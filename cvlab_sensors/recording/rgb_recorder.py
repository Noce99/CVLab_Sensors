"""Stereo RGB recorder (Basler, GigE).

Usage: rgb.py <session_folder> [--allow-unsynced]

Writes <session>/rgb_{left,right}/<host_time_us>.png, plus
  <session>/status/rgb_{left,right}.json   health, once per second
  <session>/previews/rgb_{left,right}.jpg  small preview, twice per second
  <session>/rgb_clock_sync.csv             camera <-> host clock offsets
Stop with SIGINT/SIGTERM: the frames still queued are written before exiting.
"""

import argparse
import os
import queue
import signal
import threading
import time
import traceback
from datetime import datetime

import cv2
from pypylon import pylon

from cvlab_sensors import config
from cvlab_sensors.sensors.basler import CAMERA_NAMES, measure_clock_offset, open_cameras, ptp_state
from cvlab_sensors.status import RateCounter, Throttle, write_json_atomic, write_preview_atomic

CLOCK_SYNC_PERIOD_S = 10

stop_requested = threading.Event()


def request_stop(signum, frame):
    stop_requested.set()


# ============================================================================
# CLOCKS
# ============================================================================

def wait_for_ptp(cameras, status, require_ptp, timeout_s):
    """Wait until every camera is a locked PTP slave with a small offset."""
    deadline = time.monotonic() + timeout_s
    while not stop_requested.is_set():
        states = [ptp_state(camera) for camera in cameras]
        message = " | ".join(f"{name}: {s} [{servo}] {offset} ns"
                             for name, (s, servo, offset) in zip(CAMERA_NAMES, states))
        print(f"PTP {message}")
        if all(s == "Slave" and servo == "Locked" and abs(offset) < config.RGB_PTP_MAX_OFFSET_NS
               for s, servo, offset in states):
            print("PTP synchronization complete")
            return True
        if time.monotonic() > deadline:
            if require_ptp:
                raise RuntimeError(f"PTP not synchronized after {timeout_s}s ({message}). "
                                   "Is ptp4l running? (see README)")
            print(f"WARNING: recording WITHOUT PTP sync ({message})")
            status.set_warning(f"not PTP synced: {message}")
            status.set_error(None)
            return False
        status.set_error(f"waiting for PTP: {message}")
        status.write(0)
        time.sleep(0.5)
    return False


# ============================================================================
# STATUS
# ============================================================================

class StatusReporter:
    def __init__(self, session):
        self.session = session
        self.lock = threading.Lock()
        self.saved = [RateCounter(), RateCounter()]
        self.dropped = [0, 0]     # queue full, frame not saved
        self.missed = [0, 0]      # frame never received (block id gap)
        self.failed = [0, 0]      # grab failed
        self.last_update = [None, None]
        self.error = None
        self.warning = None

    def set_error(self, message):
        with self.lock:
            self.error = message

    def set_warning(self, message):
        with self.lock:
            self.warning = message

    def frame_saved(self, idx):
        with self.lock:
            self.saved[idx].add()
            self.last_update[idx] = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def write(self, queue_size):
        try:
            self._write(queue_size)
        except OSError as e:
            print(f"WARNING: status not written: {e}")

    def _write(self, queue_size):
        with self.lock:
            for idx, name in enumerate(CAMERA_NAMES):
                write_json_atomic(os.path.join(self.session, "status", f"{name}.json"), {
                    "written_at": time.time(),
                    "fps": self.saved[idx].rate(),
                    "frames": self.saved[idx].total,
                    "dropped": self.dropped[idx],
                    "missed": self.missed[idx],
                    "failed": self.failed[idx],
                    "queue": queue_size,
                    "last_update": self.last_update[idx],
                    "error": self.error,
                    "warning": self.warning,
                })


# ============================================================================
# WRITERS
# ============================================================================

def writer_loop(frames, status):
    params = [cv2.IMWRITE_PNG_COMPRESSION, config.RGB_PNG_COMPRESSION]
    while True:
        item = frames.get()
        if item is None:
            return
        idx, path, rgb = item
        try:
            if not cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), params):
                raise IOError(f"cv2.imwrite failed for {path}")
            status.frame_saved(idx)
        except Exception as e:
            status.set_error(f"write error: {e}")
            print(f"ERROR: {e}")


# ============================================================================
# ACQUISITION
# ============================================================================

def record(session, frames, status, args, sync_log):
    cameras = open_cameras()
    try:
        require_ptp = config.RGB_REQUIRE_PTP and not args.allow_unsynced
        # When unsynced recording is allowed, only give PTP a few seconds to lock
        timeout = args.ptp_timeout if require_ptp else min(args.ptp_timeout, 5)
        wait_for_ptp(cameras, status, require_ptp, timeout)
        if stop_requested.is_set():
            return

        offsets = [None, None]

        def update_offsets():
            for idx, camera in enumerate(cameras):
                offsets[idx], uncertainty = measure_clock_offset(camera)
                sync_log.write(f"{time.time_ns()},{CAMERA_NAMES[idx]},{offsets[idx]},{uncertainty}\n")
            sync_log.flush()
            print(f"Clock offsets host-camera: left {offsets[0]} ns, right {offsets[1]} ns, "
                  f"difference {(offsets[0] - offsets[1]) / 1e6:.3f} ms")

        update_offsets()
        clock_sync = Throttle(CLOCK_SYNC_PERIOD_S)
        clock_sync.ready()
        previews = [Throttle(config.PREVIEW_PERIOD_S), Throttle(config.PREVIEW_PERIOD_S)]
        status_throttle = Throttle(config.STATUS_PERIOD_S)
        last_block_id = [None, None]
        folders = [os.path.join(session, name) for name in CAMERA_NAMES]

        cameras.StartGrabbing()
        status.set_error(None)
        print("Grabbing from left and right RGB cameras...")

        while not stop_requested.is_set():
            if status_throttle.ready():
                status.write(frames.qsize())
            if clock_sync.ready():
                update_offsets()

            grab = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            try:
                idx = grab.GetCameraContext()
                if not grab.GrabSucceeded():
                    status.failed[idx] += 1
                    print(f"Grab failed [{CAMERA_NAMES[idx]}]: {grab.GetErrorDescription()}")
                    continue

                block_id = grab.GetBlockID()
                if last_block_id[idx] is not None and block_id > last_block_id[idx] + 1:
                    status.missed[idx] += block_id - last_block_id[idx] - 1
                last_block_id[idx] = block_id

                host_us = (grab.GetTimeStamp() + offsets[idx]) // 1000
                rgb = grab.GetArray().copy()
            finally:
                grab.Release()

            if previews[idx].ready():
                try:
                    write_preview_atomic(os.path.join(session, "previews", f"{CAMERA_NAMES[idx]}.jpg"),
                                         cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                except Exception as e:
                    print(f"WARNING: preview not written: {e}")
            try:
                frames.put_nowait((idx, os.path.join(folders[idx], f"{host_us}.png"), rgb))
            except queue.Full:
                status.dropped[idx] += 1
    finally:
        if cameras.IsGrabbing():
            cameras.StopGrabbing()
        cameras.Close()


def main():
    parser = argparse.ArgumentParser(description="Stereo RGB recorder")
    parser.add_argument("session", help="Session folder (must contain rgb_left/ and rgb_right/)")
    parser.add_argument("--allow-unsynced", action="store_true",
                        help="Record even if the cameras are not PTP synchronized")
    parser.add_argument("--ptp-timeout", type=float, default=config.RGB_PTP_TIMEOUT_S,
                        help="Seconds to wait for PTP lock (default: %(default)s)")
    args = parser.parse_args()

    for name in CAMERA_NAMES + ["status", "previews"]:
        os.makedirs(os.path.join(args.session, name), exist_ok=True)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    status = StatusReporter(args.session)
    frames = queue.Queue(maxsize=config.RGB_QUEUE_SIZE)
    writers = [threading.Thread(target=writer_loop, args=(frames, status))
               for _ in range(config.RGB_WRITER_THREADS)]
    for w in writers:
        w.start()

    try:
        with open(os.path.join(args.session, "rgb_clock_sync.csv"), "a") as sync_log:
            sync_log.write("host_time_ns,camera,host_minus_camera_ns,uncertainty_ns\n")
            # Never fall back to fake frames: on failure report the error and reconnect.
            while not stop_requested.is_set():
                try:
                    record(args.session, frames, status, args, sync_log)
                except Exception as e:
                    traceback.print_exc()
                    status.set_error(str(e))
                    status.write(frames.qsize())
                    stop_requested.wait(2.0)
    finally:
        print(f"Stopping, writing {frames.qsize()} queued frames...")
        for _ in writers:
            frames.put(None)
        for w in writers:
            w.join()
        status.write(0)
        print("Done.")


if __name__ == "__main__":
    main()

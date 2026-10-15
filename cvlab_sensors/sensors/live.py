"""Live sensors for the capture tools: the Basler pair, the two EVK4s, the Livox HAP and the Hesai XT32.

SensorManager opens what the current capture page needs and releases the rest: a camera can be
opened by one process only, so nothing may be recording (see the Record page). Each sensor takes a
snapshot of its settings (cvlab_sensors.settings.store) when it opens; a sensor whose snapshot is
older than the saved settings is reopened by the next page that uses it.
"""

import collections
import multiprocessing
import os
import shutil
import signal
import subprocess
import threading
import time
import traceback

import numpy as np

from cvlab_sensors import config
from cvlab_sensors.sensors import events
from cvlab_sensors.sensors.lidar import LidarChunk, list_chunks
from cvlab_sensors.settings import store
from cvlab_sensors.status import Throttle

RETRY_DELAY_S = 2.0
RGB_BUFFER_FRAMES = 5           # newest frames kept per RGB camera
RGB_CLOCK_SYNC_PERIOD_S = 10
PTP_POLL_PERIOD_S = 5
MAX_RGB_DT_MS = 20              # left/right RGB frames further apart are not a pair
EVENT_BUFFER_US = 1_000_000     # newest events kept per event camera (~16 B each, ~320 MB at the rate cap);
                                # must cover the oldest RGB pair searched by a capture plus the event delivery delay
EVENT_SYNC_PERIOD_S = 0.1       # one (camera, host) clock pair every this
# Clock pairs kept: the camera -> host offset is the smallest one (lowest delivery
# latency) of the last 20 s. Fitting a drift on so short a window turns the delivery
# jitter (tens of ms) into a wrong slope; the real drift (tens of ppm) is ~1 ms over 20 s.
EVENT_SYNC_PAIRS = 200
EVENT_OPEN_STAGGER_S = 3.0      # opening both EVK4s at the same time fails
EVENT_WAIT_S = 1.0              # max wait for the events of the capture time to arrive
# Newest lidar points kept: must cover the longest capture plus the chunk delay.
# ~21 B per point: Livox ~450 k points/s -> ~240 MB, Hesai ~1 M points/s -> ~520 MB
LIDAR_BUFFER_S = 25
LIDAR_POLL_S = 0.05
STREAM_DIR = str(config.STREAM_DIR)   # recorder outputs, emptied as they are read


# ============================================================================
# RGB
# ============================================================================

class RgbCameras:
    """Grabs both Basler cameras, keeping the newest frames with their host time."""

    def __init__(self):
        self.lock = threading.Lock()
        self.frames = [collections.deque(maxlen=RGB_BUFFER_FRAMES) for _ in range(2)]   # (host_us, rgb)
        self.error = None
        self.ptp = [None, None]
        self.rate = [0.0, 0.0]
        self.thread = None
        self.stopping = threading.Event()
        self.settings = None   # "rgb" settings of the current opening
        self.reports = [None, None]

    def start(self):
        self.settings = store.load("rgb")
        self.reports = [None, None]
        self.stopping.clear()
        self.error = "starting"
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stopping.set()
        if self.thread is not None:
            self.thread.join(10)
            self.thread = None
        with self.lock:
            for frames in self.frames:
                frames.clear()
        self.rate = [0.0, 0.0]
        self.settings = None
        self.reports = [None, None]

    def running(self):
        return self.thread is not None

    def run(self):
        while not self.stopping.is_set():
            try:
                self.grab()
            except Exception as e:
                traceback.print_exc()
                self.error = str(e)
                self.stopping.wait(RETRY_DELAY_S)

    def grab(self):
        from pypylon import pylon

        from cvlab_sensors.sensors.basler import measure_clock_offset, open_cameras, ptp_state

        cameras, self.reports = open_cameras(self.settings)
        try:
            offsets = [measure_clock_offset(camera)[0] for camera in cameras]
            last_sync = last_ptp = 0.0
            counts, count_start = [0, 0], time.monotonic()
            cameras.StartGrabbing()
            self.error = None
            print("RGB cameras grabbing")
            while not self.stopping.is_set():
                now = time.monotonic()
                if now - last_sync > RGB_CLOCK_SYNC_PERIOD_S:
                    offsets = [measure_clock_offset(camera)[0] for camera in cameras]
                    last_sync = now
                if now - last_ptp > PTP_POLL_PERIOD_S:
                    self.ptp = [ptp_state(camera) for camera in cameras]
                    last_ptp = now
                if now - count_start > 2:
                    self.rate = [c / (now - count_start) for c in counts]
                    counts, count_start = [0, 0], now

                grab = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                try:
                    if not grab.GrabSucceeded():
                        print(f"RGB grab failed: {grab.GetErrorDescription()}")
                        continue
                    idx = grab.GetCameraContext()
                    host_us = (grab.GetTimeStamp() + offsets[idx]) // 1000
                    rgb = grab.GetArray().copy()
                finally:
                    grab.Release()
                counts[idx] += 1
                with self.lock:
                    self.frames[idx].append((host_us, rgb))
        finally:
            if cameras.IsGrabbing():
                cameras.StopGrabbing()
            cameras.Close()
            print("RGB cameras closed")

    def latest(self, idx):
        with self.lock:
            return self.frames[idx][-1] if self.frames[idx] else None

    def applied(self, idx):
        """Applied settings report of a camera (cvlab_sensors.settings.store.applied_report), None before it opens."""
        return self.reports[idx]

    def latest_pairs(self, n):
        """Up to n (t_left, left, t_right, right) pairs, newest first, each with the right
        frame closest in time to the left one."""
        with self.lock:
            left, right = list(self.frames[0]), list(self.frames[1])
        pairs = []
        for t_left, img_left in reversed(left):
            if right and len(pairs) < n:
                t_right, img_right = min(right, key=lambda f: abs(f[0] - t_left))
                if abs(t_right - t_left) <= MAX_RGB_DT_MS * 1000:
                    pairs.append((t_left, img_left, t_right, img_right))
        if not pairs and left and right:
            # Not synchronized (no PTP): take the newest frames anyway, the board must be still
            pairs.append((left[-1][0], left[-1][1], right[-1][0], right[-1][1]))
        return pairs

    def status(self, idx):
        ptp = self.ptp[idx]
        return {
            "rate": round(self.rate[idx], 1),
            "unit": "fps",
            "error": self.error,
            "warning": None if ptp is None or (ptp[0] == "Slave" and ptp[1] == "Locked")
            else f"PTP {ptp[0]} [{ptp[1]}]: left and right are not synchronized",
        }


# ============================================================================
# EVENT
# ============================================================================

class EventCamera(threading.Thread):
    """Streams one EVK4, keeping the newest events and a camera -> host clock mapping."""

    def __init__(self, serial, settings):
        super().__init__(daemon=True)
        self.serial = serial
        self.settings = settings   # "event" settings
        self.report = None         # what the camera applied
        self.lock = threading.Lock()
        self.batches = collections.deque()        # event arrays, camera time
        self.sync = collections.deque(maxlen=EVENT_SYNC_PAIRS)   # host_us - camera_us
        self.size = None                          # (width, height)
        self.error = None
        self.rate = 0.0

    def run(self):
        while True:
            try:
                self.stream()
            except Exception as e:
                traceback.print_exc()
                self.error = f"event camera {self.serial}: {e}"
                time.sleep(RETRY_DELAY_S)

    def stream(self):
        from metavision_core.event_io import EventsIterator
        from metavision_core.event_io.raw_reader import initiate_device

        from cvlab_sensors.sensors.evk4 import DELTA_T_US, configure

        device = initiate_device(path=self.serial, do_time_shifting=False)
        try:
            self.report = configure(device, self.serial, self.settings, self.settings["calibration_max_rate_mev"])
            iterator = EventsIterator.from_device(device=device, delta_t=DELTA_T_US)
            height, width = iterator.get_size()
            self.size = (width, height)
            self.error = None
            print(f"Event camera {self.serial} streaming")
            count, count_start = 0, time.monotonic()
            sync_throttle = Throttle(EVENT_SYNC_PERIOD_S)
            for evs in iterator:
                host_ns = time.time_ns()
                now = time.monotonic()
                count += len(evs)
                if now - count_start > 2:
                    self.rate, count, count_start = count / (now - count_start) / 1e6, 0, now
                if not len(evs):
                    continue
                last_t = int(evs["t"][-1])
                with self.lock:
                    self.batches.append(evs.copy())
                    if sync_throttle.ready():
                        self.sync.append(host_ns // 1000 - last_t)
                    while self.batches and self.batches[0]["t"][-1] < last_t - EVENT_BUFFER_US:
                        self.batches.popleft()
        finally:
            del device

    def offset_us(self):
        """host_us - camera_us, or None before the first events."""
        with self.lock:
            return min(self.sync) if self.sync else None

    def latest_t(self):
        with self.lock:
            return int(self.batches[-1]["t"][-1]) if self.batches else None

    def window(self, end_us, window_us=events.CALIBRATION_WINDOW_US):
        with self.lock:
            batches = [b for b in self.batches if b["t"][-1] > end_us - window_us and b["t"][0] <= end_us]
        if not batches:
            return np.zeros(0, dtype=[("x", np.uint16), ("y", np.uint16), ("p", np.int16), ("t", np.int64)])
        # Batches are sorted in time: slice them instead of masking (~60 Mev/s, keep this cheap)
        return np.concatenate([b[np.searchsorted(b["t"], end_us - window_us, "right"):
                                 np.searchsorted(b["t"], end_us, "right")] for b in batches])

    def frames_at(self, host_times_us):
        """[events.calibration_frames] of the events ending at each host time, or None."""
        offset = self.offset_us()
        if offset is None or self.size is None:
            return None
        self.wait_for(max(host_times_us) - offset)
        return [events.calibration_frames(self.window(t - offset, events.CALIBRATION_FRAMES_WINDOW_US), *self.size)
                for t in host_times_us]

    def wait_for(self, cam_us):
        """Wait until the events up to cam_us have arrived."""
        deadline = time.monotonic() + EVENT_WAIT_S
        while (self.latest_t() or 0) < cam_us and time.monotonic() < deadline:
            time.sleep(0.01)
        if (self.latest_t() or 0) < cam_us:
            lag_ms = (cam_us - (self.latest_t() or 0)) / 1e3 + EVENT_WAIT_S * 1e3
            raise RuntimeError(f"its events arrive {lag_ms:.0f} ms late ({self.rate:.0f} Mev/s): "
                               "lower the live tools rate cap on the Settings page")

    def events_before(self, host_us, duration_us):
        """Raw events of the duration_us ending at host_us (to debug failed captures)."""
        self.wait_for(host_us - self.offset_us())
        return self.window(host_us - self.offset_us(), duration_us)

    def latest_frame(self):
        """(host_us, calibration frame) of the newest events, or None."""
        t = self.latest_t()
        offset = self.offset_us()
        if t is None or offset is None or self.size is None:
            return None
        return t + offset, events.calibration_frame(self.window(t), *self.size)

    def preview_frame(self, window_us=33_000):
        """ON / OFF image of the newest events (white / orange), or None."""
        t = self.latest_t()
        if t is None or self.size is None:
            return None
        return events.colored_frame(self.window(t, window_us), *self.size)

    def applied(self):
        return self.report

    def status(self):
        cap = self.settings["calibration_max_rate_mev"]
        capped = cap is not None and self.rate > 0.9 * cap
        notes = self.report["notes"] if self.report else []
        warning = (f"event rate at the {cap:.0f} Mev/s cap: the camera is dropping events "
                   "(flickering light?)" if capped else None)
        return {"rate": round(self.rate, 2), "unit": "Mev/s", "error": self.error,
                "warning": "; ".join(filter(None, [warning, *notes])) or None}


def event_camera_process(serial, settings, conn):
    """Child process: streams one EVK4 and answers EventCamera method calls."""
    camera = EventCamera(serial, settings)
    camera.start()
    while True:
        try:
            method, args = conn.recv()
        except EOFError:   # parent gone
            os._exit(0)
        try:
            conn.send(("ok", getattr(camera, method)(*args)))
        except Exception as e:
            conn.send(("error", repr(e)))


class EventCameraProcess:
    """EventCamera in its own process.

    In one process, the two event cameras and the RGB grabber compete for the GIL:
    event batches are then read slower than real time and lag by hundreds of ms.
    """

    def __init__(self, serial):
        self.serial = serial
        self.lock = threading.Lock()
        self.conn = None
        self.process = None
        self.settings = None   # "event" settings of the current opening

    def start(self):
        self.settings = store.load("event")
        context = multiprocessing.get_context("spawn")
        self.conn, child = context.Pipe()
        self.process = context.Process(target=event_camera_process, args=(self.serial, self.settings, child),
                                       daemon=True)
        self.process.start()

    def stop(self):
        """Release the camera now (another tool can open it right away)."""
        if self.process is not None:
            self.process.terminate()
            self.process.join(5)
            self.process = None
        self.settings = None

    def running(self):
        return self.process is not None

    def call(self, method, *args):
        if self.process is None:
            raise RuntimeError(f"event camera {self.serial} is not open")
        with self.lock:
            self.conn.send((method, args))
            kind, value = self.conn.recv()
        if kind == "error":
            raise RuntimeError(f"event camera {self.serial}: {value}")
        return value

    def frames_at(self, host_times_us):
        return self.call("frames_at", host_times_us)

    def latest_frame(self):
        return self.call("latest_frame")

    def events_before(self, host_us, duration_us):
        return self.call("events_before", host_us, duration_us)

    def preview_frame(self):
        return self.call("preview_frame")

    def applied(self):
        return self.call("applied") if self.process is not None else None

    def status(self):
        if self.process is None:
            return {"rate": 0, "unit": "Mev/s", "error": "starting", "warning": None}
        return self.call("status")


# ============================================================================
# LIDARS
# ============================================================================

class LidarStream:
    """Runs a lidar recorder (lidar_recorders/save_point_cloud_{livox,hesai}) into STREAM_DIR/<name> and
    reads each H5 chunk (Livox 100 k points ~0.2 s, Hesai 300 k ~0.3 s) as soon as it is complete,
    keeping the newest LIDAR_BUFFER_S of points in memory: camera-convention xyz (mm), intensity and
    host time (us)."""

    def __init__(self, name, command):
        self.name = name
        self.command = command   # folder -> recorder command line
        self.folder = os.path.join(STREAM_DIR, name)
        self.lock = threading.Lock()
        self.batches = collections.deque()   # (t_first_us, t_last_us, xyz (N, 3) float32, intensity (N,), t_us (N,))
        self.process = None
        self.log = None
        self.thread = None
        self.stopping = threading.Event()
        self.error = None
        self.rate = 0.0   # points / s

    def start(self):
        shutil.rmtree(self.folder, ignore_errors=True)
        os.makedirs(self.folder)
        self.stopping.clear()
        self.error = "starting"
        self.log = open(os.path.join(STREAM_DIR, f"{self.name}_recorder.log"), "ab")
        command = [str(c) for c in self.command(self.folder)]
        # Own process group: we stop it with SIGINT and wait for it to flush, like cvlab_sensors.recording
        self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT, start_new_session=True)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.stopping.set()
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process = None
        if self.thread is not None:
            self.thread.join(5)
            self.thread = None
        if self.log is not None:
            self.log.close()
            self.log = None
        with self.lock:
            self.batches.clear()
        self.rate = 0.0
        shutil.rmtree(self.folder, ignore_errors=True)

    def running(self):
        return self.process is not None

    def run(self):
        count, count_start = 0, time.monotonic()
        while not self.stopping.is_set():
            code = self.process.poll()
            if code is not None:
                self.error = f"{self.name} recorder exited with code {code} (see {STREAM_DIR}/{self.name}_recorder.log)"
                return
            for _, path in list_chunks(self.folder):
                try:
                    count += self.read(path)
                except Exception as e:
                    traceback.print_exc()
                    self.error = f"unreadable {self.name} chunk: {e}"
                finally:
                    os.remove(path)
            now = time.monotonic()
            if now - count_start > 2:
                self.rate, count, count_start = count / (now - count_start), 0, now
            self.stopping.wait(LIDAR_POLL_S)

    def read(self, path):
        chunk = LidarChunk(path)
        host_ns = chunk.host_timestamps_ns()
        if host_ns is None or not len(host_ns):
            return 0
        t_us = (host_ns // 1000).astype(np.int64)
        batch = (int(t_us[0]), int(t_us[-1]), chunk.xyz_camera_mm().astype(np.float32),
                 chunk.intensity.astype(np.uint8), t_us)
        with self.lock:
            self.batches.append(batch)
            while self.batches and self.batches[0][1] < batch[1] - LIDAR_BUFFER_S * 1_000_000:
                self.batches.popleft()
        self.error = None
        return len(t_us)

    def latest_us(self):
        with self.lock:
            return self.batches[-1][1] if self.batches else None

    def points_between(self, t0_us, t1_us):
        """(xyz (N, 3) mm camera convention, intensity (N,), t_us (N,)) with t0_us <= t <= t1_us."""
        with self.lock:
            batches = [b for b in self.batches if b[1] >= t0_us and b[0] <= t1_us]
        if not batches:
            return np.zeros((0, 3), np.float32), np.zeros(0, np.uint8), np.zeros(0, np.int64)
        xyz, intensity, t = (np.concatenate(parts) for parts in zip(*(b[2:] for b in batches)))
        keep = (t >= t0_us) & (t <= t1_us)
        return xyz[keep], intensity[keep], t[keep]

    def status(self):
        latest = self.latest_us()
        warning = None
        if latest is not None:
            age_s = time.time() - latest / 1e6
            if age_s > 2:
                warning = f"newest points are {age_s:.1f} s old"
        return {"rate": round(self.rate / 1000), "unit": "k points/s", "error": self.error, "warning": warning}


# ============================================================================
# MANAGER
# ============================================================================

def stop_stale_recorders():
    """Stop the lidar recorders a previous run left writing into STREAM_DIR (a server killed without
    its cleanup leaves them running: nothing reads their chunks and the disk fills up), and empty it."""
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmdline = f.read().decode(errors="replace")
        except OSError:
            continue
        if STREAM_DIR in cmdline and "save_point_cloud" in cmdline:
            print(f"Stopping a stale lidar recorder (pid {pid}) from a previous run")
            try:
                os.kill(int(pid), signal.SIGINT)
                for _ in range(50):
                    os.kill(int(pid), 0)
                    time.sleep(0.1)
                os.kill(int(pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    shutil.rmtree(STREAM_DIR, ignore_errors=True)


class SensorManager:
    """Opens the sensors a tool needs and closes the others, in the background (opening the
    event cameras takes several seconds)."""

    NEEDS = {"rgb_event": {"rgb", "event"}, "rgb_livox": {"rgb", "livox"}, "livox_hesai": {"livox", "hesai"},
             "settings": {"rgb", "event"}, None: set()}

    def __init__(self):
        stop_stale_recorders()
        self.rgb = RgbCameras()
        self.event = {"event_left": EventCameraProcess(config.EVENT_LEFT_SERIAL),
                      "event_right": EventCameraProcess(config.EVENT_RIGHT_SERIAL)}
        self.livox = LidarStream("livox", lambda folder: [config.LIVOX_RECORDER, folder, config.LIVOX_CONFIG])
        self.hesai = LidarStream("hesai", lambda folder: [config.HESAI_RECORDER, folder, config.HESAI_IP])
        self.lock = threading.Lock()
        self.tool = None
        self.blocker = lambda: None   # -> why the sensors cannot be opened now (a recording runs), or None

    def blocked(self):
        return self.blocker()

    def use(self, tool):
        """Switch to the sensors of tool (a key of NEEDS); returns at once. Nothing is opened while
        blocked (see blocker)."""
        if tool not in self.NEEDS:
            raise ValueError(f"unknown tool {tool!r}")
        if tool is not None and self.blocked():
            return
        self.tool = tool
        threading.Thread(target=self._apply, args=(tool,), daemon=True).start()

    def _apply(self, tool):
        with self.lock:
            if tool != self.tool:   # superseded by a newer request
                return
            needed = self.NEEDS[tool]
            # Opened with settings saved since: reopen them with the new ones
            for cam in self.event.values():
                if "event" not in needed or (cam.running() and cam.settings != store.load("event")):
                    cam.stop()
            for lidar in ("livox", "hesai"):
                if lidar not in needed and getattr(self, lidar).running():
                    getattr(self, lidar).stop()
            if self.rgb.running() and ("rgb" not in needed or self.rgb.settings != store.load("rgb")):
                self.rgb.stop()
            if "rgb" in needed and not self.rgb.running():
                self.rgb.start()
            for lidar in ("livox", "hesai"):
                if lidar in needed and not getattr(self, lidar).running():
                    getattr(self, lidar).start()
            if "event" in needed:
                for cam in self.event.values():
                    if not cam.running():
                        cam.start()
                        time.sleep(EVENT_OPEN_STAGGER_S)

    def close(self):
        self.tool = None
        self._apply(None)

"""Event camera recorder (Prophesee EVK4).

Usage: event.py <session_folder> [--right-camera]

Writes one RAW file per recording into <session>/event_{left,right}/:
  <host_start_us>.raw            events (+ external triggers if EVENT_TRIGGER_IN)
  <host_start_us>.sync.csv       [camera_t_us, host_ns] pairs, ~10 per second
  <host_start_us>.json           metadata
cvlab.events.EventRecording maps the camera clock to the host clock from the sync
pairs (cvlab.timesync: lower envelope fit, clock drift included).
Also writes <session>/status/event_*.json and <session>/previews/event_*.jpg.
Stop with SIGINT/SIGTERM.
"""

import argparse
import glob
import json
import os
import signal
import sys
import threading
import time
import traceback
from datetime import datetime

from metavision_core.event_io import EventsIterator
from metavision_core.event_io.raw_reader import initiate_device
import metavision_hal

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config
from cvlab.events import colored_frame
from cvlab.status import Throttle, write_json_atomic, write_preview_atomic

SYNC_PERIOD_S = 0.1
DELTA_T_US = 10_000
RETRY_DELAY_S = 2.0

stop_requested = threading.Event()


def request_stop(signum, frame):
    stop_requested.set()


def evk4_usb_speeds_mbps():
    """USB link speed of every connected EVK4 (they expose no serial in sysfs)."""
    speeds = []
    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(dev, "idVendor")) as f:
                vendor = f.read().strip()
            with open(os.path.join(dev, "idProduct")) as f:
                product = f.read().strip()
            if (vendor, product) == ("04b4", "00f5"):
                with open(os.path.join(dev, "speed")) as f:
                    speeds.append(int(f.read().strip()))
        except (OSError, ValueError):
            pass
    return speeds


class Status:
    def __init__(self, session, name):
        self.path = os.path.join(session, "status", f"{name}.json")
        self.events = 0
        self._last_events = 0
        self._last_time = time.monotonic()
        self.last_update = None
        self.error = None
        self.warning = None

    def write(self):
        try:
            self._write()
        except OSError as e:
            print(f"WARNING: status not written: {e}")

    def _write(self):
        now = time.monotonic()
        rate = (self.events - self._last_events) / max(now - self._last_time, 1e-6)
        self._last_events, self._last_time = self.events, now
        write_json_atomic(self.path, {
            "written_at": time.time(),
            "fps": rate / 1e6,
            "unit": "Mev/s",
            "events": self.events,
            "last_update": self.last_update,
            "error": self.error,
            "warning": self.warning,
        })


def open_device(serial, status):
    while not stop_requested.is_set():
        try:
            return initiate_device(path=serial, do_time_shifting=False)
        except Exception as e:
            status.error = f"cannot open event camera {serial}: {e}"
            status.write()
            print(f"{status.error}. Retrying in {RETRY_DELAY_S}s...")
            stop_requested.wait(RETRY_DELAY_S)
    return None


def record(serial, name, session, status):
    device = open_device(serial, status)
    if device is None:
        return

    events_stream = device.get_i_events_stream()
    try:
        hot_pixels = config.EVENT_HOT_PIXELS.get(serial, [])
        if hot_pixels:
            masks = device.get_i_digital_event_mask().get_pixel_masks()
            if len(hot_pixels) > len(masks):
                print(f"WARNING: {len(hot_pixels)} hot pixels but only {len(masks)} hardware masks")
            for mask, (x, y) in zip(masks, hot_pixels):
                mask.set_mask(x, y, True)
            print(f"Masked hot pixels: {hot_pixels[:len(masks)]}")

        if config.EVENT_TRIGGER_IN:
            trigger_in = device.get_i_trigger_in()
            trigger_in.enable(metavision_hal.I_TriggerIn.Channel.MAIN)
            print("Trigger input MAIN enabled")

        out_dir = os.path.join(session, name)
        start_us = time.time_ns() // 1000
        base = os.path.join(out_dir, str(start_us))
        mv_iterator = EventsIterator.from_device(device=device, delta_t=DELTA_T_US)
        height, width = mv_iterator.get_size()
        events_stream.log_raw_data(base + ".raw")

        metadata = {
            "serial": serial,
            "host_start_us": start_us,
            "width": width,
            "height": height,
            "trigger_in": config.EVENT_TRIGGER_IN,
            "masked_pixels": hot_pixels,
            "usb_speeds_mbps": evk4_usb_speeds_mbps(),
        }
        with open(base + ".json", "w") as f:
            json.dump(metadata, f, indent=2)

        sync_throttle = Throttle(SYNC_PERIOD_S)
        preview_throttle = Throttle(config.PREVIEW_PERIOD_S)
        status_throttle = Throttle(config.STATUS_PERIOD_S)
        status.error = None
        print(f"Recording {name} ({serial}) to {base}.raw")

        with open(base + ".sync.csv", "w") as sync_log:
            sync_log.write("camera_t_us,host_ns\n")
            for evs in mv_iterator:
                host_ns = time.time_ns()
                if stop_requested.is_set():
                    break
                if len(evs) > 0:
                    status.events += len(evs)
                    status.last_update = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    if sync_throttle.ready():
                        sync_log.write(f"{int(evs['t'][-1])},{host_ns}\n")
                    if preview_throttle.ready():
                        try:
                            write_preview_atomic(os.path.join(session, "previews", f"{name}.jpg"),
                                                 colored_frame(evs, width, height))
                        except Exception as e:
                            print(f"WARNING: preview not written: {e}")
                if status_throttle.ready():
                    status.write()
                    sync_log.flush()
    finally:
        events_stream.stop_log_raw_data()
        events_stream.stop()
        del device


def main():
    parser = argparse.ArgumentParser(description="Event camera recorder")
    parser.add_argument("session", help="Session folder (must contain event_left/ and event_right/)")
    parser.add_argument("--right-camera", action="store_true", dest="use_right_camera",
                        help="Use right camera (default: left camera)")
    args = parser.parse_args()

    name = "event_right" if args.use_right_camera else "event_left"
    serial = config.EVENT_RIGHT_SERIAL if args.use_right_camera else config.EVENT_LEFT_SERIAL
    for sub in [name, "status", "previews"]:
        os.makedirs(os.path.join(args.session, sub), exist_ok=True)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    status = Status(args.session, name)
    slow_links = [s for s in evk4_usb_speeds_mbps() if s < 5000]
    if slow_links:
        status.warning = f"EVK4 connected at {slow_links} Mbps (USB2): events may be dropped, use a USB3 port"
        print(f"WARNING: {status.warning}")

    # Never fall back to fake frames: on failure report the error and reconnect.
    while not stop_requested.is_set():
        try:
            record(serial, name, args.session, status)
        except Exception as e:
            traceback.print_exc()
            status.error = str(e)
            status.write()
            stop_requested.wait(RETRY_DELAY_S)

    status.write()
    print("Done.")


if __name__ == "__main__":
    main()

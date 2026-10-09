"""Live RGB-event calibration tool (web).

Usage: calibration_server.py [--resume [processed_data/<session>]] [--port 5001]
Open http://<NUC_IP>:5001

Keeps the two Basler and the two EVK4 cameras open and shows them live. Each
"Capture" takes the newest RGB pair and the event frames ending at the same host
time, and looks for the chessboard in all four. When all four see it, the corners
are saved to processed_data/<session>/<index>/ in the format of
process_calibration_data.py, so calibrate.py can run on the folder directly.

The recorders (get_data.py / Cameras_Server) must be stopped: a camera can be
opened by one process only.
"""

import argparse
import base64
import collections
import io
import multiprocessing
import os
import shutil
import signal
import sys
import threading
import time
import traceback
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import cv2
import numpy as np
from flask import Flask, jsonify, render_template, send_file

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config, events
from cvlab.calibration import find_corners_variants, find_event_corners, to_calibration_size
from cvlab.status import Throttle
from calibrate import CAMERAS, DEDUP_PX
from process_calibration_data import PROCESSED_DATA_PATH

RETRY_DELAY_S = 2.0
RGB_BUFFER_FRAMES = 5           # newest frames kept per RGB camera
# RGB pairs searched per capture, newest first: a blinking screen caught while it redraws
# shows no board, the previous pair (0.1 s before) usually does
RGB_CAPTURE_PAIRS = 3
STILL_BOARD_PX = 1.0            # corners closer than this across pairs: the board did not move
RGB_CLOCK_SYNC_PERIOD_S = 10
PTP_POLL_PERIOD_S = 5
MAX_RGB_DT_MS = 20              # left/right RGB frames further apart are not a pair
EVENT_BUFFER_US = 1_000_000     # newest events kept per event camera (~16 B each, ~320 MB at the rate cap);
                                # must cover the oldest of RGB_CAPTURE_PAIRS plus the event delivery delay
EVENT_SYNC_PERIOD_S = 0.1       # one (camera, host) clock pair every this
# Clock pairs kept: the camera -> host offset is the smallest one (lowest delivery
# latency) of the last 20 s. Fitting a drift on so short a window turns the delivery
# jitter (tens of ms) into a wrong slope; the real drift (tens of ppm) is ~1 ms over 20 s.
EVENT_SYNC_PAIRS = 200
EVENT_OPEN_STAGGER_S = 3.0      # opening both EVK4s at the same time fails
EVENT_WAIT_S = 1.0              # max wait for the events of the capture time to arrive
PREVIEW_SIZE = (640, 360)

app = Flask(__name__)


def jpeg(image, quality=80):
    return cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()


def data_url(image):
    return "data:image/jpeg;base64," + base64.b64encode(jpeg(image)).decode()


def preview(image):
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return cv2.resize(image, PREVIEW_SIZE, interpolation=cv2.INTER_AREA)


# ============================================================================
# CAMERAS
# ============================================================================

class RgbCameras(threading.Thread):
    """Grabs both Basler cameras, keeping the newest frames with their host time."""

    def __init__(self):
        super().__init__(daemon=True)
        self.lock = threading.Lock()
        self.frames = [collections.deque(maxlen=RGB_BUFFER_FRAMES) for _ in range(2)]   # (host_us, rgb)
        self.error = None
        self.ptp = [None, None]
        self.rate = [0.0, 0.0]

    def run(self):
        while True:
            try:
                self.grab()
            except Exception as e:
                traceback.print_exc()
                self.error = str(e)
                time.sleep(RETRY_DELAY_S)

    def grab(self):
        from pypylon import pylon
        from get_data.rgb import measure_clock_offset, open_cameras, ptp_state

        cameras = open_cameras()
        try:
            offsets = [measure_clock_offset(camera)[0] for camera in cameras]
            last_sync = last_ptp = 0.0
            counts, count_start = [0, 0], time.monotonic()
            cameras.StartGrabbing()
            self.error = None
            print("RGB cameras grabbing")
            while True:
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

    def latest(self, idx):
        with self.lock:
            return self.frames[idx][-1] if self.frames[idx] else None

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


class EventCamera(threading.Thread):
    """Streams one EVK4, keeping the newest events and a camera -> host clock mapping."""

    def __init__(self, serial):
        super().__init__(daemon=True)
        self.serial = serial
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
        from get_data.event import DELTA_T_US, limit_event_rate, mask_hot_pixels

        device = initiate_device(path=self.serial, do_time_shifting=False)
        try:
            mask_hot_pixels(device, self.serial)
            limit_event_rate(device, config.EVENT_CALIBRATION_MAX_RATE)
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
                               "lower EVENT_CALIBRATION_MAX_RATE in cvlab/config.py")

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

    def status(self):
        cap = config.EVENT_CALIBRATION_MAX_RATE
        capped = cap is not None and self.rate * 1e6 > 0.9 * cap
        return {"rate": round(self.rate, 2), "unit": "Mev/s", "error": self.error,
                "warning": f"event rate at the {cap / 1e6:.0f} Mev/s cap: the camera is dropping events "
                           "(flickering light?)" if capped else None}


def event_camera_process(serial, conn):
    """Child process: streams one EVK4 and answers EventCamera method calls."""
    camera = EventCamera(serial)
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

    def start(self):
        context = multiprocessing.get_context("spawn")
        self.conn, child = context.Pipe()
        self.process = context.Process(target=event_camera_process, args=(self.serial, child), daemon=True)
        self.process.start()

    def stop(self):
        """Release the camera now (the next server can open it right away)."""
        if self.process is not None:
            self.process.terminate()
            self.process.join(5)

    def call(self, method, *args):
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

    def status(self):
        return self.call("status")


# ============================================================================
# SAMPLES
# ============================================================================

class Samples:
    """Captured boards in processed_data/<session>/<index>/ (format of process_calibration_data.py)."""

    def __init__(self, folder):
        self.folder = folder   # created with the first board: calibrate.py picks the latest folder
        self.lock = threading.Lock()
        self.corners = {}   # name -> {cam: (N, 2)}
        for name in sorted(os.listdir(folder)) if os.path.isdir(folder) else []:
            paths = {cam: os.path.join(folder, name, f"{cam}_corners.npy") for cam in CAMERAS}
            if all(os.path.exists(p) for p in paths.values()):
                self.corners[name] = {cam: np.load(p).reshape(-1, 2) for cam, p in paths.items()}

    def duplicate_of(self, corners):
        """Name of a sample calibrate.py would consider the same board (see calibrate.deduplicate)."""
        c = corners["rgb_left"]
        d = np.concatenate([c[0], c[-1]])
        for name, other in self.corners.items():
            o = other["rgb_left"]
            if np.max(np.abs(d - np.concatenate([o[0], o[-1]]))) <= DEDUP_PX:
                return name
        return None

    def add(self, corners, images, rgb_full, log):
        with self.lock:
            index = max((int(n) for n in self.corners), default=-1) + 1
            name = f"{index:05d}"
            sample = os.path.join(self.folder, name)
            os.makedirs(sample)
            for cam, c in corners.items():
                np.save(os.path.join(sample, f"{cam}_corners.npy"), c)
                cv2.imwrite(os.path.join(sample, f"{cam}_corners.png"), draw(images[cam], c))
                cv2.imwrite(os.path.join(sample, f"{cam}.png"), images[cam])
            # Full resolution RGB, to re-process with other settings later
            for cam, rgb in rgb_full.items():
                cv2.imwrite(os.path.join(sample, f"{cam}_full.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                            [cv2.IMWRITE_PNG_COMPRESSION, config.RGB_PNG_COMPRESSION])
            with open(os.path.join(sample, "log.log"), "w") as f:
                f.write(log)
            self.corners[name] = corners
            return name

    def delete(self, name):
        with self.lock:
            if name not in self.corners:
                return False
            shutil.rmtree(os.path.join(self.folder, name))
            del self.corners[name]
            return True

    def as_json(self):
        w, h = config.CALIBRATION_IMAGE_SIZE
        with self.lock:
            return [{"name": name, "corners": {cam: (c / (w, h)).round(4).tolist() for cam, c in cs.items()}}
                    for name, cs in sorted(self.corners.items())]


def draw(gray, corners):
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if corners is not None:
        cv2.drawChessboardCorners(image, config.PATTERN_SIZE, corners.reshape(-1, 1, 2), True)
    return image


# ============================================================================
# ROUTES
# ============================================================================

rgb_cameras = RgbCameras()
event_cameras = {"event_left": EventCameraProcess(config.EVENT_LEFT_SERIAL),
                 "event_right": EventCameraProcess(config.EVENT_RIGHT_SERIAL)}
samples = None
failed_folder = None   # --save-failed: images (and raw events) of the captures where a camera missed the board
FAILED_EVENTS_US = 200_000
capture_lock = threading.Lock()
pool = ThreadPoolExecutor(max_workers=12)


@app.route("/")
def index():
    return render_template("calibration.html")


@app.route("/api/status")
def status():
    sensors = {"rgb_left": rgb_cameras.status(0), "rgb_right": rgb_cameras.status(1)}
    sensors.update({name: cam.status() for name, cam in event_cameras.items()})
    return jsonify({
        "sensors": sensors,
        "folder": os.path.relpath(samples.folder, config.REPO_ROOT),
        "pattern_size": config.PATTERN_SIZE,
        "image_size": config.CALIBRATION_IMAGE_SIZE,
    })


@app.route("/api/preview/<cam>")
def live_preview(cam):
    if cam in ("rgb_left", "rgb_right"):
        frame = rgb_cameras.latest(CAMERAS.index(cam))
    elif cam in event_cameras:
        frame = event_cameras[cam].latest_frame()
    else:
        return jsonify({"error": f"unknown camera {cam}"}), 404
    if frame is None:
        return jsonify({"error": "no frame"}), 503
    host_us, image = frame
    if cam.startswith("rgb"):
        image = to_calibration_size(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    response = send_file(io.BytesIO(jpeg(preview(image), 70)), mimetype="image/jpeg")
    response.headers["Cache-Control"] = "no-store"
    # Host time of the image (end of the event window): the page shows its age
    response.headers["X-Frame-Age-Ms"] = f"{(time.time_ns() // 1000 - host_us) / 1e3:.0f}"
    return response


@app.route("/api/samples")
def list_samples():
    return jsonify(samples.as_json())


@app.route("/api/samples/<name>", methods=["DELETE"])
def delete_sample(name):
    if not samples.delete(name):
        return jsonify({"error": f"no sample {name}"}), 404
    return jsonify({"status": "deleted"})


@app.route("/api/samples/<name>/<cam>.jpg")
def sample_image(name, cam):
    if name not in samples.corners or cam not in CAMERAS:
        return jsonify({"error": "not found"}), 404
    image = cv2.imread(os.path.join(samples.folder, name, f"{cam}_corners.png"))
    return send_file(io.BytesIO(jpeg(preview(image))), mimetype="image/jpeg")


@app.route("/api/capture", methods=["POST"])
def capture():
    if not capture_lock.acquire(blocking=False):
        return jsonify({"error": "a capture is already running"}), 409
    try:
        return jsonify(do_capture())
    except CaptureError as e:
        return jsonify({"error": str(e)}), 503
    finally:
        capture_lock.release()


class CaptureError(Exception):
    pass


def do_capture():
    clicked_us = time.time_ns() // 1000
    pairs = rgb_cameras.latest_pairs(RGB_CAPTURE_PAIRS)
    if not pairs:
        raise CaptureError(f"no RGB frames ({rgb_cameras.error or 'cameras starting'})")

    # Ask for the event frames of every candidate pair now: the RGB search can take ~1 s,
    # longer than the events kept in memory (EVENT_BUFFER_US)
    pair_times = [pair[0] for pair in pairs]
    event_futures = {name: pool.submit(cam.frames_at, pair_times) for name, cam in event_cameras.items()}
    raw_futures = {name: pool.submit(cam.events_before, pair_times[0], FAILED_EVENTS_US)
                   for name, cam in event_cameras.items()} if failed_folder else {}

    # Search every camera in every candidate pair (in parallel: OpenCV releases the GIL).
    # Each search returns (corners, variant, image the corners were found in).
    def rgb_search(rgb):
        gray = to_calibration_size(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        return (*find_corners_variants(gray, config.CHESSBOARD_VARIANTS["rgb"]), gray)

    searches = {"rgb_left": [pool.submit(rgb_search, pair[1]) for pair in pairs],
                "rgb_right": [pool.submit(rgb_search, pair[3]) for pair in pairs]}
    event_frames = {}   # cam -> [events.calibration_frames] per pair
    for name, future in event_futures.items():
        try:
            event_frames[name] = future.result()
        except RuntimeError as e:
            raise CaptureError(f"{name}: {e}")
        if event_frames[name] is None:
            raise CaptureError(f"no events for the capture time from {name} "
                               f"({event_cameras[name].status()['error'] or 'camera starting'})")
        searches[name] = [pool.submit(find_event_corners, frames) for frames in event_frames[name]]
    results = {cam: [f.result() for f in searches[cam]] for cam in CAMERAS}
    times = {"rgb_left": [p[0] for p in pairs], "rgb_right": [p[2] for p in pairs],
             "event_left": pair_times, "event_right": pair_times}

    chosen, mode = choose_pairs(results)
    found = {cam: results[cam][i] for cam, i in chosen.items()}
    corners = {cam: c for cam, (c, _, _) in found.items()}
    images = {cam: image for cam, (_, _, image) in found.items()}
    image_times = {cam: times[cam][i] for cam, i in chosen.items()}

    w, h = config.CALIBRATION_IMAGE_SIZE
    rgb_dt_ms = round((image_times["rgb_right"] - image_times["rgb_left"]) / 1e3, 2)
    result = {"cameras": {}, "rgb_dt_ms": rgb_dt_ms, "saved": None, "duplicate_of": None,
              "moving": mode is None and all(c is not None for c in corners.values())}
    for cam in CAMERAS:
        c = corners[cam]
        info = {"found": c is not None, "variant": found[cam][1],
                "age_ms": round((clicked_us - image_times[cam]) / 1e3),
                "image": data_url(preview(draw(images[cam], c))),
                "corners": None if c is None else (c / (w, h)).round(4).tolist()}
        if cam in event_frames:
            info["activation"] = round(events.activation(event_frames[cam][chosen[cam]]["count"]), 3)
        result["cameras"][cam] = info

    if mode == "same" and abs(rgb_dt_ms) > MAX_RGB_DT_MS:
        result["warning"] = (f"RGB left and right are {rgb_dt_ms} ms apart (PTP not locked?): "
                             "keep the board still")
    if failed_folder and any(c is None for c in corners.values()):
        raw = {cam: raw_futures[cam].result() for cam in event_frames if corners[cam] is None}
        result["debug_saved"] = save_failed(pair_times[0], image_times, raw, images, found)
    if mode is not None and all(c is not None for c in corners.values()):
        result["duplicate_of"] = samples.duplicate_of(corners)
        if result["duplicate_of"] is None:
            log = (f"Captured {datetime.now().isoformat()} by calibration_server.py\n"
                   + ("All views from the same RGB pair (event frames end at the rgb_left time)\n" if mode == "same"
                      else f"Board still: views from different RGB pairs (corners within {STILL_BOARD_PX} px)\n")
                   + "".join(f"{cam}: host time {image_times[cam]} us, found with "
                             f"(gamma, blur, clahe[, rendering]) = {found[cam][1]}\n" for cam in CAMERAS))
            rgb_full = {"rgb_left": pairs[chosen["rgb_left"]][1], "rgb_right": pairs[chosen["rgb_right"]][3]}
            result["saved"] = samples.add(corners, images, rgb_full, log)
    return result


def choose_pairs(results):
    """Pick the candidate pair of every camera; results[cam][i] = (corners, variant, image).

    Returns ({cam: pair index}, mode):
      "same"   the newest pair where all four cameras see the board
      "still"  a blinking screen is caught in a good phase at different moments by each camera:
               when the board is still (its corners agree within STILL_BOARD_PX in every camera
               that sees it in several pairs) each camera uses its newest detection
      None     the board moved (or nothing to combine): each camera's newest detection, to
               show, not to save
    """
    n = len(next(iter(results.values())))
    for i in range(n):
        if all(results[cam][i][0] is not None for cam in CAMERAS):
            return {cam: i for cam in CAMERAS}, "same"
    detections = {cam: [r[0] for r in results[cam] if r[0] is not None] for cam in CAMERAS}
    newest = {cam: next((i for i in range(n) if results[cam][i][0] is not None), 0) for cam in CAMERAS}
    repeated = [d for d in detections.values() if len(d) >= 2]
    still = bool(repeated) and all(np.max(np.abs(c - d[0])) <= STILL_BOARD_PX for d in repeated for c in d[1:])
    return newest, "still" if still else None


def save_failed(t_newest, image_times, raw, images, found):
    """Images of a failed capture, and raw events (FAILED_EVENTS_US ending at the newest pair) of
    the event cameras that missed the board."""
    t_left = image_times["rgb_left"]
    folder = os.path.join(failed_folder, f"{t_newest}")
    os.makedirs(folder, exist_ok=True)
    for cam, evs in raw.items():
        np.save(os.path.join(folder, f"{cam}_events.npy"), evs)
    for cam, image in images.items():
        cv2.imwrite(os.path.join(folder, f"{cam}.png"), image)
    with open(os.path.join(folder, "log.log"), "w") as f:
        f.write("".join(f"{cam}: host time {t} us\n" for cam, t in image_times.items()))
        f.write(f"Raw events: the {FAILED_EVENTS_US} us ending at {t_newest} (host)\n")
        f.write("".join(f"{cam}: {'found ' + str(found[cam][1]) if found[cam][0] is not None else 'not found'}\n"
                        for cam in CAMERAS))
    return folder


# ============================================================================
# MAIN
# ============================================================================

def main():
    global samples, failed_folder
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--resume", nargs="?", const="latest", metavar="FOLDER",
                        help="Add to an existing processed_data folder (default: the latest)")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--save-failed", action="store_true",
                        help="Save the images (and raw events) of captures where a camera misses the board "
                             "(failed_captures/<folder>/<time>/), to tune the chessboard search")
    args = parser.parse_args()

    if args.resume == "latest":
        existing = sorted(p for p in os.listdir(PROCESSED_DATA_PATH)
                          if os.listdir(os.path.join(PROCESSED_DATA_PATH, p))) if os.path.isdir(PROCESSED_DATA_PATH) else []
        if not existing:
            sys.exit(f"Nothing to resume in {PROCESSED_DATA_PATH}")
        folder = os.path.join(PROCESSED_DATA_PATH, existing[-1])
    elif args.resume:
        folder = os.path.abspath(args.resume)
    else:
        folder = os.path.join(PROCESSED_DATA_PATH, datetime.now().strftime(config.SESSION_NAME_FORMAT))
    samples = Samples(folder)
    if args.save_failed:
        failed_folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "failed_captures",
                                     os.path.basename(folder))
    print(f"Saving boards to {folder} ({len(samples.corners)} already there)")

    rgb_cameras.start()
    for cam in event_cameras.values():
        cam.start()
        time.sleep(EVENT_OPEN_STAGGER_S)

    print(f"Calibration tool on http://0.0.0.0:{args.port}")
    print(f"When done: {sys.executable} {os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibrate.py')} {folder}")
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))
    try:
        app.run(host="0.0.0.0", port=args.port, debug=True, threaded=True, use_reloader=False)
    finally:
        for cam in event_cameras.values():
            cam.stop()


if __name__ == "__main__":
    main()

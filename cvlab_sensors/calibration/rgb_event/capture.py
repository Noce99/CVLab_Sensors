"""Live RGB-event calibration data capture (page of the web app, cvlab_sensors/app.py).

Keeps the two Basler and the two EVK4 cameras open and shows them live. Each "Capture" takes the
newest RGB pair and the event frames ending at the same host time, and looks for the chessboard in
all four. When all four see it, the corners are saved to data/calibration/rgb_event/captures/<session>/<index>/,
the folder rgb_event/solve.py calibrates from.
"""

import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import cv2
import numpy as np

from cvlab_sensors import config
from cvlab_sensors.calibration.common import (CaptureError, Samples, draw, find_corners_variants, find_event_corners,
                                      to_calibration_size)
from cvlab_sensors.calibration.rgb_event.solve import CAMERAS, CAPTURES_PATH, DEDUP_PX
from cvlab_sensors.sensors import events
from cvlab_sensors.sensors.live import MAX_RGB_DT_MS
from cvlab_sensors.web import capture_blueprint, data_url, preview

# RGB pairs searched per capture, newest first: a blinking screen caught while it redraws
# shows no board, the previous pair (0.1 s before) usually does
RGB_CAPTURE_PAIRS = 3
STILL_BOARD_PX = 1.0            # corners closer than this across pairs: the board did not move
FAILED_EVENTS_US = 200_000
FAILED_CAPTURES_PATH = str(config.CALIBRATION_DIR / "rgb_event" / "failed_captures")

TOOL = {
    "id": "rgb_event",
    "title": "RGB-event calibration data",
    "cameras": CAMERAS,
    "labels": {"rgb_left": "RGB left", "rgb_right": "RGB right", "event_left": "Event left", "event_right": "Event right"},
    # Pause between two previews of a camera. An event preview costs ~100 ms of
    # the camera process, which also has to keep up with up to 60 Mev/s.
    "preview_delay_ms": {"rgb_left": 150, "rgb_right": 150, "event_left": 400, "event_right": 400},
    "aspect": {cam: list(config.CALIBRATION_IMAGE_SIZE) for cam in CAMERAS},
    "pattern": True,          # blinking chessboard: event cameras only see what changes
    "save_failed": True,
    "timed_capture": False,
    "solve_url": "/rgb-event/calibrate",
}


class RgbEventCapture:
    TOOL = TOOL

    def __init__(self, sensors):
        self.sensors = sensors
        self.samples = None
        self.failed_folder = None   # images (and raw events) of the captures where a camera missed the board
        self.capture_lock = threading.Lock()
        self.pool = ThreadPoolExecutor(max_workers=12)
        self.open_session(None)

    def open_session(self, name):
        """name: an existing captures folder to add to, or None for a new one."""
        if name is None:
            name = datetime.now().strftime(config.SESSION_NAME_FORMAT)
        elif name not in os.listdir(CAPTURES_PATH):
            raise ValueError(f"no session {name!r}")
        image_sizes = {cam: config.CALIBRATION_IMAGE_SIZE for cam in CAMERAS}
        # A new session keeps the square size entered for the previous one (same board, usually)
        square = self.samples.square_size_mm if self.samples is not None else None
        self.samples = Samples(os.path.join(CAPTURES_PATH, name), CAMERAS, image_sizes, DEDUP_PX,
                               square_size_mm=square)
        if self.failed_folder is not None:
            self.set_save_failed(True)

    def set_save_failed(self, on):
        self.failed_folder = os.path.join(FAILED_CAPTURES_PATH, os.path.basename(self.samples.folder)) if on else None

    def status(self):
        s = self.sensors
        sensors = {"rgb_left": s.rgb.status(0), "rgb_right": s.rgb.status(1)}
        for name, cam in s.event.items():
            try:
                sensors[name] = cam.status()
            except RuntimeError as e:
                sensors[name] = {"rate": 0, "unit": "Mev/s", "error": str(e), "warning": None}
        return sensors

    def latest(self, cam):
        """(host_us, BGR or gray image) of the newest frame of a camera, or None."""
        if cam in ("rgb_left", "rgb_right"):
            frame = self.sensors.rgb.latest(CAMERAS.index(cam))
            if frame is None:
                return None
            return frame[0], to_calibration_size(cv2.cvtColor(frame[1], cv2.COLOR_RGB2BGR))
        return self.sensors.event[cam].latest_frame()

    def capture(self):
        clicked_us = time.time_ns() // 1000
        rgb, event_cameras, pool = self.sensors.rgb, self.sensors.event, self.pool
        pairs = rgb.latest_pairs(RGB_CAPTURE_PAIRS)
        if not pairs:
            raise CaptureError(f"no RGB frames ({rgb.error or 'cameras starting'})")

        # Ask for the event frames of every candidate pair now: the RGB search can take ~1 s,
        # longer than the events kept in memory (EVENT_BUFFER_US)
        pair_times = [pair[0] for pair in pairs]
        event_futures = {name: pool.submit(cam.frames_at, pair_times) for name, cam in event_cameras.items()}
        raw_futures = {name: pool.submit(cam.events_before, pair_times[0], FAILED_EVENTS_US)
                       for name, cam in event_cameras.items()} if self.failed_folder else {}

        # Search every camera in every candidate pair (in parallel: OpenCV releases the GIL).
        # Each search returns (corners, variant, image the corners were found in).
        def rgb_search(image):
            gray = to_calibration_size(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
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

        rgb_dt_ms = round((image_times["rgb_right"] - image_times["rgb_left"]) / 1e3, 2)
        result = {"cameras": {}, "rgb_dt_ms": rgb_dt_ms, "saved": None, "duplicate_of": None,
                  "moving": mode is None and all(c is not None for c in corners.values())}
        for cam in CAMERAS:
            c = corners[cam]
            info = {"found": c is not None, "variant": found[cam][1],
                    "age_ms": round((clicked_us - image_times[cam]) / 1e3),
                    "image": data_url(preview(draw(images[cam], c))),
                    "corners": self.samples.normalized(cam, c)}
            if cam in event_frames:
                info["activation"] = round(events.activation(event_frames[cam][chosen[cam]]["count"]), 3)
            result["cameras"][cam] = info

        if mode == "same" and abs(rgb_dt_ms) > MAX_RGB_DT_MS:
            result["warning"] = (f"RGB left and right are {rgb_dt_ms} ms apart (PTP not locked?): "
                                 "keep the board still")
        if self.failed_folder and any(c is None for c in corners.values()):
            raw = {cam: raw_futures[cam].result() for cam in event_frames if corners[cam] is None}
            result["debug_saved"] = self.save_failed(pair_times[0], image_times, raw, images, found)
        if mode is not None and all(c is not None for c in corners.values()):
            result["duplicate_of"] = self.samples.duplicate_of(corners)
            if result["duplicate_of"] is None:
                log = (f"Captured {datetime.now().isoformat()} by cvlab_sensors/calibration/rgb_event/capture.py\n"
                       + ("All views from the same RGB pair (event frames end at the rgb_left time)\n" if mode == "same"
                          else f"Board still: views from different RGB pairs (corners within {STILL_BOARD_PX} px)\n")
                       + "".join(f"{cam}: host time {image_times[cam]} us, found with "
                                 f"(gamma, blur, clahe[, rendering]) = {found[cam][1]}\n" for cam in CAMERAS))
                rgb_full = {"rgb_left": pairs[chosen["rgb_left"]][1], "rgb_right": pairs[chosen["rgb_right"]][3]}

                def save_full(sample):
                    # Full resolution RGB, to re-process with other settings later
                    for cam, image in rgb_full.items():
                        cv2.imwrite(os.path.join(sample, f"{cam}_full.png"), cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                                    [cv2.IMWRITE_PNG_COMPRESSION, config.RGB_PNG_COMPRESSION])

                result["saved"] = self.samples.add(corners, images, log, save_full)
        return result

    def save_failed(self, t_newest, image_times, raw, images, found):
        """Images of a failed capture, and raw events (FAILED_EVENTS_US ending at the newest pair) of
        the event cameras that missed the board."""
        folder = os.path.join(self.failed_folder, f"{t_newest}")
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


def choose_pairs(results):
    """Pick the candidate pair of every camera; results[cam][i] = (corners, variant, image).

    Returns ({cam: pair index}, mode):
      "same"   the newest pair where all four cameras see the board, and the RGB corners of the
               other pairs agree within STILL_BOARD_PX (board still during the event window)
      "still"  a blinking screen is caught in a good phase at different moments by each camera:
               when the board is still (its corners agree within STILL_BOARD_PX in every camera
               that sees it in several pairs) each camera uses its newest detection
      None     the board moved (or nothing to combine): each camera's newest detection, to
               show, not to save
    """
    n = len(next(iter(results.values())))
    for i in range(n):
        if all(results[cam][i][0] is not None for cam in CAMERAS):
            # The event frames integrate the events of up to 100 ms before the RGB exposure: a board
            # moving in that time is in different places for RGB and events, which shows up as a
            # wrong RGB <-> event extrinsics. The RGB corners of the other pairs (100 ms apart)
            # tell whether it moved (the event corners are too noisy for this check).
            moved = any(np.max(np.abs(results[cam][j][0] - results[cam][i][0])) > STILL_BOARD_PX
                        for cam in ("rgb_left", "rgb_right") for j in range(n)
                        if j != i and results[cam][j][0] is not None)
            return {cam: i for cam in CAMERAS}, None if moved else "same"
    detections = {cam: [r[0] for r in results[cam] if r[0] is not None] for cam in CAMERAS}
    newest = {cam: next((i for i in range(n) if results[cam][i][0] is not None), 0) for cam in CAMERAS}
    repeated = [d for d in detections.values() if len(d) >= 2]
    still = bool(repeated) and all(np.max(np.abs(c - d[0])) <= STILL_BOARD_PX for d in repeated for c in d[1:])
    return newest, "still" if still else None


def blueprint(sensors):
    tool = RgbEventCapture(sensors)
    return capture_blueprint(tool, "rgb_event_capture", "/rgb-event/capture", CAPTURES_PATH,
                             lambda body: tool.capture())

"""Live RGB-Livox calibration data capture (page of Calibration/calibrate.py).

Shows RGB-left and the Livox reflectivity image live. The Livox HAP draws a non-repetitive pattern
that needs seconds to cover the board, so a capture lasts N seconds: the board must stay still
while only the points of that time are collected. RGB-left is sampled once a second to verify it
did not move. When both see the board, the corners (and the points) are saved to
rgb_livox/processed_data/<session>/<index>/, the folder rgb_livox/solve.py calibrates from.
"""

import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import cv2
import h5py
import numpy as np

from cvlab import config
from cvlab.calibration import find_corners_variants, to_calibration_size
from Calibration.common import CaptureError, Samples, capture_blueprint, data_url, draw, preview
from Calibration.rgb_livox import lidar_image
from Calibration.rgb_livox.solve import CAMERAS, DEDUP_PX, PROCESSED_DATA_PATH

STILL_BOARD_PX = 1.5            # RGB corners closer than this over the capture: the board did not move
PREVIEW_WINDOW_S = 2            # seconds of points in the live reflectivity image
MIN_CAPTURE_S, MAX_CAPTURE_S = 1, 30
LIVOX_ARRIVAL_WAIT_S = 3.0      # max wait for the points of the end of the capture (chunks are ~0.2 s)

TOOL = {
    "id": "rgb_livox",
    "title": "RGB-Livox calibration data",
    "cameras": CAMERAS,
    "labels": {"rgb_left": "RGB left", "livox": "Livox reflectivity"},
    "preview_delay_ms": {"rgb_left": 150, "livox": 500},
    "aspect": {"rgb_left": list(config.CALIBRATION_IMAGE_SIZE), "livox": list(config.LIVOX_IMAGE_SIZE)},
    "pattern": False,         # the Livox sees reflectivity: a printed board works
    "save_failed": False,
    "timed_capture": True,
    "capture_s": config.LIVOX_CAPTURE_S,
    "solve_url": "/rgb-livox/calibrate",
}


class RgbLivoxCapture:
    TOOL = TOOL

    def __init__(self, sensors):
        self.sensors = sensors
        self.samples = None
        self.capture_lock = threading.Lock()
        self.pool = ThreadPoolExecutor(max_workers=8)
        self.open_session(None)

    def open_session(self, name):
        """name: an existing processed_data folder to add to, or None for a new one."""
        if name is None:
            name = datetime.now().strftime(config.SESSION_NAME_FORMAT)
        elif name not in os.listdir(PROCESSED_DATA_PATH):
            raise ValueError(f"no session {name!r}")
        image_sizes = {"rgb_left": config.CALIBRATION_IMAGE_SIZE, "livox": config.LIVOX_IMAGE_SIZE}
        self.samples = Samples(os.path.join(PROCESSED_DATA_PATH, name), CAMERAS, image_sizes, DEDUP_PX)

    def set_save_failed(self, on):
        pass

    def status(self):
        return {"rgb_left": self.sensors.rgb.status(0), "livox": self.sensors.livox.status()}

    def latest(self, cam):
        if cam == "rgb_left":
            frame = self.sensors.rgb.latest(0)
            if frame is None:
                return None
            return frame[0], to_calibration_size(cv2.cvtColor(frame[1], cv2.COLOR_RGB2BGR))
        end_us = self.sensors.livox.latest_us()
        if end_us is None:
            return None
        xyz, intensity, _ = self.sensors.livox.points_between(end_us - PREVIEW_WINDOW_S * 1_000_000, end_us)
        return end_us, lidar_image.display(lidar_image.reflectivity_image(xyz, intensity))

    def capture(self, seconds):
        rgb, livox = self.sensors.rgb, self.sensors.livox
        if rgb.latest(0) is None:
            raise CaptureError(f"no RGB frames ({rgb.error or 'cameras starting'})")
        if livox.latest_us() is None:
            raise CaptureError(f"no Livox points ({livox.error or 'lidar starting'})")

        # Only the points of the next `seconds`: the board must already be in place and still
        start_us = time.time_ns() // 1000
        frames = []   # (host_us, full RGB) once a second, the last one at the end
        for i in range(int(np.ceil(seconds)) + 1):
            time.sleep(max(0.0, start_us / 1e6 + min(i, seconds) - time.time()))
            frames.append(rgb.latest(0))
        if any(frame is None for frame in frames):
            raise CaptureError(f"RGB left stopped during the capture ({rgb.error or 'no frames'})")
        end_us = start_us + int(seconds * 1_000_000)
        deadline = time.monotonic() + LIVOX_ARRIVAL_WAIT_S
        while (livox.latest_us() or 0) < end_us and time.monotonic() < deadline:
            time.sleep(0.05)
        xyz, intensity, t_us = livox.points_between(start_us, end_us)
        if not len(xyz):
            raise CaptureError(f"no Livox points during the capture ({livox.error or 'is the lidar streaming?'})")

        def rgb_search(image):
            gray = to_calibration_size(cv2.cvtColor(image, cv2.COLOR_RGB2GRAY))
            return (*find_corners_variants(gray, config.CHESSBOARD_VARIANTS["rgb"]), gray)

        rgb_futures = [self.pool.submit(rgb_search, frame[1]) for frame in frames]
        reflectivity = lidar_image.reflectivity_image(xyz, intensity)
        lidar_corners, gamma = lidar_image.find_lidar_corners(reflectivity)
        rgb_results = [f.result() for f in rgb_futures]

        rgb_corners, rgb_variant, rgb_gray = rgb_results[-1]
        # Still: every RGB sample that sees the board agrees with the last one
        seen = [c for c, _, _ in rgb_results if c is not None]
        drift_px = max((np.max(np.abs(c - rgb_corners)) for c in seen), default=0.0) if rgb_corners is not None else None
        moving = rgb_corners is not None and (len(seen) < 2 or drift_px > STILL_BOARD_PX)

        lidar_display = lidar_image.display(reflectivity)
        corners = {"rgb_left": rgb_corners, "livox": lidar_corners}
        images = {"rgb_left": rgb_gray, "livox": lidar_display}
        now_us = time.time_ns() // 1000
        result = {"cameras": {}, "saved": None, "duplicate_of": None,
                  "moving": moving and lidar_corners is not None,
                  "points": int(len(xyz)), "rgb_drift_px": None if drift_px is None else round(float(drift_px), 2)}
        for cam, variant, t in [("rgb_left", rgb_variant, frames[-1][0]), ("livox", None if gamma is None else [gamma, 0, 0], end_us)]:
            c = corners[cam]
            result["cameras"][cam] = {"found": c is not None, "variant": variant,
                                      "age_ms": round((now_us - t) / 1e3),
                                      "image": data_url(preview(draw(images[cam], c))),
                                      "corners": self.samples.normalized(cam, c)}

        if all(c is not None for c in corners.values()) and not moving:
            result["duplicate_of"] = self.samples.duplicate_of(corners)
            if result["duplicate_of"] is None:
                log = (f"Captured {datetime.now().isoformat()} by Calibration/rgb_livox/capture.py\n"
                       f"Livox: {len(xyz)} points from host time {start_us} to {end_us} us ({seconds} s), "
                       f"board found with gamma {gamma}\n"
                       f"rgb_left: {len(frames)} samples, board in {len(seen)}, max drift {drift_px:.2f} px; "
                       f"last at host time {frames[-1][0]} us, found with (gamma, blur, clahe) = {rgb_variant}\n")
                full = frames[-1][1]

                def save_extra(sample):
                    cv2.imwrite(os.path.join(sample, "rgb_left_full.png"), cv2.cvtColor(full, cv2.COLOR_RGB2BGR),
                                [cv2.IMWRITE_PNG_COMPRESSION, config.RGB_PNG_COMPRESSION])
                    cv2.imwrite(os.path.join(sample, "livox_raw.png"), reflectivity)
                    # The points, to re-render with other settings and for the board plane check of solve.py
                    with h5py.File(os.path.join(sample, "livox_points.h5"), "w") as f:
                        f.create_dataset("xyz_camera_mm", data=xyz)
                        f.create_dataset("intensity", data=intensity)
                        f.create_dataset("host_us", data=t_us)

                result["saved"] = self.samples.add(corners, images, log, save_extra)
        return result


def blueprint(sensors):
    tool = RgbLivoxCapture(sensors)

    def capture(body):
        seconds = float(body.get("seconds") or config.LIVOX_CAPTURE_S)
        return tool.capture(min(max(seconds, MIN_CAPTURE_S), MAX_CAPTURE_S))

    return capture_blueprint(tool, "rgb_livox_capture", "/rgb-livox/capture", PROCESSED_DATA_PATH, capture)

"""Helpers shared by the calibration scripts and by every consumer of the results."""

import json
import os
import shutil
import threading
from datetime import datetime

import cv2
import numpy as np

from cvlab import config

SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)


def to_calibration_size(img, size=config.CALIBRATION_IMAGE_SIZE):
    """Center-crop to the aspect ratio of `size`, then resize.

    The RGB intrinsics are estimated on images transformed this way: apply it to
    every RGB image before using the calibration.
    """
    width, height = size
    h, w = img.shape[:2]
    target_ratio = width / height
    if w / h > target_ratio:
        new_w = int(round(h * target_ratio))
        x0 = (w - new_w) // 2
        img = img[:, x0:x0 + new_w]
    elif w / h < target_ratio:
        new_h = int(round(w / target_ratio))
        y0 = (h - new_h) // 2
        img = img[y0:y0 + new_h]
    if (img.shape[1], img.shape[0]) != (width, height):
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_AREA)
    return img


def full_frame_intrinsics(K, full_size, size=config.CALIBRATION_IMAGE_SIZE):
    """K of the image before to_calibration_size, from the K estimated on its output."""
    width, height = size
    w, h = full_size
    target_ratio = width / height
    x0 = y0 = 0
    if w / h > target_ratio:
        new_w, new_h = int(round(h * target_ratio)), h
        x0 = (w - new_w) // 2
    else:
        new_w, new_h = w, int(round(w / target_ratio))
        y0 = (h - new_h) // 2
    sx, sy = width / new_w, height / new_h
    K = np.array(K, dtype=np.float64)
    full = K.copy()
    full[0, 0], full[1, 1] = K[0, 0] / sx, K[1, 1] / sy
    full[0, 2] = (K[0, 2] + 0.5) / sx - 0.5 + x0
    full[1, 2] = (K[1, 2] + 0.5) / sy - 0.5 + y0
    return full


def pattern_points(pattern_size=config.PATTERN_SIZE, square_size_mm=config.SQUARE_SIZE_MM):
    """3D chessboard corners in mm, in the order of the detected corners."""
    points = np.zeros((np.prod(pattern_size), 3), np.float32)
    points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    return points * square_size_mm


def canonical_corners(corners):
    """Resolve the 180 degree ambiguity of the chessboard detector.

    A 9x6 board looks the same rotated by 180 degrees, so the detector may start
    from either end. We always start from the top-left corner of the image. This
    assumes every camera of the rig is mounted upright (no camera rotated by more
    than 90 degrees with respect to the others) and that no image is mirrored.
    """
    corners = corners.reshape(-1, 2).astype(np.float32)
    if np.dot(corners[-1] - corners[0], (1.0, 1.0)) < 0:
        corners = corners[::-1].copy()
    return corners


def find_corners(gray, subpix=True):
    """Canonically ordered chessboard corners (N, 2), or None."""
    found, corners = cv2.findChessboardCornersSB(gray, config.PATTERN_SIZE, flags=0)
    if not found:
        return None
    if subpix:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)
    return canonical_corners(corners)


def adjust_gamma(image, gamma=1.0):
    """gamma > 1: darker, gamma < 1: brighter."""
    table = np.array([((i / 255.0) ** gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    return cv2.LUT(image, table)


def find_corners_variants(gray, variants, subpix=True):
    """find_corners on (gamma, blur, clahe) variants of the image, in order (see CHESSBOARD_VARIANTS).

    Returns (corners, variant) for the first variant that finds the board, or (None, None).
    """
    for gamma, blur, clahe in variants:
        image = cv2.GaussianBlur(gray, (blur, blur), 0) if blur else gray
        if clahe:
            image = cv2.createCLAHE(clahe, (16, 16)).apply(image)
        if gamma != 1.0:
            image = adjust_gamma(image, gamma)
        corners = find_corners(image, subpix=subpix)
        if corners is not None:
            return corners, (gamma, blur, clahe)
    return None, None


def find_event_corners(frames):
    """Chessboard in an event window, in its renderings (events.calibration_frames) in the
    order of EVENT_RENDERINGS.

    Returns (corners, (gamma, blur, clahe, rendering), image the corners were found in),
    or (None, None, count frame).

    No cornerSubPix: findChessboardCornersSB is already sub-pixel, and on speckled event
    images cornerSubPix pulls the corners off (synthetic blinking boards: 1.4 px RMS error
    with it, 0.55 px without).
    """
    for rendering in config.EVENT_RENDERINGS:
        corners, variant = find_corners_variants(frames[rendering], config.CHESSBOARD_VARIANTS[f"event_{rendering}"],
                                                 subpix=False)
        if corners is not None:
            return corners, (*variant, rendering), frames[rendering]
    return None, None, frames["count"]


def stereo_rectify_maps(K1, D1, K2, D2, R, T, image_size, alpha=0):
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(K1, D1, K2, D2, image_size, R, T, alpha=alpha)
    maps1 = cv2.initUndistortRectifyMap(K1, D1, R1, P1, image_size, cv2.CV_16SC2)
    maps2 = cv2.initUndistortRectifyMap(K2, D2, R2, P2, image_size, cv2.CV_16SC2)
    return maps1, maps2, Q


def rectify_pair(img1, img2, maps1, maps2):
    return (cv2.remap(img1, maps1[0], maps1[1], cv2.INTER_LINEAR),
            cv2.remap(img2, maps2[0], maps2[1], cv2.INTER_LINEAR))


def is_vertical_baseline(T):
    """cv2.stereoRectify makes the epipolar lines vertical when the baseline is mostly vertical."""
    T = np.ravel(T)
    return abs(T[1]) > abs(T[0])


def rectification_check_image(img1, img2, step=50, vertical=False):
    """Two rectified images with lines to eyeball a rectification.

    Horizontal baseline: side by side, corresponding points on the same row.
    Vertical baseline (see is_vertical_baseline): stacked, corresponding points on the same column.
    """
    if img1.ndim == 2:
        img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    if img2.ndim == 2:
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
    if vertical:
        vis = np.vstack((img1, img2))
        for x in range(0, vis.shape[1], step):
            cv2.line(vis, (x, 0), (x, vis.shape[0]), (0, 255, 0), 1)
    else:
        vis = np.hstack((img1, img2))
        for y in range(0, vis.shape[0], step):
            cv2.line(vis, (0, y), (vis.shape[1], y), (0, 255, 0), 1)
    return vis


def load_calibration(path):
    """Load rgb_event_calibration.npz, checking it matches the current setup."""
    with np.load(path) as data:
        calib = {k: data[k] for k in data.files}
    if "pipeline_version" not in calib:
        raise ValueError(f"{path} was made by the old calibration pipeline: recalibrate")
    roi = tuple(calib["rgb_roi_fraction"])
    if roi != tuple(config.RGB_ROI_FRACTION):
        raise ValueError(f"{path}: calibrated with RGB ROI {roi}, current ROI is {config.RGB_ROI_FRACTION}")
    return calib


# ============================================================================
# CAPTURED BOARDS
# ============================================================================

SESSION_FILE = "session.json"   # per session settings, e.g. the chessboard square size
MIN_SQUARE_MM, MAX_SQUARE_MM = 1.0, 1000.0


class UserError(Exception):
    """A calibration that cannot run for a reason the user can fix (shown without a traceback)."""


class CaptureError(Exception):
    """A capture that could not run (sensor missing...): shown on the page."""


def draw(gray, corners):
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR) if gray.ndim == 2 else gray.copy()
    if corners is not None:
        cv2.drawChessboardCorners(image, config.PATTERN_SIZE, corners.reshape(-1, 1, 2).astype(np.float32), True)
    return image


def sessions(folder):
    """[{name, boards}] of the session folders in folder, newest first (boards: subfolders)."""
    if not os.path.isdir(folder):
        return []
    entries = []
    for name in sorted(os.listdir(folder), reverse=True):
        path = os.path.join(folder, name)
        if os.path.isdir(path):
            boards = sum(1 for b in os.listdir(path) if os.path.isdir(os.path.join(path, b)) and b.isdigit())
            entries.append({"name": name, "boards": boards,
                            "modified": datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M")})
    return entries


def count_boards(folder, cameras):
    """Boards with the corners of every camera, as the solvers load them."""
    return sum(1 for name in os.listdir(folder)
               if all(os.path.exists(os.path.join(folder, name, f"{cam}_corners.npy")) for cam in cameras))


def session_square_size(folder):
    """Chessboard square size (mm) of a session: the one entered on the capture page, or
    config.SQUARE_SIZE_MM for sessions made before it was recorded."""
    try:
        with open(os.path.join(folder, SESSION_FILE)) as f:
            return float(json.load(f)["square_size_mm"])
    except (OSError, KeyError, ValueError):
        return float(config.SQUARE_SIZE_MM)


def check_square_size(value):
    value = float(value)
    if not MIN_SQUARE_MM <= value <= MAX_SQUARE_MM:
        raise ValueError(f"square size {value} mm: expected {MIN_SQUARE_MM:.0f}-{MAX_SQUARE_MM:.0f} mm")
    return value


class Samples:
    """Captured boards in <captures>/<session>/<index>/: <cam>_corners.npy / .png and <cam>.png
    for every camera, plus whatever extra files the tool writes (see add)."""

    def __init__(self, folder, cameras, image_sizes, dedup_px, reference="rgb_left", square_size_mm=None):
        self.folder = folder   # created with the first board
        # An existing session keeps its own square size; a new one takes the given one
        self.square_size_mm = (session_square_size(folder) if os.path.isdir(folder) or square_size_mm is None
                               else square_size_mm)
        self.cameras = cameras
        self.image_sizes = image_sizes   # {cam: (w, h)}, to normalise the corners for the page
        self.dedup_px = dedup_px
        self.reference = reference
        self.lock = threading.Lock()
        self.corners = {}   # name -> {cam: (N, 2)}
        for name in sorted(os.listdir(folder)) if os.path.isdir(folder) else []:
            paths = {cam: os.path.join(folder, name, f"{cam}_corners.npy") for cam in cameras}
            if all(os.path.exists(p) for p in paths.values()):
                self.corners[name] = {cam: np.load(p).reshape(-1, 2) for cam, p in paths.items()}

    def duplicate_of(self, corners):
        """Name of a sample the solver would consider the same board (first and last corner of the
        reference camera within dedup_px, see solve.deduplicate)."""
        c = corners[self.reference]
        d = np.concatenate([c[0], c[-1]])
        for name, other in self.corners.items():
            o = other[self.reference]
            if np.max(np.abs(d - np.concatenate([o[0], o[-1]]))) <= self.dedup_px:
                return name
        return None

    def add(self, corners, images, log, extra=None):
        """Save a board. extra(sample_folder) writes the tool's own files. Returns the board name."""
        with self.lock:
            index = max((int(n) for n in self.corners), default=-1) + 1
            name = f"{index:05d}"
            sample = os.path.join(self.folder, name)
            os.makedirs(sample)
            self.save_session_info()
            for cam, c in corners.items():
                np.save(os.path.join(sample, f"{cam}_corners.npy"), c)
                cv2.imwrite(os.path.join(sample, f"{cam}_corners.png"), draw(images[cam], c))
                cv2.imwrite(os.path.join(sample, f"{cam}.png"), images[cam])
            if extra is not None:
                extra(sample)
            with open(os.path.join(sample, "log.log"), "w") as f:
                f.write(log)
            self.corners[name] = corners
            return name

    def set_square_size(self, value):
        self.square_size_mm = check_square_size(value)
        if os.path.isdir(self.folder):
            self.save_session_info()

    def save_session_info(self):
        with open(os.path.join(self.folder, SESSION_FILE), "w") as f:
            json.dump({"square_size_mm": self.square_size_mm}, f)

    def delete(self, name):
        with self.lock:
            if name not in self.corners:
                return False
            shutil.rmtree(os.path.join(self.folder, name))
            del self.corners[name]
            return True

    def normalized(self, cam, c):
        return None if c is None else (np.asarray(c).reshape(-1, 2) / self.image_sizes[cam]).round(4).tolist()

    def as_json(self):
        with self.lock:
            return [{"name": name, "corners": {cam: self.normalized(cam, c) for cam, c in cs.items()}}
                    for name, cs in sorted(self.corners.items())]

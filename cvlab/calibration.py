"""Helpers shared by the calibration scripts and by every consumer of the results."""

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


def rectification_check_image(img1, img2, step=50):
    """Side-by-side image with horizontal lines to eyeball a rectification."""
    if img1.ndim == 2:
        img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    if img2.ndim == 2:
        img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
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

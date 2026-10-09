"""Calibrate the RGB stereo pair, the event stereo pair and RGB-left <-> event-left.

Usage: calibrate.py [processed_data/<session>]   (default: latest)

1. Intrinsics of each camera on its own (calibrateCamera), dropping outlier boards.
2. Every stereo pair with CALIB_FIX_INTRINSIC, so R/T are consistent with the saved K/D.
3. Results in calibration_results/<time>/ and installed to config.RGB_EVENT_CALIBRATION_FILE.

All images are in CALIBRATION_IMAGE_SIZE: RGB images must go through
cvlab.calibration.to_calibration_size before using these intrinsics.
Translations are in mm.
"""

import argparse
import os
import shutil
import sys
from datetime import datetime

import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config
from cvlab.calibration import (pattern_points, rectification_check_image, rectify_pair,
                               stereo_rectify_maps)

HERE = os.path.dirname(os.path.abspath(__file__))
PROCESSED_DATA_PATH = os.path.join(HERE, "processed_data")
RESULTS_PATH = os.path.join(HERE, "calibration_results")

CAMERAS = ["rgb_left", "rgb_right", "event_left", "event_right"]
PIPELINE_VERSION = 2
DEDUP_PX = 20                 # boards closer than this (rgb_left corners) are duplicates
MAX_VIEW_ERROR_FACTOR = 3.0   # drop boards with error > factor * median


def load_samples(folder):
    samples = []
    for name in sorted(os.listdir(folder)):
        sample = os.path.join(folder, name)
        paths = {cam: os.path.join(sample, f"{cam}_corners.npy") for cam in CAMERAS}
        if os.path.isdir(sample) and all(os.path.exists(p) for p in paths.values()):
            samples.append({"name": name, **{cam: np.load(p).reshape(-1, 1, 2).astype(np.float32)
                                             for cam, p in paths.items()}})
    return samples


def deduplicate(samples):
    """Drop boards almost identical to an already kept one (the board did not move)."""
    kept, descriptors = [], []
    for s in samples:
        c = s["rgb_left"].reshape(-1, 2)
        d = np.concatenate([c[0], c[-1]])
        if not descriptors or min(np.max(np.abs(d - k)) for k in descriptors) > DEDUP_PX:
            kept.append(s)
            descriptors.append(d)
    return kept


def view_errors(objp, corners, K, D, rvecs, tvecs):
    errors = []
    for c, r, t in zip(corners, rvecs, tvecs):
        projected, _ = cv2.projectPoints(objp, r, t, K, D)
        errors.append(np.sqrt(np.mean(np.sum((projected - c) ** 2, axis=2))))
    return np.array(errors)


def calibrate_intrinsics(samples, objp, image_size):
    """Per camera intrinsics; returns ({cam: (rms, K, D)}, samples without outliers)."""
    for attempt in range(2):
        results, bad = {}, set()
        for cam in CAMERAS:
            corners = [s[cam] for s in samples]
            rms, K, D, rvecs, tvecs = cv2.calibrateCamera([objp] * len(corners), corners, image_size, None, None)
            errors = view_errors(objp, corners, K, D, rvecs, tvecs)
            results[cam] = (rms, K, D)
            bad |= set(np.where(errors > MAX_VIEW_ERROR_FACTOR * np.median(errors))[0])
            print(f"\t{cam:12s} RMS {rms:.3f} px  (median view error {np.median(errors):.3f}, max {errors.max():.3f})")
        if not bad or attempt == 1:
            return results, samples
        print(f"Dropping {len(bad)} outlier boards and recalibrating")
        samples = [s for i, s in enumerate(samples) if i not in bad]
    return results, samples


def stereo(samples, objp, cam1, cam2, intrinsics, image_size):
    _, K1, D1 = intrinsics[cam1]
    _, K2, D2 = intrinsics[cam2]
    rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
        [objp] * len(samples), [s[cam1] for s in samples], [s[cam2] for s in samples],
        K1, D1, K2, D2, image_size, flags=cv2.CALIB_FIX_INTRINSIC)
    angle = np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0]))
    print(f"\t{cam1} -> {cam2}: RMS {rms:.3f} px, T = {T.ravel().round(1)} mm "
          f"(|T| = {np.linalg.norm(T):.1f} mm), rotation {angle:.2f} deg")
    return rms, R, T


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", nargs="?", help="processed_data/<session> (default: latest)")
    parser.add_argument("--no-install", action="store_true",
                        help=f"Do not copy the result to {config.RGB_EVENT_CALIBRATION_FILE}")
    args = parser.parse_args()

    folder = args.folder or os.path.join(PROCESSED_DATA_PATH, sorted(os.listdir(PROCESSED_DATA_PATH))[-1])
    print(f"Processing folder {folder}")

    samples = load_samples(folder)
    print(f"{len(samples)} boards seen by all four cameras")
    samples = deduplicate(samples)
    print(f"{len(samples)} boards after removing duplicates")
    if len(samples) < 10:
        sys.exit("Not enough boards to calibrate (need at least 10): move the board around more")

    image_size = config.CALIBRATION_IMAGE_SIZE
    objp = pattern_points()

    print("Intrinsics:")
    intrinsics, samples = calibrate_intrinsics(samples, objp, image_size)
    print(f"Using {len(samples)} boards")

    print("Extrinsics:")
    rgb_rms, rgb_R, rgb_T = stereo(samples, objp, "rgb_left", "rgb_right", intrinsics, image_size)
    event_rms, event_R, event_T = stereo(samples, objp, "event_left", "event_right", intrinsics, image_size)
    rgb_event_rms, rgb_event_R, rgb_event_T = stereo(samples, objp, "rgb_left", "event_left", intrinsics, image_size)

    results = os.path.join(RESULTS_PATH, datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
    os.makedirs(results)

    # Visual checks on the first board
    first = os.path.join(folder, samples[0]["name"])
    img = {cam: cv2.imread(os.path.join(first, f"{cam}_corners.png")) for cam in CAMERAS}
    for (cam1, cam2, R, T) in [("rgb_left", "rgb_right", rgb_R, rgb_T),
                               ("event_left", "event_right", event_R, event_T),
                               ("rgb_left", "event_left", rgb_event_R, rgb_event_T)]:
        maps1, maps2, _ = stereo_rectify_maps(*intrinsics[cam1][1:], *intrinsics[cam2][1:], R, T, image_size)
        rect1, rect2 = rectify_pair(img[cam1], img[cam2], maps1, maps2)
        cv2.imwrite(os.path.join(results, f"rectification_{cam1}_{cam2}.png"),
                    rectification_check_image(rect1, rect2))
        if cam2 == "event_left":
            cv2.imwrite(os.path.join(results, "rgb_event_overlay.png"), cv2.addWeighted(rect1, 0.5, rect2, 0.5, 0))

    out = os.path.join(results, "rgb_event_calibration.npz")
    np.savez(
        out,
        pipeline_version=PIPELINE_VERSION,
        calibration_image_size=np.array(image_size),
        rgb_roi_fraction=np.array(config.RGB_ROI_FRACTION),
        pattern_size=np.array(config.PATTERN_SIZE),
        square_size_mm=config.SQUARE_SIZE_MM,
        n_boards=len(samples),
        rgb_K_left=intrinsics["rgb_left"][1], rgb_D_left=intrinsics["rgb_left"][2],
        rgb_K_right=intrinsics["rgb_right"][1], rgb_D_right=intrinsics["rgb_right"][2],
        event_K_left=intrinsics["event_left"][1], event_D_left=intrinsics["event_left"][2],
        event_K_right=intrinsics["event_right"][1], event_D_right=intrinsics["event_right"][2],
        rgb_R=rgb_R, rgb_T=rgb_T, rgb_rms=rgb_rms,
        event_R=event_R, event_T=event_T, event_rms=event_rms,
        rgb_event_R=rgb_event_R, rgb_event_T=rgb_event_T, rgb_event_rms=rgb_event_rms,
    )
    print(f"Results in {results}")
    if not args.no_install:
        shutil.copy(out, config.RGB_EVENT_CALIBRATION_FILE)
        print(f"Installed as {config.RGB_EVENT_CALIBRATION_FILE}")


if __name__ == "__main__":
    main()

"""Calibrate the RGB stereo pair, the event stereo pair and RGB-left <-> event-left.

Usage: python -m cvlab_sensors.calibration.rgb_event.solve [data/calibration/rgb_event/captures/<session>] [--no-install]   (default: latest)
Also run from the "Calibrate RGB <-> event" page of the web app.

1. Intrinsics of each camera on its own (calibrateCamera), dropping outlier boards.
2. Every stereo pair with CALIB_FIX_INTRINSIC, so R/T are consistent with the saved K/D.
3. Results in results/<time>/ and installed to config.RGB_EVENT_CALIBRATION_FILE.

All images are in CALIBRATION_IMAGE_SIZE: RGB images must go through
cvlab_sensors.calibration.to_calibration_size before using these intrinsics.
Translations are in mm.
"""

import argparse
import os
import shutil
import sys
from datetime import datetime

import cv2
import numpy as np

from cvlab_sensors import config
from cvlab_sensors.calibration.common import (UserError, full_frame_intrinsics, is_vertical_baseline, pattern_points,
                                      rectification_check_image, rectify_pair, session_square_size, stereo_rectify_maps)

CAPTURES_PATH = str(config.CALIBRATION_DIR / "rgb_event" / "captures")
RESULTS_PATH = str(config.CALIBRATION_DIR / "rgb_event" / "results")

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


def calibrate_intrinsics(samples, objp, image_size, log=print, step=lambda label: None):
    """Per camera intrinsics; returns ({cam: (rms, K, D)}, {cam: view errors}, samples without outliers)."""
    for attempt in range(2):
        results, errors_by_cam, bad = {}, {}, set()
        for cam in CAMERAS:
            step(f"Intrinsics {cam}" + (" (without outliers)" if attempt else ""))
            corners = [s[cam] for s in samples]
            rms, K, D, rvecs, tvecs = cv2.calibrateCamera([objp] * len(corners), corners, image_size, None, None)
            errors = view_errors(objp, corners, K, D, rvecs, tvecs)
            results[cam] = (rms, K, D)
            errors_by_cam[cam] = errors
            bad |= set(np.where(errors > MAX_VIEW_ERROR_FACTOR * np.median(errors))[0])
            log(f"\t{cam:12s} RMS {rms:.3f} px  (median view error {np.median(errors):.3f}, max {errors.max():.3f})")
        if not bad or attempt == 1:
            return results, errors_by_cam, samples
        log(f"Dropping {len(bad)} outlier boards and recalibrating")
        samples = [s for i, s in enumerate(samples) if i not in bad]
    return results, errors_by_cam, samples


def stereo(samples, objp, cam1, cam2, intrinsics, image_size, log=print):
    _, K1, D1 = intrinsics[cam1]
    _, K2, D2 = intrinsics[cam2]
    rms, _, _, _, _, R, T, _, _ = cv2.stereoCalibrate(
        [objp] * len(samples), [s[cam1] for s in samples], [s[cam2] for s in samples],
        K1, D1, K2, D2, image_size, flags=cv2.CALIB_FIX_INTRINSIC)
    angle = np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0]))
    log(f"\t{cam1} -> {cam2}: RMS {rms:.3f} px, T = {T.ravel().round(1)} mm "
          f"(|T| = {np.linalg.norm(T):.1f} mm), rotation {angle:.2f} deg")
    return rms, R, T


def board_parallax(sample, cam1, cam2, intrinsics, R, T, image_size):
    """Mean (dx, dy) in px from the board in rectified cam1 to the board in rectified cam2."""
    K1, D1 = intrinsics[cam1][1:]
    K2, D2 = intrinsics[cam2][1:]
    R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(K1, D1, K2, D2, image_size, R, T, alpha=0)
    p1 = cv2.undistortPoints(sample[cam1], K1, D1, R=R1, P=P1)
    p2 = cv2.undistortPoints(sample[cam2], K2, D2, R=R2, P=P2)
    return (p2 - p1).reshape(-1, 2).mean(axis=0)


def monotonic_radius(D, limit=3.0):
    """Normalized radius where the radial distortion stops growing (or limit).

    Beyond it the polynomial folds back: a far ray lands inside the image again, so the
    model is only invertible, and only meaningful, inside this radius."""
    k1, k2, _, _, k3 = np.ravel(D)[:5]
    r = np.linspace(0, limit, 3001)[1:]
    derivative = 1 + 3 * k1 * r ** 2 + 5 * k2 * r ** 4 + 7 * k3 * r ** 6
    bad = np.nonzero(derivative <= 0)[0]
    return r[bad[0]] if len(bad) else limit


def rectified_lookup(x, y, K, D, R_rect, image_size):
    """Source pixel (u, v) of each rectified normalized point (x, y); -1 where the camera does not see it."""
    rays = np.stack([x, y, np.ones_like(x)], -1) @ R_rect   # R_rect^T @ (x, y, 1): back to the camera frame
    z = rays[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        xn, yn = rays[..., 0] / z, rays[..., 1] / z
    k1, k2, p1, p2, k3 = np.ravel(D)[:5]
    r2 = xn ** 2 + yn ** 2
    radial = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn ** 2)
    yd = yn * radial + p1 * (r2 + 2 * yn ** 2) + 2 * p2 * xn * yn
    u = K[0, 0] * xd + K[0, 1] * yd + K[0, 2]
    v = K[1, 1] * yd + K[1, 2]
    w, h = image_size
    valid = (z > 0) & (r2 < monotonic_radius(D) ** 2) & (u >= 0) & (u <= w - 1) & (v >= 0) & (v <= h - 1)
    u, v = np.where(valid, u, -1), np.where(valid, v, -1)
    return u.astype(np.float32), v.astype(np.float32), valid


def save_blend(results, sample_folder, sample, intrinsics, R, T, image_size, log, max_side=4096):
    """Rectified rgb_left and event_left on one canvas holding both whole fields of view, the event
    image shifted by the parallax of the board: blend_rgb.png / blend_event.png for the slider of
    the solve page. Uses the full RGB frame when saved (rgb_event/capture.py does)."""
    K_rgb, D_rgb = intrinsics["rgb_left"][1:]
    K_ev, D_ev = intrinsics["event_left"][1:]
    R1, R2, *_ = cv2.stereoRectify(K_rgb, D_rgb, K_ev, D_ev, image_size, R, T, alpha=0)

    def load(name, fallback):
        path = os.path.join(sample_folder, name)
        return cv2.imread(path if os.path.exists(path) else os.path.join(sample_folder, fallback))

    rgb = load("rgb_left_full.png", "rgb_left.png")
    event = load("event_left.png", "event_left_corners.png")
    rgb_size, event_size = (rgb.shape[1], rgb.shape[0]), (event.shape[1], event.shape[0])
    if rgb_size != tuple(image_size):
        K_rgb = full_frame_intrinsics(K_rgb, rgb_size, image_size)

    # Parallax of the board, in rectified normalized coordinates
    crit = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 100, 1e-8)
    board_rgb = cv2.undistortPointsIter(sample["rgb_left"], intrinsics["rgb_left"][1], D_rgb, R1, None, crit)
    board_ev = cv2.undistortPointsIter(sample["event_left"], K_ev, D_ev, R2, None, crit)
    shift = (board_ev - board_rgb).reshape(-1, 2).mean(axis=0)

    # Canvas: bounding box of what either camera sees, at the resolution of the RGB frame
    x, y = np.meshgrid(np.linspace(-3, 3, 1201), np.linspace(-3, 3, 1201))
    seen = rectified_lookup(x, y, K_rgb, D_rgb, R1, rgb_size)[2] | \
        rectified_lookup(x + shift[0], y + shift[1], K_ev, D_ev, R2, event_size)[2]
    x0, x1, y0, y1 = x[seen].min(), x[seen].max(), y[seen].min(), y[seen].max()
    f = min(K_rgb[0, 0], max_side / max(x1 - x0, y1 - y0))
    width, height = int(np.ceil((x1 - x0) * f)), int(np.ceil((y1 - y0) * f))
    x, y = np.meshgrid(x0 + (np.arange(width) + 0.5) / f, y0 + (np.arange(height) + 0.5) / f)

    blend = {"rgb": "blend_rgb.png", "event": "blend_event.png"}
    for key, image, lookup in [("rgb", rgb, rectified_lookup(x, y, K_rgb, D_rgb, R1, rgb_size)),
                               ("event", event, rectified_lookup(x + shift[0], y + shift[1], K_ev, D_ev, R2, event_size))]:
        out = cv2.remap(image, lookup[0], lookup[1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        cv2.imwrite(os.path.join(results, blend[key]), out)
    log(f"\tRGB <-> event blend: {width}x{height} px canvas")
    # For the slider of the solve page: images[0] at the left end, images[1] at the right end
    return {"title": "RGB ↔ event", "images": [blend["event"], blend["rgb"]], "labels": ["Event", "RGB"],
            "note": "Rectified left cameras, whole field of view of both, event shifted by the board parallax "
                    "(black: not seen)"}


def camera_poses(extrinsics):
    """Centre (mm) and viewing direction of every camera in the rgb_left frame (OpenCV axes:
    x right, y down, z forward). extrinsics[(a, b)] = (rms, R, T) with x_b = R x_a + T."""
    rotations = {"rgb_left": np.eye(3)}
    translations = {"rgb_left": np.zeros((3, 1))}
    for a, b in [("rgb_left", "rgb_right"), ("rgb_left", "event_left"), ("event_left", "event_right")]:
        _, R, T = extrinsics[(a, b)]
        rotations[b] = R @ rotations[a]
        translations[b] = R @ translations[a] + T
    return {cam: {"center": (-rotations[cam].T @ translations[cam]).ravel().tolist(),
                  "axis": rotations[cam].T[:, 2].tolist()}
            for cam in CAMERAS}


def calibrate(folder, install=True, log=print, progress=lambda fraction, label: None):
    """Full calibration of a captures/<session> folder.

    progress(fraction in [0, 1], label) is called before every step. Returns a
    summary dict (see the end of the function); raises UserError.
    """
    # Steps: load, 4 intrinsics + 4 if outliers are dropped, 3 stereo, 3 rectification checks, save
    total = 16
    done = 0

    def step(label):
        nonlocal done
        progress(min(done / total, 0.99), label)
        done += 1

    step("Loading boards")
    log(f"Processing folder {folder}")
    samples = load_samples(folder)
    n_found = len(samples)
    log(f"{n_found} boards seen by all four cameras")
    samples = deduplicate(samples)
    n_unique = len(samples)
    log(f"{n_unique} boards after removing duplicates")
    if n_unique < 10:
        raise UserError(f"Not enough boards to calibrate ({n_unique}, need at least 10): move the board around more")

    image_size = config.CALIBRATION_IMAGE_SIZE
    square_mm = session_square_size(folder)
    log(f"Chessboard: {config.PATTERN_SIZE[0]}x{config.PATTERN_SIZE[1]} inner corners, {square_mm:g} mm squares")
    objp = pattern_points(square_size_mm=square_mm)

    log("Intrinsics:")
    intrinsics, view_errs, samples = calibrate_intrinsics(samples, objp, image_size, log, step)
    done = 9   # skips the second intrinsics round when no outlier was dropped
    log(f"Using {len(samples)} boards")

    log("Extrinsics:")
    pairs = [("rgb_left", "rgb_right"), ("event_left", "event_right"), ("rgb_left", "event_left")]
    extrinsics = {}
    for cam1, cam2 in pairs:
        step(f"Stereo {cam1} -> {cam2}")
        extrinsics[(cam1, cam2)] = stereo(samples, objp, cam1, cam2, intrinsics, image_size, log)
    (rgb_rms, rgb_R, rgb_T), (event_rms, event_R, event_T), (rgb_event_rms, rgb_event_R, rgb_event_T) = \
        [extrinsics[p] for p in pairs]

    results = os.path.join(RESULTS_PATH, datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
    os.makedirs(results)

    # Visual checks on the first board
    first = os.path.join(folder, samples[0]["name"])
    img = {cam: cv2.imread(os.path.join(first, f"{cam}_corners.png")) for cam in CAMERAS}
    images = []
    for cam1, cam2 in pairs:
        step(f"Rectification check {cam1} / {cam2}")
        _, R, T = extrinsics[(cam1, cam2)]
        maps1, maps2, _ = stereo_rectify_maps(*intrinsics[cam1][1:], *intrinsics[cam2][1:], R, T, image_size)
        rect1, rect2 = rectify_pair(img[cam1], img[cam2], maps1, maps2)
        name = f"rectification_{cam1}_{cam2}.png"
        cv2.imwrite(os.path.join(results, name),
                    rectification_check_image(rect1, rect2, vertical=is_vertical_baseline(T)))
        images.append(name)
        if cam2 == "event_left":
            # The two images differ by the parallax along the baseline, which depends on the
            # distance: shift the event image by the parallax of the board so that the board overlaps.
            shift = board_parallax(samples[0], cam1, cam2, intrinsics, R, T, image_size)
            log(f"\tparallax of the first board {cam1} -> {cam2}: dx {shift[0]:.1f}, dy {shift[1]:.1f} px")
            moved = cv2.warpAffine(rect2, np.float32([[1, 0, -shift[0]], [0, 1, -shift[1]]]), image_size)
            cv2.imwrite(os.path.join(results, "rgb_event_overlay.png"), cv2.addWeighted(rect1, 0.5, moved, 0.5, 0))
            images.append("rgb_event_overlay.png")
            blend = save_blend(results, first, samples[0], intrinsics, R, T, image_size, log)

    step("Saving")
    out = os.path.join(results, "rgb_event_calibration.npz")
    np.savez(
        out,
        pipeline_version=PIPELINE_VERSION,
        calibration_image_size=np.array(image_size),
        rgb_roi_fraction=np.array(config.RGB_ROI_FRACTION),
        pattern_size=np.array(config.PATTERN_SIZE),
        square_size_mm=square_mm,
        n_boards=len(samples),
        rgb_K_left=intrinsics["rgb_left"][1], rgb_D_left=intrinsics["rgb_left"][2],
        rgb_K_right=intrinsics["rgb_right"][1], rgb_D_right=intrinsics["rgb_right"][2],
        event_K_left=intrinsics["event_left"][1], event_D_left=intrinsics["event_left"][2],
        event_K_right=intrinsics["event_right"][1], event_D_right=intrinsics["event_right"][2],
        rgb_R=rgb_R, rgb_T=rgb_T, rgb_rms=rgb_rms,
        event_R=event_R, event_T=event_T, event_rms=event_rms,
        rgb_event_R=rgb_event_R, rgb_event_T=rgb_event_T, rgb_event_rms=rgb_event_rms,
    )
    log(f"Results in {results}")
    if install:
        shutil.copy(out, config.RGB_EVENT_CALIBRATION_FILE)
        log(f"Installed as {config.RGB_EVENT_CALIBRATION_FILE}")
    progress(1.0, "Done")

    return {
        "folder": folder,
        "square_size_mm": square_mm,
        "results": results,
        "run": os.path.basename(results),
        "installed": install,
        "n_found": n_found,
        "n_unique": n_unique,
        "n_used": len(samples),
        "intrinsics": {cam: {"rms": float(rms), "K": K.tolist(), "D": D.ravel().tolist(),
                             "median_view_error": float(np.median(view_errs[cam])),
                             "max_view_error": float(view_errs[cam].max())}
                       for cam, (rms, K, D) in intrinsics.items()},
        "extrinsics": [{"from": cam1, "to": cam2, "rms": float(rms), "T": T.ravel().tolist(),
                        "distance_mm": float(np.linalg.norm(T)),
                        "rotation_deg": float(np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0])))}
                       for (cam1, cam2), (rms, R, T) in extrinsics.items()],
        "images": images,
        "blend": blend,
        "poses": camera_poses(extrinsics),
        "rig_pairs": [["rgb_left", "rgb_right"], ["event_left", "event_right"],
                      ["rgb_left", "event_left"], ["rgb_right", "event_right"]],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", nargs="?", help="captures/<session> (default: latest)")
    parser.add_argument("--no-install", action="store_true",
                        help=f"Do not copy the result to {config.RGB_EVENT_CALIBRATION_FILE}")
    args = parser.parse_args()

    folder = args.folder or os.path.join(CAPTURES_PATH, sorted(os.listdir(CAPTURES_PATH))[-1])
    try:
        calibrate(folder, install=not args.no_install)
    except UserError as e:
        sys.exit(str(e))


if __name__ == "__main__":
    main()

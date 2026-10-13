"""Calibrate RGB-left <-> Livox (extrinsics only).

Usage: python -m cvlab.calibration.rgb_livox.solve [data/calibration/rgb_livox/captures/<session>] [--no-install]   (default: latest)
Also run from the "Calibrate RGB <-> Livox" page of the web app.

The Livox is a virtual camera (rgb_livox/lidar_image.py): its reflectivity images give chessboard
corners like a camera, so the extrinsics come from cv2.stereoCalibrate with both intrinsics fixed:
  - RGB-left: the rig intrinsics of the installed RGB-event calibration (one camera model for the
    whole rig),
  - Livox: the virtual pinhole, exact by construction.
A check independent of the corners: the Livox points on the board must lie on the board plane
seen by the camera. Results in data/calibration/rgb_livox/results/<time>/, installed to
config.RGB_LIVOX_CALIBRATION_FILE. Translations are in mm.

Method from ~/Downloads/LidarCameraCalibration_Solution.ipynb.
"""

import argparse
import os
import shutil
import sys
from datetime import datetime

import cv2
import h5py
import numpy as np

from cvlab import config
from cvlab.calibration.common import (UserError, full_frame_intrinsics, is_vertical_baseline, load_calibration,
                                      pattern_points, rectification_check_image, rectify_pair, session_square_size,
                                      stereo_rectify_maps)
from cvlab.calibration.rgb_event.solve import camera_poses, deduplicate, monotonic_radius
from cvlab.calibration.rgb_livox import lidar_image

CAPTURES_PATH = str(config.CALIBRATION_DIR / "rgb_livox" / "captures")
RESULTS_PATH = str(config.CALIBRATION_DIR / "rgb_livox" / "results")

CAMERAS = ["rgb_left", "livox"]
PIPELINE_VERSION = 1
DEDUP_PX = 20                 # boards closer than this (rgb_left corners) are duplicates
MIN_BOARDS = 6
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


def rig_intrinsics():
    """rgb_left K, D and the RGB-event calibration they come from."""
    path = config.RGB_EVENT_CALIBRATION_FILE
    if not os.path.exists(path):
        raise UserError(f"No RGB-event calibration installed ({path}): RGB-Livox uses its rgb_left intrinsics, "
                        "calibrate RGB <-> event first")
    try:
        calib = load_calibration(path)
    except ValueError as e:
        raise UserError(str(e))
    if tuple(calib["calibration_image_size"]) != tuple(config.CALIBRATION_IMAGE_SIZE):
        raise UserError(f"{path} was made for images of {tuple(calib['calibration_image_size'])}, "
                        f"now they are {config.CALIBRATION_IMAGE_SIZE}: recalibrate RGB <-> event")
    return calib["rgb_K_left"], calib["rgb_D_left"], calib


def stereo(samples, objp, K, D):
    """(rms, R, T, per-board rms) RGB-left -> lidar camera, both intrinsics fixed."""
    out = cv2.stereoCalibrateExtended(
        [objp] * len(samples), [s["rgb_left"] for s in samples], [s["livox"] for s in samples],
        K, D, lidar_image.lidar_K(), lidar_image.LIDAR_D, config.CALIBRATION_IMAGE_SIZE, None, None,
        flags=cv2.CALIB_FIX_INTRINSIC)
    rms, R, T, per_view = out[0], out[5], out[6], out[-1]
    return rms, R, T, np.sqrt(np.mean(per_view ** 2, axis=1))


def load_points(sample_folder):
    with h5py.File(os.path.join(sample_folder, "livox_points.h5"), "r") as f:
        return f["xyz_camera_mm"][:], f["intensity"][:]


def board_plane_residuals(sample_folder, sample, objp, K, D, R, T):
    """Signed distances (mm) of the Livox points on the board to the board plane seen by RGB-left.
    The points are picked inside the board in the lidar image (independent of R and T), the plane
    comes from solvePnP on the RGB corners with the known square size."""
    xyz, _ = load_points(sample_folder)
    w, h = config.LIVOX_IMAGE_SIZE
    mask = np.zeros((h, w), np.uint8)
    cv2.fillConvexPoly(mask, cv2.convexHull(sample["livox"].reshape(-1, 2)).astype(np.int32), 1)
    flat, idx = lidar_image.pixels(xyz)
    on_board = idx[mask.ravel()[flat] > 0]
    P = (xyz[on_board].astype(np.float64) - T.reshape(1, 3)) @ R   # rgb_left frame
    _, rvec, tvec = cv2.solvePnP(objp, sample["rgb_left"], K, D)
    normal = cv2.Rodrigues(rvec)[0][:, 2]
    return (P - tvec.ravel()) @ normal


def rig_poses(R, T, rig_calib):
    """Camera poses for the rig view: the RGB-event rig (installed calibration) plus the Livox."""
    extrinsics = {("rgb_left", "rgb_right"): (None, rig_calib["rgb_R"], rig_calib["rgb_T"]),
                  ("rgb_left", "event_left"): (None, rig_calib["rgb_event_R"], rig_calib["rgb_event_T"]),
                  ("event_left", "event_right"): (None, rig_calib["event_R"], rig_calib["event_T"])}
    poses = camera_poses(extrinsics)
    poses["livox"] = {"center": (-R.T @ T).ravel().tolist(), "axis": R.T[:, 2].tolist()}
    return poses


def calibrate(folder, install=True, log=print, progress=lambda fraction, label: None):
    """Full calibration of a captures/<session> folder; returns a summary dict for the solve
    page (see the end of the function), raises UserError."""
    total, done = 8, 0

    def step(label):
        nonlocal done
        progress(min(done / total, 0.99), label)
        done += 1

    step("Loading boards")
    log(f"Processing folder {folder}")
    samples = load_samples(folder)
    n_found = len(samples)
    log(f"{n_found} boards seen by RGB left and the Livox")
    samples = deduplicate(samples)
    n_unique = len(samples)
    log(f"{n_unique} boards after removing duplicates")
    if n_unique < MIN_BOARDS:
        raise UserError(f"Not enough boards to calibrate ({n_unique}, need at least {MIN_BOARDS}): "
                        "capture more, at different places and angles")

    step("RGB-left intrinsics of the rig")
    K, D, rig_calib = rig_intrinsics()
    log(f"RGB-left intrinsics from {config.RGB_EVENT_CALIBRATION_FILE}: fx {K[0, 0]:.1f} fy {K[1, 1]:.1f} "
        f"cx {K[0, 2]:.1f} cy {K[1, 2]:.1f}")
    square_mm = session_square_size(folder)
    log(f"Chessboard: {config.PATTERN_SIZE[0]}x{config.PATTERN_SIZE[1]} inner corners, {square_mm:g} mm squares")
    objp = pattern_points(square_size_mm=square_mm)

    step("Stereo RGB left -> Livox")
    rms, R, T, per_view = stereo(samples, objp, K, D)
    log(f"\tRMS {rms:.3f} px, worst board {per_view.max():.2f} px")
    bad = per_view > MAX_VIEW_ERROR_FACTOR * np.median(per_view)
    if bad.any() and len(samples) - bad.sum() >= MIN_BOARDS:
        log(f"Dropping {bad.sum()} outlier boards ({', '.join(s['name'] for s, b in zip(samples, bad) if b)}) "
            "and recalibrating")
        samples = [s for s, b in zip(samples, bad) if not b]
        rms, R, T, per_view = stereo(samples, objp, K, D)
        log(f"\tRMS {rms:.3f} px, worst board {per_view.max():.2f} px")
    angle = np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0]))
    log(f"Using {len(samples)} boards: T = {T.ravel().round(1)} mm (|T| = {np.linalg.norm(T):.1f} mm), "
        f"rotation {angle:.2f} deg")

    step("Board plane check")
    plane = []
    for s, err in zip(samples, per_view):
        d = board_plane_residuals(os.path.join(folder, s["name"]), s, objp, K, D, R, T)
        median = float(np.median(d)) if len(d) else None
        spread = float(1.4826 * np.median(np.abs(d - median))) if len(d) else None
        plane.append({"board": s["name"], "rms_px": float(err), "points": int(len(d)),
                      "offset_mm": median, "spread_mm": spread})
    offsets = [p["offset_mm"] for p in plane if p["offset_mm"] is not None]
    log(f"Livox points on the board vs the plane seen by RGB left: mean offset {np.mean(offsets):+.1f} mm, "
        f"worst {np.max(np.abs(offsets)):.1f} mm")

    results = os.path.join(RESULTS_PATH, datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
    os.makedirs(results)
    first = os.path.join(folder, samples[0]["name"])
    image_size = config.CALIBRATION_IMAGE_SIZE

    step("Rectification check")
    maps1, maps2, _ = stereo_rectify_maps(K, D, lidar_image.lidar_K(), lidar_image.LIDAR_D, R, T, image_size)
    rect1, rect2 = rectify_pair(cv2.imread(os.path.join(first, "rgb_left_corners.png")),
                                cv2.imread(os.path.join(first, "livox_corners.png")), maps1, maps2)
    images = ["rectification_rgb_left_livox.png"]
    cv2.imwrite(os.path.join(results, images[0]),
                rectification_check_image(rect1, rect2, vertical=is_vertical_baseline(T)))

    step("Livox depth on RGB left")
    blend = save_depth_blend(results, first, K, D, R, T, log)

    step("Saving")
    axes = config.LIDAR_TO_CAMERA_AXES["livox"].astype(np.float64)
    rgb_from_livox = np.eye(4)
    rgb_from_livox[:3, :3] = R.T @ axes          # P_rgb = R^T (A P_native - T)
    rgb_from_livox[:3, 3] = (-R.T @ T).ravel()
    out = os.path.join(results, "rgb_livox_calibration.npz")
    np.savez(
        out,
        pipeline_version=PIPELINE_VERSION,
        calibration_image_size=np.array(image_size),
        rgb_roi_fraction=np.array(config.RGB_ROI_FRACTION),
        livox_image_size=np.array(config.LIVOX_IMAGE_SIZE),
        pattern_size=np.array(config.PATTERN_SIZE),
        square_size_mm=square_mm,
        n_boards=len(samples),
        rgb_K_left=K, rgb_D_left=D, livox_K=lidar_image.lidar_K(), livox_D=lidar_image.LIDAR_D,
        R=R, T=T, rms=rms,                     # P_livox_camera = R P_rgb_left + T (livox camera: camera-convention axes)
        livox_to_camera_axes=axes,             # P_livox_camera = livox_to_camera_axes P_livox_native
        rgb_from_livox=rgb_from_livox,         # homogeneous: P_rgb_left = rgb_from_livox P_livox_native
    )
    log(f"Results in {results}")
    if install:
        shutil.copy(out, config.RGB_LIVOX_CALIBRATION_FILE)
        log(f"Installed as {config.RGB_LIVOX_CALIBRATION_FILE}")
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
        "intrinsics_note": f"RGB left: rig intrinsics from {os.path.relpath(config.RGB_EVENT_CALIBRATION_FILE, config.REPO_ROOT)}; "
                           f"Livox: virtual camera {config.LIVOX_IMAGE_SIZE[0]}x{config.LIVOX_IMAGE_SIZE[1]}, "
                           f"f = {config.LIVOX_FOCAL_PX:.0f} px",
        "extrinsics": [{"from": "rgb_left", "to": "livox", "rms": float(rms), "T": T.ravel().tolist(),
                        "distance_mm": float(np.linalg.norm(T)), "rotation_deg": float(angle)}],
        "plane": plane,
        "images": images,
        "blend": blend,
        "poses": rig_poses(R, T, rig_calib),
        "rig_pairs": [["rgb_left", "rgb_right"], ["event_left", "event_right"], ["rgb_left", "event_left"],
                      ["rgb_left", "livox"]],
    }


def save_depth_blend(results, sample_folder, K, D, R, T, log):
    """Livox depth of the first board's points on the full RGB-left frame (the same frame that was
    saved with the board): blend_depth.png / blend_rgb.png for the slider of the solve page."""
    full = cv2.imread(os.path.join(sample_folder, "rgb_left_full.png"))
    size = (full.shape[1], full.shape[0])
    K_full = full_frame_intrinsics(K, size)
    xyz, _ = load_points(sample_folder)
    # The full frame has ~2.5x the pixels of the calibration image per point: a bigger window closes the gaps
    depth = lidar_image.clean_depth(lidar_image.depth_map(xyz, K_full, D, R, T, size, monotonic_radius(D)), window=9)
    log(f"\tLivox depth on {size[0]}x{size[1]}: {np.mean(depth > 0):.1%} of the pixels")
    cv2.imwrite(os.path.join(results, "blend_rgb.png"), full)
    cv2.imwrite(os.path.join(results, "blend_depth.png"), lidar_image.colour_depth(depth))
    return {"title": "RGB ↔ Livox depth", "images": ["blend_depth.png", "blend_rgb.png"],
            "labels": ["Livox depth", "RGB"],
            "note": "Full RGB-left frame of the first board; Livox points of its capture, red = near, blue = far"}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", nargs="?", help="captures/<session> (default: latest)")
    parser.add_argument("--no-install", action="store_true",
                        help=f"Do not copy the result to {config.RGB_LIVOX_CALIBRATION_FILE}")
    args = parser.parse_args()

    folder = args.folder or os.path.join(CAPTURES_PATH, sorted(os.listdir(CAPTURES_PATH))[-1])
    try:
        calibrate(folder, install=not args.no_install)
    except UserError as e:
        sys.exit(str(e))


if __name__ == "__main__":
    main()

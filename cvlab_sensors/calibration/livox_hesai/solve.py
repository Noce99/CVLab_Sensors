"""Calibrate Livox <-> Hesai without a target, from static stops of the rig.

Usage: python -m cvlab_sensors.calibration.livox_hesai.solve [data/calibration/livox_hesai/captures/<session>] [--no-install]   (default: latest)
Also run from the "Calibrate Livox <-> Hesai" page of the web app.

Each stop holds the points of both lidars over the same seconds with the rig still. The Hesai
points are registered on the Livox surfaces (livox_hesai/registration.py) jointly over every stop,
from the initial guess (a ruler measurement is enough). Every stop is also solved alone: their
spread is the honest uncertainty, and the conditioning tells which motions the scenes leave weakly
constrained (planes only facing one way cannot fix a translation along them).
Results in data/calibration/livox_hesai/results/<time>/, installed to config.LIVOX_HESAI_CALIBRATION_FILE.
"""

import argparse
import os
import shutil
import sys
from datetime import datetime

import cv2
import h5py
import numpy as np
from scipy.spatial.transform import Rotation

from cvlab_sensors import config
from cvlab_sensors.calibration.common import UserError, load_calibration
from cvlab_sensors.calibration.livox_hesai import registration
from cvlab_sensors.calibration.rgb_event.solve import camera_poses

CAPTURES_PATH = str(config.CALIBRATION_DIR / "livox_hesai" / "captures")
RESULTS_PATH = str(config.CALIBRATION_DIR / "livox_hesai" / "results")

CAMERAS = ["livox", "hesai"]
PIPELINE_VERSION = 1
MIN_STOPS = 1
RECOMMENDED_STOPS = 3
WEAK_STRENGTH = 0.1           # conditioning strength below this: a weakly constrained direction
SPREAD_WARN = (10.0, 0.2)     # mm, deg: stops further than this from the joint solution are suspicious
VIEW_SIZE_PX = 1200    # width of the views; their scale follows the scene


def load_stop(stop_folder):
    """(hesai, livox) points in metres, camera convention."""
    clouds = []
    for cam in ("hesai", "livox"):
        with h5py.File(os.path.join(stop_folder, f"{cam}.h5"), "r") as f:
            clouds.append(f["xyz_camera_m"][:].astype(np.float64))
    return tuple(clouds)


def list_stops(folder):
    return [name for name in sorted(os.listdir(folder))
            if all(os.path.exists(os.path.join(folder, name, f"{cam}.h5")) for cam in CAMERAS)]


def initial_guess(initial_mm=None):
    """4x4 Hesai -> Livox (camera convention, m): identity rotation (the coarse axes of
    config.LIDAR_TO_CAMERA_AXES already align the sensors) and the measured translation."""
    T = np.eye(4)
    T[:3, 3] = np.array(initial_mm if initial_mm is not None else config.LIVOX_HESAI_INITIAL_T_MM) / 1000
    return T


def view_extent(livox):
    """Range (m) of the views: where 98 % of the Livox points are, 5 to 30 m."""
    reach = max(np.percentile(np.abs(livox[:, 0]), 98), np.percentile(livox[:, 2], 98))
    height = np.percentile(np.abs(livox[:, 1]), 98)
    return float(np.clip(np.ceil(reach), 5, 30)), float(np.clip(np.ceil(height), 2, 10))


def view(hesai, livox, T, kind, extent):
    """BGR image of both clouds seen from above ("top", forward up the page) or from the right
    ("side", forward to the right), 1 m grid, Livox magenta, Hesai green: white where they agree.
    extent: (reach, height) m from view_extent."""
    reach, height = extent
    # (axis on the width, its range), (axis growing up the page, its range); camera-convention
    # axes: 0 x right, 1 y down (shown as up = -y), 2 z forward
    (ha, (h0, h1)), (va, (v0, v1)) = {"top": ((0, (-reach, reach)), (2, (0, reach))),
                                      "side": ((2, (0, reach)), (1, (-height, height)))}[kind]
    s = VIEW_SIZE_PX / (h1 - h0)
    w, h = VIEW_SIZE_PX, int((v1 - v0) * s)
    up = (lambda p: -p[:, 1]) if va == 1 else (lambda p: p[:, va])

    image = np.zeros((h, w, 3), np.uint8)
    for m in range(int(np.ceil(h0)), int(h1) + 1):
        cv2.line(image, (int((m - h0) * s), 0), (int((m - h0) * s), h - 1), (50, 50, 50), 1)
    for m in range(int(np.ceil(v0)), int(v1) + 1):
        cv2.line(image, (0, int((v1 - m) * s)), (w - 1, int((v1 - m) * s)), (50, 50, 50), 1)
    hesai = hesai @ T[:3, :3].T + T[:3, 3]
    for points, channels in [(livox, (0, 2)), (hesai, (1,))]:
        u = (points[:, ha] - h0) * s
        v = (v1 - up(points)) * s
        inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        for c in channels:
            image[v[inside].astype(int), u[inside].astype(int), c] = 255
    cv2.circle(image, (int(-h0 * s), int(v1 * s)), 6, (255, 255, 255), 2)   # the Livox
    return image


def rig_poses(T):
    """Poses for the rig view: in the rgb_left frame when an RGB-Livox calibration is installed
    (with the RGB-event rig if installed too), else in the Livox frame."""
    hesai_in_livox = {"center": (T[:3, 3] * 1000).tolist(), "axis": T[:3, 2].tolist()}
    if not os.path.exists(config.RGB_LIVOX_CALIBRATION_FILE):
        return {"livox": {"center": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0]}, "hesai": hesai_in_livox}, "Livox"
    rl = np.load(config.RGB_LIVOX_CALIBRATION_FILE)
    R, t = rl["R"], rl["T"].reshape(3)             # P_livox = R P_rgb + t (mm)
    poses = {}
    if os.path.exists(config.RGB_EVENT_CALIBRATION_FILE):
        try:
            calib = load_calibration(config.RGB_EVENT_CALIBRATION_FILE)
            poses = camera_poses({("rgb_left", "rgb_right"): (None, calib["rgb_R"], calib["rgb_T"]),
                                  ("rgb_left", "event_left"): (None, calib["rgb_event_R"], calib["rgb_event_T"]),
                                  ("event_left", "event_right"): (None, calib["event_R"], calib["event_T"])})
        except ValueError:
            pass
    poses.setdefault("rgb_left", {"center": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0]})
    to_rgb = lambda p: R.T @ (np.asarray(p) - t)   # point in the Livox frame (mm) -> rgb_left
    poses["livox"] = {"center": to_rgb([0, 0, 0]).tolist(), "axis": (R.T @ [0, 0, 1]).tolist()}
    poses["hesai"] = {"center": to_rgb(hesai_in_livox["center"]).tolist(),
                      "axis": (R.T @ np.asarray(hesai_in_livox["axis"])).tolist()}
    return poses, "RGB left"


def calibrate(folder, install=True, log=print, progress=lambda fraction, label: None,
              initial_x_mm=None, initial_y_mm=None, initial_z_mm=None):
    """Joint registration over the stops of a captures/<session> folder; returns a summary dict
    for the solve page, raises UserError."""
    names = list_stops(folder)
    if len(names) < MIN_STOPS:
        raise UserError(f"No stop recorded in {folder}: record at least {RECOMMENDED_STOPS} on the capture page")
    initial_mm = None
    if None not in (initial_x_mm, initial_y_mm, initial_z_mm):
        initial_mm = (initial_x_mm, initial_y_mm, initial_z_mm)
    T0 = initial_guess(initial_mm)
    total, done = len(names) + 3, 0

    def step(label):
        nonlocal done
        progress(min(done / total, 0.99), label)
        done += 1

    step("Loading stops")
    log(f"Processing folder {folder}: {len(names)} stops")
    stops = [load_stop(os.path.join(folder, n)) for n in names]
    log(f"Initial guess: Hesai at {np.round(T0[:3, 3] * 1000, 1)} mm in the Livox frame")

    per_stop = []
    for name, stop in zip(names, stops):
        step(f"Stop {name} alone")
        T, report = registration.align([stop], T0)
        per_stop.append((T, report["stops"][0]))
        log(f"Stop {name}: {registration.describe(T)}")

    step("All stops together")
    log("Joint registration:")
    T, report = registration.align(stops, T0, log=log)
    rot = Rotation.from_matrix(T[:3, :3])
    log(f"Result: {registration.describe(T)}, rmse {report['rmse_m'] * 1000:.1f} mm")

    deviations = [registration.difference(P, T) for P, _ in per_stop]
    conditioning = registration.conditioning(report["hessian"])
    weak = [c["direction"] for c in conditioning if c["strength"] < WEAK_STRENGTH]
    warnings = []
    if len(names) < RECOMMENDED_STOPS:
        warnings.append(f"only {len(names)} stop(s): record at least {RECOMMENDED_STOPS} in different places")
    if weak:
        warnings.append(f"weakly constrained: {', '.join(weak)} — add stops with surfaces facing other ways "
                        "(ground, ramps, kerbs, tilted planes)")
    far = [n for n, (dt, dr) in zip(names, deviations) if dt > SPREAD_WARN[0] or dr > SPREAD_WARN[1]]
    if far:
        warnings.append(f"stops {', '.join(far)} disagree with the joint result by more than "
                        f"{SPREAD_WARN[0]:.0f} mm / {SPREAD_WARN[1]} deg: did the rig or the scene move?")
    for w in warnings:
        log(f"WARNING: {w}")

    results = os.path.join(RESULTS_PATH, datetime.now().strftime("%Y_%m_%d_%H_%M_%S"))
    os.makedirs(results)
    step("Views")
    hesai, livox = stops[0]
    images = []
    extent = view_extent(livox)
    for kind in ("top", "side"):
        cv2.imwrite(os.path.join(results, f"{kind}_view.png"), view(hesai, livox, T, kind, extent))
        images.append(f"{kind}_view.png")
    cv2.imwrite(os.path.join(results, "top_view_initial.png"), view(hesai, livox, T0, "top", extent))

    A_l = config.LIDAR_TO_CAMERA_AXES["livox"].astype(np.float64)
    A_h = config.LIDAR_TO_CAMERA_AXES["hesai"].astype(np.float64)
    livox_from_hesai = np.eye(4)                    # native frames, mm
    livox_from_hesai[:3, :3] = A_l.T @ T[:3, :3] @ A_h
    livox_from_hesai[:3, 3] = A_l.T @ (T[:3, 3] * 1000)
    out = os.path.join(results, "livox_hesai_calibration.npz")
    np.savez(out, pipeline_version=PIPELINE_VERSION, n_stops=len(names),
             R=T[:3, :3], t=T[:3, 3] * 1000,       # P_livox_camera = R P_hesai_camera + t (mm, camera convention)
             livox_to_camera_axes=A_l, hesai_to_camera_axes=A_h,
             livox_from_hesai=livox_from_hesai,    # homogeneous: P_livox_native = livox_from_hesai P_hesai_native
             rmse_mm=report["rmse_m"] * 1000)
    log(f"Results in {results}")
    if install:
        shutil.copy(out, config.LIVOX_HESAI_CALIBRATION_FILE)
        log(f"Installed as {config.LIVOX_HESAI_CALIBRATION_FILE}")
    progress(1.0, "Done")

    poses, origin = rig_poses(T)
    return {
        "folder": folder,
        "results": results,
        "run": os.path.basename(results),
        "installed": install,
        "n_found": len(names),
        "n_used": len(names),
        "count_labels": ["stops", "used"],
        "warnings": warnings,
        "rms_unit": "mm",
        "extrinsics": [{"from": "livox", "to": "hesai", "rms": report["rmse_m"] * 1000, "T": (T[:3, 3] * 1000).tolist(),
                        "distance_mm": float(np.linalg.norm(T[:3, 3]) * 1000),
                        "rotation_deg": float(np.degrees(rot.magnitude()))}],
        "extrinsics_note": "T: Hesai origin in the Livox frame (x right, y down, z forward); rotation on top of "
                           "the coarse axes of config.LIDAR_TO_CAMERA_AXES. RMS: point-to-plane distance of the "
                           "matched points.",
        "tables": [
            {"title": "Stops", "note": "each stop solved alone vs all together: their spread is the uncertainty",
             "columns": ["Stop", "Overlap", "RMS mm", "T alone (mm)", "Off by mm", "Off by deg"],
             "rows": [[n, f"{s['fitness']:.0%}", f"{s['rmse_m'] * 1000:.1f}",
                       ", ".join(f"{v:.1f}" for v in P[:3, 3] * 1000), f"{dt:.1f}", f"{dr:.3f}"]
                      for n, (P, s), (dt, dr) in zip(names, per_stop, deviations)],
             "flags": [["", "", "", "", "bad" if dt > SPREAD_WARN[0] else "ok", "bad" if dr > SPREAD_WARN[1] else "ok"]
                       for dt, dr in deviations]},
            {"title": "Constraint by direction",
             "note": "how strongly the surfaces of all stops fix each motion, relative to the strongest "
                     f"(rotations as their effect at 5 m); below {WEAK_STRENGTH} the result can drift that way",
             "columns": ["Direction", "Strength"],
             "rows": [[c["direction"], f"{c['strength']:.3f}"] for c in conditioning],
             "flags": [["", "bad" if c["strength"] < WEAK_STRENGTH else "warn" if c["strength"] < 2 * WEAK_STRENGTH else "ok"]
                       for c in conditioning]},
        ],
        "images": images,
        "images_title": "Calibrated clouds",
        "images_note": f"stop {names[0]}: top view (forward up) and side view (forward right), 1 m grid; "
                       "Livox magenta, Hesai green, white where they agree",
        "blend": {"title": "Initial guess ↔ calibrated", "images": ["top_view_initial.png", "top_view.png"],
                  "labels": ["Initial guess", "Calibrated"],
                  "note": f"Top view of stop {names[0]}, forward up, 1 m grid: Livox magenta, Hesai green, white where they agree"},
        "poses": poses,
        "rig_origin": origin,
        "rig_pairs": [["rgb_left", "rgb_right"], ["event_left", "event_right"], ["rgb_left", "event_left"],
                      ["rgb_left", "livox"], ["livox", "hesai"]],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder", nargs="?", help="captures/<session> (default: latest)")
    parser.add_argument("--initial", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="Hesai origin in the Livox frame, mm (default: config.LIVOX_HESAI_INITIAL_T_MM)")
    parser.add_argument("--no-install", action="store_true",
                        help=f"Do not copy the result to {config.LIVOX_HESAI_CALIBRATION_FILE}")
    args = parser.parse_args()

    folder = args.folder or os.path.join(CAPTURES_PATH, sorted(os.listdir(CAPTURES_PATH))[-1])
    initial = dict(zip(["initial_x_mm", "initial_y_mm", "initial_z_mm"], args.initial)) if args.initial else {}
    try:
        calibrate(folder, install=not args.no_install, **initial)
    except UserError as e:
        sys.exit(str(e))


if __name__ == "__main__":
    main()

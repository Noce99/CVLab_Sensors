"""The rig as the installed calibrations describe it: every sensor they place, in one frame."""

import os

import numpy as np

from cvlab import config
from cvlab.calibration.common import load_calibration
from cvlab.calibration.rgb_event.solve import camera_poses

PAIRS = [["rgb_left", "rgb_right"], ["event_left", "event_right"], ["rgb_left", "event_left"],
         ["rgb_right", "event_right"], ["rgb_left", "livox"], ["livox", "hesai"]]
ORIGIN = {"center": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0]}


def installed_rig():
    """{"poses": {sensor: {"center" mm, "axis"}}, "pairs", "origin", "notes"} from the installed
    calibrations, in the rgb_left frame when a camera calibration is installed, else in the Livox
    frame. Sensors no installed calibration links to the others are left out (see notes)."""
    notes = []
    camera_rig = {}
    if os.path.exists(config.RGB_EVENT_CALIBRATION_FILE):
        try:
            c = load_calibration(config.RGB_EVENT_CALIBRATION_FILE)
            camera_rig = camera_poses({("rgb_left", "rgb_right"): (None, c["rgb_R"], c["rgb_T"]),
                                       ("rgb_left", "event_left"): (None, c["rgb_event_R"], c["rgb_event_T"]),
                                       ("event_left", "event_right"): (None, c["event_R"], c["event_T"])})
        except (ValueError, KeyError) as e:
            notes.append(f"RGB ↔ event calibration not shown: {e}")

    livox_in_rgb = None   # P_rgb = R_rl^T (P_livox - T_rl), mm
    if os.path.exists(config.RGB_LIVOX_CALIBRATION_FILE):
        rl = np.load(config.RGB_LIVOX_CALIBRATION_FILE)
        livox_in_rgb = (rl["R"], rl["T"].reshape(3))

    hesai_in_livox = None   # P_livox = R_lh P_hesai + t_lh, mm
    if os.path.exists(config.LIVOX_HESAI_CALIBRATION_FILE):
        lh = np.load(config.LIVOX_HESAI_CALIBRATION_FILE)
        hesai_in_livox = (lh["R"], lh["t"].reshape(3))

    lidars = {}
    if hesai_in_livox is not None:
        R, t = hesai_in_livox
        lidars = {"livox": ORIGIN, "hesai": {"center": t.tolist(), "axis": R[:, 2].tolist()}}

    if livox_in_rgb is not None:
        R, T = livox_in_rgb
        to_rgb = lambda p: (R.T @ (np.asarray(p, dtype=np.float64) - T)).tolist()
        turn = lambda a: (R.T @ np.asarray(a, dtype=np.float64)).tolist()
        poses = dict(camera_rig) or {"rgb_left": ORIGIN}
        poses["livox"] = {"center": to_rgb([0, 0, 0]), "axis": turn([0, 0, 1])}
        if "hesai" in lidars:
            poses["hesai"] = {"center": to_rgb(lidars["hesai"]["center"]), "axis": turn(lidars["hesai"]["axis"])}
        origin = "RGB left"
    elif camera_rig:
        poses, origin = camera_rig, "RGB left"
        if lidars:
            notes.append("Livox and Hesai not shown: no RGB ↔ Livox calibration places them relative to the cameras")
    else:
        poses, origin = lidars, "Livox"
    return {"poses": poses, "pairs": PAIRS, "origin": origin, "notes": notes}

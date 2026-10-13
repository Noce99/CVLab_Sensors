"""Livox <-> Hesai calibration data capture (page of the web app, cvlab/app.py).

No target: the rig stands still in a structured place and both lidars record the same seconds of
the scene (a "stop"). Each stop is checked right away: the rig must not have moved (the first and
second half of the Livox points must coincide), the two lidars must overlap, and the page shows
which motions the scene constrains, so you know what the next stop should add. Stops are saved to
data/calibration/livox_hesai/captures/<session>/<index>/ (points on a 1 cm grid), solve.py uses them all.
"""

import io
import json
import os
import shutil
import threading
import time
from datetime import datetime

import cv2
import h5py
import numpy as np
from flask import Blueprint, jsonify, render_template, request, send_file

from cvlab import config
from cvlab.calibration.common import CaptureError, sessions
from cvlab.calibration.livox_hesai import registration
from cvlab.calibration.livox_hesai.solve import CAMERAS, CAPTURES_PATH, WEAK_STRENGTH, initial_guess, list_stops
from cvlab.calibration.rgb_livox import lidar_image
from cvlab.web import data_url, jpeg, preview

SAVE_VOXEL_M = 0.01             # stops are saved on this grid (the finest registration level is 2 cm)
PREVIEW_WINDOW_S = 2
MIN_CAPTURE_S, MAX_CAPTURE_S = 3, 20
ARRIVAL_WAIT_S = 3.0            # max wait for the points of the end of the capture
MOVED = (10.0, 0.1)             # mm, deg between the two halves of the Livox points: the rig moved
QUICK_SCHEDULE = registration.SCHEDULE[:3]   # down to 5 cm: enough to judge a stop

TOOL = {
    "id": "livox_hesai",
    "title": "Livox-Hesai calibration data",
    "cameras": CAMERAS,
    "labels": {"livox": "Livox reflectivity", "hesai": "Hesai reflectivity"},
    "preview_delay_ms": {"livox": 500, "hesai": 500},
    "aspect": {cam: list(config.LIVOX_IMAGE_SIZE) for cam in CAMERAS},
    "capture_s": config.LIVOX_HESAI_CAPTURE_S,
    "solve_url": "/livox-hesai/calibrate",
}


class LivoxHesaiCapture:
    def __init__(self, sensors):
        self.sensors = sensors
        self.folder = None
        self.capture_lock = threading.Lock()
        self.open_session(None)

    def open_session(self, name):
        if name is None:
            name = datetime.now().strftime(config.SESSION_NAME_FORMAT)
        elif name not in os.listdir(CAPTURES_PATH):
            raise ValueError(f"no session {name!r}")
        self.folder = os.path.join(CAPTURES_PATH, name)   # created with the first stop

    def streams(self):
        return {"livox": self.sensors.livox, "hesai": self.sensors.hesai}

    def status(self):
        return {cam: stream.status() for cam, stream in self.streams().items()}

    def latest(self, cam):
        stream = self.streams()[cam]
        end_us = stream.latest_us()
        if end_us is None:
            return None
        xyz, intensity, _ = stream.points_between(end_us - PREVIEW_WINDOW_S * 1_000_000, end_us)
        return end_us, lidar_image.display(lidar_image.reflectivity_image(xyz, intensity))

    def stops(self):
        out = []
        for name in list_stops(self.folder) if os.path.isdir(self.folder) else []:
            with open(os.path.join(self.folder, name, "check.json")) as f:
                out.append({"name": name, **json.load(f)})
        return out

    def delete(self, name):
        if name not in (list_stops(self.folder) if os.path.isdir(self.folder) else []):
            return False
        shutil.rmtree(os.path.join(self.folder, name))
        return True

    def capture(self, seconds):
        streams = self.streams()
        for cam, stream in streams.items():
            if stream.latest_us() is None:
                raise CaptureError(f"no {cam} points ({stream.error or 'lidar starting'})")

        # Only the points of the next `seconds`, with the rig and the scene still
        start_us = time.time_ns() // 1000
        end_us = start_us + int(seconds * 1_000_000)
        time.sleep(seconds)
        deadline = time.monotonic() + ARRIVAL_WAIT_S
        while any((s.latest_us() or 0) < end_us for s in streams.values()) and time.monotonic() < deadline:
            time.sleep(0.05)
        points = {cam: s.points_between(start_us, end_us) for cam, s in streams.items()}
        for cam, (xyz, _, _) in points.items():
            if not len(xyz):
                raise CaptureError(f"no {cam} points during the capture ({streams[cam].error or 'is it streaming?'})")

        # Rig still: the Livox points of the second half must lie on those of the first half
        xyz, _, t = points["livox"]
        middle = (start_us + end_us) // 2
        halves = [registration.in_range(registration.downsample(xyz[m] / 1000, SAVE_VOXEL_M))
                  for m in (t >= middle, t < middle)]
        T_halves, _ = registration.align([tuple(halves)], np.eye(4), QUICK_SCHEDULE)
        moved_mm, moved_deg = registration.difference(T_halves, np.eye(4))
        moved = moved_mm > MOVED[0] or moved_deg > MOVED[1]

        clouds = {cam: registration.in_range(registration.downsample(xyz / 1000, SAVE_VOXEL_M))
                  for cam, (xyz, _, _) in points.items()}
        T, report = registration.align([(clouds["hesai"], clouds["livox"])], initial_guess(), QUICK_SCHEDULE)
        weakest = registration.conditioning(report["hessian"])[0]
        check = {
            "seconds": seconds,
            "points": {cam: int(len(points[cam][0])) for cam in CAMERAS},
            "overlap": round(report["stops"][0]["fitness"], 3),
            "rmse_mm": round(report["stops"][0]["rmse_m"] * 1000, 1) if report["stops"][0]["rmse_m"] else None,
            "t_mm": [round(v, 1) for v in T[:3, 3] * 1000],
            "weakest": weakest["direction"],
            "weakest_strength": round(weakest["strength"], 3),
            "moved_mm": round(moved_mm, 1),
            "moved_deg": round(moved_deg, 3),
        }
        images = {cam: lidar_image.reflectivity_image(xyz, intensity) for cam, (xyz, intensity, _) in points.items()}
        result = {"saved": None, "moved": moved, "check": check,
                  "cameras": {cam: {"image": data_url(preview(lidar_image.display(images[cam]))),
                                    "points": check["points"][cam]} for cam in CAMERAS}}
        if moved:
            return result

        index = max((int(n) for n in list_stops(self.folder)), default=-1) + 1 if os.path.isdir(self.folder) else 0
        name = f"{index:05d}"
        stop = os.path.join(self.folder, name)
        os.makedirs(stop)
        for cam in CAMERAS:
            with h5py.File(os.path.join(stop, f"{cam}.h5"), "w") as f:
                f.create_dataset("xyz_camera_m", data=clouds[cam].astype(np.float32))
                f.attrs["voxel_m"] = SAVE_VOXEL_M
                f.attrs["host_us"] = [start_us, end_us]
            cv2.imwrite(os.path.join(stop, f"{cam}.png"), lidar_image.display(images[cam]))
        with open(os.path.join(stop, "check.json"), "w") as f:
            json.dump(check, f, indent=1)
        with open(os.path.join(stop, "log.log"), "w") as f:
            f.write(f"Captured {datetime.now().isoformat()} by cvlab/calibration/livox_hesai/capture.py\n"
                    f"Host time {start_us} to {end_us} us ({seconds} s), points on a {SAVE_VOXEL_M * 100:.0f} cm grid\n"
                    f"{json.dumps(check)}\n")
        result["saved"] = name
        return result


def blueprint(sensors):
    tool = LivoxHesaiCapture(sensors)
    prefix = "/livox-hesai/capture"
    bp = Blueprint("livox_hesai_capture", __name__, url_prefix=prefix)

    @bp.route("")
    def page():
        busy = sensors.blocked()
        if busy:
            return render_template("busy.html", title=TOOL["title"], reason=busy)
        sensors.use("livox_hesai")
        return render_template("scene_capture.html", tool=dict(TOOL, api=f"{prefix}/api"),
                               weak_strength=WEAK_STRENGTH)

    @bp.route("/api/status")
    def status():
        return jsonify({"sensors": tool.status(), "folder": os.path.relpath(tool.folder, config.REPO_ROOT),
                        "session": os.path.basename(tool.folder)})

    @bp.route("/api/sessions")
    def list_sessions():
        return jsonify(sessions(CAPTURES_PATH))

    @bp.route("/api/session", methods=["POST"])
    def set_session():
        try:
            tool.open_session(request.get_json(force=True).get("name"))
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        return jsonify({"session": os.path.basename(tool.folder)})

    @bp.route("/api/preview/<cam>")
    def live_preview(cam):
        if cam not in CAMERAS:
            return jsonify({"error": f"unknown sensor {cam}"}), 404
        frame = tool.latest(cam)
        if frame is None:
            return jsonify({"error": "no points"}), 503
        host_us, image = frame
        response = send_file(io.BytesIO(jpeg(preview(image), 70)), mimetype="image/jpeg")
        response.headers["Cache-Control"] = "no-store"
        response.headers["X-Frame-Age-Ms"] = f"{(time.time_ns() // 1000 - host_us) / 1e3:.0f}"
        return response

    @bp.route("/api/stops")
    def stops():
        return jsonify(tool.stops())

    @bp.route("/api/stops/<name>", methods=["DELETE"])
    def delete_stop(name):
        if not tool.delete(name):
            return jsonify({"error": f"no stop {name}"}), 404
        return jsonify({"status": "deleted"})

    @bp.route("/api/stops/<name>/<cam>.jpg")
    def stop_image(name, cam):
        if cam not in CAMERAS or name not in list_stops(tool.folder):
            return jsonify({"error": "not found"}), 404
        image = cv2.imread(os.path.join(tool.folder, name, f"{cam}.png"))
        return send_file(io.BytesIO(jpeg(preview(image))), mimetype="image/jpeg")

    @bp.route("/api/capture", methods=["POST"])
    def capture():
        if not tool.capture_lock.acquire(blocking=False):
            return jsonify({"error": "a capture is already running"}), 409
        try:
            seconds = float((request.get_json(silent=True) or {}).get("seconds") or config.LIVOX_HESAI_CAPTURE_S)
            return jsonify(tool.capture(min(max(seconds, MIN_CAPTURE_S), MAX_CAPTURE_S)))
        except CaptureError as e:
            return jsonify({"error": str(e)}), 503
        finally:
            tool.capture_lock.release()

    return bp

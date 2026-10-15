"""Replay recorded sessions (page of the web app, cvlab_sensors/app.py).

Every frame is rendered on the server at a requested host time, from the recorded data at its
resolution: one sensor alone, or all of them reprojected on RGB-left with an alpha per sensor
(see views.Rig). The page plays them back along a timeline.
"""

import io
import json
import os
import time

import numpy as np
from flask import Blueprint, jsonify, render_template, request, send_file

from cvlab_sensors import config
from cvlab_sensors.calibration.common import to_calibration_size
from cvlab_sensors.recording.recorders import discard_session, save_session
from cvlab_sensors.replay import views
from cvlab_sensors.replay.session import (DEFAULT_EVENT_ACCUMULATION_MS, DEFAULT_EVENT_REPRESENTATION, EVENTS, LIDARS,
                                  MAX_EVENT_ACCUMULATION_MS, MIN_EVENT_ACCUMULATION_MS, RGB, SENSORS, Preparation,
                                  Session, event_cache_name, find_sessions)
from cvlab_sensors.sensors.events import EVENT_REPRESENTATIONS
from cvlab_sensors.web import jpeg

LABELS = {"rgb_left": "RGB left", "rgb_right": "RGB right", "event_left": "Event left",
          "event_right": "Event right", "livox": "Livox", "hesai": "Hesai"}
DEFAULT_ALPHA = {"rgb_left": 1.0, "rgb_right": 0.0, "event_left": 0.6, "event_right": 0.0, "livox": 0.8, "hesai": 0.8}
CALIBRATION_FILES = (config.RGB_EVENT_CALIBRATION_FILE, config.RGB_LIVOX_CALIBRATION_FILE,
                     config.LIVOX_HESAI_CALIBRATION_FILE)
EVENT_MIN_LEVEL = 12    # event frames are black where nothing happened: dimmer pixels stay transparent


class Replay:
    def __init__(self):
        self.sessions = {}      # name -> Session, loaded once
        self.preparation = Preparation()
        self.rig = None
        self.rig_stamp = None

    def session(self, name):
        if name not in self.sessions:
            match = [s for s in find_sessions() if s["name"] == name]
            if not match:
                raise KeyError(name)
            self.sessions[name] = Session(match[0]["path"])
        return self.sessions[name]

    def keep_or_discard(self, name, recording, keep):
        """Save (keep=True) or discard a session left in staging: one recorded before the server
        restarted, which the Record page no longer knows. Returns the saved path or None."""
        match = [s for s in find_sessions() if s["name"] == name]
        if not match:
            raise KeyError(name)
        if match[0]["place"] != "staging":
            raise ValueError(f"{name} is already saved")
        if recording.session is not None and recording.session.name == name:
            raise ValueError(f"{name} is the Record page's session: save or discard it there")
        status = self.preparation.status()
        if status["state"] == "running" and status["session"] == name:
            raise ValueError("its events are being prepared: wait for the end")
        self.sessions.pop(name, None)   # its path changes or disappears
        if keep:
            return save_session(match[0]["path"])
        discard_session(match[0]["path"])
        return None

    def current_rig(self):
        """The installed calibrations, reloaded when one of them changes."""
        stamp = tuple(os.path.getmtime(p) if os.path.exists(p) else None for p in CALIBRATION_FILES)
        if stamp != self.rig_stamp:
            self.rig, self.rig_stamp = views.Rig(), stamp
        return self.rig

    def render(self, session, t, view, full_res, depth_m, lidar_ms, alphas, cache, warp="lidar"):
        """(BGR image, info) of the time t, events from the cache named `cache` (event_cache_name): info = {"times": {sensor: host_us}, "notes": {sensor: why not
        shown}, "warp": how the other cameras were reprojected}. warp: "lidar" (per-pixel lidar depth,
        depth_m where the lidars see nothing) or "plane" (everything at depth_m)."""
        times, notes = {}, {}
        window_us = int(lidar_ms * 1000)
        if view in RGB:
            frame = session.rgb_frame(view, t)
            if frame is None:
                return None, {"times": times, "notes": {view: "no frame near this time"}}
            times[view] = frame[0]
            return (frame[1] if full_res else to_calibration_size(frame[1])), {"times": times, "notes": notes}
        if view in EVENTS:
            frame = session.event_frame(cache, view, t)
            if frame is None:
                note = ("prepare the events first" if session.event_index(cache, view) is None
                        else "no events near this time")
                return None, {"times": times, "notes": {view: note}}
            times[view] = frame[0]
            return frame[1], {"times": times, "notes": notes}
        if view in LIDARS:
            xyz, intensity = session.lidar_points(view, t, window_us)
            if not len(xyz):
                return None, {"times": times, "notes": {view: "no points in this window"}}
            times[view] = t
            image = views.livox_view(xyz, intensity) if view == "livox" else views.panorama(xyz, intensity)
            return image, {"times": times, "notes": notes}

        # All sensors on RGB-left
        rig = self.current_rig()
        base = session.rgb_frame("rgb_left", t)
        if base is not None:
            times["rgb_left"] = base[0]
            image = base[1] if full_res else to_calibration_size(base[1])
        else:
            notes["rgb_left"] = "no frame near this time"
            w, h = config.CALIBRATION_IMAGE_SIZE
            image = np.zeros((h, w, 3), np.uint8)
        out = (image.astype(np.float32) * alphas.get("rgb_left", 1.0)).astype(np.uint8)
        size = (out.shape[1], out.shape[0])
        if rig.K_left is None:
            notes.update({s: rig.missing.get(s, "no calibration") for s in SENSORS if s != "rgb_left"})
            return out, {"times": times, "notes": notes}
        depth_mm = depth_m * 1000
        cameras = [s for s in ("rgb_right", "event_left", "event_right") if alphas.get(s, 0) > 0]
        depth = depth_mm
        info_warp = f"plane at {depth_m:g} m"
        if cameras and warp == "lidar":
            clouds = [(s, session.lidar_points(s, t, window_us)[0]) for s in LIDARS if s in rig.lidars]
            clouds = [(s, xyz) for s, xyz in clouds if len(xyz)]
            if clouds:
                depth, coverage = rig.depth_map(clouds, size, depth_mm)
                info_warp = (f"lidar depth ({' + '.join(s.capitalize() for s, _ in clouds)}) on {coverage:.0%} "
                             f"of the image, plane at {depth_m:g} m elsewhere")
            else:
                why = rig.missing.get("livox") or "no lidar points in the window"
                info_warp = f"plane at {depth_m:g} m (no lidar depth: {why})"

        for sensor in cameras:
            alpha = alphas[sensor]
            if sensor == "rgb_right":
                frame = session.rgb_frame(sensor, t)
                layer = None if frame is None else to_calibration_size(frame[1])
            else:
                frame = session.event_frame(cache, sensor, t)
                layer = None if frame is None else frame[1]
            if frame is None:
                notes[sensor] = ("prepare the events first"
                                 if sensor in EVENTS and session.event_index(cache, sensor) is None
                                 else "no data near this time")
                continue
            times[sensor] = frame[0]
            warped, mask = rig.warp(sensor, layer, size, depth)
            if sensor in EVENTS:   # only the pixels with events, the black background stays transparent
                mask &= warped.max(axis=2) > EVENT_MIN_LEVEL
            views.blend(out, warped, mask, alpha)

        for sensor in LIDARS:
            alpha = alphas.get(sensor, 0)
            if alpha <= 0:
                continue
            if sensor not in rig.lidars:
                notes[sensor] = rig.missing.get(sensor, "no calibration")
                continue
            xyz, _ = session.lidar_points(sensor, t, window_us)
            if not len(xyz):
                notes[sensor] = "no points in this window"
                continue
            uv, depth = rig.project_lidar(sensor, xyz, size)
            if not len(depth):
                notes[sensor] = "no points in the RGB-left view"
                continue
            times[sensor] = t
            order = np.argsort(-depth)        # far first, near on top
            layer, mask = views.draw_points(out.shape, uv[order], views.depth_colours(depth[order]),
                                            dot=views.LIDAR_DOT_PX * (2 if full_res else 1))
            views.blend(out, layer, mask, alpha)
        return out, {"times": times, "notes": notes, "warp": info_warp if cameras else None}


def event_settings(args):
    """(representation, accumulation ms) of a request: ?events=<representation>&acc_ms=<ms>."""
    representation = args.get("events", DEFAULT_EVENT_REPRESENTATION)
    accumulation_ms = int(float(args.get("acc_ms", DEFAULT_EVENT_ACCUMULATION_MS)))
    event_cache_name(representation, accumulation_ms)   # ValueError when out of range
    return representation, accumulation_ms


def blueprint(sensors, replay, recording):
    bp = Blueprint("replay", __name__, url_prefix="/replay")

    @bp.route("")
    def page():
        sensors.use(None)
        return render_template("replay.html", labels=LABELS, sensors=SENSORS, alphas=DEFAULT_ALPHA,
                               representations=EVENT_REPRESENTATIONS, default_representation=DEFAULT_EVENT_REPRESENTATION,
                               default_accumulation_ms=DEFAULT_EVENT_ACCUMULATION_MS,
                               accumulation_range=(MIN_EVENT_ACCUMULATION_MS, MAX_EVENT_ACCUMULATION_MS))

    @bp.route("/api/sessions")
    def sessions():
        return jsonify(find_sessions())

    @bp.route("/api/session/<name>")
    def session_summary(name):
        try:
            session = replay.session(name)
        except KeyError:
            return jsonify({"error": f"no session {name}"}), 404
        try:
            cache = event_cache_name(*event_settings(request.args))
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        rig = replay.current_rig()
        return jsonify({**session.summary(cache), "rig_missing": rig.missing})

    @bp.route("/api/session/<name>/prepare", methods=["POST"])
    def prepare(name):
        try:
            session = replay.session(name)
        except KeyError:
            return jsonify({"error": f"no session {name}"}), 404
        try:
            representation, accumulation_ms = event_settings(request.args)
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        if not replay.preparation.start(session, representation, accumulation_ms):
            return jsonify({"error": "already preparing a session"}), 409
        return jsonify({"started": name})

    @bp.route("/api/prepare")
    def prepare_status():
        return jsonify(replay.preparation.status())

    @bp.route("/api/session/<name>/frame")
    def frame(name):
        try:
            session = replay.session(name)
        except KeyError:
            return jsonify({"error": f"no session {name}"}), 404
        args = request.args
        try:
            t = int(args["t"])
            alphas = {k: float(v) for k, v in json.loads(args.get("layers", "{}")).items() if k in SENSORS}
            depth_m = min(max(float(args.get("depth", 5)), 0.5), 200)
            lidar_ms = min(max(float(args.get("lidar_ms", 100)), 10), 10_000)
            cache = event_cache_name(*event_settings(args))
        except (KeyError, ValueError) as e:
            return jsonify({"error": f"bad request: {e}"}), 400
        view = args.get("view", "overlay")
        if view != "overlay" and view not in SENSORS:
            return jsonify({"error": f"unknown view {view}"}), 400
        started = time.monotonic()
        image, info = replay.render(session, t, view, args.get("res") == "full", depth_m, lidar_ms, alphas, cache,
                                    "plane" if args.get("warp") == "plane" else "lidar")
        if image is None:
            w, h = config.CALIBRATION_IMAGE_SIZE
            image = np.zeros((h, w, 3), np.uint8)
        response = send_file(io.BytesIO(jpeg(image, 85)), mimetype="image/jpeg")
        response.headers["Cache-Control"] = "no-store"
        info["render_ms"] = round((time.monotonic() - started) * 1000)
        info["size"] = [image.shape[1], image.shape[0]]
        response.headers["X-Frame-Info"] = json.dumps(info)
        return response

    @bp.route("/api/session/<name>/save", methods=["POST"])
    def save(name):
        return keep_or_discard(name, True)

    @bp.route("/api/session/<name>/discard", methods=["POST"])
    def discard(name):
        return keep_or_discard(name, False)

    def keep_or_discard(name, keep):
        try:
            saved = replay.keep_or_discard(name, recording, keep)
        except KeyError:
            return jsonify({"error": f"no session {name}"}), 404
        except (ValueError, FileExistsError) as e:
            return jsonify({"error": str(e)}), 409
        return jsonify({"saved": saved.name} if saved else {"discarded": name})

    @bp.route("/api/session/<name>/neighbours")
    def neighbours(name):
        """Previous and next RGB-left frame times around t, to step frame by frame."""
        try:
            session = replay.session(name)
        except KeyError:
            return jsonify({"error": f"no session {name}"}), 404
        previous, following = session.rgb_neighbours(int(request.args.get("t", 0)))
        return jsonify({"previous": previous, "next": following})

    return bp

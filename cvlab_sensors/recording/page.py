"""Record page: the recorders (cvlab_sensors.recording.rgb_recorder, cvlab_sensors.recording.event_recorder,
lidar_recorders/save_point_cloud_{livox,hesai}) started and stopped from the web.

A session is recorded in config.STAGING_DIR, "Save" moves it to config.SESSIONS_DIR, "Discard" deletes it. Recording
stops by itself below config.MIN_FREE_DISK_GB. The live previews are the small images the recorders
write twice a second (<session>/previews/), not the recorded data: replay the session for that.
"""

import io
import os
import subprocess
import threading
import time
from datetime import datetime

from flask import Blueprint, jsonify, render_template, send_file

from cvlab_sensors import config
from cvlab_sensors.recording.recorders import (build_recorders, check_recorders_exist, create_session, discard_session,
                                       free_disk_gb, save_session)
from cvlab_sensors.replay import views
from cvlab_sensors.sensors.lidar import LidarChunk, list_chunks
from cvlab_sensors.settings import page as settings_page
from cvlab_sensors.settings import store
from cvlab_sensors.web import jpeg

GROUP_LABELS = {"rgb": "RGB left + right", "event": "Event left + right", "lidar_livox": "Livox", "lidar_hesai": "Hesai"}
OUTPUT_LABELS = {"rgb_left": "RGB left", "rgb_right": "RGB right", "event_left": "Event left",
                 "event_right": "Event right", "lidar_livox": "Livox", "lidar_hesai": "Hesai"}


def orphan_recorders_message():
    """A warning when recorders of a previous run still write into the staging folder (the server
    was killed while recording: they go on until stopped, without the disk guard)."""
    pids = []
    for pid in os.listdir("/proc"):
        if pid.isdigit():
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as f:
                    cmdline = f.read().decode(errors="replace")
            except OSError:
                continue
            if str(config.STAGING_DIR) in cmdline and ("cvlab_sensors.recording" in cmdline or "save_point_cloud" in cmdline):
                pids.append(pid)
    if not pids:
        return None
    return (f"recorders of a previous run are still recording into {config.STAGING_DIR} (pids {', '.join(pids)}): "
            "stop them (kill -INT) before recording again")


class Recording:
    """The recorders, the current session and the disk guard."""

    def __init__(self):
        self.groups = build_recorders()
        self.recorders = [r for group in self.groups.values() for r in group]
        self.outputs = {out: r for r in self.recorders for out in r.outputs}
        self.lock = threading.Lock()
        self.session = None
        self.started = None
        self.message = None
        self.lidar_cache = {}   # output -> (chunk path, preview image)
        self.message = orphan_recorders_message()
        threading.Thread(target=self.disk_guard, daemon=True).start()

    def any_running(self):
        return any(r.is_running() for r in self.recorders)

    def disk_guard(self):
        while True:
            if self.any_running() and free_disk_gb() < config.MIN_FREE_DISK_GB:
                self.message = f"Less than {config.MIN_FREE_DISK_GB} GB free: recording stopped"
                print(f"WARNING: {self.message}")
                self.stop(None)
            time.sleep(1.0)

    def start(self, groups):
        missing = check_recorders_exist()
        if missing:
            raise RuntimeError("missing recorders: " + ", ".join(missing))
        if free_disk_gb() < config.MIN_FREE_DISK_GB:
            raise RuntimeError(f"only {free_disk_gb():.1f} GB free: free some space first")
        with self.lock:
            self.message = None
            if self.session is None:
                self.session = create_session()
                self.started = datetime.now()
            if "event" in groups and not any(r.is_running() for r in self.groups["event"]):
                # Power-cycle the event cameras so they open reliably
                result = subprocess.run(["sudo", "-n", str(config.RESET_EVENT_CAMERAS_SCRIPT)],
                                        capture_output=True, text=True)
                if result.returncode != 0:
                    self.message = f"event camera reset failed: {result.stderr.strip() or result.returncode}"
            for group in groups:
                for recorder in self.groups[group]:
                    recorder.start(self.session)

    def stop(self, groups):
        """Stop some groups (None: all); each recorder flushes its data."""
        threads = [threading.Thread(target=r.stop) for name, group in self.groups.items()
                   if groups is None or name in groups for r in group]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

    def save(self):
        with self.lock:
            if self.any_running():
                raise RuntimeError("stop the recording first")
            if self.session is None:
                raise RuntimeError("no session")
            destination = save_session(self.session)
            self.session = None
            return destination

    def discard(self):
        with self.lock:
            if self.any_running():
                raise RuntimeError("stop the recording first")
            if self.session is not None:
                discard_session(self.session)
            self.session = None

    def status(self):
        outputs = {}
        for out, recorder in self.outputs.items():
            outputs[out] = {**recorder.status()[out], "running": recorder.is_running(), "label": OUTPUT_LABELS[out]}
        size_gb = None
        if self.session is not None and self.session.exists():
            size_gb = sum(f.stat().st_size for f in self.session.rglob("*") if f.is_file()) / 1e9
        return {"session": None if self.session is None else self.session.name,
                "started": None if self.started is None or self.session is None else self.started.strftime("%H:%M:%S"),
                "size_gb": None if size_gb is None else round(size_gb, 2),
                "free_gb": round(free_disk_gb(), 1),
                "groups": {g: {"label": GROUP_LABELS[g], "running": any(r.is_running() for r in rs),
                               "outputs": [o for r in rs for o in r.outputs]} for g, rs in self.groups.items()},
                "outputs": outputs, "message": self.message,
                "settings": settings_page.summary(self),
                "event_max_rate": _ev_s(store.load("event")["record_max_rate_mev"])}

    def preview(self, output):
        """Live preview of an output: the recorder's own small JPEG, or the newest lidar chunk."""
        if self.session is None:
            return None
        if not output.startswith("lidar"):
            try:
                return (self.session / "previews" / f"{output}.jpg").read_bytes()
            except OSError:
                return None
        chunks = list_chunks(self.session / output)
        if not chunks:
            return None
        path = chunks[-1][1]
        cached = self.lidar_cache.get(output)
        if cached is None or cached[0] != path:
            chunk = LidarChunk(path)
            xyz = chunk.xyz_camera_mm()
            image = (views.livox_view(xyz, chunk.intensity) if output == "lidar_livox"
                     else views.panorama(xyz, chunk.intensity))
            cached = (path, jpeg(image, 80))
            self.lidar_cache[output] = cached
        return cached[1]


def _ev_s(mev):
    return None if mev is None else mev * 1e6


def blueprint(sensors, recording):
    bp = Blueprint("record", __name__, url_prefix="/record")

    @bp.route("")
    def page():
        sensors.use(None)   # the recorders open the sensors themselves
        # Lists, not dicts: tojson sorts dict keys and the page shows them in this order
        return render_template("record.html", outputs=list(OUTPUT_LABELS.items()), groups=list(GROUP_LABELS.items()),
                               rgb_size=store.rgb_size(store.load("rgb")))

    @bp.route("/api/status")
    def status():
        return jsonify(recording.status())

    @bp.route("/api/preview/<output>")
    def preview(output):
        if output not in recording.outputs:
            return jsonify({"error": f"unknown sensor {output}"}), 404
        data = recording.preview(output)
        if data is None:
            return jsonify({"error": "no preview yet"}), 503
        response = send_file(io.BytesIO(data), mimetype="image/jpeg")
        response.headers["Cache-Control"] = "no-store"
        return response

    def action(function):
        try:
            result = function()
        except (RuntimeError, FileExistsError, ValueError) as e:
            return jsonify({"error": str(e)}), 409
        return jsonify({"status": "ok", **(result or {})})

    @bp.route("/api/start/<group>", methods=["POST"])
    def start(group):
        if group != "all" and group not in recording.groups:
            return jsonify({"error": f"unknown group {group}"}), 404
        return action(lambda: recording.start(list(recording.groups) if group == "all" else [group]))

    @bp.route("/api/stop/<group>", methods=["POST"])
    def stop(group):
        return action(lambda: recording.stop(None if group == "all" else [group]))

    @bp.route("/api/save", methods=["POST"])
    def save():
        return action(lambda: {"saved": str(recording.save().name)})

    @bp.route("/api/discard", methods=["POST"])
    def discard():
        return action(recording.discard)

    return bp

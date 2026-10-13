"""Pieces shared by the calibration tools: captured boards, background jobs, image helpers."""

import base64
import io
import os
import shutil
import threading
import time
import traceback
from datetime import datetime

import cv2
import numpy as np
from flask import Blueprint, jsonify, render_template, request, send_file, send_from_directory

from cvlab import config

PREVIEW_WIDTH = 640


def jpeg(image, quality=80):
    return cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])[1].tobytes()


def data_url(image):
    return "data:image/jpeg;base64," + base64.b64encode(jpeg(image)).decode()


def preview(image):
    """BGR image PREVIEW_WIDTH wide, aspect ratio kept."""
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    h, w = image.shape[:2]
    return cv2.resize(image, (PREVIEW_WIDTH, round(h * PREVIEW_WIDTH / w)), interpolation=cv2.INTER_AREA)


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


# ============================================================================
# SAMPLES
# ============================================================================

class Samples:
    """Captured boards in <processed_data>/<session>/<index>/: <cam>_corners.npy / .png and <cam>.png
    for every camera, plus whatever extra files the tool writes (see add)."""

    def __init__(self, folder, cameras, image_sizes, dedup_px, reference="rgb_left"):
        self.folder = folder   # created with the first board
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


# ============================================================================
# BACKGROUND JOBS
# ============================================================================

class UserError(Exception):
    """A calibration that cannot run for a reason the user can fix (shown without a traceback)."""


class Job:
    """A calibration running (or last run) in the background. One at a time."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = "idle"   # idle, running, done, failed
        self.folder = None
        self.progress = 0.0
        self.label = ""
        self.log = []
        self.result = None
        self.error = None

    def start(self, function, folder, **kwargs):
        """Run function(folder, log=..., progress=..., **kwargs) in a thread. False if one is running."""
        with self.lock:
            if self.state == "running":
                return False
            self.state, self.folder = "running", os.path.basename(folder)
            self.progress, self.label, self.log = 0.0, "Starting", []
            self.result = self.error = None
        threading.Thread(target=self.run, args=(function, folder, kwargs), daemon=True).start()
        return True

    def run(self, function, folder, kwargs):
        try:
            result = function(folder, log=self.add_log, progress=self.set_progress, **kwargs)
            with self.lock:
                self.result, self.state = result, "done"
        except UserError as e:
            self.fail(str(e))
        except Exception as e:
            traceback.print_exc()
            self.fail(f"{type(e).__name__}: {e}")

    def add_log(self, line):
        print(line)
        with self.lock:
            self.log.append(line)

    def set_progress(self, fraction, label):
        with self.lock:
            self.progress, self.label = fraction, label

    def fail(self, error):
        self.add_log(error)
        with self.lock:
            self.error, self.state = error, "failed"

    def status(self):
        with self.lock:
            return {"state": self.state, "folder": self.folder, "progress": self.progress, "label": self.label,
                    "log": list(self.log), "result": self.result, "error": self.error}


# ============================================================================
# CAPTURE PAGES
# ============================================================================

class CaptureError(Exception):
    """A capture that could not run (sensor missing...): shown on the page."""


def capture_blueprint(tool, name, url_prefix, processed_data_path, capture_function):
    """Routes shared by the capture tools; tool: an object like RgbEventCapture (samples, status(),
    latest(cam), open_session(), capture_lock), capture_function(body) -> result dict."""
    bp = Blueprint(name, __name__, url_prefix=url_prefix)
    cameras = tool.TOOL["cameras"]

    @bp.route("")
    def page():
        tool.sensors.use(tool.TOOL["id"])
        return render_template("capture.html", tool=dict(tool.TOOL, api=f"{url_prefix}/api"),
                               pattern_size=config.PATTERN_SIZE)

    @bp.route("/api/status")
    def status():
        return jsonify({"sensors": tool.status(),
                        "folder": os.path.relpath(tool.samples.folder, config.REPO_ROOT),
                        "session": os.path.basename(tool.samples.folder),
                        "save_failed": getattr(tool, "failed_folder", None) is not None,
                        "pattern_size": config.PATTERN_SIZE})

    @bp.route("/api/sessions")
    def list_sessions():
        return jsonify(sessions(processed_data_path))

    @bp.route("/api/session", methods=["POST"])
    def set_session():
        body = request.get_json(force=True)
        try:
            if "name" in body:
                tool.open_session(body["name"])
            if "save_failed" in body:
                tool.set_save_failed(bool(body["save_failed"]))
        except ValueError as e:
            return jsonify({"error": str(e)}), 400
        return jsonify({"session": os.path.basename(tool.samples.folder)})

    @bp.route("/api/preview/<cam>")
    def live_preview(cam):
        if cam not in cameras:
            return jsonify({"error": f"unknown camera {cam}"}), 404
        try:
            frame = tool.latest(cam)
        except RuntimeError as e:
            return jsonify({"error": str(e)}), 503
        if frame is None:
            return jsonify({"error": "no frame"}), 503
        host_us, image = frame
        response = send_file(io.BytesIO(jpeg(preview(image), 70)), mimetype="image/jpeg")
        response.headers["Cache-Control"] = "no-store"
        # Host time of the image (end of the event window): the page shows its age
        response.headers["X-Frame-Age-Ms"] = f"{(time.time_ns() // 1000 - host_us) / 1e3:.0f}"
        return response

    @bp.route("/api/samples")
    def list_samples():
        return jsonify(tool.samples.as_json())

    @bp.route("/api/samples/<name>", methods=["DELETE"])
    def delete_sample(name):
        if not tool.samples.delete(name):
            return jsonify({"error": f"no sample {name}"}), 404
        return jsonify({"status": "deleted"})

    @bp.route("/api/samples/<name>/<cam>.jpg")
    def sample_image(name, cam):
        if name not in tool.samples.corners or cam not in cameras:
            return jsonify({"error": "not found"}), 404
        image = cv2.imread(os.path.join(tool.samples.folder, name, f"{cam}_corners.png"))
        return send_file(io.BytesIO(jpeg(preview(image))), mimetype="image/jpeg")

    @bp.route("/api/capture", methods=["POST"])
    def capture():
        if not tool.capture_lock.acquire(blocking=False):
            return jsonify({"error": "a capture is already running"}), 409
        try:
            return jsonify(capture_function(request.get_json(silent=True) or {}))
        except CaptureError as e:
            return jsonify({"error": str(e)}), 503
        finally:
            tool.capture_lock.release()

    return bp


# ============================================================================
# SOLVE PAGES
# ============================================================================

def count_boards(folder, cameras):
    """Boards with the corners of every camera, as the solvers load them."""
    return sum(1 for name in os.listdir(folder)
               if all(os.path.exists(os.path.join(folder, name, f"{cam}_corners.npy")) for cam in cameras))


def solve_blueprint(tool, name, url_prefix, processed_data_path, results_path, calibrate, install_path):
    """Routes of a solve page; tool: {"id", "title", "cameras", "min_boards", "capture_url"[, "params":
    [{"name", "label", "value"}] numbers passed to calibrate as keyword arguments][, "count": folder ->
    number of usable boards / stops]},
    calibrate(folder, install, log, progress) -> result dict (see rgb_event/solve.py)."""
    bp = Blueprint(name, __name__, url_prefix=url_prefix)
    job = Job()

    @bp.route("")
    def page():
        tool["sensors"].use(None)
        page_tool = {k: v for k, v in tool.items() if k not in ("sensors", "count")}
        return render_template("solve.html", tool=page_tool | {"api": f"{url_prefix}/api"})

    @bp.route("/api/folders")
    def folders():
        entries = []
        for entry in sessions(processed_data_path):
            path = os.path.join(processed_data_path, entry["name"])
            entry["boards"] = tool["count"](path) if "count" in tool else count_boards(path, tool["cameras"])
            entries.append(entry)
        return jsonify({"folders": entries, "install_path": os.path.relpath(install_path, config.REPO_ROOT)})

    @bp.route("/api/calibrate", methods=["POST"])
    def start():
        body = request.get_json(force=True)
        folder = body.get("folder", "")
        if folder not in os.listdir(processed_data_path):
            return jsonify({"error": f"Unknown folder {folder!r}"}), 400
        # Extra solver parameters declared by the tool (e.g. the initial guess of livox_hesai)
        params = {}
        for param in tool.get("params", []):
            try:
                params[param["name"]] = float(body.get("params", {})[param["name"]])
            except (KeyError, TypeError, ValueError):
                return jsonify({"error": f"{param['label']}: a number is needed"}), 400
        if not job.start(calibrate, os.path.join(processed_data_path, folder), install=bool(body.get("install", True)),
                         **params):
            return jsonify({"error": "A calibration is already running"}), 409
        return jsonify({"started": folder})

    @bp.route("/api/job")
    def job_status():
        return jsonify(job.status())

    @bp.route("/api/results/<run>/<file>")
    def result_file(run, file):
        if run not in os.listdir(results_path):
            return jsonify({"error": "not found"}), 404
        return send_from_directory(os.path.join(results_path, run), file)

    return bp

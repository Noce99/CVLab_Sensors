"""Web GUI for calibrate.py: pick a processed_data/<session> folder, run the calibration, look at the results.

Usage: calibrate_server.py [--port 5002]

Same calibration as calibrate.py (it calls calibrate.calibrate), run in a
background thread so the page can show its progress.
"""

import argparse
import os
import sys
import threading
import traceback
from datetime import datetime

from flask import Flask, abort, jsonify, render_template, request, send_from_directory

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config
from calibrate import CAMERAS, PROCESSED_DATA_PATH, RESULTS_PATH, NotEnoughBoards, calibrate

app = Flask(__name__)


class Job:
    """The calibration running (or last run) in the background. One at a time."""

    def __init__(self):
        self.lock = threading.Lock()
        self.state = "idle"   # idle, running, done, failed
        self.folder = None
        self.progress = 0.0
        self.label = ""
        self.log = []
        self.result = None
        self.error = None

    def start(self, folder, install):
        with self.lock:
            if self.state == "running":
                return False
            self.state, self.folder = "running", folder
            self.progress, self.label, self.log = 0.0, "Starting", []
            self.result = self.error = None
        threading.Thread(target=self.run, args=(folder, install), daemon=True).start()
        return True

    def run(self, folder, install):
        try:
            result = calibrate(os.path.join(PROCESSED_DATA_PATH, folder), install=install,
                               log=self.add_log, progress=self.set_progress)
            with self.lock:
                self.result, self.state = result, "done"
        except NotEnoughBoards as e:
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


job = Job()


def count_boards(folder):
    """Boards seen by all four cameras, as calibrate.load_samples counts them."""
    n = 0
    for name in os.listdir(folder):
        sample = os.path.join(folder, name)
        if os.path.isdir(sample) and all(os.path.exists(os.path.join(sample, f"{cam}_corners.npy")) for cam in CAMERAS):
            n += 1
    return n


@app.route("/")
def index():
    return render_template("calibrate.html")


@app.route("/api/folders")
def folders():
    entries = []
    for name in sorted(os.listdir(PROCESSED_DATA_PATH), reverse=True):
        path = os.path.join(PROCESSED_DATA_PATH, name)
        if os.path.isdir(path):
            entries.append({"name": name, "boards": count_boards(path),
                            "modified": datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y-%m-%d %H:%M")})
    return jsonify({"folders": entries, "install_path": os.path.relpath(config.RGB_EVENT_CALIBRATION_FILE, config.REPO_ROOT)})


@app.route("/api/calibrate", methods=["POST"])
def start_calibration():
    body = request.get_json(force=True)
    folder = body.get("folder", "")
    if folder not in os.listdir(PROCESSED_DATA_PATH):
        return jsonify({"error": f"Unknown folder {folder!r}"}), 400
    if not job.start(folder, bool(body.get("install", True))):
        return jsonify({"error": "A calibration is already running"}), 409
    return jsonify({"started": folder})


@app.route("/api/job")
def job_status():
    return jsonify(job.status())


@app.route("/api/results/<run>/<name>")
def result_file(run, name):
    if run not in os.listdir(RESULTS_PATH):
        abort(404)
    return send_from_directory(os.path.join(RESULTS_PATH, run), name)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=5002)
    args = parser.parse_args()
    print(f"Calibration GUI on http://0.0.0.0:{args.port}")
    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()

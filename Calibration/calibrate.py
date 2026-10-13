"""Calibration tools of the rig, in one web page.

Usage: python Calibration/calibrate.py [--port 5001]
Open http://<NUC_IP>:5001 and pick a tool:
  - Get RGB-event calibration data   live capture of the chessboard in the 2 Basler + 2 EVK4
  - Calibrate RGB <-> event          intrinsics + extrinsics of the four cameras (rgb_event/solve.py)
  - Get RGB-Livox calibration data   live capture of the chessboard in RGB-left + Livox reflectivity
  - Calibrate RGB <-> Livox          extrinsics RGB-left <-> Livox (rgb_livox/solve.py)
  - Get Livox-Hesai calibration data  static stops of the rig, both lidars recording the same scene
  - Calibrate Livox <-> Hesai         target-free registration over all stops (livox_hesai/solve.py)

Capture pages open the sensors they need and release the others (one process can own a camera:
stop get_data.py / Cameras_Server first); the home page and the calibrate pages release them all.
"""

import argparse
import os
import signal
import sys
from datetime import datetime

from flask import Flask, render_template

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from Calibration import common
from Calibration.livox_hesai import capture as livox_hesai_capture
from Calibration.livox_hesai import solve as livox_hesai_solve
from Calibration.rgb_event import capture as rgb_event_capture
from Calibration.rgb_event import solve as rgb_event_solve
from Calibration.rgb_livox import capture as rgb_livox_capture
from Calibration.rgb_livox import solve as rgb_livox_solve
from Calibration.sensors import SensorManager
from cvlab import config

app = Flask(__name__)
sensors = SensorManager()

SOLVERS = {
    "rgb_event": dict(title="Calibrate RGB ↔ event", cameras=rgb_event_solve.CAMERAS, min_boards=10,
                      capture_url="/rgb-event/capture", module=rgb_event_solve,
                      install_path=config.RGB_EVENT_CALIBRATION_FILE, url="/rgb-event/calibrate"),
    "rgb_livox": dict(title="Calibrate RGB ↔ Livox", cameras=rgb_livox_solve.CAMERAS,
                      min_boards=rgb_livox_solve.MIN_BOARDS, capture_url="/rgb-livox/capture",
                      module=rgb_livox_solve, install_path=config.RGB_LIVOX_CALIBRATION_FILE,
                      url="/rgb-livox/calibrate"),
    "livox_hesai": dict(title="Calibrate Livox ↔ Hesai", cameras=livox_hesai_solve.CAMERAS,
                        min_boards=livox_hesai_solve.MIN_STOPS, capture_url="/livox-hesai/capture",
                        module=livox_hesai_solve, install_path=config.LIVOX_HESAI_CALIBRATION_FILE,
                        url="/livox-hesai/calibrate", count=lambda folder: len(livox_hesai_solve.list_stops(folder)),
                        count_unit="stops",
                        params_title="Hesai at (mm, Livox frame)",
                        params_note="Initial guess: Hesai origin in the Livox frame, x right, y down, z forward. "
                                    "A ruler measurement is enough (the registration converges from +-30 cm / +-10 deg).",
                        params=[{"name": f"initial_{axis}_mm", "label": axis, "value": value}
                                for axis, value in zip("xyz", config.LIVOX_HESAI_INITIAL_T_MM)]),
}

for solver in SOLVERS.values():
    os.makedirs(solver["module"].PROCESSED_DATA_PATH, exist_ok=True)
    os.makedirs(solver["module"].RESULTS_PATH, exist_ok=True)

app.register_blueprint(rgb_event_capture.blueprint(sensors))
app.register_blueprint(rgb_livox_capture.blueprint(sensors))
app.register_blueprint(livox_hesai_capture.blueprint(sensors))
for tool_id, solver in SOLVERS.items():
    tool = {"id": tool_id, "title": solver["title"], "cameras": solver["cameras"], "min_boards": solver["min_boards"],
            "capture_url": solver["capture_url"], "sensors": sensors,
            **{k: solver[k] for k in ("params", "params_title", "params_note", "count", "count_unit") if k in solver}}
    module = solver["module"]
    app.register_blueprint(common.solve_blueprint(tool, f"{tool_id}_solve", solver["url"], module.PROCESSED_DATA_PATH,
                                                  module.RESULTS_PATH, module.calibrate, solver["install_path"]))


def tool_summary(tool_id):
    """Latest session and installed calibration of a tool, for the home page cards."""
    solver = SOLVERS[tool_id]
    module = solver["module"]
    sessions = common.sessions(module.PROCESSED_DATA_PATH)
    latest = None
    if sessions:
        path = os.path.join(module.PROCESSED_DATA_PATH, sessions[0]["name"])
        boards = solver["count"](path) if "count" in solver else common.count_boards(path, solver["cameras"])
        latest = dict(sessions[0], boards=boards)
    installed = None
    if os.path.exists(solver["install_path"]):
        installed = datetime.fromtimestamp(os.path.getmtime(solver["install_path"])).strftime("%Y-%m-%d %H:%M")
    return {"latest": latest, "sessions": len(sessions), "installed": installed,
            "install_path": os.path.relpath(solver["install_path"], config.REPO_ROOT)}


@app.route("/")
def home():
    sensors.use(None)
    return render_template("home.html", summaries={tool_id: tool_summary(tool_id) for tool_id in SOLVERS})


@app.route("/pattern")
def blinking_pattern():
    """Blinking chessboard to show on a screen: event cameras only see what changes."""
    cols, rows = config.PATTERN_SIZE
    return render_template("pattern.html", cols=cols, rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=5001)
    args = parser.parse_args()
    print(f"Calibration tools on http://0.0.0.0:{args.port}")
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))
    try:
        app.run(host="0.0.0.0", port=args.port, threaded=True, use_reloader=False)
    finally:
        sensors.close()


if __name__ == "__main__":
    main()

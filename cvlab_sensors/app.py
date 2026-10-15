"""The rig's web app: record and replay sessions, calibrate the sensors.

Usage: python -m cvlab_sensors [--port 5001]   (from the repo root)
Open http://<NUC_IP>:5001 and pick a tool:
  - Record                            the recorders of every sensor, save / discard the session
  - Replay                            recorded sessions on a timeline, sensors alone or on RGB left
  - Settings                          camera and Livox settings applied when a sensor opens, Hesai checked
  - Get RGB-event calibration data    live capture of the chessboard in the 2 Basler + 2 EVK4
  - Calibrate RGB <-> event           intrinsics + extrinsics of the four cameras
  - Get RGB-Livox calibration data    live capture of the chessboard in RGB-left + Livox reflectivity
  - Calibrate RGB <-> Livox           extrinsics RGB-left <-> Livox
  - Get Livox-Hesai calibration data  static stops of the rig, both lidars recording the same scene
  - Calibrate Livox <-> Hesai         target-free registration over all stops

Capture pages open the sensors they need and release the others (a sensor can be opened by one
process only: no capture while the Record page records); the other pages release them all.
"""

import argparse
import os
import signal
import socket
import sys
from datetime import datetime

from flask import Flask, render_template

from cvlab_sensors import config, web
from cvlab_sensors.calibration import common
from cvlab_sensors.calibration.livox_hesai import capture as livox_hesai_capture
from cvlab_sensors.calibration.livox_hesai import solve as livox_hesai_solve
from cvlab_sensors.calibration.rgb_event import capture as rgb_event_capture
from cvlab_sensors.calibration.rgb_event import solve as rgb_event_solve
from cvlab_sensors.calibration.rgb_livox import capture as rgb_livox_capture
from cvlab_sensors.calibration.rgb_livox import solve as rgb_livox_solve
from cvlab_sensors.calibration.rig import installed_rig
from cvlab_sensors.recording import page as record
from cvlab_sensors.replay import page as replay
from cvlab_sensors.settings import page as settings_page
from cvlab_sensors.sensors.hesai import HesaiMonitor
from cvlab_sensors.sensors.live import SensorManager

SOLVERS = {
    "rgb_event": dict(title="Calibrate RGB ↔ event", cameras=rgb_event_solve.CAMERAS, min_boards=10,
                      capture_url="/rgb-event/capture", module=rgb_event_solve, square_size=True,
                      install_path=config.RGB_EVENT_CALIBRATION_FILE, url="/rgb-event/calibrate"),
    "rgb_livox": dict(title="Calibrate RGB ↔ Livox", cameras=rgb_livox_solve.CAMERAS,
                      min_boards=rgb_livox_solve.MIN_BOARDS, capture_url="/rgb-livox/capture",
                      module=rgb_livox_solve, install_path=config.RGB_LIVOX_CALIBRATION_FILE, square_size=True,
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


def tool_summary(tool_id):
    """Latest session and installed calibration of a tool, for the home page cards."""
    solver = SOLVERS[tool_id]
    module = solver["module"]
    sessions = common.sessions(module.CAPTURES_PATH)
    latest = None
    if sessions:
        path = os.path.join(module.CAPTURES_PATH, sessions[0]["name"])
        boards = solver["count"](path) if "count" in solver else common.count_boards(path, solver["cameras"])
        latest = dict(sessions[0], boards=boards)
    installed = None
    if os.path.exists(solver["install_path"]):
        installed = datetime.fromtimestamp(os.path.getmtime(solver["install_path"])).strftime("%Y-%m-%d %H:%M")
    return {"latest": latest, "sessions": len(sessions), "installed": installed,
            "install_path": os.path.relpath(solver["install_path"], config.REPO_ROOT)}


def create_app():
    """(app, sensors, recording). Built here and not at import: the sensor and event-cache
    processes are spawned, and a spawned process imports this module again."""
    app = Flask(__name__)
    sensors = SensorManager()
    recording = record.Recording(HesaiMonitor())
    # A sensor can be opened by one program: no calibration capture while a recording runs
    sensors.blocker = lambda: "a recording is running: stop it on the Record page first" if recording.any_running() else None

    for solver in SOLVERS.values():
        os.makedirs(solver["module"].CAPTURES_PATH, exist_ok=True)
        os.makedirs(solver["module"].RESULTS_PATH, exist_ok=True)

    app.register_blueprint(rgb_event_capture.blueprint(sensors))
    app.register_blueprint(rgb_livox_capture.blueprint(sensors))
    app.register_blueprint(livox_hesai_capture.blueprint(sensors))
    app.register_blueprint(record.blueprint(sensors, recording))
    app.register_blueprint(replay.blueprint(sensors, replay.Replay(), recording))
    app.register_blueprint(settings_page.blueprint(sensors, recording))
    for tool_id, solver in SOLVERS.items():
        tool = {"id": tool_id, "title": solver["title"], "cameras": solver["cameras"], "min_boards": solver["min_boards"],
                "capture_url": solver["capture_url"], "sensors": sensors,
                **{k: solver[k] for k in ("params", "params_title", "params_note", "count", "count_unit", "square_size") if k in solver}}
        module = solver["module"]
        app.register_blueprint(web.solve_blueprint(tool, f"{tool_id}_solve", solver["url"], module.CAPTURES_PATH,
                                                      module.RESULTS_PATH, module.calibrate, solver["install_path"]))

    @app.route("/")
    def home():
        sensors.use(None)
        return render_template("home.html", summaries={tool_id: tool_summary(tool_id) for tool_id in SOLVERS},
                               rig=installed_rig(), recording=recording.status(),
                               settings=settings_page.summary(recording),
                               settings_warnings=settings_page.warnings(recording),
                               n_sessions=len(replay.find_sessions()))

    @app.route("/pattern")
    def blinking_pattern():
        """Blinking chessboard to show on a screen: event cameras only see what changes."""
        cols, rows = config.PATTERN_SIZE
        return render_template("pattern.html", cols=cols, rows=rows)

    return app, sensors, recording


def detach_from_terminal(log_path):
    """Write the output of the server (and of the processes it starts) to log_path, not to the
    terminal, and survive the terminal closing.

    The terminal is an SSH session over the Wi-Fi, which drops during runs: printing to a stalled
    session blocks the printing thread (a request, the disk guard, a recorder stop), printing to a
    closed one fails, and its hangup signal would kill the server and leave the recorders running
    without the disk guard. Ctrl+C still stops the server.
    """
    log_path.parent.mkdir(parents=True, exist_ok=True)
    if log_path.exists() and log_path.stat().st_size > config.APP_LOG_MAX_BYTES:
        log_path.replace(log_path.with_name(log_path.name + ".1"))
    sys.stdout.flush()
    sys.stderr.flush()
    fd = os.open(log_path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o644)
    os.dup2(fd, 1)
    os.dup2(fd, 2)
    os.close(fd)
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
    signal.signal(signal.SIGHUP, signal.SIG_IGN)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=5001)
    args = parser.parse_args()
    # Checked here, while errors still reach the terminal: a server started earlier survives its terminal
    with socket.socket() as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)   # as the server: TIME_WAIT is not "in use"
        try:
            s.bind(("0.0.0.0", args.port))
        except OSError as e:
            sys.exit(f"Port {args.port} not available ({e.strerror}): is the server already running? "
                     f"(pgrep -af cvlab_sensors)")
    print(f"Rig tools on http://0.0.0.0:{args.port}\n"
          f"Output in {config.APP_LOG} (tail -f it). The server survives the terminal closing; Ctrl+C stops it.")
    detach_from_terminal(config.APP_LOG)
    print(f"\n=== {datetime.now():%Y-%m-%d %H:%M:%S} server started on port {args.port} (pid {os.getpid()})")
    app, sensors, recording = create_app()
    signal.signal(signal.SIGTERM, lambda signum, frame: sys.exit(0))
    try:
        app.run(host="0.0.0.0", port=args.port, threaded=True, use_reloader=False)
    finally:
        recording.stop(None)   # the recorders flush their data
        sensors.close()


if __name__ == "__main__":
    main()

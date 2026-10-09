"""
Camera Monitoring Server
=========================

Flask dashboard to start/stop the sensor recorders and monitor them:
- RGB stereo cameras (Basler)          get_data/rgb.py
- Event stereo cameras (Prophesee)     get_data/event.py
- LiDAR Livox / Hesai                  get_data/cpp/save_point_cloud_{livox,hesai}

The recorders are the same ones used by get_data/get_data.py. Each Start
opens a recording session (if none is open) in get_data/data/<session>;
"Save" moves it to CVLab_live_imgs/, "Discard" deletes it.
"""

import base64
import os
import sys
import threading

import cv2
from flask import Flask, jsonify, render_template
from flask_cors import CORS

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import lidar
from cvlab import config
from cvlab.recording import (build_recorders, check_recorders_exist, create_session,
                             discard_session, free_disk_gb, save_session)

app = Flask(__name__)
CORS(app)

# --allow-unsynced: record RGB even without PTP (until ptp4l is set up)
recorder_groups = build_recorders(allow_unsynced="--allow-unsynced" in sys.argv)
recorders = [r for group in recorder_groups.values() for r in group]
outputs = {out: r for r in recorders for out in r.outputs}

session_lock = threading.Lock()
current_session = None

# Lidar previews are rendered from the newest H5 file, cached by file path
lidar_preview_cache = {}


disk_stopped = threading.Event()


def disk_guard():
    """Stop every recorder when the staging disk is almost full."""
    while True:
        if any_running() and free_disk_gb() < config.MIN_FREE_DISK_GB:
            print(f"WARNING: less than {config.MIN_FREE_DISK_GB} GB free, stopping all recorders")
            disk_stopped.set()
            stop_all()
        threading.Event().wait(1.0)


def any_running():
    return any(r.is_running() for r in recorders)


def busy_response(action):
    return jsonify({"status": "error", "error": f"Stop all sensors before you {action} the session"}), 409


# ============================================================================
# STATUS / FRAMES
# ============================================================================

@app.route("/")
def index():
    return render_template("dashboard.html")


@app.route("/api/status")
def get_status():
    status = {}
    for r in recorders:
        status.update(r.status())
    for out, recorder in outputs.items():
        status[out]["running"] = recorder.is_running()
    return jsonify({
        "sensors": status,
        "session": current_session.name if current_session else None,
        "recording": any_running(),
        "disk_free_gb": round(free_disk_gb(), 1),
        "disk_stopped": disk_stopped.is_set(),
    })


def latest_lidar_preview(folder):
    chunks = lidar.list_chunks(folder)
    if not chunks:
        return None
    path = chunks[-1][1]
    if path not in lidar_preview_cache:
        lidar_preview_cache.clear()
        lidar_preview_cache[path] = lidar.render_chunk(path, dot_radius=2)
    return lidar_preview_cache[path]


@app.route("/api/frame/<camera_name>")
def get_frame(camera_name):
    if camera_name not in outputs:
        return jsonify({"error": "Camera not found"}), 404
    if current_session is None:
        return jsonify({"error": "No session"}), 404

    if camera_name.startswith("lidar"):
        frame = latest_lidar_preview(current_session / camera_name)
        if frame is None:
            return jsonify({"error": "No frame available"}), 404
        _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        data = buffer.tobytes()
    else:
        try:
            data = (current_session / "previews" / f"{camera_name}.jpg").read_bytes()
        except OSError:
            return jsonify({"error": "No frame available"}), 404

    return jsonify({
        "frame": f"data:image/jpeg;base64,{base64.b64encode(data).decode('utf-8')}",
        "timestamp": outputs[camera_name].status()[camera_name]["last_update"],
    })


# ============================================================================
# CONTROL
# ============================================================================

def start_group(group):
    global current_session
    if free_disk_gb() < config.MIN_FREE_DISK_GB:
        raise RuntimeError(f"Only {free_disk_gb():.1f} GB free: free some space first")
    disk_stopped.clear()
    with session_lock:
        if current_session is None:
            current_session = create_session()
            print(f"INFO: new session {current_session}")
        return [r.start(current_session) for r in recorder_groups[group]]


@app.route("/api/start/<camera_type>", methods=["POST"])
def start_camera(camera_type):
    if camera_type not in recorder_groups:
        return jsonify({"error": f"Camera type not found: {camera_type}"}), 404
    try:
        started = start_group(camera_type)
        return jsonify({"status": "started" if any(started) else "already_running", "camera": camera_type})
    except Exception as e:
        return jsonify({"status": "error", "camera": camera_type, "error": str(e)}), 500


@app.route("/api/stop/<camera_type>", methods=["POST"])
def stop_camera(camera_type):
    if camera_type not in recorder_groups:
        return jsonify({"error": f"Camera type not found: {camera_type}"}), 404
    for r in recorder_groups[camera_type]:
        r.stop()
    return jsonify({"status": "stopped", "camera": camera_type})


@app.route("/api/start_all", methods=["POST"])
def start_all():
    results = {}
    for group in recorder_groups:
        try:
            results[group] = "started" if any(start_group(group)) else "already_running"
        except Exception as e:
            results[group] = f"error: {e}"
    return jsonify({"status": "completed", "results": results})


def stop_all():
    # Stop in parallel: each recorder may need a few seconds to flush
    threads = [threading.Thread(target=r.stop) for r in recorders]
    for t in threads:
        t.start()
    for t in threads:
        t.join()


@app.route("/api/stop_all", methods=["POST"])
def stop_all_route():
    stop_all()
    return jsonify({"status": "completed"})


@app.route("/api/clear_all", methods=["POST"])
def clear_all():
    """Discard the current session (all its data is deleted)."""
    global current_session
    with session_lock:
        if any_running():
            return busy_response("discard")
        if current_session is None:
            return jsonify({"status": "completed", "message": "No session to discard"})
        discard_session(current_session)
        print(f"INFO: discarded session {current_session}")
        current_session = None
    return jsonify({"status": "completed"})


@app.route("/api/move_all", methods=["POST"])
def move_all():
    """Save the current session to CVLab_live_imgs/."""
    global current_session
    with session_lock:
        if any_running():
            return busy_response("save")
        if current_session is None:
            return jsonify({"status": "completed", "message": "No session to save"})
        destination = save_session(current_session)
        print(f"INFO: session saved to {destination}")
        current_session = None
    return jsonify({"status": "completed", "destination": str(destination)})


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("CAMERA MONITORING SERVER")
    print("=" * 60)

    missing = check_recorders_exist()
    if missing:
        sys.exit("ERROR: missing recorders:\n  " + "\n  ".join(missing))

    print("\nServer starting on http://0.0.0.0:5000")
    print("Connect from mobile: http://<COMPUTER_IP>:5000")
    print("Press Ctrl+C to stop")
    print("=" * 60 + "\n")

    threading.Thread(target=disk_guard, daemon=True).start()
    try:
        # use_reloader=False prevents double recorder spawning in debug mode
        app.run(host="0.0.0.0", port=5000, debug=True, threaded=True, use_reloader=False)
    finally:
        print("Stopping all recorders...")
        stop_all()

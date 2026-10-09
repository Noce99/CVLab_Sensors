"""Recording sessions and recorder subprocesses, shared by get_data.py and the camera server."""

import os
import shutil
import signal
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

from cvlab import config
from cvlab.status import RateCounter, read_json

# A sensor is considered stalled when its newest output is older than this
STALL_TIMEOUT_S = 5.0


# ============================================================================
# SESSIONS
# ============================================================================

def free_disk_gb(path=None):
    path = Path(path or config.STAGING_DIR)
    while not path.exists():
        path = path.parent
    return shutil.disk_usage(path).free / 1e9


def create_session():
    name = datetime.now().strftime(config.SESSION_NAME_FORMAT)
    path = config.STAGING_DIR / name
    for sub in config.SESSION_SUBFOLDERS:
        (path / sub).mkdir(parents=True)
    return path


def save_session(path):
    """Move a finished session to SESSIONS_DIR and return the new path.

    On the same filesystem this is an atomic rename, otherwise a copy + delete.
    """
    config.SESSIONS_DIR.mkdir(parents=True, exist_ok=True)
    destination = config.SESSIONS_DIR / Path(path).name
    if destination.exists():
        raise FileExistsError(destination)
    shutil.move(str(path), str(destination))
    return destination


def discard_session(path):
    path = Path(path).resolve()
    if path.parent != config.STAGING_DIR.resolve():
        raise ValueError(f"Refusing to delete {path}: not a session in {config.STAGING_DIR}")
    shutil.rmtree(path)


# ============================================================================
# RECORDER PROCESSES
# ============================================================================

class Recorder:
    """One recorder subprocess producing one or more sensor outputs.

    Health is computed by a watcher thread:
      - Python recorders write <session>/status/<output>.json once per second
      - lidar recorders are watched through the files they produce
    """

    STOP_TIMEOUT_S = 15

    def __init__(self, name, outputs, build_command, writes_status_json):
        self.name = name
        self.outputs = outputs
        self._build_command = build_command
        self._writes_status_json = writes_status_json
        self.process = None
        self.session = None
        self._log = None
        self._lock = threading.Lock()
        self._status = {out: self._empty_status() for out in outputs}
        self._watcher = None

    @staticmethod
    def _empty_status():
        return {"active": False, "fps": 0, "last_update": None, "error": None}

    # -- control --------------------------------------------------------------

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, session):
        if self.is_running():
            return False
        self.session = Path(session)
        log_path = self.session / "logs" / f"{self.name}.log"
        self._log = open(log_path, "ab")
        command = [str(c) for c in self._build_command(self.session)]
        self._log.write(f"$ {' '.join(command)}\n".encode())
        self._log.flush()
        # Own process group: a Ctrl+C in the terminal does not reach the recorder
        # directly, we stop it ourselves with SIGINT and wait for it to flush.
        self.process = subprocess.Popen(command, stdout=self._log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        with self._lock:
            self._status = {out: self._empty_status() for out in self.outputs}
        self._watcher = threading.Thread(target=self._watch, daemon=True)
        self._watcher.start()
        return True

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=self.STOP_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                print(f"WARNING: {self.name} did not stop in {self.STOP_TIMEOUT_S}s, killing it")
                self.process.kill()
                self.process.wait()
        if self._watcher is not None:
            self._watcher.join(timeout=2)
        if self._log is not None:
            self._log.close()
            self._log = None

    # -- status ---------------------------------------------------------------

    def status(self):
        with self._lock:
            return {out: dict(s) for out, s in self._status.items()}

    def _watch(self):
        counters = {out: RateCounter() for out in self.outputs}
        while True:
            exit_code = self.process.poll()
            with self._lock:
                for out in self.outputs:
                    if self._writes_status_json:
                        self._status[out] = self._status_from_json(out)
                    else:
                        self._status[out] = self._status_from_files(out, counters[out])
                    if exit_code is not None:
                        self._status[out]["active"] = False
                        if exit_code != 0:
                            self._status[out]["error"] = f"recorder exited with code {exit_code} (see logs/{self.name}.log)"
            if exit_code is not None:
                return
            time.sleep(config.STATUS_PERIOD_S)

    def _status_from_json(self, output):
        data = read_json(self.session / "status" / f"{output}.json")
        if data is None:
            return {**self._empty_status(), "error": "starting..."}
        age = time.time() - data.get("written_at", 0)
        status = {
            "active": age < STALL_TIMEOUT_S and data.get("error") is None,
            "fps": round(data.get("fps", 0), 1),
            "last_update": data.get("last_update"),
            "error": data.get("error"),
        }
        if age >= STALL_TIMEOUT_S:
            status["error"] = f"no status update for {age:.0f}s"
        return status

    def _status_from_files(self, output, counter):
        folder = self.session / output
        newest_mtime, count = 0.0, 0
        with os.scandir(folder) as it:
            for entry in it:
                if entry.name.endswith(".h5"):
                    count += 1
                    newest_mtime = max(newest_mtime, entry.stat().st_mtime)
        counter.add(count - counter.total)
        files_per_s = counter.rate()
        if count == 0:
            return {**self._empty_status(), "error": "waiting for data..."}
        age = time.time() - newest_mtime
        return {
            "active": age < STALL_TIMEOUT_S,
            "fps": round(files_per_s, 1),
            "last_update": datetime.fromtimestamp(newest_mtime).strftime("%H:%M:%S"),
            "error": None if age < STALL_TIMEOUT_S else f"no new file for {age:.0f}s",
        }


def python_recorder(script, *args):
    return lambda session: [config.PYTHON, "-u", script, session, *args]


def build_recorders(allow_unsynced=False):
    """All recorders, grouped the way the dashboard controls them.

    allow_unsynced: record RGB even if the cameras are not PTP synchronized.
    """
    rgb_args = ["--allow-unsynced"] if allow_unsynced else []
    return {
        "rgb": [
            Recorder("rgb", ["rgb_left", "rgb_right"],
                     python_recorder(config.RGB_RECORDER, *rgb_args), writes_status_json=True),
        ],
        "event": [
            Recorder("event_left", ["event_left"],
                     python_recorder(config.EVENT_RECORDER), writes_status_json=True),
            Recorder("event_right", ["event_right"],
                     python_recorder(config.EVENT_RECORDER, "--right-camera"), writes_status_json=True),
        ],
        "lidar_livox": [
            Recorder("lidar_livox", ["lidar_livox"],
                     lambda s: [config.LIVOX_RECORDER, s / "lidar_livox", config.LIVOX_CONFIG],
                     writes_status_json=False),
        ],
        "lidar_hesai": [
            Recorder("lidar_hesai", ["lidar_hesai"],
                     lambda s: [config.HESAI_RECORDER, s / "lidar_hesai", config.HESAI_IP],
                     writes_status_json=False),
        ],
    }


def check_recorders_exist():
    """List of missing recorder files (empty if everything is in place)."""
    required = [config.PYTHON, config.RGB_RECORDER, config.EVENT_RECORDER,
                config.LIVOX_RECORDER, config.HESAI_RECORDER, config.LIVOX_CONFIG]
    return [str(p) for p in required if not Path(p).exists()]

"""Hesai XT32 settings, read over its PTC protocol (TCP 9347). Read only: the Hesai keeps its settings
in its own memory, they are changed on its web page (store.HESAI_WEB_PAGE).

Config info (PTC command 0x08) layout, checked on 2026-10-15 against the web API (dest port 2369,
GPS port 10110, clock source, UDP sequence): ip, mask, gateway, dest ip (4 B each), dest port,
GPS port, spin rate rpm (2 B each), sync (1), sync angle, start angle, stop angle (2 B, 0.1 deg),
clock source, UDP sequence, trigger method, return mode, standby, motor status (1 B each)...
"""

import socket
import struct
import threading
import time
from datetime import datetime

from cvlab_sensors import config

PTC_PORT = 9347
PTC_GET_CONFIG = 0x08
TIMEOUT_S = 2.0
POLL_PERIOD_S = 30.0
RETURN_MODES = {0: "last", 1: "strongest", 2: "dual"}
CLOCK_SOURCES = {0: "GPS / free running", 1: "PTP"}


def _recv(sock, n):
    data = b""
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            raise ConnectionError("Hesai closed the PTC connection")
        data += chunk
    return data


def read_config(ip=config.HESAI_IP):
    """{"spin_rpm", "return_mode", "fov_start_deg", "fov_stop_deg", "clock_source", "standby"}, in the
    units of the "hesai" settings."""
    with socket.create_connection((ip, PTC_PORT), timeout=TIMEOUT_S) as sock:
        sock.sendall(struct.pack(">BBBBI", 0x47, 0x74, PTC_GET_CONFIG, 0, 0))
        _, _, _, code, length = struct.unpack(">BBBBI", _recv(sock, 8))
        body = _recv(sock, length)
    if code != 0 or length < 35:
        raise RuntimeError(f"Hesai PTC config query failed (code {code}, {length} bytes)")
    spin, = struct.unpack(">H", body[20:22])
    start, stop = struct.unpack(">HH", body[25:29])
    return {"spin_rpm": str(spin), "return_mode": RETURN_MODES.get(body[32], f"code {body[32]}"),
            "fov_start_deg": start / 10, "fov_stop_deg": stop / 10,
            "clock_source": CLOCK_SOURCES.get(body[29], f"code {body[29]}"), "standby": bool(body[33])}


def problems(readback):
    """Settings that break the recordings whatever the expected values (see tools/check_ptp.py)."""
    found = []
    if readback.get("clock_source") == "PTP":
        found.append("clock source is PTP: its clock jumps (it cannot follow our two-step master). "
                     "Set it back to GPS on its web page")
    if readback.get("standby"):
        found.append("in standby: it sends no points")
    return found


class HesaiMonitor:
    """Reads the Hesai settings in the background, so pages never wait on the network."""

    def __init__(self):
        self.lock = threading.Lock()
        self.result = {"readback": None, "error": "not read yet", "read_at": None}
        self.wake = threading.Event()
        threading.Thread(target=self.run, daemon=True).start()

    def run(self):
        while True:
            self.read()
            self.wake.wait(POLL_PERIOD_S)
            self.wake.clear()

    def read(self):
        try:
            result = {"readback": read_config(), "error": None}
        except (OSError, RuntimeError) as e:
            result = {"readback": None, "error": f"cannot read the Hesai settings: {e}"}
        result["read_at"] = datetime.now().isoformat(timespec="seconds")
        with self.lock:
            self.result = result
        return result

    def latest(self):
        with self.lock:
            return dict(self.result)

    def refresh(self, max_age_s=0):
        """Read now (in this thread) unless the last read is younger than max_age_s."""
        latest = self.latest()
        if latest["read_at"] and max_age_s and \
                time.time() - datetime.fromisoformat(latest["read_at"]).timestamp() < max_age_s:
            return latest
        return self.read()

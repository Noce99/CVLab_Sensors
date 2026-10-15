"""Hesai XT32 settings, read over its PTC protocol (TCP 9347). Read only: the Hesai keeps its settings
in its own memory, they are changed on its web page (store.HESAI_WEB_PAGE).

Config info (PTC command 0x08) layout, checked on 2026-10-15 against the web API (dest port 2369,
GPS port 10110, clock source, UDP sequence): ip, mask, gateway, dest ip (4 B each), dest port,
GPS port, spin rate rpm (2 B each), sync (1), sync angle, start angle, stop angle (2 B, 0.1 deg),
clock source, UDP sequence, trigger method, return mode, standby, motor status (1 B each)...
Its start / stop angles are NOT the FoV set on the web page: that one is PTC command 0x23
(mode 0: one range, start and end in 0.1 deg; mode 3: one range per channel group), checked on
2026-10-15 by setting 130-230 deg on the web page.
"""

import socket
import struct
import threading
import time
from datetime import datetime

from cvlab_sensors import config

PTC_PORT = 9347
PTC_GET_CONFIG = 0x08
PTC_GET_FOV = 0x23
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


def _query(sock, command):
    """Payload of the answer to a PTC command without payload."""
    sock.sendall(struct.pack(">BBBBI", 0x47, 0x74, command, 0, 0))
    _, _, _, code, length = struct.unpack(">BBBBI", _recv(sock, 8))
    body = _recv(sock, length)
    if code != 0:
        raise RuntimeError(f"Hesai PTC command 0x{command:02x} failed (code {code})")
    return body


def read_config(ip=config.HESAI_IP):
    """{"spin_rpm", "return_mode", "fov", "clock_source", "standby"}, in the units of the "hesai"
    fields of cvlab_sensors.settings.store."""
    with socket.create_connection((ip, PTC_PORT), timeout=TIMEOUT_S) as sock:
        body = _query(sock, PTC_GET_CONFIG)
        fov = _query(sock, PTC_GET_FOV)
    if len(body) < 35:
        raise RuntimeError(f"Hesai PTC config answer too short ({len(body)} bytes)")
    spin, = struct.unpack(">H", body[20:22])
    if len(fov) >= 5 and fov[0] == 0:
        start, end = struct.unpack(">HH", fov[1:5])
        fov_text = f"{start / 10:g}° – {end / 10:g}°"
    elif fov and fov[0] == 3:
        fov_text = f"per channel group ({fov[1] if len(fov) > 1 else '?'} ranges): see its web page"
    else:
        fov_text = f"unknown (mode {fov[0] if fov else '?'})"
    return {"spin_rpm": str(spin), "return_mode": RETURN_MODES.get(body[32], f"code {body[32]}"), "fov": fov_text,
            "clock_source": CLOCK_SOURCES.get(body[29], f"code {body[29]}"), "standby": bool(body[33])}


def problems(readback):
    """Settings that break the recordings (see tools/check_ptp.py)."""
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

"""Show whether every Ethernet sensor follows the ptp4l grandmaster.

Usage: check_ptp.py [--skip-livox]

- Basler: PTP port state, servo state and offset (read from the cameras)
- Hesai:  clock source (read from its web API). It must NOT be on PTP: it cannot
          follow our two-step master; its clock is mapped through the H5 sync pairs
- Livox:  runs the recorder for up to 25 s and reads the time source of the packets
ptp4l itself: journalctl -u ptp4l-cvlab -f
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.request

from pypylon import pylon

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config


def check_basler():
    tl = pylon.TlFactory.GetInstance().CreateTl("BaslerGigE")
    for name, ip in [("RGB left", config.RGB_LEFT_IP), ("RGB right", config.RGB_RIGHT_IP)]:
        try:
            info = tl.CreateDeviceInfo()
            info.SetIpAddress(ip)
            camera = pylon.InstantCamera(tl.CreateDevice(info))
            camera.Open()
            camera.PtpDataSetLatch.Execute()
            state, servo = camera.PtpStatus.Value, camera.PtpServoStatus.Value
            offset = camera.PtpOffsetFromMaster.Value
            camera.Close()
            ok = state == "Slave" and servo == "Locked"
            print(f"{'OK ' if ok else 'NO '} {name:10s} {state} [{servo}] offset {offset / 1000:.1f} us")
        except Exception as e:
            print(f"ERR {name:10s} {e}")


def check_hesai():
    url = f"http://{config.HESAI_IP}/pandar.cgi?action=get&object=lidar_config"
    try:
        body = json.load(urllib.request.urlopen(url, timeout=5))["Body"]
    except Exception as e:
        print(f"ERR Hesai      {e}")
        return
    on_ptp = body.get("ClockSource") == "1"
    print(f"{'NO ' if on_ptp else 'OK '} {'Hesai':10s} clock source {'PTP' if on_ptp else 'GPS/free running'}"
          + (" -> set it back to GPS on its web page: its clock jumps when on PTP" if on_ptp else
             " (mapped to host time through the H5 sync pairs)"))


def check_livox(timeout_s=25):
    with tempfile.TemporaryDirectory(dir="/dev/shm") as tmp:
        process = subprocess.Popen([str(config.LIVOX_RECORDER), tmp, str(config.LIVOX_CONFIG)],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        source = None
        deadline = time.monotonic() + timeout_s
        try:
            for line in process.stdout:
                if line.startswith("Livox time source:"):
                    source = line.split(":", 1)[1].strip()
                    break
                if time.monotonic() > deadline:
                    break
        finally:
            process.terminate()
            process.wait()
    ok = source == "livox_ptp"
    print(f"{'OK ' if ok else 'NO '} {'Livox':10s} {source or 'no data within %ds' % timeout_s}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--skip-livox", action="store_true", help="Do not start the Livox (takes ~15 s)")
    args = parser.parse_args()

    check_basler()
    check_hesai()
    if not args.skip_livox:
        check_livox()


if __name__ == "__main__":
    main()

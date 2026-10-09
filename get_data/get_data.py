"""Record every sensor into a new session from the terminal.

Usage: get_data.py [--no-event-reset] [--allow-unsynced] [--only rgb event lidar_livox lidar_hesai]

Ctrl+C stops all recorders (each one flushes its data), then the session is
moved to CVLab_live_imgs/ if every sensor produced data.
"""

import argparse
import datetime
import os
import subprocess
import sys
import time

from tabulate import tabulate

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config
from cvlab.recording import (build_recorders, check_recorders_exist, create_session, free_disk_gb,
                             save_session)


def print_recap_table(recorders):
    rows = []
    for recorder in recorders:
        for output, status in recorder.status().items():
            rows.append([output, "running" if recorder.is_running() else "stopped",
                         "yes" if status["active"] else "no", status["fps"],
                         status["last_update"], status["error"] or ""])
    table = tabulate(rows, headers=["Sensor", "Process", "Active", "Rate", "Last update", "Error"],
                     tablefmt="grid")
    lines = table.split("\n")
    print(datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S"))
    print(table)
    # Move the cursor back up so the next table overwrites this one
    print("\033[F" * (len(lines) + 1), end="", flush=True)


def main():
    parser = argparse.ArgumentParser(description="Record all sensors")
    parser.add_argument("--no-event-reset", action="store_true",
                        help="Do not power-cycle the event cameras before starting")
    parser.add_argument("--allow-unsynced", action="store_true",
                        help="Record RGB even if the cameras are not PTP synchronized")
    parser.add_argument("--only", nargs="+", choices=list(build_recorders()),
                        help="Record only these sensor groups")
    args = parser.parse_args()

    missing = check_recorders_exist()
    if missing:
        sys.exit("Missing recorders:\n  " + "\n  ".join(missing))

    groups = build_recorders(allow_unsynced=args.allow_unsynced)
    if args.only:
        groups = {k: v for k, v in groups.items() if k in args.only}
    recorders = [r for group in groups.values() for r in group]

    if "event" in groups and not args.no_event_reset:
        # Be sure event cameras are ready
        result = subprocess.run(["sudo", "-n", str(config.RESET_EVENT_CAMERAS_SCRIPT)], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"WARNING: event camera reset failed: {result.stderr.strip()}")

    if free_disk_gb() < config.MIN_FREE_DISK_GB:
        sys.exit(f"Only {free_disk_gb():.1f} GB free on {config.STAGING_DIR}: free some space first")

    session = create_session()
    print(f"Recording session {session}")

    try:
        for recorder in recorders:
            recorder.start(session)
        while True:
            print_recap_table(recorders)
            free = free_disk_gb(session)
            if free < config.MIN_FREE_DISK_GB:
                print("\n" * 20 + f"Only {free:.1f} GB free: stopping the recording")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        # Whatever happens, never leave recorders running in the background
        print("\n" * 20 + "Stopping recorders (waiting for them to flush)...")
        for recorder in recorders:
            recorder.stop()

    final = {out: s for r in recorders for out, s in r.status().items()}
    empty = [out for out in final if not any(os.scandir(session / out))]
    exit_codes = {r.name: r.process.returncode for r in recorders}
    print(f"Exit codes: {exit_codes}")

    if empty:
        print(f"Not all sensors produced data (empty: {', '.join(empty)}). "
              f"Session left in {session}")
    else:
        print(f"Session saved to {save_session(session)}")


if __name__ == "__main__":
    main()

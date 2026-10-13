"""Find the chessboard in time-aligned RGB and event frames of a recording session.

Record the chessboard with:   python get_data/get_data.py --only rgb event
Then:                          python Calibration/rgb_event/process_session.py [session_folder]

For every RGB-left frame (one every --step), the closest RGB-right frame and the
event frames ending at the same host time are rendered; when all four show the
chessboard their corners are saved to processed_data/<session>/<index>/.
"""

import argparse
import os
import shutil
import sys
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
from tqdm import tqdm

sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from cvlab import config, events
from cvlab.calibration import find_corners_variants, find_event_corners, to_calibration_size

PROCESSED_DATA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "processed_data")


def latest_session():
    sessions = sorted(p for p in os.listdir(config.SESSIONS_DIR) if p[:4].isdigit())
    if not sessions:
        sys.exit(f"No session in {config.SESSIONS_DIR}")
    return os.path.join(config.SESSIONS_DIR, sessions[-1])


def image_timestamps(folder):
    return np.array(sorted(int(f[:-4]) for f in os.listdir(folder) if f.endswith(".png")), dtype=np.int64)


def closest(timestamps, t):
    i = np.searchsorted(timestamps, t)
    candidates = [j for j in (i - 1, i) if 0 <= j < len(timestamps)]
    j = min(candidates, key=lambda j: abs(timestamps[j] - t))
    return int(timestamps[j]), abs(int(timestamps[j]) - t)


def find_rgb_corners(gray):
    """(corners, image they were found in) like find_event_corners."""
    return find_corners_variants(gray, config.CHESSBOARD_VARIANTS["rgb"])[0], gray


def find_event_frames_corners(frames):
    corners, _, image = find_event_corners(frames)
    return corners, image


def load_rgb(path):
    return to_calibration_size(cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2GRAY))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", nargs="?", help="Session folder (default: latest in CVLab_live_imgs)")
    parser.add_argument("--step", type=int, default=1, help="Use one RGB-left frame every STEP")
    parser.add_argument("--max-rgb-dt-ms", type=float, default=20, help="Max time between RGB left and right")
    args = parser.parse_args()

    session = os.path.abspath(args.session or latest_session())
    name = os.path.basename(session.rstrip("/"))
    print(f"Processing session {session}")

    rgb_left_ts = image_timestamps(os.path.join(session, "rgb_left"))[::args.step]
    rgb_right_ts = image_timestamps(os.path.join(session, "rgb_right"))

    pairs = {}
    for t in rgb_left_ts:
        t_right, dt = closest(rgb_right_ts, int(t))
        if dt <= args.max_rgb_dt_ms * 1000:
            pairs[int(t)] = (t_right, dt)
    print(f"{len(pairs)} RGB pairs (of {len(rgb_left_ts)} left frames)")

    out_folder = os.path.join(PROCESSED_DATA_PATH, name)
    if os.path.exists(out_folder):
        shutil.rmtree(out_folder)
    os.makedirs(out_folder)

    counters = {"rgb_left": 0, "rgb_right": 0, "event_left": 0, "event_right": 0, "all": 0, "inactive_events": 0}
    pool = ThreadPoolExecutor(max_workers=4)

    frames = events.stereo_frames_at(session, list(pairs), window_us=events.CALIBRATION_FRAMES_WINDOW_US,
                                     render=events.calibration_frames)
    for t, event_l, event_r in tqdm(frames, total=len(pairs), desc="Searching chessboards"):
        if (events.activation(event_l["count"]) < events.MIN_ACTIVATION
                or events.activation(event_r["count"]) < events.MIN_ACTIVATION):
            counters["inactive_events"] += 1
            continue
        t_right, dt_right = pairs[t]
        rgb_l = load_rgb(os.path.join(session, "rgb_left", f"{t}.png"))
        rgb_r = load_rgb(os.path.join(session, "rgb_right", f"{t_right}.png"))

        images = {"rgb_left": rgb_l, "rgb_right": rgb_r, "event_left": event_l, "event_right": event_r}
        finders = {"rgb_left": find_rgb_corners, "rgb_right": find_rgb_corners,
                   "event_left": find_event_frames_corners, "event_right": find_event_frames_corners}
        futures = {k: pool.submit(finders[k], img) for k, img in images.items()}
        found = {k: f.result() for k, f in futures.items()}
        corners = {k: c for k, (c, _) in found.items()}
        images = {k: image for k, (_, image) in found.items()}
        for k, c in corners.items():
            counters[k] += c is not None
        if any(c is None for c in corners.values()):
            continue

        counters["all"] += 1
        sample = os.path.join(out_folder, f"{counters['all'] - 1:05d}")
        os.makedirs(sample)
        for k, c in corners.items():
            np.save(os.path.join(sample, f"{k}_corners.npy"), c)
            drawn = cv2.drawChessboardCorners(cv2.cvtColor(images[k], cv2.COLOR_GRAY2BGR),
                                              config.PATTERN_SIZE, c.reshape(-1, 1, 2), True)
            cv2.imwrite(os.path.join(sample, f"{k}_corners.png"), drawn)
        with open(os.path.join(sample, "log.log"), "w") as f:
            f.write(f"Reference timestamp (rgb_left) = {t} us\n")
            f.write(f"RGB right dt = {dt_right / 1e3:.2f} ms\n")
            f.write("Event frames end at the reference timestamp\n")

    print("Chessboard search:")
    for k, v in counters.items():
        print(f"\t{k:16s} {v}")
    print(f"Saved to {out_folder}")


if __name__ == "__main__":
    main()

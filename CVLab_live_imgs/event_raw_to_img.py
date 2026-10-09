"""Render the event recordings of a folder to PNG frames.

Usage: event_raw_to_img.py <session>/event_left <output_folder> [--fps 10] [--window-ms 33]
Frames are named <host_time_us>.png.
"""

import argparse
import os
import sys

import cv2

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import events


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder")
    parser.add_argument("output")
    parser.add_argument("--fps", type=float, default=10)
    parser.add_argument("--window-ms", type=float, default=33)
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    for rec in events.find_recordings(args.folder):
        period = int(1e6 / args.fps)
        times = range(rec.first_host_us + period, rec.last_host_us, period)
        for t, frame in rec.frames_at(times, int(args.window_ms * 1000), render=events.colored_frame):
            cv2.imwrite(os.path.join(args.output, f"{t}.png"), frame)


if __name__ == "__main__":
    main()

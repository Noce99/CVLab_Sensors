"""Step through the rectified RGB or event pairs of a session (any key: next, q: quit).

Usage: rectification.py {rgb,event} [session_folder]
"""

import argparse
import bisect
import os
import sys

import cv2

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config, events
from cvlab.calibration import (load_calibration, rectification_check_image, rectify_pair,
                               stereo_rectify_maps, to_calibration_size)
from all_images_rect import find_session, image_timestamps


def show(left, right, maps):
    left_rect, right_rect = rectify_pair(left, right, maps[0], maps[1])
    cv2.imshow("Comparison", cv2.resize(rectification_check_image(left_rect, right_rect), None, fx=0.6, fy=0.6))
    return cv2.waitKey(0) & 0xFF != ord("q")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("what", choices=["rgb", "event"])
    parser.add_argument("session", nargs="?")
    args = parser.parse_args()

    session = args.session or find_session()
    calib = load_calibration(config.RGB_EVENT_CALIBRATION_FILE)
    p = args.what
    maps = stereo_rectify_maps(calib[f"{p}_K_left"], calib[f"{p}_D_left"], calib[f"{p}_K_right"],
                               calib[f"{p}_D_right"], calib[f"{p}_R"], calib[f"{p}_T"],
                               config.CALIBRATION_IMAGE_SIZE)

    left_ts = image_timestamps(os.path.join(session, "rgb_left"))
    if args.what == "rgb":
        right_ts = image_timestamps(os.path.join(session, "rgb_right"))
        for t in left_ts:
            i = min(bisect.bisect_left(right_ts, t), len(right_ts) - 1)
            left = to_calibration_size(cv2.imread(os.path.join(session, "rgb_left", f"{t}.png")))
            right = to_calibration_size(cv2.imread(os.path.join(session, "rgb_right", f"{right_ts[i]}.png")))
            if not show(left, right, maps):
                break
    else:
        for _, left, right in events.stereo_frames_at(session, left_ts, render=events.colored_frame):
            if not show(left, right, maps):
                break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

"""Render every sensor of a session side by side, time-aligned on RGB-left, and make a video.

Usage: all_images_rect.py [session_folder] [--step N] [--no-video]

Top row:    RGB left | RGB right (rectified)    | Livox
Bottom row: Event left | Event right (rectified) | Hesai
Output in <session>/rectified/ (<i>.png and video.mp4).
"""

import argparse
import bisect
import os
import subprocess
import sys

import cv2
import numpy as np
from tqdm import tqdm

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config, events, lidar
from cvlab.calibration import load_calibration, rectify_pair, stereo_rectify_maps, to_calibration_size

EVENT_WINDOW_US = 33_000
LIDAR_HALF_WINDOW_US = 50_000
TILE_SIZE = (640, 360)


def find_session():
    sessions = sorted(p for p in os.listdir(config.SESSIONS_DIR) if p[:4].isdigit())
    if not sessions:
        sys.exit(f"No session in {config.SESSIONS_DIR}")
    return os.path.join(config.SESSIONS_DIR, sessions[-1])


def image_timestamps(folder):
    return sorted(int(f[:-4]) for f in os.listdir(folder) if f.endswith(".png"))


def closest(timestamps, t):
    i = bisect.bisect_left(timestamps, t)
    candidates = [j for j in (i - 1, i) if 0 <= j < len(timestamps)]
    return timestamps[min(candidates, key=lambda j: abs(timestamps[j] - t))]


def tile(img, label):
    if img is None:
        img = np.full((TILE_SIZE[1], TILE_SIZE[0], 3), 60, dtype=np.uint8)
    img = cv2.resize(img, TILE_SIZE, interpolation=cv2.INTER_AREA)
    cv2.putText(img, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    return img


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", nargs="?", help="Session folder (default: latest)")
    parser.add_argument("--step", type=int, default=1, help="Use one RGB-left frame every STEP")
    parser.add_argument("--no-video", action="store_true")
    args = parser.parse_args()

    session = os.path.abspath(args.session or find_session())
    print(f"Session: {session}")

    calib = load_calibration(config.RGB_EVENT_CALIBRATION_FILE)
    size = config.CALIBRATION_IMAGE_SIZE
    rgb_maps = stereo_rectify_maps(calib["rgb_K_left"], calib["rgb_D_left"], calib["rgb_K_right"],
                                   calib["rgb_D_right"], calib["rgb_R"], calib["rgb_T"], size)
    event_maps = stereo_rectify_maps(calib["event_K_left"], calib["event_D_left"], calib["event_K_right"],
                                     calib["event_D_right"], calib["event_R"], calib["event_T"], size)

    rgb_left = image_timestamps(os.path.join(session, "rgb_left"))[::args.step]
    rgb_right = image_timestamps(os.path.join(session, "rgb_right"))
    livox = lidar.LidarFolder(os.path.join(session, "lidar_livox"))
    hesai = lidar.LidarFolder(os.path.join(session, "lidar_hesai"))

    out_dir = os.path.join(session, "rectified")
    os.makedirs(out_dir, exist_ok=True)

    # Event frames are generated in one pass over the raw files, only for the RGB times
    frames = events.stereo_frames_at(session, rgb_left, EVENT_WINDOW_US, render=events.colored_frame)
    index = 0
    for t, event_l, event_r in tqdm(frames, total=len(rgb_left)):
        t_right = closest(rgb_right, t)
        img_l = to_calibration_size(cv2.imread(os.path.join(session, "rgb_left", f"{t}.png")))
        img_r = to_calibration_size(cv2.imread(os.path.join(session, "rgb_right", f"{t_right}.png")))
        rgb_l_rect, rgb_r_rect = rectify_pair(img_l, img_r, rgb_maps[0], rgb_maps[1])
        event_l_rect, event_r_rect = rectify_pair(event_l, event_r, event_maps[0], event_maps[1])

        # No lidar extrinsics yet: each lidar is shown from its own point of view
        livox_img = lidar.depth_image(livox.points_around(t, LIDAR_HALF_WINDOW_US), dot_radius=2)
        hesai_img = lidar.depth_image(hesai.points_around(t, LIDAR_HALF_WINDOW_US), dot_radius=2)

        dt_ms = (t_right - t) / 1e3
        grid = np.vstack([
            np.hstack([tile(rgb_l_rect, "RGB left"), tile(rgb_r_rect, f"RGB right ({dt_ms:+.1f} ms)"),
                       tile(livox_img, "Livox")]),
            np.hstack([tile(event_l_rect, "Event left"), tile(event_r_rect, "Event right"),
                       tile(hesai_img, "Hesai")]),
        ])
        # Contiguous numbering, so ffmpeg does not stop at a gap
        cv2.imwrite(os.path.join(out_dir, f"{index}.png"), grid)
        index += 1

    print(f"{index} frames written to {out_dir}")
    if index and not args.no_video:
        subprocess.run([
            "ffmpeg", "-framerate", str(config.RGB_FPS // args.step or 1),
            "-i", os.path.join(out_dir, "%d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-y",
            os.path.join(out_dir, "video.mp4"),
        ], check=True, capture_output=True)
        print(f"Video: {os.path.join(out_dir, 'video.mp4')}")


if __name__ == "__main__":
    main()

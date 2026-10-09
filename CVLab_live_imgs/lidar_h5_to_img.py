"""Render every lidar H5 file of a folder to a depth image.

Usage: lidar_h5_to_img.py <session>/lidar_livox <output_folder>
"""

import argparse
import os
import sys

import cv2

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import lidar


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("folder")
    parser.add_argument("output")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)
    for t, path in lidar.list_chunks(args.folder):
        cv2.imwrite(os.path.join(args.output, f"{t}.png"), lidar.render_chunk(path, dot_radius=2))


if __name__ == "__main__":
    main()

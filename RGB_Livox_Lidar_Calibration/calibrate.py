import os
from datetime import datetime
import shutil
from tqdm import tqdm
import cv2
import concurrent.futures
from functools import partial
import numpy as np
from typing import List, Tuple

import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config
from cvlab.calibration import pattern_points as make_pattern_points


PATTERN_SIZE = config.PATTERN_SIZE
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)


def rectify_stereo_pair(left_img, right_img, left_map1, left_map2, right_map1, right_map2):
    left_rectified = cv2.remap(left_img, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_img, right_map1, right_map2, cv2.INTER_LINEAR)
    return left_rectified, right_rectified

def test_rectification(left_img, right_img, left_map1, left_map2, right_map1, right_map2):

    # Rectify the images
    left_rect, right_rect = rectify_stereo_pair(left_img, right_img, left_map1, left_map2, right_map1, right_map2)
    
    # Create a side-by-side visualization
    vis = np.hstack((left_rect, right_rect))
    
    # Draw horizontal lines to verify rectification
    h, w = left_rect.shape[:2]
    for y in range(0, h, 50):
        cv2.line(vis, (0, y), (w*2, y), (0, 255, 0), 1)
    
    return left_rect, right_rect, vis

class CornerSelector:
    px_threshold : int
    accepted_corners : int
    corners_describers : List[np.ndarray]
    corners_lidar : List[np.ndarray]
    corners_rgb : List[np.ndarray]

    def __init__(self, px_threshold=5):
        self.px_threshold = px_threshold
        self.accepted_corners = 0
        self.corners_describers = []
        self.corners_lidar   = []
        self.corners_rgb   = []

    def add_corners(self,
        c_lidar: np.ndarray,
        c_rgb: np.ndarray,
    ):
        current_corner_describer = np.array([c_rgb[0, 0], c_rgb[0, 1], c_rgb[-1, 0], c_rgb[-1, 1]])

        if self.accepted_corners == 0:
            self.accepted_corners += 1
            self.corners_describers.append(current_corner_describer)
            self.corners_lidar.append(c_lidar)
            self.corners_rgb.append(c_rgb)
        else:
            diffs = []
            for cd in self.corners_describers:
                diffs.append(np.max(np.abs(cd - current_corner_describer)))
            min_diffs = min(diffs)
            if min_diffs > self.px_threshold:
                self.accepted_corners += 1
                self.corners_describers.append(current_corner_describer)
                self.corners_lidar.append(c_lidar)
                self.corners_rgb.append(c_rgb)
            # print(f"Min Diff = {min_diffs}  [{self.accepted_corners}]")


def main():
    processed_data_path = "./processed_data"
    if not os.path.exists(processed_data_path):
        raise Exception(f"Processed data folder not found at {processed_data_path}")

    folders = os.listdir(processed_data_path)
    folders.sort()
    folder = folders[-1]
    data_folder = os.path.join(processed_data_path, folder)
    print(f"Processing folder {data_folder}")


    calibration_results_path = "./calibration_results"
    if not os.path.exists(calibration_results_path):
        os.mkdir(calibration_results_path)
    current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    current_folder_calib_results = os.path.join("./calibration_results", current_time)
    os.mkdir(current_folder_calib_results)

    chessboards_calib_results = os.path.join(current_folder_calib_results, "chessboards")
    os.mkdir(chessboards_calib_results)

    cs = CornerSelector(px_threshold=5)

    for i, index_folder in tqdm(enumerate(os.listdir(data_folder)), total=len(os.listdir(data_folder))):
        if not os.path.exists(os.path.join(data_folder, index_folder, "lidar_corners.npy")):
            continue  # "discarded" and anything else that is not a sample
        lidar_corners_path   = os.path.join(data_folder, index_folder, "lidar_corners.npy")
        rgb_corners_path   = os.path.join(data_folder, index_folder, "rgb_corners.npy")

        corners_lidar_fine   = np.load(lidar_corners_path)
        corners_rgb_fine   = np.load(rgb_corners_path)

        cs.add_corners(
            corners_lidar_fine,
            corners_rgb_fine,
        )
            

    print(f"I have detected the following chessboards:")
    print(f"\t{len(cs.corners_lidar)} Lidar")
    print(f"\t{len(cs.corners_rgb)} RGB")
    print(f"Let's Calibrate!")

    index_folder = next(f for f in sorted(os.listdir(data_folder))
                        if os.path.exists(os.path.join(data_folder, f, "lidar_corners.png")))
    lidar_path   = os.path.join(data_folder, index_folder, "lidar_corners.png")
    rgb_path   = os.path.join(data_folder, index_folder, "rgb_corners.png")

    lidar = cv2.imread(lidar_path, cv2.IMREAD_GRAYSCALE)
    rgb   = cv2.imread(rgb_path,   cv2.IMREAD_GRAYSCALE)

    lidar = cv2.cvtColor(lidar, cv2.COLOR_GRAY2BGR)
    rgb   = cv2.cvtColor(rgb,   cv2.COLOR_GRAY2BGR)

    rgb_image_size = (rgb.shape[1], rgb.shape[0])
    lidar_image_size = (lidar.shape[1], lidar.shape[0])

    pattern_points = make_pattern_points()  # mm
    pattern_points_repeated = [pattern_points] * len(cs.corners_rgb)

    # RGB and Lidar Stereo Calibrate

    err, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(
        pattern_points_repeated, cs.corners_rgb, cs.corners_lidar, None, None, None, None, rgb_image_size, 
        flags=0
    )

    T = T
    R = R
    rgb_K= Kl
    rgb_D= Dl
    lidar_K = Kr
    lidar_D = Dr

    print(f"RGB Lidar Reprojection Error = {err:.4f} px (<0.5 Excellent, 0.5<1.0 Good, >1.0 Problematic)")
    print(f"RGB-Lidar T = {T.flatten()} mm")

    # Let's Rectify RGB
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        rgb_K, rgb_D, lidar_K, lidar_D, rgb_image_size, R, T, alpha=-1
    )
    rgb_map1, rgb_map2 = cv2.initUndistortRectifyMap(
        rgb_K, rgb_D, R1, P1, rgb_image_size, cv2.CV_16SC2
    )
    lidar_map1, lidar_map2 = cv2.initUndistortRectifyMap(
        lidar_K, lidar_D, R2, P2, rgb_image_size, cv2.CV_16SC2
    )

    rgb_rect, lidar_rect, comparison = test_rectification(rgb, lidar, rgb_map1, rgb_map2, lidar_map1, lidar_map2)
    cv2.imwrite(os.path.join(current_folder_calib_results, "rgb_lidar_rectification_test.png"), comparison)

    np.savez(
        os.path.join(current_folder_calib_results, "rgb_lidar_calibration.npz"),
        rgb_K=rgb_K, rgb_D=rgb_D, lidar_K=lidar_K, lidar_D=lidar_D,
        R=R, T=T, rms=err, image_size=np.array(rgb_image_size),
        square_size_mm=config.SQUARE_SIZE_MM, n_boards=len(cs.corners_rgb),
    )
    print(f"Saved to {current_folder_calib_results}")


if __name__ == "__main__":
    main()
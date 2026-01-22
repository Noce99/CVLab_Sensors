import os
from datetime import datetime
import shutil
from tqdm import tqdm
import cv2
import concurrent.futures
from functools import partial
import numpy as np
from typing import List, Tuple

from scipy.spatial.transform import Rotation

from process_calibration_data import adjust_gamma
from warp import undistort, compute_homography, warp_image


PATTERN_SIZE = (9, 6)
SIDE_SIZE_CM = 3.0
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
    corners_rgb_l : List[np.ndarray]
    corners_rgb_r : List[np.ndarray]
    corners_event_l : List[np.ndarray]
    corners_event_r : List[np.ndarray]

    def __init__(self, px_threshold=5):
        self.px_threshold = px_threshold
        self.accepted_corners = 0
        self.corners_describers = []
        self.corners_rgb_l   = []
        self.corners_rgb_r   = []
        self.corners_event_l = []
        self.corners_event_r = []

    def add_corners(self,
        c_rgb_l: np.ndarray,
        c_rgb_r: np.ndarray,
        c_event_l: np.ndarray,
        c_event_r: np.ndarray
    ):
        current_corner_describer = np.array([c_rgb_l[0, 0], c_rgb_l[0, 1], c_rgb_l[-1, 0], c_rgb_l[-1, 1]])

        if self.accepted_corners == 0:
            self.accepted_corners += 1
            self.corners_describers.append(current_corner_describer)
            self.corners_rgb_l.append(c_rgb_l)
            self.corners_rgb_r.append(c_rgb_r)
            self.corners_event_l.append(c_event_l)
            self.corners_event_r.append(c_event_r)
        else:
            diffs = []
            for cd in self.corners_describers:
                diffs.append(np.max(np.abs(cd - current_corner_describer)))
            min_diffs = min(diffs)
            if min_diffs > self.px_threshold:
                self.accepted_corners += 1
                self.corners_describers.append(current_corner_describer)
                self.corners_rgb_l.append(c_rgb_l)
                self.corners_rgb_r.append(c_rgb_r)
                self.corners_event_l.append(c_event_l)
                self.corners_event_r.append(c_event_r)
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

    cs = CornerSelector(px_threshold=10)

    for i, index_folder in tqdm(enumerate(os.listdir(data_folder)), total=len(os.listdir(data_folder))):
        rgb_l_corners_path   = os.path.join(data_folder, index_folder, "left_rgb_corners.npy")
        rgb_r_corners_path   = os.path.join(data_folder, index_folder, "right_rgb_corners.npy")
        event_l_corners_path = os.path.join(data_folder, index_folder, "left_event_corners.npy")
        event_r_corners_path = os.path.join(data_folder, index_folder, "right_event_corners.npy")

        corners_rgb_l_fine   = np.load(rgb_l_corners_path)
        corners_rgb_r_fine   = np.load(rgb_r_corners_path)
        corners_event_l_fine = np.load(event_l_corners_path)[::-1]
        corners_event_r_fine = np.load(event_r_corners_path)[::-1]

        cs.add_corners(
            corners_rgb_l_fine,
            corners_rgb_r_fine,
            corners_event_l_fine,
            corners_event_r_fine,
        )
            

    print(f"I have detected the following chessboards:")
    print(f"\t{len(cs.corners_rgb_l)} Left RGB")
    print(f"\t{len(cs.corners_rgb_r)} Right RGB")
    print(f"\t{len(cs.corners_event_l)} Left Event")
    print(f"\t{len(cs.corners_event_r)} Right Event")
    print(f"Let's Calibrate!")

    index_folder = os.listdir(data_folder)[0]
    rgb_l_path   = os.path.join(data_folder, index_folder, "rgb_left_corners.png")
    rgb_r_path   = os.path.join(data_folder, index_folder, "rgb_right_corners.png")
    event_l_path = os.path.join(data_folder, index_folder, "event_left_corners.png")
    event_r_path = os.path.join(data_folder, index_folder, "event_right_corners.png")

    rgb_l = cv2.imread(  rgb_l_path,   cv2.IMREAD_GRAYSCALE)
    rgb_r = cv2.imread(  rgb_r_path,   cv2.IMREAD_GRAYSCALE)
    event_l = cv2.imread(event_l_path, cv2.IMREAD_GRAYSCALE)
    event_r = cv2.imread(event_r_path, cv2.IMREAD_GRAYSCALE)

    rgb_l = cv2.cvtColor(rgb_l, cv2.COLOR_GRAY2BGR)
    rgb_r = cv2.cvtColor(rgb_r, cv2.COLOR_GRAY2BGR)
    event_l = cv2.cvtColor(event_l, cv2.COLOR_GRAY2BGR)
    event_r = cv2.cvtColor(event_r, cv2.COLOR_GRAY2BGR)


    rgb_image_size = (rgb_l.shape[1], rgb_l.shape[0])
    event_image_size = (event_l.shape[1], event_l.shape[0])

    pattern_points = np.zeros((np.prod(PATTERN_SIZE), 3), np.float32)
    pattern_points[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
    pattern_points *= SIDE_SIZE_CM
    pattern_points_repeated = [pattern_points] * len(cs.corners_rgb_l)

    # Left Rgb anf Right Rgb Stereo Calibrate

    err, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(
        pattern_points_repeated, cs.corners_rgb_l, cs.corners_rgb_r, None, None, None, None, rgb_image_size, flags=0)
    
    rgb_T = T
    rgb_R = R
    rgb_K_left = Kl
    rgb_D_left = Dl
    rgb_K_right = Kr
    rgb_D_right = Dr

    print(f"RGB Reprojection Error = {err:.4f} px (<0.5 Excellent, 0.5<1.0 Good, >1.0 Problematic)")
    print(f"RGB T = {T.flatten()}")

    # Left Event and Right Event Stereo Calibrate

    err, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(
        pattern_points_repeated, cs.corners_event_l, cs.corners_event_r, None, None, None, None, event_image_size, flags=0)

    event_T = T
    event_R = R
    event_K_left = Kl
    event_D_left = Dl
    event_K_right = Kr
    event_D_right = Dr

    print(f"Event Reprojection Error = {err:.4f} px (<0.5 Excellent, 0.5<1.0 Good, >1.0 Problematic)")
    print(f"Event T = {T.flatten()}")

    # Left Rgb and Left Event Stereo Calibrate

    rgb_event_K_left = rgb_K_left.copy()
    rgb_event_D_left = rgb_D_left.copy()
    rgb_event_K_right = event_K_right.copy()
    rgb_event_D_right = event_D_right.copy()

    err, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        pattern_points_repeated, cs.corners_rgb_l, cs.corners_event_l, rgb_event_K_left, rgb_event_D_left, rgb_event_K_right, rgb_event_D_right, rgb_image_size, flags=0)

    rgb_event_T = T
    rgb_event_R = R

    print(f"RGB-Event Reprojection Error = {err:.4f} px (<0.5 Excellent, 0.5<1.0 Good, >1.0 Problematic)")
    print(f"RGB-Event T = {rgb_event_T.flatten()}")


    # Let's Rectify RGB
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        rgb_K_left, rgb_D_left, rgb_K_right, rgb_D_right, rgb_image_size, rgb_R, rgb_T, alpha=-1
    )
    rgb_l_map1, rgb_l_map2 = cv2.initUndistortRectifyMap(
        rgb_K_left, rgb_D_left, R1, P1, rgb_image_size, cv2.CV_16SC2
    )
    rgb_r_map1, rgb_r_map2 = cv2.initUndistortRectifyMap(
        rgb_K_right, rgb_D_right, R2, P2, rgb_image_size, cv2.CV_16SC2
    )

    rgb_l_rect, rgb_r_rect, comparison = test_rectification(rgb_l, rgb_r, rgb_l_map1, rgb_l_map2, rgb_r_map1, rgb_r_map2)
    cv2.imwrite(os.path.join(current_folder_calib_results, "rgb_rectification_test.png"), comparison)

    # Let's Rectify Event
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        event_K_left, event_D_left, event_K_right, event_D_right, event_image_size, event_R, event_T, alpha=-1
    )
    event_l_map1, event_l_map2 = cv2.initUndistortRectifyMap(
        event_K_left, event_D_left, R1, P1, event_image_size, cv2.CV_16SC2
    )
    event_r_map1, event_r_map2 = cv2.initUndistortRectifyMap(
        event_K_right, event_D_right, R2, P2, event_image_size, cv2.CV_16SC2
    )

    event_l_rect, event_r_rect, comparison = test_rectification(event_l, event_r, event_l_map1, event_l_map2, event_r_map1, event_r_map2)
    cv2.imwrite(os.path.join(current_folder_calib_results, "event_rectification_test.png"), comparison)

    # Let's Rectify RGB-Event
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        rgb_K_left, rgb_D_left, event_K_left, event_D_left, rgb_image_size, rgb_event_R, rgb_event_T, alpha=-1, 
    )
    rgb_event_l_map1, rgb_event_l_map2 = cv2.initUndistortRectifyMap(
        rgb_K_left, rgb_D_left, R1, P1, rgb_image_size, cv2.CV_16SC2
    )
    rgb_event_r_map1, rgb_event_r_map2 = cv2.initUndistortRectifyMap(
        event_K_left, event_D_left, R2, P2, rgb_image_size, cv2.CV_16SC2
    )

    rgb_event_l_rect, rgb_event_r_rect, comparison = test_rectification(rgb_l, event_l, rgb_event_l_map1, rgb_event_l_map2, rgb_event_r_map1, rgb_event_r_map2)
    cv2.imwrite(os.path.join(current_folder_calib_results, "rgb_event_rectification_test.png"), comparison)

    # TODO: Rectification between everything and RGB Left

    print(f"RGB T = {rgb_T}")
    print(f"Event T = {event_T}")

    T_rgb_to_pattern = []
    T_event_to_pattern = []

    for corner_rgb_l, corner_event_l in tqdm(zip(cs.corners_rgb_l, cs.corners_event_l), total=len(cs.corners_rgb_l)):
        
        # There we compute the rotation and translation from the pattern to the camera
        # Where patterns points are in the pattern coordinate system (3D points in real world space)
        # and corners are the detected 2D points in the image
        rgb_success, rgb_rvec, rgb_tvec = cv2.solvePnP(
            pattern_points, # shape (N, 3)
            corner_rgb_l,   # shape (N, 2)
            rgb_K_left,     # shape (3, 3) Camera Matrix
            rgb_D_left,     # shape (1, 5) or (1, 8) Distortion Coefficients
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        event_success, event_rvec, event_tvec = cv2.solvePnP(
            pattern_points,
            corner_event_l,
            event_K_left,
            event_D_left,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if rgb_success and event_success:
            R_rgb, _ = cv2.Rodrigues(rgb_rvec)
            t_rgb    = rgb_tvec.flatten()
            R_event, _ = cv2.Rodrigues(event_rvec)
            t_event    = event_tvec.flatten()

            T_rgb = np.eye(4) 
            T_rgb[:3, :3] = R_rgb
            T_rgb[:3, 3] = t_rgb
            
            T_event = np.eye(4)
            T_event[:3, :3] = R_event
            T_event[:3, 3] = t_event
            
            T_rgb_to_pattern.append(T_rgb)
            T_event_to_pattern.append(T_event)

            # Debug: Check the first pattern point reprojection
            first_pattern_point = pattern_points[0, :]
            homogeneous_first_pattern_point = np.ones((4, 1))
            homogeneous_first_pattern_point[:3, 0] = first_pattern_point

            l_rgb_rs = T_rgb @ homogeneous_first_pattern_point
            # print(f"First Pattern Point in Left RGB Camera Reference System:\n{l_rgb_rs}")

            l_event_rs = T_event @ homogeneous_first_pattern_point
            # print(f"First Pattern Point in Left Event Camera Reference System:\n{l_event_rs}")
            diff = (l_rgb_rs - l_event_rs).flatten()[:3]
            # print(f"DIFF = {diff}")
        else:
            print("WTF PnP failed!")

    T_estimates = []
    for T_rgb, T_event in zip(T_rgb_to_pattern, T_event_to_pattern):
        T_rgb_inv = np.linalg.inv(T_rgb)
        T_rgb_to_event = T_event @ T_rgb_inv
        T_estimates.append(T_rgb_to_event)
    
    # Media delle trasformazioni
    rotations = [T[:3, :3] for T in T_estimates]
    translations = [T[:3, 3] for T in T_estimates]
    
    R_mean = Rotation.from_matrix(rotations).mean().as_matrix()
    t_mean = np.mean(translations, axis=0)

    print(f"T RGB_to_Event = {t_mean}")
    print(f"R RGB_to_Event = {R_mean}") 


    # Let's warp event on rgb:

    h, w = rgb_l.shape[:2]

    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        rgb_event_K_left,  rgb_event_D_left,
        rgb_event_K_right, rgb_event_D_right,
        (w, h),
        rgb_event_R,
        rgb_event_T,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=-1
    )

    map1_l, map2_l = cv2.initUndistortRectifyMap(
        rgb_event_K_left, rgb_event_D_left,
        R1, P1,
        (w, h),
        cv2.CV_32FC1
    )

    map1_r, map2_r = cv2.initUndistortRectifyMap(
        rgb_event_K_right, rgb_event_D_right,
        R2, P2,
        (w, h),
        cv2.CV_32FC1
    )

    rect_left  = cv2.remap(rgb_l,   map1_l, map2_l, cv2.INTER_LINEAR)
    rect_right = cv2.remap(event_l, map1_r, map2_r, cv2.INTER_LINEAR)

    overlay = cv2.addWeighted(rect_left, 0.5, rect_right, 0.5, 0)

    cv2.imwrite(os.path.join(current_folder_calib_results, "rgb_event_overlay.png"), overlay)

    # Stack rect_left (top) and rect_right (bottom), save and display
    vertical_stack = np.vstack((rect_left, rect_right))
    cv2.imwrite(os.path.join(current_folder_calib_results, "rgb_event_vertical_stack.png"), vertical_stack)




if __name__ == "__main__":
    main()
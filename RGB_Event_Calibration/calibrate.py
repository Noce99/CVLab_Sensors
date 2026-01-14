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



PATTERN_SIZE = (9, 6)
SIDE_SIZE_CM = 2.93
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
            print(f"Min Diff = {min_diffs}  [{self.accepted_corners}]")


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
        rgb_l_path   = os.path.join(data_folder, index_folder, "left_rgb.png")
        rgb_r_path   = os.path.join(data_folder, index_folder, "right_rgb.png")
        event_l_path = os.path.join(data_folder, index_folder, "left_event.png")
        event_r_path = os.path.join(data_folder, index_folder, "right_event.png")

        rgb_l = cv2.imread(  rgb_l_path,   cv2.IMREAD_GRAYSCALE)
        rgb_r = cv2.imread(  rgb_r_path,   cv2.IMREAD_GRAYSCALE)
        event_l = cv2.imread(event_l_path, cv2.IMREAD_GRAYSCALE)
        event_r = cv2.imread(event_r_path, cv2.IMREAD_GRAYSCALE)

        flags_events = (cv2.CALIB_CB_ADAPTIVE_THRESH + 
            cv2.CALIB_CB_NORMALIZE_IMAGE + 
            cv2.CALIB_CB_FILTER_QUADS)
        cb_rgb_l,   corner_rgb_l   = cv2.findChessboardCornersSB(rgb_l, PATTERN_SIZE, flags=0)
        cb_rgb_r,   corner_rgb_r   = cv2.findChessboardCornersSB(rgb_r, PATTERN_SIZE, flags=0)
        cb_event_l, corner_event_l = cv2.findChessboardCorners(event_l, PATTERN_SIZE, flags=flags_events)
        cb_event_r, corner_event_r = cv2.findChessboardCorners(event_r, PATTERN_SIZE, flags=flags_events)

        if not (cb_rgb_l and cb_rgb_r and cb_event_l and cb_event_r):
            print(f"WTF not able to process {index_folder}!")
            print(f"RGB_left =\t\t{cb_rgb_l}\nRGB_right =\t\t{cb_rgb_r}\nEvent_left =\t\t{cb_event_l}\nEvent_right =\t\t{cb_event_r}")
        else:
            corners_rgb_l_fine = cv2.cornerSubPix(rgb_l, corner_rgb_l, (11,11), (-1,-1), criteria)
            corners_rgb_r_fine = cv2.cornerSubPix(rgb_r, corner_rgb_r, (11,11), (-1,-1), criteria)
            corners_event_l_fine = cv2.cornerSubPix(event_l, corner_event_l, (11,11), (-1,-1), criteria)
            corners_event_r_fine = cv2.cornerSubPix(event_r, corner_event_r, (11,11), (-1,-1), criteria)

            corners_rgb_l_fine   = corners_rgb_l_fine.reshape(-1, 2)
            corners_rgb_r_fine   = corners_rgb_r_fine.reshape(-1, 2)
            corners_event_l_fine = corners_event_l_fine.reshape(-1, 2)
            corners_event_r_fine = corners_event_r_fine.reshape(-1, 2)

            # Save chessboard images with drawn corners
            rgb_l_3_channels = cv2.cvtColor(rgb_l, cv2.COLOR_GRAY2BGR)
            rgb_r_3_channels = cv2.cvtColor(rgb_r, cv2.COLOR_GRAY2BGR)
            event_l_3_channels = cv2.cvtColor(event_l, cv2.COLOR_GRAY2BGR)
            event_r_3_channels = cv2.cvtColor(event_r, cv2.COLOR_GRAY2BGR)
            rgb_l_corners_img = cv2.drawChessboardCorners(rgb_l_3_channels, PATTERN_SIZE, corners_rgb_l_fine, True)
            rgb_r_corners_img = cv2.drawChessboardCorners(rgb_r_3_channels, PATTERN_SIZE, corners_rgb_r_fine, True)
            event_l_corners_img = cv2.drawChessboardCorners(event_l_3_channels, PATTERN_SIZE, corners_event_l_fine, True)
            event_r_corners_img = cv2.drawChessboardCorners(event_r_3_channels, PATTERN_SIZE, corners_event_r_fine, True)

            drawChessboardCorners_path = os.path.join(chessboards_calib_results, index_folder)
            os.mkdir(drawChessboardCorners_path)
            cv2.imwrite(os.path.join(drawChessboardCorners_path, "rgb_left_corners.png"), rgb_l_corners_img)
            cv2.imwrite(os.path.join(drawChessboardCorners_path, "rgb_right_corners.png"), rgb_r_corners_img)
            cv2.imwrite(os.path.join(drawChessboardCorners_path, "event_left_corners.png"), event_l_corners_img)
            cv2.imwrite(os.path.join(drawChessboardCorners_path, "event_right_corners.png"), event_r_corners_img)

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
    rgb_l_path   = os.path.join(data_folder, index_folder, "left_rgb.png")
    rgb_r_path   = os.path.join(data_folder, index_folder, "right_rgb.png")
    event_l_path = os.path.join(data_folder, index_folder, "left_event.png")
    event_r_path = os.path.join(data_folder, index_folder, "right_event.png")

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

    err, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(
        pattern_points_repeated, cs.corners_rgb_l, cs.corners_rgb_r, None, None, None, None, rgb_image_size, flags=0)
    
    rgb_T = T
    rgb_R = R
    rgb_K_left = Kl
    rgb_D_left = Dl
    rgb_K_right = Kr
    rgb_D_right = Dr

    print(f"RGB Reprojection Error = {err:.4f} px (<0.5 Excellent, 0.5<1.0 Good, >1.0 Problematic)")

    err, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(
        pattern_points_repeated, cs.corners_event_l, cs.corners_event_r, None, None, None, None, event_image_size, flags=0)

    event_T = T
    event_R = R
    event_K_left = Kl
    event_D_left = Dl
    event_K_right = Kr
    event_D_right = Dr

    print(f"Event Reprojection Error = {err:.4f} px (<0.5 Excellent, 0.5<1.0 Good, >1.0 Problematic)")

    # Let's Rectify RGB
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        rgb_K_left, rgb_D_left, rgb_K_right, rgb_D_right, rgb_image_size, rgb_R, rgb_T, alpha=0.9
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
        event_K_left, event_D_left, event_K_right, event_D_right, event_image_size, event_R, event_T, alpha=0.9
    )
    event_l_map1, event_l_map2 = cv2.initUndistortRectifyMap(
        event_K_left, event_D_left, R1, P1, event_image_size, cv2.CV_16SC2
    )
    event_r_map1, event_r_map2 = cv2.initUndistortRectifyMap(
        event_K_right, event_D_right, R2, P2, event_image_size, cv2.CV_16SC2
    )

    event_l_rect, event_r_rect, comparison = test_rectification(event_l, event_r, event_l_map1, event_l_map2, event_r_map1, event_r_map2)
    cv2.imwrite(os.path.join(current_folder_calib_results, "event_rectification_test.png"), comparison)

    # TODO: Rectification between everything and RGB Left

    print(f"RGB T = {rgb_T}")
    print(f"Event T = {event_T}")

    T_rgb_to_pattern = []
    T_event_to_pattern = []

    for corner_rgb_l, corner_event_l in tqdm(zip(cs.corners_rgb_l, cs.corners_event_l), total=len(cs.corners_rgb_l)):

        rgb_success, rgb_rvec, rgb_tvec = cv2.solvePnP(
            pattern_points,
            corner_rgb_l,
            rgb_K_left,
            rgb_D_left,
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


if __name__ == "__main__":
    main()
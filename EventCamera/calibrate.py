import os
import cv2

from typing import Tuple, Dict
from tqdm import tqdm
from collections import OrderedDict

import shutil
import numpy as np
import time


PATTERN_SIZE = (9, 6)
SQUARE_SIZE_MM = 29.0  # same as cvlab.config.SQUARE_SIZE_MM

def read_calibration_data(delete_calibration_couples=True) -> Tuple[Dict[str,str], Dict[str,str]]:
    calibration_data_folder_path = "./calibration_data"
    data_folder = os.listdir(calibration_data_folder_path)
    data_folder.sort()

    left_path = os.path.join("./calibration_data", data_folder[-1], "left")
    right_path = os.path.join("./calibration_data", data_folder[-1], "right")
    output_path = os.path.join("./calibration_data", data_folder[-1], "calibration_couples")

    if os.path.exists(output_path):
        if delete_calibration_couples:
            shutil.rmtree(output_path)
            os.mkdir(output_path)
    else:
        os.mkdir(output_path)

    left_dict =  {int(img_file_name.split(".")[0]): os.path.join(left_path, img_file_name)  for img_file_name in os.listdir(left_path) }
    right_dict = {int(img_file_name.split(".")[0]): os.path.join(right_path, img_file_name) for img_file_name in os.listdir(right_path)}

    left_dict =     OrderedDict(sorted(left_dict.items()))
    right_dict =    OrderedDict(sorted(right_dict.items()))

    return left_dict, right_dict, output_path


def find_closer_right_timestamp(timestamp, last_other_timestamp_i, other_timestamps):
    last_dist = abs(timestamp-other_timestamps[last_other_timestamp_i])
    closer_timestamp_i = last_other_timestamp_i
    for i in range(last_other_timestamp_i+1, len(other_timestamps)):
        if abs(timestamp-other_timestamps[i]) > last_dist:
            break
        else:
            last_dist = abs(timestamp-other_timestamps[i])
            closer_timestamp_i = i

    return closer_timestamp_i, other_timestamps[closer_timestamp_i]

def process_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    image[image<40] = 0
    image[image>=40] = 255

    image = cv2.GaussianBlur(image, (15, 15), 0)

    # new_height, new_width = image.shape[0] // 2, image.shape[1] // 2
    # image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
    return image

if __name__ == "__main__":

    left_dict, right_dict, output_path = read_calibration_data()

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

    chess_i = 0
    last_right_timestamp_i = 0

    all_corners_left = []
    all_corners_right = []

    img_size = None

    for left_timestamp in tqdm(left_dict):

        left_img = cv2.imread(left_dict[left_timestamp])
        left_img_gray = process_image(left_img)

        if img_size is None:
            img_size = (left_img.shape[1], left_img.shape[0])

        res_left, corners_left = cv2.findChessboardCorners(left_img_gray, PATTERN_SIZE)

        if res_left:

            corners_left_fine = cv2.cornerSubPix(left_img_gray, corners_left, (11,11), (-1,-1), criteria)

            last_right_timestamp_i, right_timestamp = find_closer_right_timestamp(left_timestamp, last_right_timestamp_i, list(right_dict.keys()))

            right_img = cv2.imread(right_dict[right_timestamp])
            right_image_gray = process_image(right_img)

            # cv2.drawChessboardCorners(left_img, PATTERN_SIZE, corners_left_fine, res_left)
            # cv2.imwrite(os.path.join(output_path, f"{time.time_ns()}_left.png"), left_img)
            # cv2.imwrite(os.path.join(output_path, f"{time.time_ns()}_right_grey.png"), right_image_gray)

            res_right, corners_right = cv2.findChessboardCorners(right_image_gray, PATTERN_SIZE)

            if res_right:
                corners_right_fine = cv2.cornerSubPix(right_image_gray, corners_right, (11,11), (-1,-1), criteria)

                cv2.drawChessboardCorners(left_img, PATTERN_SIZE, corners_left_fine, res_left)
                cv2.imwrite(os.path.join(output_path, f"{chess_i}_left.png"), left_img)

                cv2.drawChessboardCorners(right_img, PATTERN_SIZE, corners_right_fine, res_right)
                cv2.imwrite(os.path.join(output_path, f"{chess_i}_right.png"), right_img)

                chess_i += 1

                all_corners_left.append(corners_left_fine)
                all_corners_right.append(corners_right_fine)

    # Now in corners we have the corners from the left and right images
    assert len(all_corners_left) == len(all_corners_right)

    pattern_points = np.zeros((np.prod(PATTERN_SIZE), 3), np.float32)
    pattern_points[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
    pattern_points *= SQUARE_SIZE_MM
    pattern_points = [pattern_points] * len(all_corners_left)

    err, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(
        pattern_points, all_corners_left, all_corners_right, None, None, None, None, img_size, flags=0)

    print(f"T = {T}")
    print(f"R = {R}")

    # After stereo calibration, compute rectification maps
    R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
        Kl, Dl, Kr, Dr, img_size, R, T, alpha=0.9
    )

    # Compute rectification maps
    left_map1, left_map2 = cv2.initUndistortRectifyMap(
        Kl, Dl, R1, P1, img_size, cv2.CV_16SC2
    )
    right_map1, right_map2 = cv2.initUndistortRectifyMap(
        Kr, Dr, R2, P2, img_size, cv2.CV_16SC2
    )

    np.save(os.path.join(output_path, 'stereo.npy'), {'left_map1': left_map1, 'left_map2': left_map2, 'right_map1': right_map1, 'right_map2': right_map2})



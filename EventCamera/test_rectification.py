import os
import cv2

from typing import Tuple, Dict
from tqdm import tqdm
from collections import OrderedDict
from calibrate import read_calibration_data, find_closer_right_timestamp

import shutil
import numpy as np
from pypylon import genicam
from pypylon import pylon
from calibrate import read_calibration_data
import time


def rectify_stereo_pair(left_img, right_img, left_map1, left_map2, right_map1, right_map2):
    left_rectified = cv2.remap(left_img, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_img, right_map1, right_map2, cv2.INTER_LINEAR)
    return left_rectified, right_rectified

def test_rectification(left_img, right_img, left_map1, left_map2, right_map1, right_map2, output_path, rect_i):
    # Rectify the images
    left_rect, right_rect = rectify_stereo_pair(left_img, right_img, left_map1, left_map2, right_map1, right_map2)
    
    # Create a side-by-side visualization
    vis = np.hstack((left_rect, right_rect))
    
    # Draw horizontal lines to verify rectification
    h, w = left_rect.shape[:2]
    for y in range(0, h, 50):
        cv2.line(vis, (0, y), (w*2, y), (0, 255, 0), 1)
    
    # Save the rectified images
    #cv2.imwrite(os.path.join(output_path, f"rectified_{rect_i}.png"), left_rect)
    #cv2.imwrite(os.path.join(output_path, "rectified_right.png"), right_rect)
    cv2.imwrite(os.path.join(output_path, f"rectified_{rect_i}.png"), vis)
    
    return left_rect, right_rect

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

def main():
    # Let's Rectify

    left_dict, right_dict, output_path = read_calibration_data(delete_calibration_couples=False)

    stereo_maps = np.load(os.path.join(output_path, 'stereo.npy'), allow_pickle=True).item()

    # Access the individual maps
    left_map1 = stereo_maps['left_map1']
    left_map2 = stereo_maps['left_map2'] 
    right_map1 = stereo_maps['right_map1']
    right_map2 = stereo_maps['right_map2']

    # Run the test
    last_right_timestamp_i = 0
    rect_i = 0
    for left_timestamp in left_dict:
        left_img = cv2.imread(left_dict[left_timestamp])

        last_right_timestamp_i, right_timestamp = find_closer_right_timestamp(left_timestamp, last_right_timestamp_i, list(right_dict.keys()))
        right_img = cv2.imread(right_dict[right_timestamp])

        left_rectified, right_rectified = test_rectification(left_img, right_img, left_map1, left_map2, right_map1, right_map2, output_path, rect_i)

        rect_i += 1


if __name__ == "__main__":
    main()
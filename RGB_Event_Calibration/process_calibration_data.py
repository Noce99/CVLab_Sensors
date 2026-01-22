import os
from datetime import datetime
import shutil
from tqdm import tqdm
import cv2
import concurrent.futures
from functools import partial
import numpy as np


PATTERN_SIZE = (9, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)


class DataFolder:
    def __init__(self, path):
        self.path = path
        self.timestamps = [int(fname.split(".")[0]) for fname in os.listdir(self.path) if fname.endswith(".png")]
        self.timestamps.sort()
    
    def get_min_timestamp(self):
        return min(self.timestamps)
    
    def get_max_timestamp(self):
        return max(self.timestamps)

    def discard_previous_to(self, timestamp):
        self.timestamps = [ts for ts in self.timestamps if ts >= timestamp]

    def discard_after_to(self, timestamp):
        self.timestamps = [ts for ts in self.timestamps if ts <= timestamp]

    def get_closer_timestamp(self, timestamp):
        closer_timestamp = min(self.timestamps, key=lambda ts: abs(ts - timestamp))
        return closer_timestamp, abs(timestamp - closer_timestamp)
    
    def file_from_timestamp(self, timestamp):
        return os.path.join(self.path, f"{timestamp}.png")

def process_event_image(image):
    r = image[:, :, 0]
    b = image[:, :, 2]
    g = image[:, :, 1]

    image_mono = np.zeros_like(r)
    print(f"Min r: {r.min()}\tMax r: {r.max()}")
    print(f"Min b: {b.min()}\tMax b: {b.max()}")
    print(f"Min g: {g.min()}\tMax g: {g.max()}")

    image_mono[r < 150] = 0
    image_mono[b < 150] = 0

    cv2.imshow("event_mono", b)
    cv2.waitKey(1)

    return image_mono

def main():
    calibration_data_path = "./calibration_data"
    folders = os.listdir(calibration_data_path)
    folders.sort()

    folder = folders[-1]

    print("Processing folder:", folder)

    left_rgb_df    = DataFolder(os.path.join(calibration_data_path, folder, "left_rgb"))
    right_rgb_df   = DataFolder(os.path.join(calibration_data_path, folder, "right_rgb"))
    left_event_df  = DataFolder(os.path.join(calibration_data_path, folder, "left_event"))
    right_event_df = DataFolder(os.path.join(calibration_data_path, folder, "right_event"))

    starting_timestamp = max(
        left_rgb_df.get_min_timestamp(),
        right_rgb_df.get_min_timestamp(),
        left_event_df.get_min_timestamp(),
        right_event_df.get_min_timestamp(),
    )

    left_rgb_df.discard_previous_to(starting_timestamp)
    right_rgb_df.discard_previous_to(starting_timestamp)
    left_event_df.discard_previous_to(starting_timestamp)
    right_event_df.discard_previous_to(starting_timestamp)

    ending_timestamp = min(
        left_rgb_df.get_max_timestamp(),
        right_rgb_df.get_max_timestamp(),
        left_event_df.get_max_timestamp(),
        right_event_df.get_max_timestamp(),
    )

    left_rgb_df.discard_after_to(ending_timestamp)
    right_rgb_df.discard_after_to(ending_timestamp)
    left_event_df.discard_after_to(ending_timestamp)
    right_event_df.discard_after_to(ending_timestamp)

    print(f"Calibration data ranges from:\n{datetime.fromtimestamp(starting_timestamp / 1e6)}\nto\n{datetime.fromtimestamp(ending_timestamp / 1e6)}")
    print(f"Total of:")
    print(f"\tLeft RGB = {len(left_rgb_df.timestamps)}\n\tRight RGB = {len(right_rgb_df.timestamps)}")
    print(f"\tLeft Event = {len(left_event_df.timestamps)}\n\tRight Event = {len(right_event_df.timestamps)}")

    processed_data_path = "./processed_data"
    if not os.path.exists(processed_data_path):
        os.mkdir(processed_data_path)
    
    processed_folder_path = os.path.join(processed_data_path, folder)
    if os.path.exists(processed_folder_path):
        shutil.rmtree(processed_folder_path)
    os.mkdir(processed_folder_path)

    i = 0
    total_dt_rgb_r = 0
    total_dt_event_l = 0
    total_dt_event_r = 0

    counter_rgb_l = 0
    counter_rgb_r = 0
    counter_event_l = 0
    counter_event_r = 0
    counter_all = 0

    for timestamp in tqdm(left_rgb_df.timestamps, desc="Searching Chessboards in time aligned frames!"):
        rgb_l_path = left_rgb_df.file_from_timestamp(timestamp)

        closer_t_rgb_r, diff_rgb_r = right_rgb_df.get_closer_timestamp(timestamp)
        rgb_r_path = right_rgb_df.file_from_timestamp(closer_t_rgb_r)
        total_dt_rgb_r += diff_rgb_r

        closer_t_event_l, diff_event_l = left_event_df.get_closer_timestamp(timestamp)
        event_l_path = left_event_df.file_from_timestamp(closer_t_event_l)
        total_dt_event_l += diff_event_l

        closer_t_event_r, diff_event_r = right_event_df.get_closer_timestamp(timestamp)
        event_r_path = right_event_df.file_from_timestamp(closer_t_event_r)
        total_dt_event_r += diff_event_r

        rgb_l = cv2.imread(rgb_l_path)
        rgb_r = cv2.imread(rgb_r_path)
        event_l = cv2.imread(event_l_path, cv2.IMREAD_GRAYSCALE)
        event_r = cv2.imread(event_r_path, cv2.IMREAD_GRAYSCALE)

        rgb_l =  cv2.cvtColor(rgb_l, cv2.COLOR_BGR2GRAY)
        rgb_r =  cv2.cvtColor(rgb_r, cv2.COLOR_BGR2GRAY)

        rgb_l = cv2.GaussianBlur(rgb_l, (5,5), 0)
        rgb_r = cv2.GaussianBlur(rgb_r, (5,5), 0)

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(20,20))
        # rgb_l = clahe.apply(rgb_l)
        # rgb_r = clahe.apply(rgb_r)

        flags_events = (cv2.CALIB_CB_ADAPTIVE_THRESH + 
            cv2.CALIB_CB_NORMALIZE_IMAGE + 
            cv2.CALIB_CB_FILTER_QUADS)
        # cb_rgb_l   = cv2.findChessboardCorners(rgb_l,   PATTERN_SIZE)[0]
        # cb_rgb_r   = cv2.findChessboardCorners(rgb_r,   PATTERN_SIZE)[0]


        # _, rgb_l = cv2.threshold(rgb_l, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # _, rgb_r = cv2.threshold(rgb_r, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        rgb_l = adjust_gamma(rgb_l, gamma=0.1)
        rgb_r = adjust_gamma(rgb_r, gamma=0.1)

        cv2.imshow("Event Left", event_l)
        cv2.imshow("Event Right", event_r)
        cv2.imshow("RGB Left", rgb_l)
        cv2.imshow("RGB Right", rgb_r)
        cv2.waitKey(1)

        cb_rgb_l, corner_rgb_l = cv2.findChessboardCornersSB(rgb_l, PATTERN_SIZE, flags=0)
        cb_rgb_r, corner_rgb_r = cv2.findChessboardCornersSB(rgb_r, PATTERN_SIZE, flags=0)

        cb_event_l, corner_event_l = cv2.findChessboardCornersSB(event_l, PATTERN_SIZE, flags=0)
        cb_event_r, corner_event_r = cv2.findChessboardCornersSB(event_r, PATTERN_SIZE, flags=0)
        # print(f"RGB_left =\t\t{cb_rgb_l}\nRGB_right =\t\t{cb_rgb_r}\nEvent_left =\t\t{cb_event_l}\nEvent_right =\t\t{cb_event_r}")

        counter_rgb_l += cb_rgb_l
        counter_rgb_r += cb_rgb_r
        counter_event_l += cb_event_l
        counter_event_r += cb_event_r

        if cb_rgb_l and cb_rgb_r and cb_event_l and cb_event_r:
            counter_all += 1
            subfolder_path = os.path.join(processed_folder_path, f"{i:05d}")
            os.mkdir(subfolder_path)

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

            cv2.imwrite(os.path.join(subfolder_path, "rgb_left_corners.png"), rgb_l_corners_img)
            cv2.imwrite(os.path.join(subfolder_path, "rgb_right_corners.png"), rgb_r_corners_img)
            cv2.imwrite(os.path.join(subfolder_path, "event_left_corners.png"), event_l_corners_img)
            cv2.imwrite(os.path.join(subfolder_path, "event_right_corners.png"), event_r_corners_img)

            # Save corners as numpy arrays
            np.save(os.path.join(subfolder_path, 'left_rgb_corners.npy'), corners_rgb_l_fine)
            np.save(os.path.join(subfolder_path, 'right_rgb_corners.npy'), corners_rgb_r_fine)
            np.save(os.path.join(subfolder_path, 'left_event_corners.npy'), corners_event_l_fine)
            np.save(os.path.join(subfolder_path, 'right_event_corners.npy'), corners_event_r_fine)

            with open(os.path.join(subfolder_path, "log.log"), "w") as f:
                f.write(f"Reference Timestamp = {timestamp / 1e3:.2f} ms\n")
                f.write(f"Left RGB = 0.00 ms\n")
                f.write(f"Right RGB = {diff_rgb_r / 1e3:.2f} ms\n")
                f.write(f"Left Event = {diff_event_l / 1e3:.2f} ms\n")
                f.write(f"Right Event = {diff_event_r / 1e3:.2f} ms\n")

            i += 1
    
    print("Chessboard Searching Log:")
    print(f"\tRGB Left =    {counter_rgb_l}")
    print(f"\tRGB Right =   {counter_rgb_r}")
    print(f"\tEvent Left =  {counter_event_l}")
    print(f"\tEvent Right = {counter_event_r}")
    print(f"\tAll simultaneously = {counter_all}")

    print("Time distanses from RGB Left:")
    print(f"\tAverage dt RGB Right =\t\t\t{total_dt_rgb_r / len(left_rgb_df.timestamps) / 1e3:.2f} ms")
    print(f"\tAverage dt Event Left =\t\t\t{total_dt_event_l / len(left_rgb_df.timestamps) / 1e3:.2f} ms")
    print(f"\tAverage dt Event Right =\t\t{total_dt_event_r / len(left_rgb_df.timestamps) / 1e3:.2f} ms")

def adjust_gamma(image, gamma=1.0):
    """
    gamma < 1: brighter
    gamma > 1: darker
    """
    # Build lookup table
    inv_gamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** inv_gamma) * 255
                      for i in np.arange(0, 256)]).astype("uint8")
    
    return cv2.LUT(image, table)


if __name__ == "__main__":
    main()
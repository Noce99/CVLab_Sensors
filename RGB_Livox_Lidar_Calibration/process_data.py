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

def set_new_size(width, height, img):
    aspect_ration = width / height
    new_img_height = img.shape[1] / aspect_ration
    height_diff = abs(img.shape[0] - new_img_height)
    if height_diff != 0:
        img = img[int(height_diff/2):int(-height_diff/2), :]

    img = cv2.resize(img, (width, height))
    return img

def main():
    calibration_data_path = "./calibration_data"
    folders = os.listdir(calibration_data_path)
    folders.sort()

    folder = folders[-1]

    print("Processing folder:", folder)

    lidar_df = DataFolder(os.path.join(calibration_data_path, folder, "lidar"))
    rgb_df   = DataFolder(os.path.join(calibration_data_path, folder, "rgb"))

    starting_timestamp = max(
        lidar_df.get_min_timestamp(),
        rgb_df.get_min_timestamp(),
    )

    lidar_df.discard_previous_to(starting_timestamp)
    rgb_df.discard_previous_to(starting_timestamp)


    ending_timestamp = min(
        lidar_df.get_max_timestamp(),
        rgb_df.get_max_timestamp(),
    )

    lidar_df.discard_after_to(ending_timestamp)
    rgb_df.discard_after_to(ending_timestamp)

    print(f"Calibration data ranges from:\n{datetime.fromtimestamp(starting_timestamp / 1e6)}\nto\n{datetime.fromtimestamp(ending_timestamp / 1e6)}")
    print(f"Total of:")
    print(f"\tLidar = {len(lidar_df.timestamps)}")
    print(f"\tRGB   = {len(rgb_df.timestamps)}")

    processed_data_path = "./processed_data"
    if not os.path.exists(processed_data_path):
        os.mkdir(processed_data_path)
    
    processed_folder_path = os.path.join(processed_data_path, folder)
    if os.path.exists(processed_folder_path):
        shutil.rmtree(processed_folder_path)
    os.mkdir(processed_folder_path)

    i = 0
    j = 0
    total_rgb = 0

    counter_lidar = 0
    counter_rgb = 0
    counter_all = 0

    os.mkdir(os.path.join(processed_folder_path, "discarded"))

    for timestamp in tqdm(lidar_df.timestamps, desc="Searching Chessboards in time aligned frames!"):
        lidar_path = lidar_df.file_from_timestamp(timestamp)

        closer_t_rgb, diff_rgb = rgb_df.get_closer_timestamp(timestamp)
        rgb_path = rgb_df.file_from_timestamp(closer_t_rgb)
        total_rgb += diff_rgb

        lidar = cv2.imread(lidar_path, cv2.IMREAD_GRAYSCALE)
        rgb = cv2.imread(rgb_path, cv2.IMREAD_GRAYSCALE)


        height = min(lidar.shape[0], rgb.shape[0])
        width = min(lidar.shape[1], rgb.shape[1])

        rgb = set_new_size(width, height, rgb)
        lidar = set_new_size(width, height, lidar)    

        # lidar = adjust_gamma(lidar, gamma=15.0) # 23

        # lidar[lidar > 100] = 255
        # lidar[lidar <= 100] = 0

        # lidar = cv2.bitwise_not(lidar)
        # kernel = np.ones((3,3), np.uint8)
        # lidar = cv2.dilate(lidar, kernel, iterations=1)
        # lidar = cv2.bitwise_not(lidar)

        cb_lidar, corner_lidar = cv2.findChessboardCornersSB(lidar, PATTERN_SIZE, flags=0)
        cb_rgb, corner_rgb = cv2.findChessboardCornersSB(rgb, PATTERN_SIZE, flags=0)
        # print(f"RGB_left =\t\t{cb_rgb_l}\nRGB_right =\t\t{cb_rgb_r}\nEvent_left =\t\t{cb_event_l}\nEvent_right =\t\t{cb_event_r}")

        counter_lidar += cb_lidar
        counter_rgb += cb_rgb

        if cb_lidar and cb_rgb:
            counter_all += 1
            subfolder_path = os.path.join(processed_folder_path, f"{i:05d}")
            os.mkdir(subfolder_path)

            corners_lidar_fine = corner_lidar# cv2.cornerSubPix(lidar, corner_lidar, (11,11), (-1,-1), criteria)
            corner_rgb_fine    = corner_rgb# cv2.cornerSubPix(rgb, corner_rgb, (11,11), (-1,-1), criteria)

            corners_lidar_fine   = corners_lidar_fine.reshape(-1, 2)
            corner_rgb_fine      = corner_rgb_fine.reshape(-1, 2)

            # Save chessboard images with drawn corners
            lidar_3_channels = cv2.cvtColor(lidar, cv2.COLOR_GRAY2BGR)
            rgb_3_channels = cv2.cvtColor(rgb, cv2.COLOR_GRAY2BGR)

            lidar_corners_img = cv2.drawChessboardCorners(lidar_3_channels, PATTERN_SIZE, corners_lidar_fine, True)
            rgb_corners_img = cv2.drawChessboardCorners(rgb_3_channels, PATTERN_SIZE, corner_rgb_fine, True)

            cv2.imwrite(os.path.join(subfolder_path, "lidar_corners.png"), lidar_corners_img)
            cv2.imwrite(os.path.join(subfolder_path, "rgb_corners.png"), rgb_corners_img)

            # Save corners as numpy arrays
            np.save(os.path.join(subfolder_path, 'lidar_corners.npy'), corners_lidar_fine)
            np.save(os.path.join(subfolder_path, 'rgb_corners.npy'), corner_rgb_fine)

            with open(os.path.join(subfolder_path, "log.log"), "w") as f:
                f.write(f"Reference Timestamp = {timestamp / 1e3:.2f} ms\n")
                f.write(f"Lidar = 0.00 ms\n")
                f.write(f"RGB = {diff_rgb / 1e3:.2f} ms\n")

            i += 1
        else:
            subfolder_path = os.path.join(processed_folder_path, "discarded", f"{j:05d}")
            os.mkdir(subfolder_path)
            if not cb_lidar:
                cv2.imwrite(os.path.join(subfolder_path, "lidar_not_ok.png"), lidar)
            else:
                cv2.imwrite(os.path.join(subfolder_path, "lidar_ok.png"), lidar)
            if not cb_rgb:
                cv2.imwrite(os.path.join(subfolder_path, "rgb_not_ok.png"), rgb)
            else:
                cv2.imwrite(os.path.join(subfolder_path, "rgb_ok.png"), rgb)
            j += 1
    
    print("Chessboard Searching Log:")
    print(f"\tLidar =    {counter_lidar}")
    print(f"\tRGB =   {counter_rgb}")
    print(f"\tAll simultaneously = {counter_all}")

    # print("Time distanses from RGB Left:")
    # print(f"\tAverage dt RGB Right =\t\t\t{total_dt_rgb_r / len(left_rgb_df.timestamps) / 1e3:.2f} ms")
    # print(f"\tAverage dt Event Left =\t\t\t{total_dt_event_l / len(left_rgb_df.timestamps) / 1e3:.2f} ms")
    # print(f"\tAverage dt Event Right =\t\t{total_dt_event_r / len(left_rgb_df.timestamps) / 1e3:.2f} ms")

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
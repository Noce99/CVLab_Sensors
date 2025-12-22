import os
from datetime import datetime
import shutil
from tqdm import tqdm
import cv2
import concurrent.futures
from functools import partial
import numpy as np


PATTERN_SIZE = (9, 6)

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

    print(f"Calibration data ranges from {datetime.fromtimestamp(starting_timestamp / 1e6)} to {datetime.fromtimestamp(ending_timestamp / 1e6)}!")
    print(f"Total of:")
    print(f"Left RGB = {len(left_rgb_df.timestamps)}\t\tRight RGB = {len(right_rgb_df.timestamps)}")
    print(f"Left Event = {len(left_event_df.timestamps)}\t\tRight Event = {len(right_event_df.timestamps)}")

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
    for timestamp in tqdm(left_rgb_df.timestamps):
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
        event_l = cv2.imread(event_l_path)
        event_r = cv2.imread(event_r_path)

        rgb_l =  cv2.cvtColor(rgb_l, cv2.COLOR_BGR2GRAY)
        rgb_l = 255 - rgb_l
        rgb_r =  cv2.cvtColor(rgb_r, cv2.COLOR_BGR2GRAY)
        rgb_r = 255 - rgb_r
        # event_l = process_event_image(event_l)
        # event_r = process_event_image(event_r)

        # cv2.imshow("event_l", event_l)
        # cv2.imshow("event_r", event_r)
        #cv2.waitKey(1)

        flags_events = (cv2.CALIB_CB_ADAPTIVE_THRESH + 
            cv2.CALIB_CB_NORMALIZE_IMAGE + 
            cv2.CALIB_CB_FILTER_QUADS)
        cb_rgb_l   = cv2.findChessboardCorners(rgb_l,   PATTERN_SIZE)[0]
        cb_rgb_r   = cv2.findChessboardCorners(rgb_r,   PATTERN_SIZE)[0]
        cb_event_l = cv2.findChessboardCorners(event_l, PATTERN_SIZE, flags=flags_events)[0]
        cb_event_r = cv2.findChessboardCorners(event_r, PATTERN_SIZE, flags=flags_events)[0]

        print(f"RGB_left =\t\t{cb_rgb_l}\nRGB_right =\t\t{cb_rgb_r}\nEvent_left =\t\t{cb_event_l}\nEvent_right =\t\t{cb_event_r}")


        if cb_rgb_l and cb_rgb_r and cb_event_l and cb_event_r:
            subfolder_path = os.path.join(processed_folder_path, f"{i:05d}")
            os.mkdir(subfolder_path)

            cv2.imwrite(os.path.join(subfolder_path, "left_rgb.png"),    rgb_l  )
            cv2.imwrite(os.path.join(subfolder_path, "right_rgb.png"),   rgb_r  )
            cv2.imwrite(os.path.join(subfolder_path, "left_event.png"),  event_l)
            cv2.imwrite(os.path.join(subfolder_path, "right_event.png"), event_r)

            with open(os.path.join(subfolder_path, "log.log"), "w") as f:
                f.write(f"Reference Timestamp = {timestamp / 1e3:.2f} ms\n")
                f.write(f"Left RGB = 0.00 ms\n")
                f.write(f"Right RGB = {diff_rgb_r / 1e3:.2f} ms\n")
                f.write(f"Left Event = {diff_event_l / 1e3:.2f} ms\n")
                f.write(f"Right Event = {diff_event_r / 1e3:.2f} ms\n")

            i += 1
    
    print(f"Average dt RGB Right =\t\t\t{total_dt_rgb_r / len(left_rgb_df.timestamps) / 1e3:.2f} ms")
    print(f"Average dt Event Left =\t\t\t{total_dt_event_l / len(left_rgb_df.timestamps) / 1e3:.2f} ms")
    print(f"Average dt Event Right =\t\t{total_dt_event_r / len(left_rgb_df.timestamps) / 1e3:.2f} ms")




if __name__ == "__main__":
    main()
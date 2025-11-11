"""
There I will take data from the cameras and I will save them on the disk!
"""
import os
os.environ["PYLON_CAMEMU"] = "0" # "3"
from pypylon import genicam
from pypylon import pylon
import sys
import time
import cv2
from datetime import datetime
import subprocess


def launch_save_images(left_camera: bool, path: str):
    cmd = ["/home/ubm/CVLab_Sensors/venv_3_12/bin/python", "/home/ubm/CVLab_Sensors/EventCamera/save_images.py", path]
    if left_camera is False:
        cmd += ["--right-camera"]

    try:
        return subprocess.Popen(cmd)
    except Exception as e:
        print(f"Error running command: {e}")

def main():

    if not os.path.exists("./calibration_data"):
        os.mkdir("./calibration_data")

    current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    current_folder = os.path.join("./calibration_data", current_time)
    os.mkdir(current_folder)

    left_folder = os.path.join(current_folder, "left")
    right_folder = os.path.join(current_folder, "right")

    os.mkdir(left_folder)
    os.mkdir(right_folder)

    left_result = launch_save_images(left_camera=True, path=left_folder)
    right_result = launch_save_images(left_camera=False, path=right_folder)

    print("BACK to main!")
    time.sleep(180)

    left_result.kill()
    right_result.kill()




if __name__ == "__main__":
    main()
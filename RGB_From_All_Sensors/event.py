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


class EventSaveStereoImages:

    def __init__(self, left_path):
        self.left_thread  = launch_save_images(left_camera = True,  path = left_path)
    
    def kill_all(self):
        self.left_thread.kill()

if __name__ == "__main__":
    esi = EventSaveStereoImages("./event_images")
    time.sleep(10)
    esi.kill_all()
import threading
import time
import cv2
import numpy as np
from pypylon import genicam
from pypylon import pylon
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RGBCamera.setup_cameras import setup_camera


LEFT_SERIAL = "40663113"
RIGHT_SERIAL = "40665216"

class RGBSaveStereoImages:
    def __init__(self, path_left, path_right):
        cameras = pylon.InstantCameraArray(2) # 0 -> left, 1 --> right
        self.camera_thread = \
            threading.Thread(
                target=RGBSaveStereoImages.save_data,
                args = (cameras, path_left, path_right),
                daemon=True,
            )
        self.camera_thread.start()        

    @staticmethod
    def save_data(cameras, path_left, path_right):
        tlFactory = pylon.TlFactory.GetInstance()
        left_camera_found = False
        right_camera_found = False
        while not left_camera_found or not right_camera_found:
            devices = tlFactory.EnumerateDevices()
            for d in devices:
                if d.GetSerialNumber() == LEFT_SERIAL:
                    cameras[0].Attach(tlFactory.CreateDevice(d))
                    cameras[0].Open()
                    setup_camera(cameras[0])
                    left_camera_found = True
                elif d.GetSerialNumber() == RIGHT_SERIAL:
                    cameras[1].Attach(tlFactory.CreateDevice(d))
                    cameras[1].Open()
                    setup_camera(cameras[1])
                    right_camera_found = True
                else:
                    print(f"Uknown camera with serial: {d.GetSerialNumber()}")

        while True:
            all_locked = True
            for i, camera in enumerate(cameras):
                camera.PtpDataSetLatch.Execute()
                time.sleep(0.1)
                print(f"[{i}] -- {camera.PtpStatus.Value} [{camera.PtpServoStatus.Value}] [{camera.PtpOffsetFromMaster.Value} ns]")
                if camera.PtpServoStatus.Value != "Locked":
                    all_locked = False
            if all_locked:
                break
            time.sleep(0.1)

        print(f"Starting taking data from Left and Right RGB cameras...")

        cameras.StartGrabbing()

        how_many_images_between_ptp_updates = 50

        while True:
            grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            cameraContextValue = grabResult.GetCameraContext()

            folder_path = [
                path_left,
                path_right,
            ]

            if grabResult.GrabSucceeded():
                timestamp = grabResult.GetTimeStamp()
                path_to_save = os.path.join(folder_path[cameraContextValue], f"{int(timestamp / 1e3)}.png")
                cv2.imwrite(path_to_save, grabResult.GetArray())
 
import threading
import time
import cv2
import numpy as np
from pypylon import genicam
from pypylon import pylon
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from setup_camera import setup_camera


LEFT_SERIAL = "40663113"
RIGHT_SERIAL = "40665216"
CAMERA_TO_USE = LEFT_SERIAL

class RGBSaveImages:
    def __init__(self, path):
        cameras = pylon.InstantCameraArray(1) # 0 -> left, 1 --> right
        self.info = {"getting_data": False}
        self.camera_thread = \
            threading.Thread(
                target=RGBSaveImages.save_data,
                args = (cameras, path, self.info),
                daemon=True,
            )
        self.camera_thread.start()     


    @staticmethod
    def save_data(cameras, path, info):
        tlFactory = pylon.TlFactory.GetInstance()
        camera_found = False
        while not camera_found:
            devices = tlFactory.EnumerateDevices()
            for d in devices:
                if d.GetSerialNumber() == CAMERA_TO_USE:
                    cameras[0].Attach(tlFactory.CreateDevice(d))
                    cameras[0].Open()
                    setup_camera(cameras[0])
                    camera_found = True
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

        print(f"Starting taking data camera...")
        info["getting_data"] = True

        cameras.StartGrabbing()

        how_many_images_between_ptp_updates = 50

        while True:
            grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            cameraContextValue = grabResult.GetCameraContext()

            if grabResult.GrabSucceeded():
                timestamp = grabResult.GetTimeStamp()
                path_to_save = os.path.join(path, f"{int(timestamp / 1e3)}.png")
                cv2.imwrite(path_to_save, grabResult.GetArray())
                # cv2.imshow(f"Camera {cameraContextValue}", grabResult.GetArray())
                # cv2.waitKey(1)
 
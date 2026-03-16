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
import traceback


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
        cameras = pylon.InstantCameraArray(1)
        tlf = pylon.TlFactory.GetInstance()
        tl = tlf.CreateTl('BaslerGigE')
        tlf.EnumerateDevices()

        cam_left_info = tl.CreateDeviceInfo()
        cam_left_info.SetIpAddress('192.168.1.70')

        cam_right_info = tl.CreateDeviceInfo()
        cam_right_info.SetIpAddress('192.168.1.71')

        cameras[0].Attach(tl.CreateDevice(cam_left_info))
        cameras[0].Open()
        setup_camera(cameras[0])

        cameras[0].StreamGrabber.TransmissionType ="Multicast"
        cameras[0].SetCameraContext(0)
        cameras[0].StreamGrabber.DestinationAddr ="192.168.1.52"
        cameras[0].StreamGrabber.DestinationPort.Value = 5556

        print(f"Starting taking data from Left and Right RGB cameras...")
        print(f"PTH = {path}")

        cameras.StartGrabbing()

        pc_ts  = time.time() * 1_000_000
        cameras[0].TimestampLatch.Execute()
        camera_ts = cameras[0].TimestampLatchValue.Value / 1_000
        delta_ts = pc_ts - camera_ts

        counter = 0

        while True:
            try:
                grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

                cameraContextValue = grabResult.GetCameraContext()

                if grabResult.GrabSucceeded():
                    camera_index = grabResult.GetCameraContext()
                    ts = grabResult.GetTimeStamp() / 1_000 + delta_ts
                    cv2.imwrite(os.path.join(path, f"{int(ts)}.png"), cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_RGB2BGR))

                    if cameraContextValue == 0 and counter % 100 == 0:
                        counter = 0
                        pc_ts  = time.time() * 1_000_000
                        cameras[0].TimestampLatch.Execute()
                        camera_ts = cameras[0].TimestampLatchValue.Value / 1_000
                        delta_ts = pc_ts - camera_ts

                    counter += 1
                else:
                    if cameraContextValue == 0:
                        camera = "left"
                    else:
                        camera = "right"
                    print(f"Grab does not succeeded! [camera = {camera}]")
            except Exception as e:
                traceback.print_exc() 
 
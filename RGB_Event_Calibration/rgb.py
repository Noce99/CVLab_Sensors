import threading
import time
import cv2
import numpy as np
from pypylon import genicam
from pypylon import pylon
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RGBCamera.setup_stereo_cameras import setup_stereo_cameras

import traceback

LEFT_SERIAL = "40663113"
RIGHT_SERIAL = "40665216"

class RGBSaveStereoImages:
    def __init__(self, path_left, path_right):
        cameras = pylon.InstantCameraArray(2) # 0 -> left, 1 --> right
        self.info = {"getting_data": False}
        self.camera_thread = \
            threading.Thread(
                target=RGBSaveStereoImages.save_data,
                args = (cameras, path_left, path_right, self.info),
                daemon=True,
            )
        self.camera_thread.start()     


    @staticmethod
    def save_data(cameras, path_left, path_right, info):
        tlf = pylon.TlFactory.GetInstance()
            
        left_found = False
        right_found = False
        
        print("INFO: Searching for RGB cameras...")
        tl = tlf.CreateTl('BaslerGigE')
        tlf.EnumerateDevices()
        
        cam_left_info = tl.CreateDeviceInfo()
        cam_left_info.SetIpAddress('192.168.1.70')

        cam_right_info = tl.CreateDeviceInfo()
        cam_right_info.SetIpAddress('192.168.1.71')

        cameras[0].Attach(tl.CreateDevice(cam_left_info))
        cameras[0].Open()
        setup_stereo_cameras(cameras[0])

        cameras[1].Attach(tl.CreateDevice(cam_right_info))
        cameras[1].Open()
        setup_stereo_cameras(cameras[1])

        cameras[1].StreamGrabber.TransmissionType ="Multicast"
        cameras[1].SetCameraContext(1)
        cameras[1].StreamGrabber.DestinationAddr ="192.168.1.52"
        cameras[1].StreamGrabber.DestinationPort.Value = 5555
        cameras[0].StreamGrabber.TransmissionType ="Multicast"
        cameras[0].SetCameraContext(0)
        cameras[0].StreamGrabber.DestinationAddr ="192.168.1.52"
        cameras[0].StreamGrabber.DestinationPort.Value = 5556

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
        info["getting_data"] = True

        pc_ts  = time.time() * 1_000_000
        camera.TimestampLatch.Execute()
        camera_ts = camera.TimestampLatchValue.Value / 1_000
        delta_ts = pc_ts - camera_ts

        counter = 0
        folder_path = [
            path_left,
            path_right,
        ]

        while True:
            try:
                grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

                cameraContextValue = grabResult.GetCameraContext()

                if grabResult.GrabSucceeded():
                    camera_index = grabResult.GetCameraContext()
                    ts = grabResult.GetTimeStamp() / 1_000 + delta_ts
                    cv2.imwrite(os.path.join(folder_path[cameraContextValue], f"{int(ts)}.png"), cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_RGB2BGR))

                    if cameraContextValue == 0 and counter % 100 == 0:
                        counter = 0
                        pc_ts  = time.time() * 1_000_000
                        camera.TimestampLatch.Execute()
                        camera_ts = camera.TimestampLatchValue.Value / 1_000
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
        

        

        # print(f"Starting taking data from Left and Right RGB cameras...")
        # info["getting_data"] = True

        # cameras.StartGrabbing()

        # how_many_images_between_ptp_updates = 50

        # while True:
        #     grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        #     cameraContextValue = grabResult.GetCameraContext()

        #     folder_path = [
        #         path_left,
        #         path_right,
        #     ]

        #     if grabResult.GrabSucceeded():
        #         timestamp = grabResult.GetTimeStamp()
        #         path_to_save = os.path.join(folder_path[cameraContextValue], f"{int(timestamp / 1e3)}.png")
        #         cv2.imwrite(path_to_save, grabResult.GetArray())
        #         # cv2.imshow(f"Camera {cameraContextValue}", grabResult.GetArray())
        #         # cv2.waitKey(1)
 
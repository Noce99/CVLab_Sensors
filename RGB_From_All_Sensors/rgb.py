import threading
import time
import cv2
import numpy as np
from pypylon import genicam
from pypylon import pylon
import sys
import os
import socket

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RGBCamera.setup_stereo_cameras import setup_stereo_cameras


LEFT_SERIAL = "40663113"
RIGHT_SERIAL = "40665216"

class RGBSaveStereoImages:
    def __init__(self, path_left):
        cameras = pylon.InstantCameraArray(2) # 0 -> left, 1 --> right
        self.info = {"getting_data": False}
        self.camera_thread = \
            threading.Thread(
                target=RGBSaveStereoImages.save_data,
                args = (cameras, path_left, self.info),
                daemon=True,
            )
        self.camera_thread.start()     


    @staticmethod
    def save_data(cameras, path_left, info):
        print(f"Starting taking data!")
        # tlFactory = pylon.TlFactory.GetInstance()

        tlf = pylon.TlFactory.GetInstance()
        tl = tlf.CreateTl('BaslerGigE')
        tlf.EnumerateDevices()

        cam_left_info = tl.CreateDeviceInfo()
        cam_left_info.SetIpAddress('192.168.1.70')

        cam_right_info = tl.CreateDeviceInfo()
        cam_right_info.SetIpAddress('192.168.1.71')

        """
        tlf = pylon.TlFactory.GetInstance()
        tlf.EnumerateDevices() # Note this additional line to @SMA2016a's answer above
        cam_info = pylon.DeviceInfo()
        cam_info.SetIpAddress(ip_address)
        cam = pylon.InstantCamera(tlf.CreateDevice(cam_info))
        """

        # device_cam_left = tlf.CreateDevice(cam_left_info)
        # device_cam_right = tlf.CreateDevice(cam_right_info)

        """print(type(device_cam_left))

        cam_left = pylon.InstantCamera(device_cam_left)
        cam_right = pylon.InstantCamera(device_cam_right)

        print(type(cam_left))"""

        cameras[0].Attach(tl.CreateDevice(cam_left_info))
        cameras[0].Open()
        setup_stereo_cameras(cameras[0])
        # cameras[0].GetStreamGrabberParams().DestinationAddr = "239.0.0.1"
        # cameras[0].GetStreamGrabberParams().DestinationPort = 49154

        cameras[1].Attach(tl.CreateDevice(cam_right_info))
        cameras[1].Open()
        setup_stereo_cameras(cameras[1])


        """left_camera_found = False
        right_camera_found = False
        while not left_camera_found or not right_camera_found:
            devices = tlFactory.EnumerateDevices()
            for d in devices:
                if d.GetSerialNumber() == LEFT_SERIAL:
                    cameras[0].Attach(tlFactory.CreateDevice(d))
                    cameras[0].Open()
                    setup_stereo_cameras(cameras[0])
                    left_camera_found = True
                    print("Left camera found!")
                elif d.GetSerialNumber() == RIGHT_SERIAL:
                    cameras[1].Attach(tlFactory.CreateDevice(d))
                    cameras[1].Open()
                    setup_stereo_cameras(cameras[1])
                    right_camera_found = True
                    print("Right camera found!")
                else:
                    print(f"Uknown camera with serial: {d.GetSerialNumber()}")"""

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
        info["getting_data"] = True

        cameras.StartGrabbing()

        while True:
            grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            cameraContextValue = grabResult.GetCameraContext()

            if cameraContextValue == 0:

                if grabResult.GrabSucceeded():
                    timestamp = grabResult.GetTimeStamp()
                    path_to_save = os.path.join(path_left, f"{int(timestamp / 1e3)}.png")
                    rgb_image = cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_BGR2RGB)
                    cv2.imwrite(path_to_save, rgb_image)
                    # cv2.imshow(f"Camera {cameraContextValue}", grabResult.GetArray())
                    # cv2.waitKey(1)
 
 
if __name__ == "__main__":
    a = RGBSaveStereoImages("./rgb_images")
    time.sleep(100)

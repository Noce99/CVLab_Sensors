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


def save_data(ip, path, is_left):

    print(f"Starting taking data!")
    # tlFactory = pylon.TlFactory.GetInstance()

    tlf = pylon.TlFactory.GetInstance()
    tl = tlf.CreateTl('BaslerGigE')
    tlf.EnumerateDevices()

    cam_info = tl.CreateDeviceInfo()
    cam_info.SetIpAddress(ip)

    camera = pylon.InstantCamera(tl.CreateDevice(cam_info))
    camera.Open()
    setup_stereo_cameras(camera)

    camera.StreamGrabber.TransmissionType.Value ="Multicast"
    camera.StreamGrabber.DestinationAddr.Value ="192.168.1.52"


    while True:
        all_locked = True
        camera.PtpDataSetLatch.Execute()
        time.sleep(0.1)
        print(f"[{camera.PtpStatus.Value} [{camera.PtpServoStatus.Value}] [{camera.PtpOffsetFromMaster.Value} ns]")
        if camera.PtpServoStatus.Value != "Locked":
            all_locked = False
        if all_locked:
            break
        time.sleep(0.1)

    print(f"Starting taking data from camera {ip}...")

    camera.StartGrabbing()

    if is_left:
        camera_name = "left"
    else:
        camera_name = "right"

    while True:
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        
        if grabResult.GrabSucceeded():
            timestamp = grabResult.GetTimeStamp()
            path_to_save = os.path.join(path, f"{camera_name}_{int(timestamp / 1e3)}.png")
            rgb_image = cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_BGR2RGB)
            cv2.imwrite(path_to_save, rgb_image)
            # cv2.imshow(f"Camera {cameraContextValue}", grabResult.GetArray())
            # cv2.waitKey(1)
            print("YEA")
        else:
            # print("FUCK")
            pass
        grabResult.Release()
 
 
if __name__ == "__main__":
    a = save_data("192.168.1.70", "./rgb_images", True)
    # b = save_data("192.168.1.71", "./rgb_images", False)
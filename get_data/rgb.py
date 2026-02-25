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


def save_data(cameras, path_left, path_right):
    print(f"Starting taking data!")
    # tlFactory = pylon.TlFactory.GetInstance()

    tlf = pylon.TlFactory.GetInstance()
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

    while True:
        grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        cameraContextValue = grabResult.GetCameraContext()

        if grabResult.GrabSucceeded():
            if cameraContextValue == 0:
                timestamp = grabResult.GetTimeStamp()
                path_to_save = os.path.join(path_left, f"{int(timestamp / 1e3)}.png")
                rgb_image = cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_BGR2RGB)
                cv2.imwrite(path_to_save, rgb_image)
                # cv2.imshow(f"Camera {cameraContextValue}", grabResult.GetArray())
                # cv2.waitKey(1)
            elif cameraContextValue == 1:
                timestamp = grabResult.GetTimeStamp()
                path_to_save = os.path.join(path_right, f"{int(timestamp / 1e3)}.png")
                rgb_image = cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_BGR2RGB)
                cv2.imwrite(path_to_save, rgb_image)
                # cv2.imshow(f"Camera {cameraContextValue}", grabResult.GetArray())
                # cv2.waitKey(1)
            else:
                print(f"Uknown RGB camera? cameraContextValue = {cameraContextValue}")
        else:
            if cameraContextValue == 0:
                camera = "left"
            else:
                camera = "right"
            print(f"Grab does not succeeded! [camera = {camera}]")
            
 
 
if __name__ == "__main__":
    if len(sys.argv) > 1:
        data_path =  sys.argv[1]
    else:
        print("Please provide the path to save images as an argument.")
        exit(1)
    
    if not os.path.exists(data_path):
        print(f"Path {data_path} does not exists!")
        exit(1)

    rgb_data_path = os.path.join(data_path, "rgb")
    os.makedirs(rgb_data_path, exist_ok=True)
    
    path_left = os.path.join(rgb_data_path, "left")
    os.makedirs(path_left, exist_ok=True)
    path_right = os.path.join(rgb_data_path, "right")
    os.makedirs(path_right, exist_ok=True)


    print(f"I will save data in {rgb_data_path}")

    save_data(
        cameras    = pylon.InstantCameraArray(2),
        path_left  = path_left,
        path_right = path_right,
    )

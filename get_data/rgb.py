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

import traceback


def save_data(path_left, path_right):
    print(f"Starting taking data!")
    # tlFactory = pylon.TlFactory.GetInstance()

    cameras = pylon.InstantCameraArray(2)
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

    path = [path_left, path_right]

    pc_ts  = time.time() * 1_000_000
    camera.TimestampLatch.Execute()
    camera_ts = camera.TimestampLatchValue.Value / 1_000
    delta_ts = pc_ts - camera_ts

    counter = 0

    while True:
        try:
            grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            cameraContextValue = grabResult.GetCameraContext()

            if grabResult.GrabSucceeded():
                camera_index = grabResult.GetCameraContext()
                ts = grabResult.GetTimeStamp() / 1_000 + delta_ts
                cv2.imwrite(os.path.join(path[camera_index], f"{int(ts)}.png"), cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_RGB2BGR))

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


if __name__ == "__main__":
    if len(sys.argv) > 1:
        data_path =  sys.argv[1]
    else:
        print("Please provide the path to save images as an argument.")
        exit(1)
    
    if not os.path.exists(data_path):
        print(f"Path {data_path} does not exists!")
        exit(1)


    path_left = os.path.join(data_path, "rgb_left")
    path_right = os.path.join(data_path, "rgb_right")


    print(f"I will save data in {path_left} and {path_right}")

    save_data(
        path_left  = path_left,
        path_right = path_right,
    )

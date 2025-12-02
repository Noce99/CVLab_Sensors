import os
import cv2

from typing import Tuple, Dict
from tqdm import tqdm
from collections import OrderedDict
from calibrate import read_calibration_data, find_closer_right_timestamp

import shutil
import numpy as np
from pypylon import genicam
from pypylon import pylon
from setup_cameras import setup_camera
import time



def rectify_stereo_pair(left_img, right_img, left_map1, left_map2, right_map1, right_map2):
    left_rectified = cv2.remap(left_img, left_map1, left_map2, cv2.INTER_LINEAR)
    right_rectified = cv2.remap(right_img, right_map1, right_map2, cv2.INTER_LINEAR)
    return left_rectified, right_rectified

def test_rectification(left_img, right_img, left_map1, left_map2, right_map1, right_map2, output_path):
    # Rectify the images
    left_rect, right_rect = rectify_stereo_pair(left_img, right_img, left_map1, left_map2, right_map1, right_map2)
    
    # Create a side-by-side visualization
    vis = np.hstack((left_rect, right_rect))
    
    # Draw horizontal lines to verify rectification
    h, w = left_rect.shape[:2]

    for y in range(0, h, 50):
        cv2.line(vis, (0, y), (w*2, y), (0, 255, 0), 1)
    
    # Save the rectified images
    cv2.imwrite(os.path.join(output_path, f"rectified_left.png"), left_rect)
    cv2.imwrite(os.path.join(output_path, f"rectified_right.png"), right_rect)
    cv2.imwrite(os.path.join(output_path, f"rectified_comparison.png"), vis)
    
    return left_rect, right_rect

def main():
    left_right_serial = {
        "40663113": "left",
        "40665216": "right",
    }
    left_camera = {
        "40663113": False,
        "40665216": True,
    }



    # Number of images to be grabbed.
    countOfImagesToGrab = 10000
    maxCamerasToUse = 2

    # Get the transport layer factory.
    tlFactory = pylon.TlFactory.GetInstance()

    # Get all attached devices and exit application if no device is found.
    devices = tlFactory.EnumerateDevices()
    if len(devices) == 0:
        raise pylon.RuntimeException("No camera present.")

    # Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
    cameras = pylon.InstantCameraArray(min(len(devices), maxCamerasToUse))

    # Create and attach all Pylon Devices.
    camera_serials = {}
    for i, camera in enumerate(cameras):
        camera.Attach(tlFactory.CreateDevice(devices[i]))
        camera.Open()

        # Print the serial of the camera.
        print(f"[{i}] Using device ", camera.GetDeviceInfo().GetSerialNumber())

        camera_serials[i] = camera.GetDeviceInfo().GetSerialNumber()
        
        setup_camera(camera, left_camera[camera_serials[i]])

    check_iterations = 0
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
        check_iterations += 1
    print()
    print()

    cameras.StartGrabbing()

    left_img = None
    right_img = None

    # Grab c_countOfImagesToGrab from the cameras.
    while True:
        if not cameras.IsGrabbing():
            break

        grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        cameraContextValue = grabResult.GetCameraContext()
        timestamp = grabResult.GetTimeStamp()

        img = grabResult.GetArray()
        if img is not None:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            if left_right_serial[camera_serials[cameraContextValue]] == "left":
                left_img = img
            elif left_right_serial[camera_serials[cameraContextValue]] == "right":
                right_img = img
            
            if left_img is not None and right_img is not None:

                # Let's Rectify
                _, _, output_path = read_calibration_data(delete_calibration_couples=False)

                stereo_maps = np.load(os.path.join(output_path, 'stereo.npy'), allow_pickle=True).item()

                # Access the individual maps
                left_map1 = stereo_maps['left_map1']
                left_map2 = stereo_maps['left_map2'] 
                right_map1 = stereo_maps['right_map1']
                right_map2 = stereo_maps['right_map2']

                # Run the test
                left_rectified, right_rectified = test_rectification(left_img, right_img, left_map1, left_map2, right_map1, right_map2, output_path)

                print(left_rectified.shape)

                left_img = None
                right_img = None


if __name__ == "__main__":
    main()
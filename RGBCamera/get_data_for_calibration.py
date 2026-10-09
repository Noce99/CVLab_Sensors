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

from setup_stereo_cameras import setup_stereo_cameras


def main():

    if not os.path.exists("./calibration_data"):
        os.mkdir("./calibration_data")

    current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    current_folder = os.path.join("./calibration_data", current_time)
    os.mkdir(current_folder)

    left_folder = os.path.join(current_folder, "left")
    right_folder = os.path.join(current_folder, "right")

    os.mkdir(left_folder)
    os.mkdir(right_folder)

    folder_path = {
        "40663113": left_folder,
        "40665216": right_folder,
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
        
        setup_stereo_cameras(camera)

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

    # Grab c_countOfImagesToGrab from the cameras.
    for i in range(countOfImagesToGrab):
        if not cameras.IsGrabbing():
            break

        grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        cameraContextValue = grabResult.GetCameraContext()
        timestamp = grabResult.GetTimeStamp()

        img = grabResult.GetArray()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        path_to_save = os.path.join(folder_path[camera_serials[cameraContextValue]], f"{timestamp}.png")
        
        print(path_to_save)

        cv2.imwrite(path_to_save, img)

    sys.exit(0)


if __name__ == "__main__":
    main()
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

from setup_stereo_cameras import setup_stereo_cameras

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
for i, camera in enumerate(cameras):
    camera.Attach(tlFactory.CreateDevice(devices[i]))
    camera.Open()

    # Print the serial of the camera.
    print("Using device ", camera.GetDeviceInfo().GetSerialNumber())
    
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

    print(f"Camera {cameraContextValue} : {grabResult.GrabSucceeded()}")
    img = grabResult.GetArray()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(f"{cameraContextValue}-{timestamp}.png", img)

# Comment the following two lines to disable waiting on exit.
sys.exit(0)
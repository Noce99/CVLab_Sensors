"""
There I will take data from the cameras and I will save them on the disk!
"""
from pypylon import pylon
from datetime import datetime, timezone
import threading
import time
import sys


def save_data_from_a_camera(camera_serial, path_to_save):
    tlf = pylon.TlFactory.GetInstance()
    camera = None
    while camera is None:
        print(f"Searching for serial: {camera_serial}")
        devices = tlf.EnumerateDevices()
        for d in devices:
            if d.GetSerialNumber() == camera_serial:
                camera = pylon.InstantCamera(tlf.CreateDevice(d))
    print(f"Found out serial: {camera_serial}")
    camera.Open()

    # First let's enable PTP
    camera.PtpEnable.Value = False
    camera.BslPtpPriority1.Value = 255
    # Enable end-to-end delay measurement
    camera.BslPtpProfile.Value = "DelayRequestResponseDefaultProfile"
    # Set the network mode to unicast
    camera.BslPtpNetworkMode.Value = "Multicast"
    # Enable PTP Management Protocol
    camera.BslPtpManagementEnable.Value = True
    # Disable two-step operation
    camera.BslPtpTwoStep.Value = False
    # ** Enable PTP on the current device **
    camera.PtpEnable.Value = True

    # Let's setup FPS
    camera.AcquisitionFrameRate.Value = 20

    to_print = ""
    check_iterations = 0
    while camera.PtpServoStatus.Value != "Locked" or check_iterations < 5:
        camera.PtpDataSetLatch.Execute()
        old_len = len(to_print)
        to_print = f"{camera.PtpStatus.Value} [{camera.PtpServoStatus.Value}] [{camera.PtpOffsetFromMaster.Value}]"
        print('\b' * old_len + " " * old_len, end="")
        print('\b' * old_len + to_print, end="")
        sys.stdout.flush()
        time.sleep(0.1)
        check_iterations += 1
    print()
    print()

    camera.StartGrabbing()
    img = pylon.PylonImage()
    while True:
        with camera.RetrieveResult(2000) as result:

            # Calling AttachGrabResultBuffer creates another reference to the
            # grab result buffer. This prevents the buffer's reuse for grabbing.
            img.AttachGrabResultBuffer(result)

            filename = f"{result.GetTimeStamp()}_{camera_serial}.png"
            img.Save(pylon.ImageFileFormat_Png, filename)
            camera.PtpDataSetLatch.Execute()
            print(f"{camera_serial} - [{result.GetID()}] {camera.PtpStatus.Value} [{camera.PtpServoStatus.Value}] [{camera.PtpOffsetFromMaster.Value}]")

            # In order to make it possible to reuse the grab result for grabbing
            # again, we have to release the image (effectively emptying the
            # image object).
            img.Release()




if __name__ == "__main__":
    save_data_from_a_camera("40663113", "/home/ubm/Downloads")
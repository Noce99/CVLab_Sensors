
from pypylon import pylon
import socket
import os, sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RGBCamera.setup_stereo_cameras import setup_stereo_cameras

# 192.168.1.70 -> 192.168.1.52
# 192.168.1.71 -> 192.168.1.53

tlf = pylon.TlFactory.GetInstance()
tl = tlf.CreateTl('BaslerGigE')
tlf.EnumerateDevices()

cam_info = tl.CreateDeviceInfo()
cam_info.SetIpAddress('192.168.1.71')

cam = pylon.InstantCamera(tlf.CreateDevice(cam_info))

cam.Open()

setup_stereo_cameras(cam)


print(cam.StreamGrabber.TransmissionType.Value)
cam.StreamGrabber.TransmissionType.Value ="Multicast"
cam.StreamGrabber.DestinationAddr.Value ="192.168.1.53"
print(cam.StreamGrabber.DestinationAddr.Value)
print(cam.StreamGrabber.DestinationPort.Value)

# cam.StreamGrabber.DestinationAddr.Value = "192.168.1.52"
# cam.StreamGrabber.DestinationPort.Value = PORT

cam.StartGrabbing()

print(cam.StreamGrabber.TransmissionType.Value)
print(cam.StreamGrabber.DestinationAddr.Value)
print(cam.StreamGrabber.DestinationPort.Value)



while True:
    grabResult = cam.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException) # 5000

    if grabResult.GrabSucceeded():
        print("YEA")
        """timestamp = grabResult.GetTimeStamp()
        path_to_save = os.path.join(path_left, f"{int(timestamp / 1e3)}.png")
        rgb_image = cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_BGR2RGB)
        cv2.imwrite(path_to_save, rgb_image)"""
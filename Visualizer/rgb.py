import threading
import time
import pygame
import cv2
import numpy as np
from pypylon import genicam
from pypylon import pylon
from utils import Section, resize_with_aspect_ratio
from RGBCamera.setup_stereo_cameras import setup_stereo_cameras


LEFT_SERIAL = "40663113"
RIGHT_SERIAL = "40665216"

class RGBSection(Section):
    def __init__(self, name, window):
        super().__init__(name, window)
        self.window = window
        self.font = pygame.font.Font("./assets/UbuntuMono-R.ttf", 32)
        self.small_font = pygame.font.Font("./assets/UbuntuMono-R.ttf", 20)

        self.info = {
            "state": "Inizializing Cameras",
            "left_camera_found": False,
            "right_camera_found": False,
            "left_good_count": 0,
            "left_bad_count": 0,
            "left_ptp_status": None,
            "left_ptp_slave_status": None,
            "left_ptp_offset": None,
            "left_last_image": None,
            "left_FPS": 0.,
            "right_good_count": 0,
            "right_bad_count": 0,
            "right_ptp_status": None,
            "right_ptp_slave_status": None,
            "right_ptp_offset": None,
            "right_last_image": None,
            "right_FPS": 0.,
        }
        self.cameras = pylon.InstantCameraArray(2) # 0 -> left, 1 --> right
        self.camera_thread = \
            threading.Thread(
                target=RGBSection.camera_handler,
                args = (self.cameras, self.info),
                daemon=True,
            )
        self.camera_thread.start()        

    @staticmethod
    def camera_handler(cameras, info):
        print("Starting camera initialization!")

        tlf = pylon.TlFactory.GetInstance()
        tl = tlf.CreateTl('BaslerGigE')
        tlf.EnumerateDevices()

        cam_info_left = tl.CreateDeviceInfo()
        cam_info_left.SetIpAddress('192.168.1.70')

        cam_info_right = tl.CreateDeviceInfo()
        cam_info_right.SetIpAddress('192.168.1.71')

        cameras[0].Attach(tlf.CreateDevice(cam_info_left))
        cameras[0].Open()
        cameras[0].StreamGrabber.TransmissionType.Value ="Multicast"
        cameras[0].StreamGrabber.DestinationAddr.Value ="192.168.1.52"
        cameras[0].StreamGrabber.DestinationPort.Value =49152
        setup_stereo_cameras(cameras[0], False)

        cameras[1].Attach(tlf.CreateDevice(cam_info_right))
        cameras[1].Open()
        cameras[1].StreamGrabber.TransmissionType.Value ="Multicast"
        cameras[1].StreamGrabber.DestinationAddr.Value ="192.168.1.53"
        cameras[1].StreamGrabber.DestinationPort.Value =49153
        setup_stereo_cameras(cameras[1], True)
        
        info["state"] = "Receiving Data"

        cameras.StartGrabbing()

        start = time.time()

        how_many_images_between_ptp_updates = 50

        while True:
            grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            cameraContextValue = grabResult.GetCameraContext()

            if cameraContextValue == 0:
                camera_to_select = "left_last_image"
                if grabResult.GrabSucceeded():
                    info["left_good_count"] += 1
                else:
                    info["left_bad_count"] += 1
                if info["left_good_count"] % how_many_images_between_ptp_updates == 0:
                    try:
                        cameras[0].PtpDataSetLatch.Execute()
                        info["left_ptp_status"] = cameras[0].PtpStatus.Value
                        info["left_ptp_slave_status"] = cameras[0].PtpServoStatus.Value
                        info["left_ptp_offset"] = cameras[0].PtpOffsetFromMaster.Value
                    except Exception as e:
                        print("Camera 0 error!")
                        print(e)
            else:
                camera_to_select = "right_last_image"
                if grabResult.GrabSucceeded():
                    info["right_good_count"] += 1
                else:
                    info["right_bad_count"] += 1
                if info["right_good_count"] % how_many_images_between_ptp_updates == 0:
                    try:
                        cameras[cameraContextValue].PtpDataSetLatch.Execute()
                        info["right_ptp_status"] = cameras[cameraContextValue].PtpStatus.Value
                        info["right_ptp_slave_status"] = cameras[cameraContextValue].PtpServoStatus.Value
                        info["right_ptp_offset"] = cameras[cameraContextValue].PtpOffsetFromMaster.Value
                    except Exception as e:
                        print(f"Camera {cameraContextValue} error!")
                        print(e)
            if grabResult.GrabSucceeded():
                info[camera_to_select] = grabResult.GetArray()

            now = time.time()
            info["left_FPS"] = info["left_good_count"] / (now-start)
            info["right_FPS"] = info["right_good_count"] / (now-start)

    def draw(self):
        if self.info["state"] == "Inizializing Cameras":
            to_show = f"Left: {self.info['left_camera_found']} Right: {self.info['right_camera_found']}"
            text_surface = self.font.render(
                    to_show,                   # Text
                    True,                      # Antialias
                    (0, 255, 0),               # Text Color
                )
            text_rect = text_surface.get_rect()
            text_rect.centerx = self.window.get_width() // 2
            text_rect.centery = self.window.get_height() // 2

            self.window.blit(text_surface, text_rect)
        elif self.info["state"] == "Receiving Data":
            # Let's draw the figures

            image_width = int(self.window.get_width() * 0.9 // 2)
            if self.info["left_last_image"] is not None:
                img, w, h = resize_with_aspect_ratio(self.info["left_last_image"], width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() / 4)
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)
            if self.info["right_last_image"] is not None:
                img, w, h = resize_with_aspect_ratio(self.info["right_last_image"], width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() * 3 / 4)
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)

            # Let's draw the logs

            left_camera_x = self.window.get_width() // 4
            
            left_lines = [
                f"PTP {self.info['left_ptp_status']} [{self.info['left_ptp_slave_status']}]",
                f"PTP Offset {self.info['left_ptp_offset']}",
                f"Ok Frame: {self.info['left_good_count']}, Bad Frame: {self.info['left_bad_count']}",
                f"FPS: {self.info['left_FPS']:.2f}"
            ]

            for i, line in enumerate(left_lines):
                text_surface = self.small_font.render(
                    line,                   # Text
                    True,                      # Antialias
                    (255, 0, 0),               # Text Color
                )
                text_rect = text_surface.get_rect()
                text_rect.centerx = left_camera_x
                text_rect.centery = self.window.get_height() - (i+1) * (self.window.get_height() * 0.3 / (len(left_lines)+1))

                self.window.blit(text_surface, text_rect)

            right_camera_x = self.window.get_width() // 4 * 3
            
            right_lines = [
                f"PTP {self.info['right_ptp_status']} [{self.info['right_ptp_slave_status']}]",
                f"PTP Offset {self.info['right_ptp_offset']}",
                f"Ok Frame: {self.info['right_good_count']}, Bad Frame: {self.info['right_bad_count']}",
                f"FPS: {self.info['right_FPS']:.2f}"
            ]

            for i, line in enumerate(right_lines):
                text_surface = self.small_font.render(
                    line,                      # Text
                    True,                      # Antialias
                    (255, 0, 0),               # Text Color
                )
                text_rect = text_surface.get_rect()
                text_rect.centerx = right_camera_x
                text_rect.centery = self.window.get_height() - (i+1) * (self.window.get_height() * 0.3 / (len(right_lines)+1))

                self.window.blit(text_surface, text_rect)
            



        
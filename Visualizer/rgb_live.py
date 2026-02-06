import threading
import os
import time
import pygame
import cv2
import numpy as np
from datetime import datetime

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

        # Setup path
        if not os.path.exists("./calibration_data"):
            os.mkdir("./calibration_data")

        current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        current_folder = os.path.join("./calibration_data", current_time)
        os.mkdir(current_folder)

        self.left_folder = os.path.join(current_folder, "left")
        self.right_folder = os.path.join(current_folder, "right")
        os.mkdir(self.left_folder)
        os.mkdir(self.right_folder)  

        self.output_path = os.path.join("./calibration_data", current_time, "live_calibration_couples")
        os.mkdir(self.output_path)

        self.chess_i = 0

        self.left_dots_backet = []
        self.right_dots_backet = []

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
        setup_stereo_cameras(cameras[0], False)
        cameras[0].StreamGrabber.TransmissionType.Value ="Multicast"
        cameras[0].StreamGrabber.DestinationAddr.Value ="192.168.1.52"

        cameras[1].Attach(tlf.CreateDevice(cam_info_right))
        cameras[1].Open()
        setup_stereo_cameras(cameras[1], True)
        cameras[1].StreamGrabber.TransmissionType.Value ="Multicast"
        cameras[1].StreamGrabber.DestinationAddr.Value ="192.168.1.53"

        info["state"] = "Receiving Data"

        cameras.StartGrabbing()

        start = time.time()

        how_many_images_between_ptp_updates = 50

        while True:
            grabResult = cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

            cameraContextValue = grabResult.GetCameraContext()

            if cameraContextValue == 0:
                print("LEFT")
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
                print("RIGHT")
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
                #---img originale sinistra
                img = np.copy(self.info["left_last_image"])
                for x, y in self.left_dots_backet:
                    cv2.circle(img, (x,y), 10, (0,0,255), -1)
                img, w, h = resize_with_aspect_ratio(img, width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() / 4)#va modificato la larghezza 
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)

            if self.info["right_last_image"] is not None:
                #---img originale destra
                img = np.copy(self.info["right_last_image"])
                for x, y in self.right_dots_backet:
                    cv2.circle(img, (x,y), 10, (0,0,255), -1)
                img, w, h = resize_with_aspect_ratio(img, width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() * 3 / 4)
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)
            
            PATTERN_SIZE = (9, 6)     
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0001)
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS
            
            left_img = self.info["left_last_image"]
            right_img = self.info["right_last_image"]
            
            if left_img is not None and right_img is not None:

                left_img = cv2.cvtColor(left_img, cv2.COLOR_BGR2RGB)
                right_img = cv2.cvtColor(right_img, cv2.COLOR_BGR2RGB)

                left_img_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
                right_img_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

                res_left, corners_left = cv2.findChessboardCorners(left_img_gray, PATTERN_SIZE, flags=flags)
                res_right, corners_right = cv2.findChessboardCorners(right_img_gray, PATTERN_SIZE, flags=flags)

                if res_left and res_right:
                    corners_left_fine = cv2.cornerSubPix(left_img_gray, corners_left, (11,11), (-1,-1), criteria)
                    corners_right_fine = cv2.cornerSubPix(right_img_gray, corners_right, (11,11), (-1,-1), criteria)

                    for i in range(corners_left_fine.shape[0]):
                        self.left_dots_backet.append((int(corners_left_fine[i, 0, 0]), int(corners_left_fine[i, 0, 1])))
                    
                    for i in range(corners_right_fine.shape[0]):
                        self.right_dots_backet.append((int(corners_right_fine[i, 0, 0]), int(corners_right_fine[i, 0, 1])))
                    

                    cv2.imwrite(os.path.join(self.left_folder, f"{self.chess_i}.png"), left_img)
                    cv2.imwrite(os.path.join(self.right_folder, f"{self.chess_i}.png"), right_img)

                    self.chess_i += 1

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
            



        
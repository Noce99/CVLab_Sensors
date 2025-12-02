import threading
import os
import time
import pygame
import cv2
import numpy as np
from pypylon import genicam
from pypylon import pylon
from utils import Section, resize_with_aspect_ratio
from RGBCamera.setup_cameras import setup_camera


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
        tlFactory = pylon.TlFactory.GetInstance()
        while not info["left_camera_found"] or \
              not info["right_camera_found"]:
            devices = tlFactory.EnumerateDevices()
            for d in devices:
                if d.GetSerialNumber() == LEFT_SERIAL:
                    cameras[0].Attach(tlFactory.CreateDevice(d))
                    cameras[0].Open()
                    setup_camera(cameras[0], False)
                    info["left_camera_found"] = True
                elif d.GetSerialNumber() == RIGHT_SERIAL:
                    cameras[1].Attach(tlFactory.CreateDevice(d))
                    cameras[1].Open()
                    setup_camera(cameras[1], True)
                    info["right_camera_found"] = True
                else:
                    print(f"Uknown camera with serial: {d.GetSerialNumber()}")
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
                    except:
                        print("Camera 0 error!")
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
                #---img originale sinistra
                img, w, h = resize_with_aspect_ratio(self.info["left_last_image"], width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() / 4)#va modificato la larghezza 
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)
            if self.info["right_last_image"] is not None:
                #---img originale destra
                img, w, h = resize_with_aspect_ratio(self.info["right_last_image"], width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() * 3 / 4)
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)
            
            #ho disegnato le immagini pulite








            PATTERN_SIZE = (9, 6)     
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.0001)
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS
            
            #percorso salvataggio img
            calibration_data_folder_path = "./calibration_data"
            data_folder = os.listdir(calibration_data_folder_path)
            data_folder.sort()
            output_path = os.path.join("./calibration_data", data_folder[-1], "live_calibration_couples")
            os.mkdir(output_path)

            chess_i = 0

            all_corners_left = []
            all_corners_right = []

            self.last_pair_good = False
            
            left_img = self.info["left_last_image"] 
            right_img = self.info["right_last_image"]
            
            img_size = None
            if img_size is None:
               img_size = (left_img.shape[1], left_img.shape[0])

            if left_img is not None and right_img is not None:

                left_img_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
                right_img_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

                res_left, corners_left = cv2.findChessboardCorners(left_img_gray, PATTERN_SIZE, flags=flags)
                res_right, corners_right = cv2.findChessboardCorners(right_img_gray, PATTERN_SIZE, flags=flags)

                #salvo dati per il render
                self.info["return_left"]=res_left
                self.info["return_left"]=res_right
                self.info["corners_left"]=corners_left
                self.info["corners_right"]=corners_right

                if res_left and res_right:
                    corners_left_fine = cv2.cornerSubPix(left_img_gray, corners_left, (11,11), (-1,-1), criteria)
                    corners_right_fine = cv2.cornerSubPix(right_img_gray, corners_right, (11,11), (-1,-1), criteria)

                    #salvo le immagini pulite da usare per fare i corner
                    left_img_corners = left_img.copy()
                    right_img_corners = right_img.copy()


                    #disegno le immagini calibrate

                    image_width = int(self.window.get_width() * 0.9 // 2)
                    if self.info["left_last_image"] is not None:
                        #---img calibrata sinistra (ho solo inserito drawCorners)

                        img_cal, w_cal, h_cal = resize_with_aspect_ratio(cv2.drawChessboardCorners(left_img_corners, PATTERN_SIZE, corners_left_fine, res_left), width=image_width)
                        if img_cal.ndim == 2:
                            img_cal = np.stack([img_cal] * 3, axis=-1)
                        if self.last_pair_good:
                            pygame.draw.rect(self.window, (0,255,0), img, 4)
                        surfarray_cal = pygame.surfarray.make_surface(np.transpose(img_cal, (1, 0, 2)))
                        rect_cal = pygame.Rect(0, 0, w_cal, h_cal)
                        rect_cal.centerx = int(self.window.get_width() / 4)#va modificato la larghezza 
                        rect_cal.centery = int(self.window.get_height() * 0.15 + h/2)
                        self.window.blit(surfarray_cal, rect_cal)

                    if self.info["right_last_image"] is not None:
                        #---img calibrata destra (ho solo inserito drawCorners)

                        img_cal, w_cal, h_cal = resize_with_aspect_ratio(cv2.drawChessboardCorners(right_img_corners, PATTERN_SIZE, corners_right_fine, res_right), width=image_width)
                        if img_cal.ndim == 2:
                            img_cal = np.stack([img_cal] * 3, axis=-1)
                        if self.last_pair_good:
                            pygame.draw.rect(self.window, (0,255,0), img, 4)
                        surfarray_cal = pygame.surfarray.make_surface(np.transpose(img_cal, (1, 0, 2)))
                        rect_cal = pygame.Rect(0, 0, w_cal, h_cal)
                        rect_cal.centerx = int(self.window.get_width() / 4)#va modificato la larghezza 
                        rect_cal.centery = int(self.window.get_height() * 0.15 + h/2)
                        self.window.blit(surfarray_cal, rect_cal)

                    #ho disegneto live le due img
                    #Ora termino il salvataggio di tali img su disco


                    cv2.imwrite(os.path.join(output_path, f"{chess_i}_left.png"), left_img_corners)
                    cv2.imwrite(os.path.join(output_path, f"{chess_i}_right.png"), right_img_corners)

                    chess_i += 1

                    all_corners_left.append(corners_left_fine)
                    all_corners_right.append(corners_right_fine)

        
            """# Now in corners we have the corners from the left and right images
            assert len(all_corners_left) == len(all_corners_right)

            pattern_points = np.zeros((np.prod(PATTERN_SIZE), 3), np.float32)
            pattern_points[:, :2] = np.indices(PATTERN_SIZE).T.reshape(-1, 2)
            pattern_points = [pattern_points] * len(all_corners_left)

            err, Kl, Dl, Kr, Dr, R, T, E, F = cv2.stereoCalibrate(
                pattern_points, all_corners_left, all_corners_right, None, None, None, None, img_size, flags=0)

            print(f"T = {T}")
            print(f"R = {R}")

            # After stereo calibration, compute rectification maps
            R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(
                Kl, Dl, Kr, Dr, img_size, R, T, alpha=0.9
            )

            # Compute rectification maps
            left_map1, left_map2 = cv2.initUndistortRectifyMap(
                Kl, Dl, R1, P1, img_size, cv2.CV_16SC2
            )
            right_map1, right_map2 = cv2.initUndistortRectifyMap(
                Kr, Dr, R2, P2, img_size, cv2.CV_16SC2
            )

            np.save(os.path.join(output_path, 'stereo.npy'), {'left_map1': left_map1, 'left_map2': left_map2, 'right_map1': right_map1, 'right_map2': right_map2})"""















            

            
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
            



        
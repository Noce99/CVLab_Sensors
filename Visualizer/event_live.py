import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide pygame message"
import pygame
import copy
import threading
import numpy as np
import cv2
import time
from datetime import datetime
from metavision_sdk_core import PeriodicFrameGenerationAlgorithm, ColorPalette
from metavision_core.event_io.raw_reader import initiate_device
from metavision_core.event_io import EventsIterator
from utils import Section, resize_with_aspect_ratio




LEFT_SERIAL = "00051962"
RIGHT_SERIAL = "00051961"
COLOR_PALETTE = ColorPalette.Dark
# CoolWarm, Dark, Gray, Light

class EventSection(Section):
    def __init__(self, name, window):
        super().__init__(name, window)
        self.window = window
        self.font = pygame.font.Font("./assets/UbuntuMono-R.ttf", 32)
        self.small_font = pygame.font.Font("./assets/UbuntuMono-R.ttf", 20)

        self.left_info = {
            "state": "Inizializing Camera",
            "camera_found": False,
            "last_image": None,
            "last_time_step": 0,
            "FPS": 0.,
        }

        self.right_info = copy.deepcopy(self.left_info)

         # Setup path
        if not os.path.exists("./calibration_data_events"):
            os.mkdir("./calibration_data_events")

        current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        current_folder = os.path.join("./calibration_data_events", current_time)
        os.mkdir(current_folder)

        self.left_folder = os.path.join(current_folder, "left")
        self.right_folder = os.path.join(current_folder, "right")
        os.mkdir(self.left_folder)
        os.mkdir(self.right_folder)  

        self.output_path = os.path.join("./calibration_data_events", current_time, "live_calibration_couples")
        os.mkdir(self.output_path)

        self.chess_i = 0

        self.left_dots_backet = []
        self.right_dots_backet = []

        self.left_camera_thread = \
            threading.Thread(
                target =    EventSection.camera_handler,
                args =      (LEFT_SERIAL, COLOR_PALETTE, self.left_info),
                daemon =    True,
            )
        self.right_camera_thread = \
            threading.Thread(
                target =    EventSection.camera_handler,
                args =      (RIGHT_SERIAL, COLOR_PALETTE, self.right_info),
                daemon =    True,
            )

        self.left_camera_thread.start()
        self.right_camera_thread.start()  


    @staticmethod
    def camera_handler(serial, color_palette, info):

        camera = initiate_device(path=serial)
        iterator = EventsIterator.from_device(camera, delta_t=100000, mode="mixed", relative_timestamps=False)
        generator = \
            PeriodicFrameGenerationAlgorithm(
                sensor_width =      iterator.get_size()[1],
                sensor_height =     iterator.get_size()[0],
                palette =           color_palette
            )

        def show(ts, frame):
            info["last_time_step"] = ts
            info["last_image"] = frame

        generator.set_output_callback(show)

        info["state"] = "Receiving Data"

    
        for evs in iterator:
            try:
                generator.process_events(evs)
            except:
                pass

    def process_image(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        image[image<40] = 0
        image[image>=40] = 255

        image = cv2.GaussianBlur(image, (15, 15), 0)

        # new_height, new_width = image.shape[0] // 2, image.shape[1] // 2
        # image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_AREA)
        return image

    def draw(self):
        start = time.time()
        if self.left_info["state"] == "Inizializing Camera" or \
                self.right_info["state"] == "Inizializing Camera":
            to_show = f"Left: {self.left_info['state'] == 'Inizializing Camera'} \
                Right: {self.right_info['state'] == 'Inizializing Camera'}"
            text_surface = self.font.render(
                    to_show,                   # Text
                    True,                      # Antialias
                    (0, 255, 0),               # Text Color
                )
            text_rect = text_surface.get_rect()
            text_rect.centerx = self.window.get_width() // 2
            text_rect.centery = self.window.get_height() // 2

            self.window.blit(text_surface, text_rect)
        elif self.left_info["state"] == "Receiving Data" and \
                self.right_info["state"] == "Receiving Data":
            
            image_width = int(self.window.get_width() * 0.9 // 2)
            if self.left_info["last_image"] is not None:
                img = np.copy(self.left_info["last_image"])
                for x, y in self.left_dots_backet:
                    cv2.circle(img, (x,y), 10, (0,0,255), -1)
                img, w, h = resize_with_aspect_ratio(img, width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                else:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() / 4)
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)

                text_surface = self.font.render(
                    f"{self.left_info["last_time_step"]}",  # Text
                    True,                                   # Antialias
                    (0, 255, 0),                            # Text Color
                )
                text_rect = text_surface.get_rect()
                text_rect.centerx = self.window.get_width() // 4
                text_rect.centery = int(self.window.get_height() * 0.9)
                self.window.blit(text_surface, text_rect)

            if self.right_info["last_image"] is not None:
                img = np.copy(self.right_info["last_image"])
                for x, y in self.right_dots_backet:
                    cv2.circle(img, (x,y), 10, (0,0,255), -1)
                img, w, h = resize_with_aspect_ratio(img, width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                else:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() * 3 / 4)
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)

                text_surface = self.font.render(
                    f"{self.right_info["last_time_step"]}",  # Text
                    True,                                    # Antialias
                    (0, 255, 0),                             # Text Color
                )
                text_rect = text_surface.get_rect()
                text_rect.centerx = self.window.get_width() // 4 * 3
                text_rect.centery = int(self.window.get_height() * 0.9)
                self.window.blit(text_surface, text_rect)


                text_surface = self.font.render(
                    f"{abs(self.right_info["last_time_step"] - self.left_info["last_time_step"]) / 10**6:.2f} s",  # Text
                    True,                                    # Antialias
                    (255, 0, 0),                             # Text Color
                )
                text_rect = text_surface.get_rect()
                text_rect.centerx = self.window.get_width() // 2
                text_rect.centery = int(self.window.get_height() * 0.7)
                self.window.blit(text_surface, text_rect)




            PATTERN_SIZE = (9, 6)     
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
            flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS
            
            left_img = self.left_info["last_image"]
            right_img = self.right_info["last_image"]
              
            
            if left_img is not None and right_img is not None:

                left_img_gray = self.process_image(left_img)


                res_left, corners_left = cv2.findChessboardCorners(left_img_gray, PATTERN_SIZE)#è stato tolto flags=flags

                if res_left:
                    right_img_gray = self.process_image(right_img)

                    res_right, corners_right = cv2.findChessboardCorners(right_img_gray, PATTERN_SIZE)
                else:
                    res_right = False

                # print(f"Left = {res_left} Right = {res_right} [{time.time()}]")

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
            else:
                print(f"No images! {time.time()} {left_img is None} {right_img is None}")

        print(time.time()-start)
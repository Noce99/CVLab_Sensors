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
        iterator = EventsIterator.from_device(camera)
        generator = \
            PeriodicFrameGenerationAlgorithm(
                sensor_width =      iterator.get_size()[1],
                sensor_height =     iterator.get_size()[0],
                fps =               25,
                palette =           color_palette
            )

        def show(ts, frame):
            info["last_time_step"] = ts
            info["last_image"] = frame

        generator.set_output_callback(show)

        info["state"] = "Receiving Data"

        for evs in iterator:
            generator.process_events(evs)

    def draw(self):
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
                img, w, h = resize_with_aspect_ratio(self.left_info["last_image"], width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                else:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() / 4)
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)
            if self.right_info["last_image"] is not None:
                img, w, h = resize_with_aspect_ratio(self.right_info["last_image"], width=image_width)
                if img.ndim == 2:
                    img = np.stack([img] * 3, axis=-1)
                else:
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                surfarray = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
                rect = pygame.Rect(0, 0, w, h)
                rect.centerx = int(self.window.get_width() * 3 / 4)
                rect.centery = int(self.window.get_height() * 0.15 + h/2)
                self.window.blit(surfarray, rect)


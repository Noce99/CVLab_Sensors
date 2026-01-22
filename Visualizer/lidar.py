from multiprocessing import shared_memory
import numpy as np
import struct
import time
import pygame
from utils import Section
import cv2

DEPTH_SHM_NAME = "DepthMap"
REFLECTION_SHM_NAME = "ReflectionMap"


class LidarSection(Section):
    def __init__(self, name, window):
        super().__init__(name, window)
        for _ in range(10):
            try:
                self.depth_shm = shared_memory.SharedMemory(name=DEPTH_SHM_NAME)
                self.reflection = shared_memory.SharedMemory(name=REFLECTION_SHM_NAME)
                break
            except FileNotFoundError:
                print("Waiting for C++ writer...")
                time.sleep(1)
        else:
            print("Shared memory not found.")
            exit(1)
        header_format = "iii"
        self.header_size = struct.calcsize(header_format)

        header_bytes = self.depth_shm.buf[:self.header_size]
        self.rows, self.cols, self.channels = struct.unpack(header_format, header_bytes)
        self.img_size = self.rows * self.cols * self.channels
        print(f"Shared image found: {self.rows}x{self.cols}x{self.channels}")

        self.depth_or_intensity = "intensity"

    
    def draw(self):
        rect = pygame.Rect(0, 0, self.cols, self.rows)
        rect.centerx = int(self.window.get_width() // 2)
        rect.centery = int(self.window.get_height() // 2)

        if self.depth_or_intensity == "depth":
            depth_img_bytes = self.depth_shm.buf[self.header_size:self.header_size + self.img_size]
            depth_img = np.frombuffer(depth_img_bytes, dtype=np.uint8).reshape((self.rows, self.cols, self.channels))
            depth_img = cv2.applyColorMap(depth_img, cv2.COLORMAP_MAGMA)
            depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2RGB)
            depth_surfarray = pygame.surfarray.make_surface(np.transpose(depth_img, (1, 0, 2)))
            self.window.blit(depth_surfarray, rect)
        else:
            reflection_img_bytes = self.reflection.buf[self.header_size:self.header_size + self.img_size]
            reflection_img = np.frombuffer(reflection_img_bytes, dtype=np.uint8).reshape((self.rows, self.cols, self.channels))
            reflection_img = cv2.applyColorMap(reflection_img, cv2.COLORMAP_MAGMA)
            reflection_img = cv2.cvtColor(reflection_img, cv2.COLOR_BGR2RGB)
            reflection_surfarray = pygame.surfarray.make_surface(np.transpose(reflection_img, (1, 0, 2)))
            self.window.blit(reflection_surfarray, rect)
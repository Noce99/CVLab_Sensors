import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide pygame message"
import pygame
from utils import Section
from home import HomeSection
from rgb import RGBSection
from event import EventSection
from lidar import LidarSection

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


WINDOW_WIDTH = 600
WINDOW_HEIGH = 600

class Visualizer:
    def __init__(self):
            # PyGame Initialization
            pygame.init()
            self.window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGH))
            self.fullscreen = True
            pygame.display.set_caption("Sensors Visualizer")
            self.font = pygame.font.Font("./assets/UbuntuMono-R.ttf", 32)
            self.FPS = 30
            self.clock = pygame.time.Clock()

            # Visualizer Sections
            self.active_section = 0 # Index of the following listt that is the current section
            self.all_sections = [
                HomeSection("Home", self.window),
                RGBSection("RGBs", self.window),
                EventSection("Events", self.window),
                LidarSection("Lidars", self.window)
            ]
            self.section_height_ration = 1./10.
            self.section_text_color = (255, 0, 0)
            self.section_background_color = (0, 255, 0)
            self.section_selected_background_color = (0, 125, 125)

            # Clickable Rectangles
            self.click_rects = []

    def draw_frame(self):
        # First of all let's clean the screen
        self.window.fill((0, 0, 0))
        # and the clickable rects list
        self.click_rects = []

        # Change section part
        self.draw_sections()


        
        pygame.display.flip()

    def draw_sections(self):
        # First we draw the section selector
        each_section_width = self.window.get_width() // len(self.all_sections)
        each_section_height = self.window.get_height() * self.section_height_ration

        for i, section in enumerate(self.all_sections):
            text_surface = self.font.render(
                section.name,                        # Text
                True,                           # Antialias
                self.section_text_color,        # Text Color
            )
            text_rect = text_surface.get_rect()
            text_rect.centerx = each_section_width * i + each_section_width // 2
            text_rect.centery = each_section_height // 2

            background_rect = pygame.Rect(0, 0, each_section_width, each_section_height)
            background_rect.centerx = text_rect.centerx
            background_rect.centery = text_rect.centery

            if self.active_section == i:
                background_color = self.section_selected_background_color
            else:
                background_color = self.section_background_color

            pygame.draw.rect(
                self.window,                    # Surface
                background_color,               # Color
                background_rect,                # Rect
                0,                              # Width
                20,                             # Border Radius
            )
            self.window.blit(text_surface, text_rect)

            # Let's add the clickable rectangle
            self.click_rects.append((background_rect, lambda i=i: self.change_section(i)))

            if self.active_section == i:
                # Let's draw the section
                section.draw()
    
    def change_section(self, new_section):
        self.active_section = new_section

if __name__ == "__main__":
    v = Visualizer()
    while True:
        events = pygame.event.get()
        for event in events:
            if event.type == pygame.QUIT:
                pygame.quit()
            elif event.type == pygame.KEYUP:
                if event.key == 27:  # ESC
                    quit_event = pygame.event.Event(pygame.QUIT)
                    pygame.event.post(quit_event)
                    exit(1)
                if event.key == 102:  # F
                    if v.fullscreen:
                        v.window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
                    else:
                        v.window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGH))
                    v.fullscreen = not v.fullscreen
                else:
                    print(f"Uknown Key: {event.key}")
            elif event.type == pygame.MOUSEBUTTONUP:
                for rect, f in v.click_rects:
                    if rect.collidepoint(event.pos):
                        f()

        v.draw_frame()
                    
from metavision_sdk_stream import Camera, CameraStreamSlicer
from metavision_core.event_io import EventsIterator
import numpy as np
from metavision_core.event_io.raw_reader import initiate_device
from metavision_core.event_io import EventsIterator
from metavision_sdk_core import PeriodicFrameGenerationAlgorithm, ColorPalette
from metavision_sdk_ui import EventLoop, BaseWindow, MTWindow, UIAction, UIKeyEvent, Window
from metavision_hal import DeviceDiscovery, RawFileConfig

import cv2
import time

# def event_to_h5(event_h5_file_path: str):

#     config = RawFileConfig()
#     config.build_index = False
#     device = DeviceDiscovery.open_raw_file(event_h5_file_path, config)

#     mv_iterator = EventsIterator.from_device(device=device)
#     height, width = mv_iterator.get_size()  # Camera Geometry

#     with Window("Periodic frame generator", width, height, Window.RenderMode.BGR) as window:
#         # Do something whenever a frame is ready
#         def periodic_cb(ts, frame):
#             cv2.putText(frame, "Timestamp: " + str(ts), (0, 10), cv2.FONT_HERSHEY_DUPLEX, 0.5, (0, 255, 0))
#             window.show(frame)
            

#         # Instantiate the frame generator
#         periodic_gen = PeriodicFrameGenerationAlgorithm(width, height, accumulation_time_us=100000, fps=10)
#         periodic_gen.set_output_callback(periodic_cb)

#         start = time.time()
#         for evs in mv_iterator:
#             EventLoop.poll_and_dispatch()  # Dispatch system events to the window
#             periodic_gen.process_events(evs)  # Feed events to the frame generator
#             if window.should_close():
#                 break
#         end = time.time()
#         print(f"Execution time = {end - start} s")

def raw_event_to_images(path_to_raw_file, accumulation_time_us=100000, fps=10):

    config = RawFileConfig()
    config.build_index = False # Used for not creating .tmp_index files.
    device = DeviceDiscovery.open_raw_file(path_to_raw_file, config)

    mv_iterator = EventsIterator.from_device(device=device)
    height, width = mv_iterator.get_size()

    imgs = {}

    def callback(ts, frame):
        imgs[ts] = frame
    
    periodic_gen = PeriodicFrameGenerationAlgorithm(width, height, accumulation_time_us=accumulation_time_us, fps=fps, palette=ColorPalette.CoolWarm)
    periodic_gen.set_output_callback(callback)

    for evs in mv_iterator:
        periodic_gen.process_events(evs)
    
    return imgs


if __name__ == "__main__":
    # event_to_h5("/home/ubm/CVLab_Sensors/CVLab_live_imgs/2026_03_05__10_26_20/event_right/1772702771576321.raw")
    imgs = raw_event_to_images("/home/ubm/CVLab_Sensors/CVLab_live_imgs/2026_03_05__10_26_20/event_right/1772702763170099.raw")
    for ts in imgs:
        cv2.imwrite(f"/home/ubm/Downloads/{ts}.png", imgs[ts])
    
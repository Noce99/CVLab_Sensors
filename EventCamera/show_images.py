from metavision_core.event_io import EventsIterator, LiveReplayEventsIterator, is_live_camera
from metavision_sdk_core import PeriodicFrameGenerationAlgorithm, ColorPalette
from metavision_sdk_ui import EventLoop, BaseWindow, MTWindow, UIAction, UIKeyEvent
from metavision_core.event_io.raw_reader import initiate_device
import argparse
import os
import cv2


def show(ts, cd_frame):
    cd_frame = cd_frame[::2, ::2]
    cv2.imshow("event_frame", cd_frame)
    cv2.waitKey(1)

right_device = initiate_device(path="00051961")
left_device = initiate_device(path="00051962")
print(left_device.get_i_hw_identification().get_serial())
print(right_device.get_i_hw_identification().get_serial())


mv_iterator = EventsIterator.from_device(left_device)
height, width = mv_iterator.get_size()
event_frame_gen = PeriodicFrameGenerationAlgorithm(
    sensor_width=width,
    sensor_height=height, fps=25,
    palette=ColorPalette.Dark
)
event_frame_gen.set_output_callback(show)

for evs in mv_iterator:
    event_frame_gen.process_events(evs)



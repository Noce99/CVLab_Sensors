from metavision_core.event_io import EventsIterator, LiveReplayEventsIterator, is_live_camera
from metavision_sdk_core import PeriodicFrameGenerationAlgorithm, ColorPalette
from metavision_sdk_ui import EventLoop, BaseWindow, MTWindow, UIAction, UIKeyEvent
from metavision_core.event_io.raw_reader import initiate_device
from metavision_sdk_stream import Camera, CameraStreamSlicer
import argparse
import os
import cv2
import sys
import argparse

import time
from datetime import datetime
import traceback

FPS = 20.
ACCUMULATION_MS = 10

class SaveEventImages:
    start_time_us: int

    def __init__(self, start_time_us, save_path):
        self.start_time_us = start_time_us
        self.save_path = save_path

    def save(self, ts, img):
        cv2.imwrite(os.path.join(self.save_path, f"{int(self.start_time_us+ts)}.png"), img)
        # print(f"Delay s: {time.time() - (self.start_time_us+ts) * 10**-6:.4f} s")

def get_data_from_camera(left_camera: bool, path: str):
    if left_camera:
        serial = "00051962"
    else:
        serial = "00051961"
    
    while True:
        try:
            # camera = Camera.from_serial(serial)
            device = initiate_device(path=serial, do_time_shifting=False)
            """if left_camera:
                print(f"DONE LEFT!")
            else:
                print(f"DONE RIGHT!")"""
            break
        except Exception as e:
            print(f"Error initiating Event device {serial}: {e}. Retrying...")
            pass


    counter = 0
    start_wall_time_us = int(time.time() * 1_000_000)

    if device.get_i_events_stream():
        starting_timestamp_us = int(datetime.now().timestamp() * 1_000_000)
        mv_iterator = EventsIterator.from_device(device=device)
        device.get_i_events_stream().log_raw_data(os.path.join(path, f"{starting_timestamp_us}.raw"))

        for evs in mv_iterator:
            try:
                if len(evs) > 0:
                    if counter == 0:
                        device.get_i_events_stream().stop_log_raw_data()
                        starting_timestamp_us = int(start_wall_time_us + evs[0]['t'])
                        device.get_i_events_stream().log_raw_data(os.path.join(path, f"{starting_timestamp_us}.raw"))
                    counter += 1
                    if counter > 20:
                        counter = 0
            except Exception as e:
                traceback.print_exc() 


    """
    if device.get_i_events_stream():
        device.get_i_events_stream().log_raw_data(os.path.join(path, time_folder_name))
    mv_iterator = EventsIterator.from_device(device=device)
    print(f"Starting taking events! {path}")
    counter = 0
    try:
        for evs in mv_iterator:
            counter += 1
            if counter > 20:
                device.get_i_events_stream().stop_log_raw_data()
                now = datetime.now()
                microseconds_from_epoch = int(now.timestamp() * 1_000_000)
                time_folder_name = f"{microseconds_from_epoch}.h5"
                device.get_i_events_stream().log_raw_data(os.path.join(path, time_folder_name))
                counter = 0

    except KeyboardInterrupt:
        pass"""
    

def argparser():
    parser = argparse.ArgumentParser(description='Camera control application')
    
    # Add path argument with existence and directory validation
    parser.add_argument(
        'path',
        type=str,
        help='Where to save images from events. It must be an existing folder!'
    )
    
    # Add boolean camera selection argument
    parser.add_argument(
        '--right-camera',
        action='store_true',
        dest='use_right_camera',
        help='Use right camera (default: use left camera)'
    )
    
    args = parser.parse_args()
    
    # Validate that the path exists and is a directory
    if not os.path.exists(args.path):
        print(f"Error: Path '{args.path}' does not exist", file=sys.stderr)
        sys.exit(1)
    
    if not os.path.isdir(args.path):
        print(f"Error: '{args.path}' is not a directory", file=sys.stderr)
        sys.exit(1)
    
    # Use the arguments
    # print(f"Directory path: {args.path}")
    
    if args.use_right_camera:
        # print("Starting RIGHT camera")
        get_data_from_camera(left_camera=False, path=args.path)
    else:
        # print("Starting LEFT camera")
        get_data_from_camera(left_camera=True, path=args.path)

if __name__ == "__main__":
    argparser()
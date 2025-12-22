from metavision_core.event_io import EventsIterator, LiveReplayEventsIterator, is_live_camera
from metavision_sdk_core import PeriodicFrameGenerationAlgorithm, ColorPalette
from metavision_sdk_core import EventPreprocessor
from metavision_sdk_ui import EventLoop, BaseWindow, MTWindow, UIAction, UIKeyEvent
from metavision_core.event_io.raw_reader import initiate_device
import argparse
import os
import cv2
import sys
import argparse
import numpy as np

import time


ACCUMULATION_MS = 60 # 100
SATURATION = int(60 * ACCUMULATION_MS / 100)

class SaveEventImages:
    start_time_us: int

    def __init__(self, start_time_us, save_path):
        self.start_time_us = start_time_us
        self.save_path = save_path

    def save(self, ts, img):
        # print(f"Saving image at time {int(self.start_time_us+ts)} us")
        cv2.imwrite(os.path.join(self.save_path, f"{int(self.start_time_us+ts)}.png"), img)
        # print(f"Delay s: {time.time() - (self.start_time_us+ts) * 10**-6:.4f} s")

def get_data_from_camera(left_camera: bool, path: str):
    if left_camera:
        serial = "00051962"
    else:
        serial = "00051961"
    
    while True:
        try:
            device = initiate_device(path=serial)
            """if left_camera:
                print(f"DONE LEFT!")
            else:
                print(f"DONE RIGHT!")"""
            break
        except Exception as e:
            print(f"Error initiating Event device {serial}: {e}. Retrying...")
            pass

    mv_iterator = EventsIterator.from_device(
        device =  device,
        delta_t = ACCUMULATION_MS * 1e3,
        mode =    "delta_t",
    )
    height, width = mv_iterator.get_size()
    histo_processor =  EventPreprocessor.create_HardwareHistoProcessor(
        input_event_width  = width,
        input_event_height = height,
        neg_saturation = SATURATION,
        pos_saturation = SATURATION,
    )
    histo = histo_processor.init_output_tensor()
    sei = SaveEventImages(
        int(time.time() * 1_000_000),
        path,
    )

    if left_camera:
        print(f"Starting taking data from Left Event Camera...")
    else:
        print(f"Starting taking data from Right Event Camera...")
    kernel = np.ones((5, 5), np.uint8)

    for evs in mv_iterator:
        histo = histo_processor.init_output_tensor()
        histo_processor.process_events(
            cur_frame_start_ts = evs[0][3],
            events_np = evs,
            frame_tensor_np = histo,
        )
        to_write = histo.sum(axis=2) / (2 * SATURATION) * 255
        to_write = to_write.astype('uint8')
        activation_of_the_scene = cv2.countNonZero(to_write) / to_write.size
        if (activation_of_the_scene > 0.1):
            # images_to_write.append(to_write)
            edit = to_write
            """cv2.imshow("Edit1", edit)
            edit = 255 - edit
            cv2.imshow("Edit2", edit)
            edit = cv2.morphologyEx(edit, cv2.MORPH_CLOSE, kernel)
            cv2.imshow("Edit3", edit)
            edit[edit==255] = 0
            cv2.imshow("Edit4", edit)

            cv2.waitKey(10)"""

            """PATTERN_SIZE = (9, 6)
            flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + 
            cv2.CALIB_CB_NORMALIZE_IMAGE + 
            cv2.CALIB_CB_FILTER_QUADS)
            # flags = None
            res, corners = cv2.findChessboardCorners(edit, PATTERN_SIZE, flags=flags)
            if res:
                print("FOUND CHESSBOARD")
                rgb = cv2.cvtColor(edit, cv2.COLOR_GRAY2RGB)
                cv2.drawChessboardCorners(rgb, PATTERN_SIZE, corners, res)
                edit = rgb
            else:
                print("DIDN'T FIND CHESSBOARD")"""
            sei.save(evs[0][3], edit)
    

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
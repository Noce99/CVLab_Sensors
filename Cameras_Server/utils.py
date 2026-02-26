from flask import Flask, render_template, jsonify, send_file
from flask_cors import CORS
import base64
import io
import threading
import time
from datetime import datetime
import numpy as np
import cv2
import os
import sys
import subprocess
import glob
import shutil


# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

class Saver:

    def __init__(self, lime_imgs_path):
        self.lime_imgs_path = lime_imgs_path

    def save_frame_to_disk(self, sensor_name, frame, timestamp_us):
        """
        Save camera frame to disk with microsecond timestamp.
        
        Frames are saved in ./live_imgs/<sensor_name>/ directory.
        Filename format: <timestamp_microseconds>.png
        
        Args:
            sensor_name (str): Sensor identifier (rgb_left, rgb_right, event_left, 
                            event_right, lidar_livox, lidar_hesai)
            frame (numpy.ndarray): Image frame as numpy array
            timestamp_us (int): Unix timestamp in microseconds
        """
        filename = f"{timestamp_us}.png"
        
        # Construct full path and ensure directory exists
        live_path = os.path.join(self.lime_imgs_path, sensor_name)
        os.makedirs(live_path, exist_ok=True)
        
        try:
            cv2.imwrite(os.path.join(live_path, filename), frame)
        except Exception as e:
            print(f"ERROR: Failed to save frame for {sensor_name}: {e}")
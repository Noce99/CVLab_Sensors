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


class EventStereoManager:
    """
    Manager class for Event stereo camera pair (Prophesee).
    
    Handles:
    - Launching Python subprocess for each camera
    - Reading frames from temporary files
    - Frame storage to disk
    - Process cleanup on shutdown
    
    Requires save_images_no_sign_distintion.py script and Metavision SDK.
    Falls back to dummy mode if script not found.
    """
    
    def __init__(self, camera_status, status_lock, saver):
        """
        Initialize Event stereo manager.
        
        Args:
            camera_status (dict): Shared status dictionary
            status_lock (threading.Lock): Thread safety lock
        """
        self.camera_status = camera_status
        self.status_lock = status_lock
        self.running = False
        self.thread = None
        self.left_process = None
        self.right_process = None
        self.temp_path = "./temp_event_frames"
        self.saver = saver
        
    def start(self):
        """
        Start Event camera acquisition.
        
        Returns:
            bool: True if started successfully, False if already running
        """
        if self.running:
            print("WARNING: Event cameras already running")
            return False
        
        # Create temporary directories for frame storage
        left_path = os.path.join(self.temp_path, "left")
        right_path = os.path.join(self.temp_path, "right")
        os.makedirs(left_path, exist_ok=True)
        os.makedirs(right_path, exist_ok=True)
        
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        print("INFO: Event cameras started")
        return True
        
    def stop(self):
        """Stop Event camera acquisition and terminate subprocesses."""
        self.running = False
        
        # Terminate Python subprocesses
        for process in [self.left_process, self.right_process]:
            if process and process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        
        if self.thread:
            self.thread.join(timeout=2)
        
        print("INFO: Event cameras stopped")
        
        # Update status to inactive
        with self.status_lock:
            self.camera_status['event_left']['active'] = False
            self.camera_status['event_right']['active'] = False
    
    def _capture_loop(self):
        """
        Main acquisition loop for Event cameras.
        
        Process:
        1. Locate acquisition script
        2. Launch subprocess for each camera
        3. Monitor temporary directory for new frames
        4. Read, display, and save frames
        5. Cleanup old files
        """
        try:
            # Search for Event camera acquisition script
            possible_paths = [
                "../EventCamera/save_h5.py"# "../EventCamera/save_images.py",
            ]
            
            event_script = None
            for path in possible_paths:
                if os.path.exists(path):
                    event_script = path
                    break
            
            if event_script is None:
                print("WARNING: Event camera script not found - switching to dummy mode")
                self._dummy_loop()
                return
            
            # Use same Python interpreter as server
            python_executable = sys.executable
            
            # Define output paths
            left_path = os.path.join(self.temp_path, "../live_imgs/event_left")
            right_path = os.path.join(self.temp_path, "../live_imgs/event_right")
            
            # Launch subprocess for left camera
            print(f"INFO: Starting Event left camera: {event_script}")
            self.left_process = subprocess.Popen(
                [python_executable, event_script, left_path],
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE
            )
            
            # Launch subprocess for right camera (with --right-camera flag)
            print(f"INFO: Starting Event right camera: {event_script}")
            self.right_process = subprocess.Popen(
                [python_executable, event_script, right_path, "--right-camera"],
                # stdout=subprocess.PIPE,
                # stderr=subprocess.PIPE
            )
            
            # Wait for processes to initialize
            time.sleep(3)
            
            # Main loop: monitor temporary directories for new frames
            while self.running:
                """# Process left camera frames
                pattern_left = os.path.join(left_path, "*.png")
                files_left = sorted(glob.glob(pattern_left))
                
                if files_left:
                    latest_file = files_left[-1]
                    
                    try:
                        # Read frame as grayscale
                        frame = cv2.imread(latest_file, cv2.IMREAD_GRAYSCALE)
                        if frame is not None:
                            # Convert to BGR for display
                            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                            
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            # Extract timestamp from filename
                            timestamp_us = int(os.path.basename(latest_file).replace('.png', ''))
                            
                            # Update status (thread-safe)
                            with self.status_lock:
                                self.camera_status['event_left']['frame'] = frame
                                self.camera_status['event_left']['active'] = True
                                self.camera_status['event_left']['last_update'] = timestamp
                                self.camera_status['event_left']['fps'] = 10
                            
                            # Save to live_imgs directory
                            self.saver.save_frame_to_disk('event_left', frame, timestamp_us)
                        
                        # Cleanup: keep only last 5 files
                        for old_file in files_left[:-5]:
                            try:
                                os.remove(old_file)
                            except:
                                pass
                    except Exception as e:
                        print(f"WARNING: Error reading Event left frame: {e}")
                
                # Process right camera frames
                pattern_right = os.path.join(right_path, "*.png")
                files_right = sorted(glob.glob(pattern_right))
                
                if files_right:
                    latest_file = files_right[-1]
                    
                    try:
                        frame = cv2.imread(latest_file, cv2.IMREAD_GRAYSCALE)
                        if frame is not None:
                            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                            
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            timestamp_us = int(os.path.basename(latest_file).replace('.png', ''))
                            
                            with self.status_lock:
                                self.camera_status['event_right']['frame'] = frame
                                self.camera_status['event_right']['active'] = True
                                self.camera_status['event_right']['last_update'] = timestamp
                                self.camera_status['event_right']['fps'] = 10
                            
                            self.saver.save_frame_to_disk('event_right', frame, timestamp_us)
                        
                        # Cleanup
                        for old_file in files_right[:-5]:
                            try:
                                os.remove(old_file)
                            except:
                                pass
                    except Exception as e:
                        print(f"WARNING: Error reading Event right frame: {e}")
                
                # Poll every 200ms
                time.sleep(0.2)"""
                pass
                
        except Exception as e:
            print(f"ERROR: Event capture failed: {e}")
            import traceback
            traceback.print_exc()
            self._dummy_loop()
    
    def _dummy_loop(self):
        """Dummy acquisition loop for testing without hardware."""
        while self.running:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            timestamp_us = int(time.time() * 1_000_000)
            
            # Generate dummy frames
            frame_left = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
            frame_left = cv2.cvtColor(frame_left, cv2.COLOR_GRAY2BGR)
            cv2.putText(frame_left, f'EVENT LEFT {timestamp}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            frame_right = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
            frame_right = cv2.cvtColor(frame_right, cv2.COLOR_GRAY2BGR)
            cv2.putText(frame_right, f'EVENT RIGHT {timestamp}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            with self.status_lock:
                self.camera_status['event_left']['frame'] = frame_left
                self.camera_status['event_left']['active'] = True
                self.camera_status['event_left']['last_update'] = timestamp
                self.camera_status['event_left']['fps'] = 10
                
                self.camera_status['event_right']['frame'] = frame_right
                self.camera_status['event_right']['active'] = True
                self.camera_status['event_right']['last_update'] = timestamp
                self.camera_status['event_right']['fps'] = 10
            
            self.saver.save_frame_to_disk('event_left', frame_left, timestamp_us)
            self.saver.save_frame_to_disk('event_right', frame_right, timestamp_us)
            
            time.sleep(0.1)
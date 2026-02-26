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


class LidarLivoxManager:
    """
    Manager class for Livox LiDAR.
    
    Handles:
    - Launching C++ acquisition program
    - Reading intensity/reflection images
    - Applying colormap for visualization
    - Frame storage to disk
    
    Requires compiled save_intensity program.
    Falls back to dummy mode if program not found.
    """
    
    def __init__(self, camera_status, status_lock, saver):
        """
        Initialize Livox LiDAR manager.
        
        Args:
            camera_status (dict): Shared status dictionary
            status_lock (threading.Lock): Thread safety lock
        """
        self.camera_status = camera_status
        self.status_lock = status_lock
        self.running = False
        self.thread = None
        self.process = None
        self.temp_path = "./temp_livox_frames"
        self.saver = saver
        
    def start(self):
        """
        Start Livox LiDAR acquisition.
        
        Returns:
            bool: True if started successfully, False if already running
        """
        if self.running:
            print("WARNING: LiDAR Livox already running")
            return False
            
        os.makedirs(self.temp_path, exist_ok=True)
        
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        print("INFO: LiDAR Livox started")
        return True
        
    def stop(self):
        """Stop Livox LiDAR acquisition and terminate subprocess."""
        self.running = False
        
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        
        if self.thread:
            self.thread.join(timeout=2)
        
        print("INFO: LiDAR Livox stopped")
        
        with self.status_lock:
            self.camera_status['lidar_livox']['active'] = False
    
    def _capture_loop(self):
        """
        Main acquisition loop for Livox LiDAR.
        
        Process:
        1. Locate compiled C++ program
        2. Launch subprocess
        3. Monitor output directory for new frames
        4. Apply colormap and save
        """
        try:
            # Search for Livox acquisition program
            possible_paths = [
                "./save_intensity",
                "../RGB_Livox_Calibration/save_intensity",
                "/home/ubm/CVLab_Sensors/RGB_Livox_Calibration/save_intensity",
            ]
            
            livox_program = None
            for path in possible_paths:
                if os.path.exists(path):
                    livox_program = path
                    break
            
            if livox_program is None:
                print("WARNING: Livox program not found - switching to dummy mode")
                self._dummy_loop()
                return
            
            print(f"INFO: Starting LiDAR Livox: {livox_program}")
            self.process = subprocess.Popen(
                [livox_program, self.temp_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Wait for process initialization
            time.sleep(2)
            
            lidar_path = os.path.join(self.temp_path, "lidar")
            
            while self.running:
                # Find most recent file
                pattern = os.path.join(lidar_path, "*.png")
                files = sorted(glob.glob(pattern))
                
                if files:
                    latest_file = files[-1]
                    
                    try:
                        # Read grayscale intensity image
                        frame = cv2.imread(latest_file, cv2.IMREAD_GRAYSCALE)
                        if frame is not None:
                            # Convert to BGR and apply JET colormap for depth visualization
                            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
                            
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            timestamp_us = int(os.path.basename(latest_file).replace('.png', ''))
                            
                            with self.status_lock:
                                self.camera_status['lidar_livox']['frame'] = frame
                                self.camera_status['lidar_livox']['active'] = True
                                self.camera_status['lidar_livox']['last_update'] = timestamp
                                self.camera_status['lidar_livox']['fps'] = 1
                            
                            self.saver.save_frame_to_disk('lidar_livox', frame, timestamp_us)
                        
                        # Cleanup: keep only last 5 files
                        for old_file in files[:-5]:
                            try:
                                os.remove(old_file)
                            except:
                                pass
                    except Exception as e:
                        print(f"WARNING: Error reading Livox frame: {e}")
                
                time.sleep(0.5)
                
        except Exception as e:
            print(f"ERROR: Livox capture failed: {e}")
            import traceback
            traceback.print_exc()
            self._dummy_loop()
    
    def _dummy_loop(self):
        """Dummy acquisition loop for testing without hardware."""
        while self.running:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            timestamp_us = int(time.time() * 1_000_000)
            
            frame = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            cv2.putText(frame, f'LIVOX {timestamp}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            with self.status_lock:
                self.camera_status['lidar_livox']['frame'] = frame
                self.camera_status['lidar_livox']['active'] = True
                self.camera_status['lidar_livox']['last_update'] = timestamp
                self.camera_status['lidar_livox']['fps'] = 1
            
            self.saver.save_frame_to_disk('lidar_livox', frame, timestamp_us)
            
            time.sleep(1.0)

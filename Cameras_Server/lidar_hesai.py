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


class LidarHesaiManager:
    """
    Manager class for Hesai LiDAR.
    
    Similar to LidarLivoxManager but for Hesai hardware.
    Uses RAINBOW colormap instead of JET.
    """
    
    def __init__(self, camera_status, status_lock, saver):
        """
        Initialize Hesai LiDAR manager.
        
        Args:
            camera_status (dict): Shared status dictionary
            status_lock (threading.Lock): Thread safety lock
        """
        self.camera_status = camera_status
        self.status_lock = status_lock
        self.running = False
        self.thread = None
        self.process = None
        self.temp_path = "./temp_hesai_frames"
        self.saver = saver
        
    def start(self):
        """
        Start Hesai LiDAR acquisition.
        
        Returns:
            bool: True if started successfully, False if already running
        """
        if self.running:
            print("WARNING: LiDAR Hesai already running")
            return False
            
        os.makedirs(self.temp_path, exist_ok=True)
        
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        print("INFO: LiDAR Hesai started")
        return True
        
    def stop(self):
        """Stop Hesai LiDAR acquisition and terminate subprocess."""
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
        
        print("INFO: LiDAR Hesai stopped")
        
        with self.status_lock:
            self.camera_status['lidar_hesai']['active'] = False
    
    def _capture_loop(self):
        """Main acquisition loop for Hesai LiDAR."""
        try:
            # Search for Hesai acquisition program
            possible_paths = [
                "./hesai_lidar_example",
                "../RGB_Hesai_Calibration/hesai_example",
                "/home/ubm/SDKs/HesaiLidar_SDK_2.0/build/hesai_lidar_sdk_example",
            ]
            
            hesai_program = None
            for path in possible_paths:
                if os.path.exists(path):
                    hesai_program = path
                    break
            
            if hesai_program is None:
                print("WARNING: Hesai program not found - switching to dummy mode")
                self._dummy_loop()
                return
            
            print(f"INFO: Starting LiDAR Hesai: {hesai_program}")
            self.process = subprocess.Popen(
                [hesai_program, self.temp_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            time.sleep(2)
            
            lidar_path = os.path.join(self.temp_path, "lidar")
            
            while self.running:
                pattern = os.path.join(lidar_path, "*.png")
                files = sorted(glob.glob(pattern))
                
                if files:
                    latest_file = files[-1]
                    
                    try:
                        frame = cv2.imread(latest_file, cv2.IMREAD_GRAYSCALE)
                        if frame is not None:
                            # Apply RAINBOW colormap
                            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                            frame = cv2.applyColorMap(frame, cv2.COLORMAP_RAINBOW)
                            
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            timestamp_us = int(os.path.basename(latest_file).replace('.png', ''))
                            
                            with self.status_lock:
                                self.camera_status['lidar_hesai']['frame'] = frame
                                self.camera_status['lidar_hesai']['active'] = True
                                self.camera_status['lidar_hesai']['last_update'] = timestamp
                                self.camera_status['lidar_hesai']['fps'] = 20
                            
                            self.saver.save_frame_to_disk('lidar_hesai', frame, timestamp_us)
                        
                        for old_file in files[:-5]:
                            try:
                                os.remove(old_file)
                            except:
                                pass
                    except Exception as e:
                        print(f"WARNING: Error reading Hesai frame: {e}")
                
                time.sleep(0.1)
                
        except Exception as e:
            print(f"ERROR: Hesai capture failed: {e}")
            import traceback
            traceback.print_exc()
            self._dummy_loop()
    
    def _dummy_loop(self):
        """Dummy acquisition loop for testing without hardware."""
        while self.running:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            timestamp_us = int(time.time() * 1_000_000)
            
            frame = np.random.randint(0, 255, (480, 640), dtype=np.uint8)
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_RAINBOW)
            cv2.putText(frame, f'HESAI {timestamp}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            with self.status_lock:
                self.camera_status['lidar_hesai']['frame'] = frame
                self.camera_status['lidar_hesai']['active'] = True
                self.camera_status['lidar_hesai']['last_update'] = timestamp
                self.camera_status['lidar_hesai']['fps'] = 20
            
            self.saver.save_frame_to_disk('lidar_hesai', frame, timestamp_us)
            
            time.sleep(0.05)
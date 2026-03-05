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

from pypylon import pylon


sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from RGBCamera.setup_stereo_cameras import setup_stereo_cameras


class RGBStereoManager:
    """
    Manager class for RGB stereo camera pair (Basler).
    
    Handles:
    - Camera initialization and configuration
    - PTP (Precision Time Protocol) synchronization
    - Continuous frame acquisition in separate thread
    - Frame storage to disk
    - Graceful shutdown
    
    Falls back to dummy mode if pypylon is unavailable or cameras not found.
    """
    
    def __init__(self, camera_status, status_lock, saver):
        """
        Initialize RGB stereo manager.
        
        Args:
            camera_status (dict): Shared status dictionary
            status_lock (threading.Lock): Thread safety lock
        """
        self.camera_status = camera_status
        self.status_lock = status_lock
        self.running = False
        self.thread = None
        self.cameras = None
        self.temp_path = "./temp_event_frames"
        self.saver = saver
        
    def start(self):
        """
        Start RGB camera acquisition.
        
        Returns:
            bool: True if started successfully, False if already running
        """
        if self.running:
            print("WARNING: RGB cameras already running")
            return False
            
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        print("INFO: RGB cameras started")
        return True
        
    def stop(self):
        """Stop RGB camera acquisition and release resources."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
        if self.cameras:
            self.cameras.StopGrabbing()
            self.cameras.Close()
        print("INFO: RGB cameras stopped")
        
        # Update status to inactive
        with self.status_lock:
            self.camera_status['rgb_left']['active'] = False
            self.camera_status['rgb_right']['active'] = False
    
    def _start_dummy(self):
        """Start dummy mode for testing without hardware."""
        self.running = True
        self.thread = threading.Thread(target=self._dummy_loop, daemon=True)
        self.thread.start()
    
    def _dummy_loop(self):
        """
        Dummy acquisition loop generating random frames.
        Used when pypylon is unavailable or for testing.
        """
        while self.running:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            timestamp_us = int(time.time() * 1_000_000)
            
            # Generate random frame for left camera
            frame_left = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            cv2.putText(frame_left, f'RGB LEFT {timestamp}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Generate random frame for right camera
            frame_right = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            cv2.putText(frame_right, f'RGB RIGHT {timestamp}', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Update status dictionary (thread-safe)
            with self.status_lock:
                self.camera_status['rgb_left']['frame'] = frame_left
                self.camera_status['rgb_left']['active'] = True
                self.camera_status['rgb_left']['last_update'] = timestamp
                self.camera_status['rgb_left']['fps'] = 10
                
                self.camera_status['rgb_right']['frame'] = frame_right
                self.camera_status['rgb_right']['active'] = True
                self.camera_status['rgb_right']['last_update'] = timestamp
                self.camera_status['rgb_right']['fps'] = 10
            
            # Save frames to disk
            self.saver.save_frame_to_disk('rgb_left', frame_left, timestamp_us)
            self.saver.save_frame_to_disk('rgb_right', frame_right, timestamp_us)
            
            time.sleep(0.1)  # 10 FPS
        
    def _capture_loop(self):
        """
        Main acquisition loop for real RGB cameras.
        
        Process:
        1. Initialize camera array
        2. Search and attach cameras by serial number
        3. Configure cameras
        4. Synchronize via PTP
        5. Continuous frame grabbing
        6. Update status and save frames
        """
        try:
            # Initialize pylon camera array for 2 cameras
            self.cameras = pylon.InstantCameraArray(2)
            tlf = pylon.TlFactory.GetInstance()
            
            left_found = False
            right_found = False
            
            print("INFO: Searching for RGB cameras...")
            tl = tlf.CreateTl('BaslerGigE')
            tlf.EnumerateDevices()
            
            cam_left_info = tl.CreateDeviceInfo()
            cam_left_info.SetIpAddress('192.168.1.70')

            cam_right_info = tl.CreateDeviceInfo()
            cam_right_info.SetIpAddress('192.168.1.71')

            self.cameras[0].Attach(tl.CreateDevice(cam_left_info))
            self.cameras[0].Open()
            setup_stereo_cameras(self.cameras[0])

            self.cameras[1].Attach(tl.CreateDevice(cam_right_info))
            self.cameras[1].Open()
            setup_stereo_cameras(self.cameras[1])

            self.cameras[1].StreamGrabber.TransmissionType ="Multicast"
            self.cameras[1].SetCameraContext(1)
            self.cameras[1].StreamGrabber.DestinationAddr ="192.168.1.52"
            self.cameras[1].StreamGrabber.DestinationPort.Value = 5555
            self.cameras[0].StreamGrabber.TransmissionType ="Multicast"
            self.cameras[0].SetCameraContext(0)
            self.cameras[0].StreamGrabber.DestinationAddr ="192.168.1.52"
            self.cameras[0].StreamGrabber.DestinationPort.Value = 5556
            
            # Fall back to dummy mode if cameras not found
            # if not left_found or not right_found:
            #     print("ERROR: RGB cameras not found - switching to dummy mode")
            #     self._dummy_loop()
            #     return
            
            # Synchronize cameras using PTP (Precision Time Protocol)
            print("INFO: Synchronizing cameras via PTP...")
            while self.running:
                all_locked = True
                for i, camera in enumerate(self.cameras):
                    camera.PtpDataSetLatch.Execute()
                    time.sleep(0.1)
                    print(f"[{i}] -- {camera.PtpStatus.Value} [{camera.PtpServoStatus.Value}] [{camera.PtpOffsetFromMaster.Value} ns]")
                    if camera.PtpServoStatus.Value != "Locked":
                        all_locked = False
                if all_locked:
                    print("INFO: PTP synchronization complete")
                    break
                time.sleep(0.1)
            
            # Start grabbing frames
            self.cameras.StartGrabbing()
            frame_count = 0
            start_time = time.time()

            left_path = os.path.join(self.temp_path, "../live_imgs/rgb_left")
            right_path = os.path.join(self.temp_path, "../live_imgs/rgb_right")
            path = [left_path, right_path]
            
            pc_ts  = time.time() * 1_000_000
            camera.TimestampLatch.Execute()
            # Get the timestamp value
            camera_ts = camera.TimestampLatchValue.Value / 1_000
            print(f"pc_ts = {pc_ts}")
            print(f"camera_ts = {camera_ts}")
            delta_ts = pc_ts - camera_ts
            print(f"delta_ts = {delta_ts}")
            
            # Main acquisition loop
            while self.running:
                # Retrieve frame with 5 second timeout
                grabResult = self.cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                
                if grabResult.GrabSucceeded():
                    camera_index = grabResult.GetCameraContext()
                    # timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    # frame = grabResult.GetArray()
                    
                    # # Convert grayscale to BGR if necessary
                    # if len(frame.shape) == 2:
                    #     frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    # else:
                    #     frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                    
                    # # Determine camera name based on index
                    # camera_name = 'rgb_left' if camera_index == 0 else 'rgb_right'
                    
                    # # Update status dictionary (thread-safe)
                    # with self.status_lock:
                    #     self.camera_status[camera_name]['frame'] = frame
                    #     self.camera_status[camera_name]['active'] = True
                    #     self.camera_status[camera_name]['last_update'] = timestamp
                        
                    #     # Calculate FPS every 30 frames
                    #     frame_count += 1
                    #     if frame_count % 30 == 0:
                    #         elapsed = time.time() - start_time
                    #         fps = 30 / elapsed
                    #         self.camera_status[camera_name]['fps'] = int(fps)
                    #         start_time = time.time()
                    
                    # # Save frame to disk
                    # timestamp_us = int(time.time() * 1_000_000)
                    # self.saver.save_frame_to_disk(camera_name, frame, timestamp_us)
                    ts = grabResult.GetTimeStamp() / 1_000 + delta_ts
                    # print(grabResult.GetTimeStamp() / 1_000_000_000)
                    cv2.imwrite(os.path.join(path[camera_index], f"{int(ts)}.png"), cv2.cvtColor(grabResult.GetArray(), cv2.COLOR_RGB2BGR))

                
                grabResult.Release()
                
        except Exception as e:
            print(f"ERROR: RGB capture failed: {e}")
            import traceback
            traceback.print_exc()
            self._dummy_loop()
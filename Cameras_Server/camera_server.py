"""
Camera Monitoring Server
=========================

Flask-based web server for monitoring and recording data from multiple sensors:
- RGB stereo cameras (Basler, via pypylon)
- Event stereo cameras (Prophesee, via Metavision SDK)
- LiDAR Livox
- LiDAR Hesai

"""

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

# Attempt to import pypylon for Basler camera support
try:
    from pypylon import pylon
    PYLON_AVAILABLE = True
except ImportError:
    print("WARNING: pypylon not available - RGB cameras will run in dummy mode")
    PYLON_AVAILABLE = False

# Initialize Flask application
app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

# ============================================================================
# CONFIGURATION
# ============================================================================

# Basler RGB camera serial numbers
LEFT_SERIAL = "40663113"
RIGHT_SERIAL = "40665216"

# Global camera status dictionary
# Stores current state, FPS, last update time, and latest frame for each sensor
camera_status = {
    'rgb_left': {'active': False, 'last_update': None, 'fps': 0, 'frame': None},
    'rgb_right': {'active': False, 'last_update': None, 'fps': 0, 'frame': None},
    'event_left': {'active': False, 'last_update': None, 'fps': 0, 'frame': None},
    'event_right': {'active': False, 'last_update': None, 'fps': 0, 'frame': None},
    'lidar_livox': {'active': False, 'last_update': None, 'fps': 0, 'frame': None},
    'lidar_hesai': {'active': False, 'last_update': None, 'fps': 0, 'frame': None}
}

# Thread safety lock for accessing camera_status
status_lock = threading.Lock()

# Output directory for live frame storage
LIVE_IMGS_PATH = "./live_imgs"

# ============================================================================
# UTILITY FUNCTIONS
# ============================================================================

def save_frame_to_disk(sensor_name, frame, timestamp_us):
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
    live_path = os.path.join(LIVE_IMGS_PATH, sensor_name)
    os.makedirs(live_path, exist_ok=True)
    
    try:
        cv2.imwrite(os.path.join(live_path, filename), frame)
    except Exception as e:
        print(f"ERROR: Failed to save frame for {sensor_name}: {e}")

# Dictionary to store camera manager instances
camera_managers = {}


# ============================================================================
# CAMERA MANAGER CLASSES
# ============================================================================

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
    
    def __init__(self, camera_status, status_lock):
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
        
    def start(self):
        """
        Start RGB camera acquisition.
        
        Returns:
            bool: True if started successfully, False if already running
        """
        if self.running:
            print("WARNING: RGB cameras already running")
            return False
            
        if not PYLON_AVAILABLE:
            print("WARNING: pypylon not available - starting dummy mode")
            self._start_dummy()
            return True
            
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
            save_frame_to_disk('rgb_left', frame_left, timestamp_us)
            save_frame_to_disk('rgb_right', frame_right, timestamp_us)
            
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
            tlFactory = pylon.TlFactory.GetInstance()
            
            left_found = False
            right_found = False
            
            print("INFO: Searching for RGB cameras...")
            devices = tlFactory.EnumerateDevices()
            
            # Find and attach cameras by serial number
            for d in devices:
                serial = d.GetSerialNumber()
                if serial == LEFT_SERIAL:
                    self.cameras[0].Attach(tlFactory.CreateDevice(d))
                    self.cameras[0].Open()
                    self._setup_camera(self.cameras[0])
                    left_found = True
                    print(f"INFO: Left RGB camera found: {serial}")
                elif serial == RIGHT_SERIAL:
                    self.cameras[1].Attach(tlFactory.CreateDevice(d))
                    self.cameras[1].Open()
                    self._setup_camera(self.cameras[1])
                    right_found = True
                    print(f"INFO: Right RGB camera found: {serial}")
            
            # Fall back to dummy mode if cameras not found
            if not left_found or not right_found:
                print("ERROR: RGB cameras not found - switching to dummy mode")
                self._dummy_loop()
                return
            
            # Synchronize cameras using PTP (Precision Time Protocol)
            print("INFO: Synchronizing cameras via PTP...")
            while self.running:
                all_locked = True
                for i, camera in enumerate(self.cameras):
                    camera.PtpDataSetLatch.Execute()
                    time.sleep(0.1)
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
            
            # Main acquisition loop
            while self.running:
                # Retrieve frame with 5 second timeout
                grabResult = self.cameras.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
                
                if grabResult.GrabSucceeded():
                    camera_index = grabResult.GetCameraContext()
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    timestamp_us = int(time.time() * 1_000_000)
                    frame = grabResult.GetArray()
                    
                    # Convert grayscale to BGR if necessary
                    if len(frame.shape) == 2:
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    
                    # Determine camera name based on index
                    camera_name = 'rgb_left' if camera_index == 0 else 'rgb_right'
                    
                    # Update status dictionary (thread-safe)
                    with self.status_lock:
                        self.camera_status[camera_name]['frame'] = frame
                        self.camera_status[camera_name]['active'] = True
                        self.camera_status[camera_name]['last_update'] = timestamp
                        
                        # Calculate FPS every 30 frames
                        frame_count += 1
                        if frame_count % 30 == 0:
                            elapsed = time.time() - start_time
                            fps = 30 / elapsed
                            self.camera_status[camera_name]['fps'] = int(fps)
                            start_time = time.time()
                    
                    # Save frame to disk
                    save_frame_to_disk(camera_name, frame, timestamp_us)
                
                grabResult.Release()
                
        except Exception as e:
            print(f"ERROR: RGB capture failed: {e}")
            import traceback
            traceback.print_exc()
            self._dummy_loop()
    
    def _setup_camera(self, camera):
        """
        Configure camera parameters.
        
        Args:
            camera: pylon camera instance
        """
        try:
            camera.MaxNumBuffer = 5
            # Additional camera configuration can be added here
        except Exception as e:
            print(f"WARNING: Camera setup error: {e}")


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
    
    def __init__(self, camera_status, status_lock):
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
                "./save_images_no_sign_distintion.py",
                "../EventCamera/save_images_no_sign_distintion.py",
                "/home/ubm/CVLab_Sensors/EventCamera/save_images_no_sign_distintion.py",
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
            left_path = os.path.join(self.temp_path, "left")
            right_path = os.path.join(self.temp_path, "right")
            
            # Launch subprocess for left camera
            print(f"INFO: Starting Event left camera: {event_script}")
            self.left_process = subprocess.Popen(
                [python_executable, event_script, left_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Launch subprocess for right camera (with --right-camera flag)
            print(f"INFO: Starting Event right camera: {event_script}")
            self.right_process = subprocess.Popen(
                [python_executable, event_script, right_path, "--right-camera"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            # Wait for processes to initialize
            time.sleep(3)
            
            # Main loop: monitor temporary directories for new frames
            while self.running:
                # Process left camera frames
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
                            save_frame_to_disk('event_left', frame, timestamp_us)
                        
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
                            
                            save_frame_to_disk('event_right', frame, timestamp_us)
                        
                        # Cleanup
                        for old_file in files_right[:-5]:
                            try:
                                os.remove(old_file)
                            except:
                                pass
                    except Exception as e:
                        print(f"WARNING: Error reading Event right frame: {e}")
                
                # Poll every 200ms
                time.sleep(0.2)
                
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
            
            save_frame_to_disk('event_left', frame_left, timestamp_us)
            save_frame_to_disk('event_right', frame_right, timestamp_us)
            
            time.sleep(0.1)


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
    
    def __init__(self, camera_status, status_lock):
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
                            
                            save_frame_to_disk('lidar_livox', frame, timestamp_us)
                        
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
            
            save_frame_to_disk('lidar_livox', frame, timestamp_us)
            
            time.sleep(1.0)


class LidarHesaiManager:
    """
    Manager class for Hesai LiDAR.
    
    Similar to LidarLivoxManager but for Hesai hardware.
    Uses RAINBOW colormap instead of JET.
    """
    
    def __init__(self, camera_status, status_lock):
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
                            
                            save_frame_to_disk('lidar_hesai', frame, timestamp_us)
                        
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
            
            save_frame_to_disk('lidar_hesai', frame, timestamp_us)
            
            time.sleep(0.05)


# ============================================================================
# INITIALIZE CAMERA MANAGERS
# ============================================================================

camera_managers['rgb'] = RGBStereoManager(camera_status, status_lock)
camera_managers['event'] = EventStereoManager(camera_status, status_lock)
camera_managers['lidar_livox'] = LidarLivoxManager(camera_status, status_lock)
camera_managers['lidar_hesai'] = LidarHesaiManager(camera_status, status_lock)


# ============================================================================
# FLASK ROUTES
# ============================================================================

@app.route('/')
def index():
    """
    Serve main dashboard HTML page.
    
    Returns:
        HTML template
    """
    return render_template('dashboard.html')


@app.route('/api/status')
def get_status():
    """
    Get current status of all cameras.
    
    Returns:
        JSON object with status for each camera including:
        - active: boolean indicating if camera is running
        - last_update: timestamp of last frame
        - fps: current frames per second
    """
    with status_lock:
        status = {
            camera: {
                'active': info['active'],
                'last_update': info['last_update'],
                'fps': info['fps']
            }
            for camera, info in camera_status.items()
        }
    return jsonify(status)


@app.route('/api/frame/<camera_name>')
def get_frame(camera_name):
    """
    Get latest frame from specific camera.
    
    Args:
        camera_name (str): Camera identifier
    
    Returns:
        JSON object with base64-encoded JPEG frame and timestamp
        404 if camera not found or no frame available
    """
    if camera_name not in camera_status:
        return jsonify({'error': 'Camera not found'}), 404
    
    with status_lock:
        frame = camera_status[camera_name]['frame']
        if frame is None:
            return jsonify({'error': 'No frame available'}), 404
        
        # Encode frame as JPEG with 85% quality
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
        
        # Convert to base64 for transmission
        frame_base64 = base64.b64encode(buffer).decode('utf-8')
    
    return jsonify({
        'frame': f'data:image/jpeg;base64,{frame_base64}',
        'timestamp': camera_status[camera_name]['last_update']
    })


@app.route('/api/start/<camera_type>')
def start_camera(camera_type):
    """
    Start camera acquisition for specified type.
    
    Args:
        camera_type (str): Camera type (rgb, event, lidar_livox, lidar_hesai)
    
    Returns:
        JSON object with operation status
        404 if camera type not found
        500 if operation fails
    """
    if camera_type not in camera_managers:
        return jsonify({'error': f'Camera type not found: {camera_type}'}), 404
    
    try:
        success = camera_managers[camera_type].start()
        if success:
            return jsonify({
                'status': 'started', 
                'camera': camera_type,
                'message': f'{camera_type} cameras started successfully'
            })
        else:
            return jsonify({
                'status': 'already_running',
                'camera': camera_type,
                'message': f'{camera_type} cameras already running'
            })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'camera': camera_type,
            'error': str(e)
        }), 500


@app.route('/api/stop/<camera_type>')
def stop_camera(camera_type):
    """
    Stop camera acquisition for specified type.
    
    Args:
        camera_type (str): Camera type (rgb, event, lidar_livox, lidar_hesai)
    
    Returns:
        JSON object with operation status
        404 if camera type not found
        500 if operation fails
    """
    if camera_type not in camera_managers:
        return jsonify({'error': f'Camera type not found: {camera_type}'}), 404
    
    try:
        camera_managers[camera_type].stop()
        return jsonify({
            'status': 'stopped',
            'camera': camera_type,
            'message': f'{camera_type} cameras stopped successfully'
        })
    except Exception as e:
        return jsonify({
            'status': 'error',
            'camera': camera_type,
            'error': str(e)
        }), 500


@app.route('/api/start_all')
def start_all():
    """
    Start all camera acquisitions.
    
    Returns:
        JSON object with results for each camera type
    """
    results = {}
    for camera_type, manager in camera_managers.items():
        try:
            manager.start()
            results[camera_type] = 'started'
        except Exception as e:
            results[camera_type] = f'error: {str(e)}'
    
    return jsonify({
        'status': 'completed',
        'results': results
    })


@app.route('/api/stop_all')
def stop_all():
    """
    Stop all camera acquisitions.
    
    Returns:
        JSON object with results for each camera type
    """
    results = {}
    for camera_type, manager in camera_managers.items():
        try:
            manager.stop()
            results[camera_type] = 'stopped'
        except Exception as e:
            results[camera_type] = f'error: {str(e)}'
    
    return jsonify({
        'status': 'completed',
        'results': results
    })


# ============================================================================
# MAIN ENTRY POINT
# ============================================================================

if __name__ == '__main__':
    print("=" * 60)
    print("CAMERA MONITORING SERVER")
    print("=" * 60)
    print("\nAvailable sensors:")
    print("  - RGB Stereo (Left + Right) - Basler")
    print("  - Event Stereo (Left + Right)")
    print("  - LiDAR Livox")
    print("  - LiDAR Hesai")
    
    # Display pypylon availability
    if PYLON_AVAILABLE:
        print("\nINFO: pypylon available - RGB real mode")
        print(f"   Serial Left:  {LEFT_SERIAL}")
        print(f"   Serial Right: {RIGHT_SERIAL}")
    else:
        print("\nWARNING: pypylon NOT available - RGB dummy mode")
    
    # Check for Event camera script
    event_script_found = any(os.path.exists(p) for p in [
        "./save_images_no_sign_distintion.py",
        "../EventCamera/save_images_no_sign_distintion.py",
        "/home/ubm/CVLab_Sensors/EventCamera/save_images_no_sign_distintion.py"
    ])
    
    if event_script_found:
        print("INFO: Event camera script found")
    else:
        print("WARNING: Event camera script not found - dummy mode")
    
    # Check for LiDAR programs
    livox_found = any(os.path.exists(p) for p in [
        "./save_intensity",
        "../RGB_Livox_Calibration/save_intensity",
        "/home/ubm/CVLab_Sensors/RGB_Livox_Calibration/save_intensity"
    ])
    
    if livox_found:
        print("INFO: Livox program found")
    else:
        print("WARNING: Livox program not found - dummy mode")
    
    print("\nServer starting on http://0.0.0.0:5000")
    print("Connect from mobile: http://<COMPUTER_IP>:5000")
    print("Connect from local: http://localhost:5000")
    print(f"\nFrames saved to: {LIVE_IMGS_PATH}/<sensor_name>/")
    print("Press Ctrl+C to stop")
    print("=" * 60 + "\n")
    
    # Start Flask server
    # host='0.0.0.0' allows connections from any IP (required for hotspot)
    # use_reloader=False prevents double thread spawning in debug mode
    app.run(host='0.0.0.0', port=5000, debug=True, threaded=True, use_reloader=False)

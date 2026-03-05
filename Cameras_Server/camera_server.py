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
import shutil
from Reader_lidar import load_latest_lidar_frame
from Reader_event import load_latest_event_frame

from rgb import RGBStereoManager
from event import EventStereoManager
from lidar_hesai import LidarHesaiManager
from lidar_livox import LidarLivoxManager

from utils import Saver

# Initialize Flask application
app = Flask(__name__)
CORS(app)  # Enable Cross-Origin Resource Sharing

# ============================================================================
# CONFIGURATION
# ============================================================================

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

# Dictionary to store camera manager instances
camera_managers = {}

# ============================================================================
# INITIALIZE CAMERA MANAGERS
# ============================================================================

saver = Saver(LIVE_IMGS_PATH)

camera_managers['rgb'] = RGBStereoManager(camera_status, status_lock, saver)
camera_managers['event'] = EventStereoManager(camera_status, status_lock, saver)
camera_managers['lidar_livox'] = LidarLivoxManager(camera_status, status_lock, saver)
camera_managers['lidar_hesai'] = LidarHesaiManager(camera_status, status_lock, saver)


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

    # I lidar salvano in H5: legge e converte l'ultimo file ad ogni richiesta
    if camera_name == 'lidar_livox':
        frame = load_latest_lidar_frame(
            os.path.join(LIVE_IMGS_PATH, 'lidar_livox'),
            cv2.COLORMAP_JET
        )
    elif camera_name == 'lidar_hesai':
        frame = load_latest_lidar_frame(
            os.path.join(LIVE_IMGS_PATH, 'lidar_hesai'),
            cv2.COLORMAP_RAINBOW
        )
    # Le event camera salvano in .raw: legge e converte l'ultimo file ad ogni richiesta
    elif camera_name == 'event_left':
        frame = load_latest_event_frame(os.path.join(LIVE_IMGS_PATH, 'event_left'))
    elif camera_name == 'event_right':
        frame = load_latest_event_frame(os.path.join(LIVE_IMGS_PATH, 'event_right'))
    else:
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


@app.route('/api/clear_all')
def clear_all():
    """
    Clear all camera acquisitions.
    
    
    """

    cartella = "./live_imgs"

    for nome_cartella in os.listdir(cartella):
        for nome_file in os.listdir(os.path.join(cartella, nome_cartella)):
            percorso = os.path.join(cartella, nome_cartella, nome_file)
            try:
                os.remove(percorso) 
            except Exception as e:
                print(f"Exception = {e}")
                print("WARNING: img non cancellata correttamente")


    print("Cartella pulita!")
    
    return jsonify({
        'status': 'completed'
    })


@app.route('/api/move_all')
def move_all():
    """
    Move all imgs on new folder ~/Documents/CVLab_live_imgs
    """

    cartella_origine = "./live_imgs"
    cartella_destinazione = "../CVLab_live_imgs"

    # Creo la cartella di destinazione se non esiste
    os.makedirs(cartella_destinazione, exist_ok=True)

    now = datetime.now()
    time_folder_name = now.strftime("%Y_%m_%d__%H_%M_%S")

    destination = os.path.join(cartella_destinazione, time_folder_name)

    shutil.copytree(cartella_origine, destination)
    clear_all()

    print("Immagini spostate con successo!")
    
    return jsonify({
        'status': 'completed'
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
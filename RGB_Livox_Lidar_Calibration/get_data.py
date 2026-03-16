import subprocess
from rgb import RGBSaveImages
import os

from datetime import datetime

def start_getting_lidar_data(path):
    process = subprocess.Popen(["sh", "save_intensity.sh", path])
    return process

def start_getting_rgb_data(path):
    RGBSaveImages(path=path)

def kill_process(process_obj):
    """Kill the process using the process object"""
    if process_obj.poll() is None:  # Check if still running
        process_obj.terminate()  # Graceful shutdown (SIGTERM)
        try:
            process_obj.wait(timeout=5)  # Wait for it to end
        except subprocess.TimeoutExpired:
            process_obj.kill()  # Force kill (SIGKILL)
            process_obj.wait()
        print("Process terminated")
    else:
        print("Process already finished")

def main():
    calibration_data_path = "./calibration_data"
    folder_name = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    os.mkdir(os.path.join(calibration_data_path, folder_name))

    lidar_path = os.path.join(calibration_data_path, folder_name, "lidar")
    lidar_raw_path = os.path.join(calibration_data_path, folder_name, "lidar_raw")
    rgb_path = os.path.join(calibration_data_path, folder_name, "rgb")

    os.mkdir(lidar_path)
    os.mkdir(lidar_raw_path)
    os.mkdir(rgb_path)

    process = start_getting_lidar_data(os.path.join(calibration_data_path, folder_name))
    start_getting_rgb_data(rgb_path)


    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        kill_process(process)


if __name__ == "__main__":
    main()
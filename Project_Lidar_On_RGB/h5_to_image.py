import h5py
import os
from datetime import datetime
import numpy as np
import cv2

def read_a_file(path_h5_file):
    # Open file for reading
    with h5py.File(path_h5_file, 'r') as f:
        dataset = f['point_cloud']
        
        # Get dataset information
        print(f"Dataset: point_cloud")
        print(f"  Shape: {dataset.shape}")
        print(f"  Dtype: {dataset.dtype}")
        print(f"  Total points: {dataset.shape[0]}")
        
        # Read the file creation timestamp attribute
        if 'file_creation_timestamp' in dataset.attrs:
            timestamp_us = dataset.attrs['file_creation_timestamp']
            
            # Convert microseconds to datetime
            timestamp_s = timestamp_us / 1_000_000  # Convert to seconds
            dt = datetime.fromtimestamp(timestamp_s)
            print(f"  File creation time: {dt.strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"  Timestamp (μs): {timestamp_us}")
        else:
            print("  Warning: No file_creation_timestamp attribute found")
        
        return dataset[:]
    
def show_image(datasets_paths):
    H = 1000
    W = 1000
    img = np.zeros((H, W), dtype=np.uint8)
    u_0 = H / 2
    v_0 = W / 2
    fx = 500
    fy = 500

    def project_points(points):
        nonlocal img

        for x, y, z, t in points:
            u = max(0, min(int(-x*fx/z + u_0), H-1))
            v = max(0, min(int(-y*fy/z + v_0), W-1))
            img[u, v] = max(0, min(int(z/ 70), 255))

        cv2.imshow("Depth Image", img)
        cv2.waitKey(1)
        img = np.zeros((H, W), dtype=np.uint8)

    while True:
        for dataset_path in datasets_paths:
            project_points(read_a_file(dataset_path))

if __name__ == "__main__":
    calibration_data_path = "./calibration_data"
    folders = os.listdir(calibration_data_path)
    folders.sort()
    folder = folders[-1]

    lidar_folder_path = os.path.join(calibration_data_path, folder, "lidar")

    h5_files = os.listdir(lidar_folder_path)
    h5_files.sort()
    
    datasets_paths = [os.path.join(lidar_folder_path, f) for f in h5_files]

    show_image(datasets_paths)

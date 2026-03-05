import h5py
import os
from datetime import datetime
import numpy as np

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
        # if 'file_creation_timestamp' in dataset.attrs:
        #     timestamp_us = dataset.attrs['file_creation_timestamp']
            
        #     # Convert microseconds to datetime
        #     timestamp_s = timestamp_us / 1_000_000  # Convert to seconds
        #     dt = datetime.fromtimestamp(timestamp_s)
        #     print(f"  File creation time: {dt.strftime('%Y-%m-%d %H:%M:%S')}")
        #     print(f"  Timestamp (μs): {timestamp_us}")
        # else:
        #     print("  Warning: No file_creation_timestamp attribute found")
        
        return dataset[:]
    
def show_point_cloud_with_updates(datasets_paths, update_interval=0.1):
    """
    Show point cloud that updates with new points
    datasets_paths: List of file paths to point cloud data
    update_interval: Time between updates in seconds
    """
    import open3d as o3d
    import numpy as np
    import time
    
    # Initialize counter and last update time
    update_counter = 1
    last_update_time = time.time()
    
    def update_points(vis):
        """
        Update callback function
        """
        nonlocal pc, update_counter, last_update_time
        
        # Check if enough time has passed and if we have more files
        current_time = time.time()
        if (current_time - last_update_time) >= update_interval and update_counter < len(datasets_paths):
            try:
                # Read the next file
                new_points = read_a_file(datasets_paths[update_counter])
                points = np.array(new_points)
                
                pc.points = o3d.utility.Vector3dVector(points[:, :3])
                vis.update_geometry(pc)
                
                # Update counter and timestamp
                update_counter += 1
                last_update_time = current_time
                
                # Print progress
                print(f"Updated point cloud {update_counter}/{len(datasets_paths)}")
                
            except Exception as e:
                print(f"Error reading file {datasets_paths[update_counter]}: {e}")
                update_counter += 1
        
        # Stop if we've processed all files
        if update_counter >= len(datasets_paths):
            update_counter = 0
            print("All point clouds processed.")
            # Return True to stop animation if you want, or False to keep window open
            return False
            
        return False  # Continue updating
    
    # Read initial point cloud
    initial_points = read_a_file(datasets_paths[0])
    points = np.array(initial_points)
    
    # Create point cloud
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points[:, :3])        
    
    # Create visualizer with better settings
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Updating Point Cloud", width=1280, height=720)
    
    # Configure render options
    render_option = vis.get_render_option()
    render_option.point_size = 1.0
    render_option.background_color = np.asarray([0.1, 0.1, 0.1])
    
    vis.add_geometry(pc)
    
    # Set initial view
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -1])
    ctr.set_up([0, -1, 0])
    ctr.set_zoom(0.8)
    
    # Register update callback
    vis.register_animation_callback(update_points)
    
    print(f"Starting visualization with {len(datasets_paths)} point clouds")
    print(f"Update interval: {update_interval} seconds")
    print("Press 'Q' to exit")
    
    # Run visualization
    vis.run()
    vis.destroy_window()

if __name__ == "__main__":
    calibration_data_path = "./calibration_data"
    folders = os.listdir(calibration_data_path)
    folders.sort()
    folder = folders[-1]

    lidar_folder_path = os.path.join(calibration_data_path, folder, "lidar")

    lidar_folder_path = "/home/ubm/CVLab_Sensors/CVLab_live_imgs/2026_03_05__14_02_41/lidar_hesai"

    h5_files = os.listdir(lidar_folder_path)
    h5_files.sort()
    
    datasets_paths = [os.path.join(lidar_folder_path, f) for f in h5_files]

    show_point_cloud_with_updates(datasets_paths, update_interval=1.0)

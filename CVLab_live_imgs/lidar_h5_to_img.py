import os
import h5py
import numpy as np
import cv2

MIN_DISTANCE = 300 # mm
MAX_DISTANCE = 5_000 # mm

def lava_color_map():
    lava_colormap = np.zeros((256, 1, 3), dtype=np.uint8)
    
    for i in range(256):
        t = i / 255.0  # Normalized value [0, 1]
        
        if t < 0.25:  # Black to dark red
            lava_colormap[i, 0, 0] = int(255 * (t * 4))  # R
            lava_colormap[i, 0, 1] = 0  # G
            lava_colormap[i, 0, 2] = 0  # B
        elif t < 0.5:  # Dark red to bright red
            lava_colormap[i, 0, 0] = 255  # R
            lava_colormap[i, 0, 1] = int(255 * ((t - 0.25) * 4))  # G
            lava_colormap[i, 0, 2] = 0  # B
        elif t < 0.75:  # Bright red to yellow
            lava_colormap[i, 0, 0] = 255  # R
            lava_colormap[i, 0, 1] = 255  # G
            lava_colormap[i, 0, 2] = int(255 * ((t - 0.5) * 4))  # B
        else:  # Yellow to white
            lava_colormap[i, 0, 0] = 255  # R
            lava_colormap[i, 0, 1] = 255  # G
            lava_colormap[i, 0, 2] = int(255 * (0.25 + (t - 0.75) * 3))  # B
    return lava_colormap

def project_points_3d_to_2d(points_3d, K, D):
    """
    Project multiple 3D points to 2D image coordinates.
    
    Args:
        points_3d: List/array of 3D points with shape (N, 3) where each point is [x, y, z]
                  x = up/down (vertical)
                  y = left/right (horizontal)
                  z = along optical axis (distance from camera)
        K: Camera matrix (3x3)
        D: Distortion coefficients (1x5)
    
    Returns:
        points_2d: Array of 2D pixel coordinates with shape (N, 2) [u, v]
    """
    # Convert to numpy array if needed
    points_3d = np.array(points_3d, dtype=np.float32)
    
    # Handle single point case
    if points_3d.ndim == 1:
        points_3d = points_3d.reshape(1, -1)
    
    # Convert from your coordinate system to OpenCV convention
    # Your: [x, y, z] where x=up, y=right, z=forward
    # OpenCV: [x_cv, y_cv, z_cv] where x_cv=right, y_cv=down, z_cv=forward
    points_cv = np.zeros_like(points_3d)
    points_cv[:, 0] = points_3d[:, 1]  # x_cv = y (right)
    points_cv[:, 1] = -points_3d[:, 0]  # y_cv = -x (negative because your x is up)
    points_cv[:, 2] = points_3d[:, 2]   # z_cv = z (forward)
    
    # Reshape for OpenCV (N, 1, 3)
    points_cv = points_cv.reshape(-1, 1, 3)
    
    # No rotation or translation (points are in camera coordinates)
    rvec = np.zeros((3, 1), dtype=np.float32)
    tvec = np.zeros((3, 1), dtype=np.float32)
    
    # Project all points at once
    points_2d, _ = cv2.projectPoints(points_cv, rvec, tvec, K, D)
    
    # Reshape to (N, 2)
    points_2d = points_2d.reshape(-1, 2)
    
    return points_2d

def load_lidar_frame(h5_file_path, K=None, D=None, x_translate=None, y_translate=None, colormap=cv2.COLORMAP_MAGMA):
    """
    Legge l'ultimo file H5 nella cartella del lidar, proietta i punti 3D
    su un'immagine 2D e applica una colormap.

    Args:
        lidar_folder (str): Percorso della cartella contenente i file .h5
        colormap: Colormap OpenCV da applicare (es. cv2.COLORMAP_JET)
            Available colormaps that resemble lava:
                - cv2.COLORMAP_HOT (black-red-yellow-white)
                - cv2.COLORMAP_JET (blue-cyan-yellow-red)
                - cv2.COLORMAP_TURBO (similar to jet but smoother)
                - cv2.COLORMAP_INFERNO (perceptually uniform)
                - cv2.COLORMAP_MAGMA (similar to inferno)

    Returns:
        np.ndarray BGR oppure None se non ci sono file o la lettura fallisce
    """
    # if not os.path.exists(lidar_folder):
    #     return None

    # h5_files = []
    # for filename in os.listdir(lidar_folder):
    #     if filename.endswith(".h5"):
    #         h5_files.append(os.path.join(lidar_folder, filename))

    # if not h5_files:
    #     return None

    # # Ordina per nome e prende l'ultimo (point_cloud_0, point_cloud_1, ...)
    # h5_files.sort()
    # latest_file = h5_files[-1]

    try:
        with h5py.File(h5_file_path, 'r') as f:
            points = f['point_cloud'][:]   # shape (N, 4) → x, y, z, t
    except Exception as e:
        print(f"WARNING: Impossibile leggere {h5_file_path}: {e}")
        return None
    
    print(f"xs = [{points[:, 0].min()}:{points[:, 0].max()}]")
    print(f"ys = [{points[:, 1].min()}:{points[:, 1].max()}]")
    print(f"zs = [{points[:, 2].min()}:{points[:, 2].max()}]")
    print(f"Points = {points.shape} {points.dtype}")
    
    if K is None or D is None:
        # Parametri di proiezione (pinhole)
        H, W = 720 // 2, 1280 // 2
        u_0  = H / 2.0
        v_0  = W / 2.0
        fx   = 500.0
        fy   = 500.0

        normalized_depth_map = np.zeros((H, W), dtype=np.float32)


        

        for x, y, z, t in points:
            if z > 0:
                if x_translate is not None:
                    x += x_translate
                if y_translate is not None:
                    y += y_translate
                u = int(-x * fx / z + u_0)
                v = int(y * fy / z + v_0)
                u = max(0, min(u, H - 1))
                v = max(0, min(v, W - 1))
                normalized_depth_map[u, v] = min(max(MIN_DISTANCE, z), MAX_DISTANCE) / MAX_DISTANCE

        depth_map_255 = (normalized_depth_map * 255).astype(np.uint8)

        colored_bgr = cv2.applyColorMap(depth_map_255, colormap)
        # colored_rgb = cv2.cvtColor(colored_bgr, cv2.COLOR_BGR2RGB)

        colored_bgr = cv2.resize(colored_bgr, (1280, 720))
    else:
        # 1958
        # 1229
        fx = K[0, 0]
        fy = K[1, 1]
        v_0 = K[0, 2] # test
        u_0 = K[1, 2] # test
        H, W = int(u_0*2), int(v_0*2)

        normalized_depth_map = np.zeros((H, W), dtype=np.float32)
        kernel = 3

        for x, y, z, t in points:
            if z > 0:
                if x_translate is not None:
                    x += x_translate
                if y_translate is not None:
                    y += y_translate
                u = int(-x * fx / z + u_0)
                v = int(y * fy / z + v_0)
                u = max(0, min(u, H - 1))
                v = max(0, min(v, W - 1))
                normalized_depth_map[u-kernel:u+kernel, v-kernel:v+kernel] = min(max(MIN_DISTANCE, z), MAX_DISTANCE) / MAX_DISTANCE

        depth_map_255 = (normalized_depth_map * 255).astype(np.uint8)

        colored_bgr = cv2.applyColorMap(depth_map_255, colormap)
        # colored_rgb = cv2.cvtColor(colored_bgr, cv2.COLOR_BGR2RGB)

        colored_bgr = cv2.resize(colored_bgr, (1280, 720))


    return colored_bgr


if __name__ == "__main__":
    livox_folder = "/home/ubm/CVLab_Sensors/CVLab_live_imgs/2026_03_05__15_53_46/lidar_hesai"
    # rgb_image = load_lidar_frame("/home/ubm/CVLab_Sensors/CVLab_live_imgs/2026_03_05__15_31_13/lidar_livox", livox_or_hesai=0)
    # cv2.imwrite("/home/ubm/Downloads/livox.png", rgb_image)
    # rgb_image = load_lidar_frame("/home/ubm/CVLab_Sensors/CVLab_live_imgs/2026_03_05__15_31_13/lidar_hesai", livox_or_hesai=1)
    # cv2.imwrite("/home/ubm/Downloads/hesai.png", rgb_image)

    i = 0
    files = os.listdir(livox_folder)
    files.sort()
    for filename in files:
        if filename.endswith(".h5"):
            rgb_img = load_lidar_frame(
                os.path.join(livox_folder, filename)
            )
            cv2.imwrite(f"/home/ubm/Downloads/{i}.png", rgb_img)
            i += 1
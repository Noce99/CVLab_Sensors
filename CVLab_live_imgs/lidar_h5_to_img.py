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

def load_lidar_frame(h5_file_path, colormap=cv2.COLORMAP_MAGMA):
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

    # Parametri di proiezione (pinhole)
    H, W = 720 // 2, 1280 // 2
    u_0  = H / 2.0
    v_0  = W / 2.0
    fx   = 500.0
    fy   = 500.0

    normalized_depth_map = np.zeros((H, W), dtype=np.float32)


    print(f"xs = [{points[:, 0].min()}:{points[:, 0].max()}]")
    print(f"ys = [{points[:, 1].min()}:{points[:, 1].max()}]")
    print(f"zs = [{points[:, 2].min()}:{points[:, 2].max()}]")
    print(f"Points = {points.shape} {points.dtype}")

    for x, y, z, t in points:
        if z > 0:
            u = int(-x * fx / z + u_0)
            v = int(y * fy / z + v_0)
            u = max(0, min(u, H - 1))
            v = max(0, min(v, W - 1))
            normalized_depth_map[u, v] = min(max(MIN_DISTANCE, z), MAX_DISTANCE) / MAX_DISTANCE

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
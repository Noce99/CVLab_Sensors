import os
import h5py
import numpy as np
import cv2


def load_latest_lidar_frame(lidar_folder, colormap):
    """
    Legge l'ultimo file H5 nella cartella del lidar, proietta i punti 3D
    su un'immagine 2D e applica una colormap.

    Args:
        lidar_folder (str): Percorso della cartella contenente i file .h5
        colormap: Colormap OpenCV da applicare (es. cv2.COLORMAP_JET)

    Returns:
        np.ndarray BGR oppure None se non ci sono file o la lettura fallisce
    """
    if not os.path.exists(lidar_folder):
        return None

    h5_files = []
    for filename in os.listdir(lidar_folder):
        if filename.endswith(".h5"):
            h5_files.append(os.path.join(lidar_folder, filename))

    if not h5_files:
        return None

    # Ordina per nome e prende l'ultimo (point_cloud_0, point_cloud_1, ...)
    h5_files.sort()
    latest_file = h5_files[-1]

    try:
        with h5py.File(latest_file, 'r') as f:
            points = f['point_cloud'][:]   # shape (N, 4) → x, y, z, t
    except Exception as e:
        print(f"WARNING: Impossibile leggere {latest_file}: {e}")
        return None

    # Parametri di proiezione (pinhole)
    H, W = 480, 640
    u_0  = H / 2.0
    v_0  = W / 2.0
    fx   = 500.0
    fy   = 500.0

    img = np.zeros((H, W), dtype=np.uint8)

    for x, y, z, t in points:
        if z > 0:
            u = int(-x * fx / z + u_0)
            v = int(-y * fy / z + v_0)
            u = max(0, min(u, H - 1))
            v = max(0, min(v, W - 1))
            intensity = 255 - max(0, min(int(z / 30000 * 255), 255))
            img[u, v] = intensity

    if img.max() == 0:
        placeholder = np.full((H, W, 3), 40, dtype=np.uint8)
        cv2.putText(placeholder, "No points projected", (120, H // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        return placeholder

    img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img_bgr = cv2.applyColorMap(img_bgr, colormap)
    return img_bgr

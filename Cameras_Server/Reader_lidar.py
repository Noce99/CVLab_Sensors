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

    # Ordina per nome (timestamp_us come nome file, es. 1773132483660719.h5) e prende l'ultimo
    h5_files.sort()
    h5_files.reverse()  # dal più recente al più vecchio

    points = None
    for latest_file in h5_files:
        try:
            with h5py.File(latest_file, 'r') as f:
                points = f['point_cloud'][:]
            break   # lettura riuscita, esci dal loop
        except Exception as e:
            print(f"WARNING: File troncato o corrotto, salto: {os.path.basename(latest_file)} ({e})")
            continue

    if points is None:
        print(f"WARNING: Nessun file H5 leggibile in {lidar_folder}")
        return None

    # Parametri di proiezione (pinhole)
    H, W = 480, 640
    u_0  = H / 2.0
    v_0  = W / 2.0
    fx   = 500.0
    fy   = 500.0

    img = np.zeros((H, W), dtype=np.uint8)

    # Filtra i punti con z <= 0 (non validi)
    valid = points[points[:, 2] > 0]
    if len(valid) == 0:
        placeholder = np.full((H, W, 3), 40, dtype=np.uint8)
        cv2.putText(placeholder, "No valid points (z <= 0)", (80, H // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
        return placeholder

    # Normalizzazione dinamica: usa il 5° e 95° percentile di z
    # per evitare che outlier schiaccino tutto il contrasto
    z_min = float(np.percentile(valid[:, 2], 5))
    z_max = float(np.percentile(valid[:, 2], 95))
    z_range = z_max - z_min if z_max > z_min else 1.0

    print(f"  [LiDAR] {os.path.basename(lidar_folder)} — punti validi: {len(valid)} | "
          f"z range: [{z_min:.1f}, {z_max:.1f}]")

    for x, y, z, t in valid:
        u = int(-x * fx / z + u_0)
        v = int(-y * fy / z + v_0)
        u = max(0, min(u, H - 1))
        v = max(0, min(v, W - 1))
        # Normalizza z sul range reale: vicino → chiaro, lontano → scuro
        intensity = int(255 * (1.0 - (z - z_min) / z_range))
        intensity = max(0, min(intensity, 255))
        img[u, v] = intensity

    if img.max() == 0:
        placeholder = np.full((H, W, 3), 40, dtype=np.uint8)
        cv2.putText(placeholder, "No points projected", (120, H // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        return placeholder

    # Equalizzazione istogramma: distribuisce i valori su tutto il range 0-255
    # migliora il contrasto quando i punti si concentrano in un range ristretto di z
    img = cv2.equalizeHist(img)

    img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img_bgr = cv2.applyColorMap(img_bgr, colormap)
    return img_bgr
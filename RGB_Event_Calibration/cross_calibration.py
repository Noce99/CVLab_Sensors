"""
Cross-calibration tra coppie stereo RGB e Event cameras

Struttura attesa:
project/
├── RGB/
│   ├── calibration.npz                    # generato da calibrate.py
│   └── calibration_data/YYYY_MM_DD_HH_MM_SS/
│       ├── left/*.png
│       └── right/*.png
├── EVENT/
│   ├── calibration.npz
│   └── calibration_data/YYYY_MM_DD_HH_MM_SS/
│       ├── left/*.png
│       └── right/*.png
└── cross_calibration.py                   # questo script
"""



"""NON UTILIZZATO"""


import os
import cv2
import numpy as np
from typing import List, Tuple
from scipy.spatial.transform import Rotation
from collections import OrderedDict


PATTERN_SIZE = (9, 6)  # (cols, rows) corner interni della scacchiera
SQUARE_SIZE = 0.025    # dimensione quadrato in metri (25mm)


class StereoToStereoCalibration:
    """Calibra la trasformazione tra due coppie stereo (RGB e Event cameras)"""
    
    def __init__(self, rgb_stereo_params, event_stereo_params):
        self.rgb_params = rgb_stereo_params
        self.event_params = event_stereo_params
        
    def detect_pattern(self, img, pattern_size=PATTERN_SIZE):
        """Rileva scacchiera nell'immagine"""
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img
            
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            return corners.reshape(-1, 2)
        return None
    
    def get_pattern_3d_points(self, pattern_size=PATTERN_SIZE, square_size=SQUARE_SIZE):
        """Genera coordinate 3D del pattern"""
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        objp *= square_size
        return objp
    
    def estimate_pose_pnp(self, object_points, image_points, K, D):
        """Stima posa del pattern rispetto alla camera"""
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, K, D, 
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        
        if not success:
            return None, None
            
        R, _ = cv2.Rodrigues(rvec)
        return R, tvec.flatten()
    
    def calibrate_from_images(self, rgb_pairs: List[Tuple], event_pairs: List[Tuple],
                              pattern_size=PATTERN_SIZE, square_size=SQUARE_SIZE):
        """
        Calibra trasformazione tra i due sistemi stereo da immagini del pattern
        
        Returns:
            R_rgb_to_event: rotazione da RGB a Event
            t_rgb_to_event: traslazione da RGB a Event
        """
        object_points = self.get_pattern_3d_points(pattern_size, square_size)
        
        T_rgb_to_pattern = []
        T_event_to_pattern = []
        
        print(f"\nElaborando {len(rgb_pairs)} coppie di immagini...")
        
        for i, (rgb_pair, event_pair) in enumerate(zip(rgb_pairs, event_pairs)):
            rgb_left, rgb_right = rgb_pair
            event_left, event_right = event_pair
            
            # Rileva pattern
            rgb_corners_left = self.detect_pattern(rgb_left, pattern_size)
            event_corners_left = self.detect_pattern(event_left, pattern_size)
            
            if rgb_corners_left is None or event_corners_left is None:
                print(f"  Frame {i}: pattern non rilevato, skip")
                continue
            
            # Stima posa del pattern rispetto a ciascun sistema
            R_rgb, t_rgb = self.estimate_pose_pnp(
                object_points, rgb_corners_left,
                self.rgb_params['K_left'], self.rgb_params['D_left']
            )
            
            R_event, t_event = self.estimate_pose_pnp(
                object_points, event_corners_left,
                self.event_params['K_left'], self.event_params['D_left']
            )
            
            if R_rgb is None or R_event is None:
                print(f"  Frame {i}: PnP fallito, skip")
                continue
            
            # Salva trasformazioni
            T_rgb = np.eye(4)
            T_rgb[:3, :3] = R_rgb
            T_rgb[:3, 3] = t_rgb
            
            T_event = np.eye(4)
            T_event[:3, :3] = R_event
            T_event[:3, 3] = t_event
            
            T_rgb_to_pattern.append(T_rgb)
            T_event_to_pattern.append(T_event)
            
            print(f"  Frame {i}: ✓")
        
        if len(T_rgb_to_pattern) < 3:
            raise ValueError(f"Troppo poche pose valide ({len(T_rgb_to_pattern)}), servono almeno 3")
        
        print(f"\nCalcolo trasformazione da {len(T_rgb_to_pattern)} pose...")
        
        # Calcola trasformazione RGB -> Event
        T_estimates = []
        for T_rgb, T_event in zip(T_rgb_to_pattern, T_event_to_pattern):
            T_rgb_inv = np.linalg.inv(T_rgb)
            T_rgb_to_event = T_event @ T_rgb_inv
            T_estimates.append(T_rgb_to_event)
        
        # Media delle trasformazioni
        rotations = [T[:3, :3] for T in T_estimates]
        translations = [T[:3, 3] for T in T_estimates]
        
        R_mean = Rotation.from_matrix(rotations).mean().as_matrix()
        t_mean = np.mean(translations, axis=0)
        
        print("\n✓ Calibrazione completata!")
        print(f"  Deviazione std traslazione: {np.std(translations, axis=0)} m")
        
        return R_mean, t_mean
    
    def transform_point(self, point_rgb, R_rgb_to_event, t_rgb_to_event):
        """Trasforma punto da sistema RGB a sistema Event"""
        point_rgb = np.asarray(point_rgb)
        if point_rgb.ndim == 1:
            return R_rgb_to_event @ point_rgb + t_rgb_to_event
        else:
            return (R_rgb_to_event @ point_rgb.T).T + t_rgb_to_event
    
    def save_calibration(self, R, t, filename='rgb_to_event_calibration.npz'):
        """Salva calibrazione su file"""
        np.savez(filename, R=R, t=t)
        print(f"✓ Calibrazione salvata in {filename}")
    
    @staticmethod
    def load_calibration(filename='rgb_to_event_calibration.npz'):
        """Carica calibrazione da file"""
        data = np.load(filename)
        return data['R'], data['t']


def load_stereo_calibration(calib_file):
    """Carica parametri di calibrazione stereo da file .npz"""
    if not os.path.exists(calib_file):
        raise FileNotFoundError(f"File di calibrazione non trovato: {calib_file}")
    
    data = np.load(calib_file)
    return {
        'K_left': data['K_left'],
        'K_right': data['K_right'],
        'D_left': data['D_left'],
        'D_right': data['D_right'],
        'R': data['R'],
        'T': data['T']
    }


def get_latest_calibration_folder(base_path):
    """Trova l'ultima cartella di calibrazione"""
    calib_data_path = os.path.join(base_path, "calibration_data")
    if not os.path.exists(calib_data_path):
        raise FileNotFoundError(f"Cartella calibration_data non trovata in {base_path}")
    
    folders = sorted(os.listdir(calib_data_path))
    if not folders:
        raise FileNotFoundError(f"Nessuna cartella di calibrazione trovata in {calib_data_path}")
    
    return os.path.join(calib_data_path, folders[-1])


def load_image_pairs_from_timestamps(left_path, right_path):
    """
    Carica coppie di immagini sincronizzate per timestamp
    (usa la stessa logica di calibrate.py)
    """
    # Leggi tutte le immagini left e right
    left_dict = {int(f.split(".")[0]): os.path.join(left_path, f) 
                 for f in os.listdir(left_path) if f.endswith('.png')}
    right_dict = {int(f.split(".")[0]): os.path.join(right_path, f)
                  for f in os.listdir(right_path) if f.endswith('.png')}
    
    left_dict = OrderedDict(sorted(left_dict.items()))
    right_dict = OrderedDict(sorted(right_dict.items()))
    
    # Accoppia per timestamp più vicino
    pairs = []
    right_timestamps = list(right_dict.keys())
    last_right_i = 0
    
    for left_ts in left_dict:
        # Trova timestamp right più vicino
        closest_right_i = last_right_i
        min_dist = abs(left_ts - right_timestamps[last_right_i])
        
        for i in range(last_right_i + 1, len(right_timestamps)):
            dist = abs(left_ts - right_timestamps[i])
            if dist > min_dist:
                break
            min_dist = dist
            closest_right_i = i
        
        last_right_i = closest_right_i
        right_ts = right_timestamps[closest_right_i]
        
        left_img = cv2.imread(left_dict[left_ts])
        right_img = cv2.imread(right_dict[right_ts])
        
        if left_img is not None and right_img is not None:
            pairs.append((left_img, right_img))
    
    return pairs


if __name__ == "__main__":
    print("="*60)
    print("CROSS-CALIBRATION RGB <-> EVENT CAMERAS")
    print("="*60)
    
    # === CONFIGURAZIONE ===
    RGB_BASE_PATH = './RGBCamera'
    EVENT_BASE_PATH = './EventCamera'
    OUTPUT_FILE = 'rgb_to_event_calibration.npz'
    
    try:
        # === CARICA CALIBRAZIONI STEREO ===
        print("\n1. Caricamento calibrazioni stereo...")
        rgb_calib_file = os.path.join(RGB_BASE_PATH, 'calibration.npz')
        event_calib_file = os.path.join(EVENT_BASE_PATH, 'calibration.npz')
        
        rgb_params = load_stereo_calibration(rgb_calib_file)
        event_params = load_stereo_calibration(event_calib_file)
        print("   ✓ Calibrazioni stereo caricate")
        
        # === TROVA ULTIME CARTELLE DI CALIBRAZIONE ===
        print("\n2. Ricerca immagini di calibrazione...")
        rgb_folder = get_latest_calibration_folder(RGB_BASE_PATH)
        event_folder = get_latest_calibration_folder(EVENT_BASE_PATH)
        
        print(f"   RGB:   {rgb_folder}")
        print(f"   Event: {event_folder}")
        
        # === CARICA IMMAGINI ===
        print("\n3. Caricamento immagini...")
        rgb_left_path = os.path.join(rgb_folder, "left")
        rgb_right_path = os.path.join(rgb_folder, "right")
        event_left_path = os.path.join(event_folder, "left")
        event_right_path = os.path.join(event_folder, "right")
        
        rgb_pairs = load_image_pairs_from_timestamps(rgb_left_path, rgb_right_path) # Aggiungere lista con timestamp che viene ritornata dalla funzione
        event_pairs = load_image_pairs_from_timestamps(event_left_path, event_right_path)

        # Confronti e cerchi le coppie RGB e coppie Events più vicine basandoti sui timestamp e controllando che siano sotto una soglia minima
        
        print(f"   ✓ Caricate {len(rgb_pairs)} coppie RGB")
        print(f"   ✓ Caricate {len(event_pairs)} coppie Event")
        
        if len(rgb_pairs) != len(event_pairs):
            print(f"\n   ⚠ ATTENZIONE: numero diverso di immagini!")
            min_len = min(len(rgb_pairs), len(event_pairs))
            rgb_pairs = rgb_pairs[:min_len]
            event_pairs = event_pairs[:min_len]
            print(f"   → Uso solo le prime {min_len} coppie")
        
        if len(rgb_pairs) < 3:
            raise ValueError("Servono almeno 3 coppie di immagini valide!")
        
        # === CALIBRAZIONE ===
        print("\n4. Esecuzione cross-calibration...")
        print("="*60)
        
        calibrator = StereoToStereoCalibration(rgb_params, event_params)
        
        R, t = calibrator.calibrate_from_images(
            rgb_pairs, event_pairs,
            pattern_size=PATTERN_SIZE,
            square_size=SQUARE_SIZE
        )
        
        print("\n" + "="*60)
        print("RISULTATI CALIBRAZIONE")
        print("="*60)
        print(f"\nTrasformazione RGB → Event:")
        print(f"\nRotazione R:\n{R}")
        print(f"\nTraslazione t (m): {t}")
        print(f"Distanza tra sistemi: {np.linalg.norm(t):.4f} m")
        print(f"\nAngoli di Eulero (gradi): {Rotation.from_matrix(R).as_euler('xyz', degrees=True)}")
        
        # Salva risultati
        calibrator.save_calibration(R, t, OUTPUT_FILE)
        
        print("\n" + "="*60)
        print("✓ CALIBRAZIONE COMPLETATA CON SUCCESSO")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ ERRORE: {e}")
        import traceback
        traceback.print_exc()
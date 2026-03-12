import os
import sys

import cv2
import numpy as np

from rectification import resize_rgb, rectify

from metavision_core.event_io.raw_reader import initiate_device
from metavision_core.event_io import EventsIterator
from metavision_sdk_core import PeriodicFrameGenerationAlgorithm, ColorPalette

from event_raw_to_img import raw_event_to_images
from lidar_h5_to_img import load_lidar_frame

import traceback
import subprocess

#versione 2

# ── Configurazione ───────────────────────────────────────────────────────────
BASE_DIR = "/home/ubm/CVLab_Sensors/CVLab_live_imgs"
TILE_W   = 640
TILE_H   = 480
# ─────────────────────────────────────────────────────────────────────────────


# ── Trova la sessione più recente ─────────────────────────────────────────
from datetime import datetime

def find_session():
    all_sessions = []
    for entry in os.listdir(BASE_DIR):
        full_path = os.path.join(BASE_DIR, entry)
        try:
            datetime.strptime(entry, "%Y_%m_%d__%H_%M_%S")  # verifica formato
            all_sessions.append(full_path)
        except ValueError:
            pass  # nome non conforme, ignorato

    all_sessions.sort()
    session = all_sessions[-1]
    # session = "/home/ubm/CVLab_Sensors/CVLab_live_imgs/2026_03_03__16_47_58"
    print(f"Sessione selezionata: {session}")
    return session


# ── Ottieni lista immagini ordinate per timestamp ────────────────
def get_images_sorted(folder):
    images = []
    for filename in os.listdir(folder):
        name, ext = os.path.splitext(filename)
        try:
            ts = int(name)
            full_path = os.path.join(folder, filename)
            images.append((ts, full_path))
        except ValueError:
            print(f"  [WARN] Nome non numerico ignorato: {filename}")

    images.sort(key=lambda x: x[0])
    return images


# ── Trova l'immagine con timestamp più vicino a 'target_ts' ─────
def find_closest_image(images, target_ts):
    best_path = None
    best_diff = float("inf")
    best_timestamp = None


    for ts, path in images:
        diff = abs(ts - target_ts)
        if diff < best_diff:
            best_diff = diff
            best_path = path
            best_timestamp = ts

    cam_name = images[0][1].split('/')[-2]
    print(f"{cam_name}  differenza temporale: {best_diff / 10**6:.4f} s")
    return best_path, best_timestamp


# ── Carica immagine a risoluzione piena (senza resize) ───────────────────────
def load_image(path):
    img = cv2.imread(path)
    if img is None:
        print(f"  [WARN] Impossibile caricare: {path}")
        return None

    # Converte in BGR se è in scala di grigi (tipico event camera)
    if len(img.shape) == 2 or img.shape[2] == 1:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    return img


# ── Ridimensiona per la griglia di visualizzazione ───────────────────────────
def resize_for_grid(img):
    if img is None:
        return np.full((TILE_H, TILE_W, 3), 60, dtype=np.uint8)  # tile grigio placeholder
    return cv2.resize(img, (int(img.shape[1]/1), int(img.shape[0]/1)))




# ── Estrae tutti i frame da un file .raw con i loro timestamp assoluti ────────
def get_event_frames_from_raw(folder):
    """
    Trova l'unico file .raw nella cartella, lo legge con Metavision SDK
    e restituisce una lista di tuple (timestamp_assoluto_us, frame_numpy)
    ordinate cronologicamente.

    Il timestamp assoluto = nome_file (start_ts) + timestamp_relativo_del_frame.
    """
    raw_files = []
    for filename in os.listdir(folder):
        if filename.endswith(".raw"):
            raw_files.append(os.path.join(folder, filename))

    if not raw_files:
        print(f"  [WARN] Nessun file .raw trovato in: {folder}")
        return []

    raw_files.sort()

    frames = []   # lista di (timestamp_assoluto_us, frame)

    for raw_file in raw_files:
        raw_filename = os.path.splitext(os.path.basename(raw_file))[0]
        try:
            start_ts = int(raw_filename)
        except ValueError:
            Exception(f"[Error] Nome file .raw non numerico: {raw_filename}, start_ts = 0")

        imgs = raw_event_to_images(
            raw_file,
            accumulation_time_us=100000,
            fps=10,
        )

        for img in imgs: # Le key sono timestamp in us dall'inizio del file
            frames.append((start_ts + img, imgs[img]))

    return frames


# ── Trova il frame con timestamp più vicino a 'target_ts' ────────────────────
def find_closest_frame(frames, target_ts):
    """
    Come find_closest_image ma lavora su una lista di (ts, frame_numpy)
    invece che (ts, path). Restituisce il frame numpy più vicino.
    """
    best_frame = None
    best_diff  = float("inf")
    best_timestamp = None

    for ts, frame in frames:
        diff = abs(ts - target_ts)
        if diff < best_diff:
            best_diff  = diff
            best_frame = frame
            best_timestamp = ts

    print(f"  differenza temporale event: {best_diff / 10**6:.4f} s")
    return best_frame, best_timestamp


def main():

    session = find_session()

    # Carica i parametri di calibrazione (una volta sola, fuori dal ciclo)
    calib_path = "rgb_event_calibration.npz"
    if not os.path.exists(calib_path):
        sys.exit(f"[ERRORE] File di calibrazione non trovato: {calib_path}")

    with np.load(calib_path) as data:
        rgb_K_left    = data['rgb_K_left']
        rgb_D_left    = data['rgb_D_left']
        rgb_K_right   = data['rgb_K_right']
        rgb_D_right   = data['rgb_D_right']
        event_K_left  = data['event_K_left']
        event_D_left  = data['event_D_left']
        event_K_right = data['event_K_right']
        event_D_right = data['event_D_right']
        rgb_R         = data['rgb_R']
        rgb_T         = data['rgb_T']
        event_R       = data['event_R']
        event_T       = data['event_T']

    print(f"rgb_K_left = {rgb_K_left}")

    print("Parametri di calibrazione caricati.")

    # Crea cartella rectified
    rectified_dir = os.path.join(session, "rectified")
    os.makedirs(rectified_dir, exist_ok=True)

    # Crea finestra
    cv2.namedWindow("CVLab - All Camera View", cv2.WINDOW_AUTOSIZE)

    # rgb_left è la telecamera di riferimento
    rgb_left_imgs = get_images_sorted(os.path.join(session, "rgb_left"))
    if not rgb_left_imgs:
        sys.exit("[ERRORE] Nessuna immagine in rgb_left")

    # Carica le liste delle altre 3 telecamere una volta sola (fuori dal ciclo)
    rgb_right_imgs   = get_images_sorted(os.path.join(session, "rgb_right"))
    event_left_frames  = get_event_frames_from_raw(os.path.join(session, "event_left"))
    event_right_frames = get_event_frames_from_raw(os.path.join(session, "event_right"))
    lidar_livox_files = get_images_sorted(os.path.join(session, "lidar_livox"))
    lidar_hesai_files = get_images_sorted(os.path.join(session, "lidar_hesai"))
    
    for i in range(len(rgb_left_imgs)):
        
        try:
            ref_ts, ref_path = rgb_left_imgs[i]
            print(f"\n--- Frame {i+1}/{len(rgb_left_imgs)} ---")
            print(f"Riferimento (rgb_left): ts={ref_ts}")

            # Trova le immagini più vicine nelle altre 3 cartelle
            rgb_right_path, rgb_right_ts     = find_closest_image(rgb_right_imgs,   ref_ts)
            img_event_left, event_left_ts    = find_closest_frame(event_left_frames,  ref_ts)
            img_event_right, event_right_ts  = find_closest_frame(event_right_frames, ref_ts)
            lidar_livox_path, lidar_livox_ts = find_closest_image(lidar_livox_files,   ref_ts)
            lidar_hesai_path, lidar_hesai_ts = find_closest_image(lidar_hesai_files,   ref_ts)

            # ── Carica le 4 immagini a risoluzione piena ─────────────────────────
            img_rgb_left    = load_image(ref_path)
            img_rgb_right   = load_image(rgb_right_path)
            livox_img       = load_lidar_frame(lidar_livox_path, event_K_left, event_D_left, )#x_translate=0, y_translate=200)
            hesai_img       = load_lidar_frame(lidar_hesai_path, event_K_left, event_D_left, )#x_translate=150, y_translate=200)



            # img_event_left e img_event_right sono già array numpy pronti

            # ── Rettifica la coppia RGB ───────────────────────────────────────────
            # resize_rgb ritaglia e porta le rgb a 1280x720 prima di rettificare
            img_rgb_left  = resize_rgb(img_rgb_left)
            img_rgb_right = resize_rgb(img_rgb_right)

            img_rgb_left_rect, img_rgb_right_rect, comparison = rectify(
                rgb_K_left, rgb_D_left,
                rgb_K_right, rgb_D_right,
                rgb_R, rgb_T,
                img_rgb_left, img_rgb_right
            )


            # img_rgb_left_rect = img_rgb_left
            # img_rgb_right_rect = img_rgb_right

            # ── Rettifica la coppia Event ─────────────────────────────────────────
            # Le event camera non hanno bisogno di resize_rgb (risoluzione nativa diversa)
            img_event_left_rect, img_event_right_rect, comparison = rectify(
                event_K_left, event_D_left,
                event_K_right, event_D_right,
                event_R, event_T,
                img_event_left, img_event_right
            )
            
            # img_event_left_rect = img_event_left
            # img_event_right_rect = img_event_right

            # ── Costruisci la griglia 2x2 (ridimensiona per la visualizzazione) ───
            top_row    = np.hstack([resize_for_grid(img_rgb_left_rect),   resize_for_grid(img_rgb_right_rect), resize_for_grid(livox_img)])
            bottom_row = np.hstack([resize_for_grid(img_event_left_rect), resize_for_grid(img_event_right_rect), resize_for_grid(hesai_img)])
            
            grid       = np.vstack([top_row, bottom_row])



            # ── Salva nella cartella rectified ────────────────────────────────────
            output_path = os.path.join(rectified_dir, f"{i}.png")
            cv2.imwrite(output_path, grid)
            print(f"Immagine salvata in: {output_path}")

            # ── Mostra a schermo ──────────────────────────────────────────────────
            # cv2.imshow("CVLab - All Camera View", grid)
            # cv2.moveWindow("CVLab - All Camera View", 0, 0)
            # key = cv2.waitKey(0)
            # if key == ord('q') or key == 27:   # Q o ESC per uscire prima della fine
            #     print("Uscita anticipata.")
            #     break
        except Exception as e:
            traceback.print_exc()

    cmd = [
        "ffmpeg",
        "-framerate", "10",
        "-i", f"{rectified_dir}/%d.png",
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-y",  # Overwrite output file without asking
        f"{rectified_dir}/cool_video.mp4",
    ]

    result = subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        check=True
    )

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
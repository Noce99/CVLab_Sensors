import os
from datetime import datetime

from rgb import RGBSaveStereoImages
from event import EventSaveStereoImages
import time

import subprocess


def main():
    subprocess.run(["sudo", "./scripts/disconnect_and_reconnect_event_cameras.sh"], capture_output=True, text=True)

    if not os.path.exists("./calibration_data"):
        os.mkdir("./calibration_data")

    current_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    current_folder = os.path.join("./calibration_data", current_time)
    os.mkdir(current_folder)

    left_rgb_folder = os.path.join(current_folder, "left_rgb")
    right_rgb_folder = os.path.join(current_folder, "right_rgb")
    left_event_folder = os.path.join(current_folder, "left_event")
    right_event_folder = os.path.join(current_folder, "right_event")
    os.mkdir(left_rgb_folder)
    os.mkdir(right_rgb_folder)
    os.mkdir(left_event_folder)
    os.mkdir(right_event_folder)

    rgb = RGBSaveStereoImages(
        left_rgb_folder,
        right_rgb_folder,
    )

    while not rgb.info["getting_data"]:
        time.sleep(0.1)

    event = EventSaveStereoImages(
        left_event_folder,
        right_event_folder,
    )

    # stessa cosa con gli eventi

    # uno script che controlli il timestamp delle 4 tipi di immagine e li associ in un unica cartella (es: cartella 1 contiene 4 img, cartella 2 4 img...etc) 
    # controllando anche  il timestamp tra event e rgb minore a 70 ms


    # associated_date = os.path.join("./calibration_data", "associated_date")
    # os.mkdir(associated_date)
    

    #
    try:
        input("Press Enter to stop...\n")
    except KeyboardInterrupt:
        pass

    event.kill_all()

if __name__ == "__main__":
    main()
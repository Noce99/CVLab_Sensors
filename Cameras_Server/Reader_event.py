import os
import cv2

from metavision_core.event_io.raw_reader import initiate_device
from metavision_core.event_io import EventsIterator
from metavision_sdk_core import PeriodicFrameGenerationAlgorithm, ColorPalette


def load_latest_event_frame(event_folder):
    """
    Legge l'ultimo file .raw nella cartella della event camera,
    genera i frame tramite Metavision SDK e restituisce l'ultimo frame.

    Args:
        event_folder (str): Percorso della cartella con i file .raw

    Returns:
        np.ndarray BGR oppure None se non ci sono file o la lettura fallisce
    """
    if not os.path.exists(event_folder):
        return None

    raw_files = []
    for filename in os.listdir(event_folder):
        if filename.endswith(".raw"):
            raw_files.append(os.path.join(event_folder, filename))

    if not raw_files:
        return None

    raw_files.sort()
    latest_file = raw_files[-1]

    try:
        device = initiate_device(path=latest_file)
        mv_iterator = EventsIterator.from_device(device=device)
        height, width = mv_iterator.get_size()

        frame_gen = PeriodicFrameGenerationAlgorithm(
            sensor_width=width,
            sensor_height=height,
            fps=25,
            palette=ColorPalette.Dark
        )

        last_frame = None

        def on_frame(ts, frame):
            nonlocal last_frame
            last_frame = frame.copy()

        frame_gen.set_output_callback(on_frame)

        for evs in mv_iterator:
            frame_gen.process_events(evs)

        if last_frame is None:
            return None

        # Converte in BGR se necessario (Metavision restituisce BGR o GRAY a seconda della palette)
        if len(last_frame.shape) == 2:
            last_frame = cv2.cvtColor(last_frame, cv2.COLOR_GRAY2BGR)

        return last_frame

    except Exception as e:
        print(f"WARNING: Impossibile leggere {latest_file}: {e}")
        return None
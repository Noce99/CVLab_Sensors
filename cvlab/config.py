"""Single source of truth for paths, sensor addresses and acquisition settings."""

import os
from pathlib import Path

import numpy as np

# ============================================================================
# PATHS
# ============================================================================

REPO_ROOT = Path(__file__).resolve().parent.parent
PYTHON = REPO_ROOT / "venv_3_12" / "bin" / "python"

# Sessions are recorded into STAGING_DIR and moved (atomic rename, same
# filesystem) to SESSIONS_DIR when saved: <SESSIONS_DIR>/<YYYY_MM_DD__HH_MM_SS>/
STAGING_DIR = Path(os.environ.get("CVLAB_STAGING_DIR", REPO_ROOT / "get_data" / "data"))
SESSIONS_DIR = Path(os.environ.get("CVLAB_SESSIONS_DIR", REPO_ROOT / "CVLab_live_imgs"))
SESSION_NAME_FORMAT = "%Y_%m_%d__%H_%M_%S"

# Recorders
GET_DATA_DIR = REPO_ROOT / "get_data"
RGB_RECORDER = GET_DATA_DIR / "rgb.py"
EVENT_RECORDER = GET_DATA_DIR / "event.py"
LIVOX_RECORDER = GET_DATA_DIR / "cpp" / "save_point_cloud_livox"
HESAI_RECORDER = GET_DATA_DIR / "cpp" / "save_point_cloud_hesai"
LIVOX_CONFIG = GET_DATA_DIR / "config" / "livox_hap.json"
RESET_EVENT_CAMERAS_SCRIPT = GET_DATA_DIR / "scripts" / "disconnect_and_reconnect_event_cameras.sh"

# Recording stops automatically when the staging disk has less free space than this
MIN_FREE_DISK_GB = 2.0

# Sub-folders of a session
SESSION_SUBFOLDERS = [
    "rgb_left", "rgb_right",
    "event_left", "event_right",
    "lidar_livox", "lidar_hesai",
    "logs", "previews", "status",
]

# ============================================================================
# RGB CAMERAS (Basler, GigE)
# ============================================================================

RGB_LEFT_IP = "192.168.1.70"
RGB_RIGHT_IP = "192.168.1.71"

# Each camera streams to its own host NIC
RGB_STREAM = {
    0: {"destination": "192.168.1.52", "port": 5556},
    1: {"destination": "192.168.1.52", "port": 5555},
}

RGB_FPS = 10
# Region of interest, as a fraction of the full 2448x2048 sensor (centered)
RGB_SENSOR_SIZE = (2448, 2048)
RGB_ROI_FRACTION = (0.8, 0.6)
RGB_PIXEL_FORMAT = "RGB8"

# The rig is on a moving car: keep exposures short and let the gain compensate.
RGB_EXPOSURE_LOWER_US = 20.0
RGB_EXPOSURE_UPPER_US = 4000.0
RGB_GAIN_UPPER_DB = 24.0

# PTP: cameras must be PTP slaves with an offset below this before grabbing
RGB_REQUIRE_PTP = True
RGB_PTP_MAX_OFFSET_NS = 1_000_000
RGB_PTP_TIMEOUT_S = 60

# Saving
RGB_PNG_COMPRESSION = 1     # 0-9, low = fast
RGB_WRITER_THREADS = 4
RGB_QUEUE_SIZE = 60         # frames waiting to be written; beyond this frames are dropped (and counted)

# ============================================================================
# EVENT CAMERAS (Prophesee EVK4, USB)
# ============================================================================

EVENT_LEFT_SERIAL = "00051962"
EVENT_RIGHT_SERIAL = "00051961"
# Record the EVK4 trigger input (MAIN channel). Wire a Basler output line
# (LineSource = ExposureActive) into it to timestamp every RGB exposure in the event clock.
EVENT_TRIGGER_IN = False
# Pixels masked in hardware (x, y). Find them with get_data/find_hot_pixels.py.
EVENT_HOT_PIXELS = {
    EVENT_LEFT_SERIAL: [(1186, 491), (397, 348), (798, 380)],  # 87% of all events on 2026-10-09
    EVENT_RIGHT_SERIAL: [(1240, 176)],  # 83% of all events on 2026-10-09
}

# ============================================================================
# LIDARS
# ============================================================================

HESAI_IP = "192.168.1.91"   # XT32
LIVOX_IP = "192.168.1.100"  # HAP (set in LIVOX_CONFIG)

# Coarse rotation from each lidar's native frame to the camera convention
# (x right, y down, z forward). Replace with calibrated extrinsics when available.
#   Livox native: x forward, y left, z up
#   Hesai: same mapping that was used by the old recorder (verify on the car)
LIDAR_TO_CAMERA_AXES = {
    "livox": np.array([[0, -1, 0],
                       [0, 0, -1],
                       [1, 0, 0]], dtype=np.float32),
    "hesai": np.array([[-1, 0, 0],
                       [0, 0, -1],
                       [0, -1, 0]], dtype=np.float32),
}

# ============================================================================
# CALIBRATION
# ============================================================================

PATTERN_SIZE = (9, 6)       # inner corners (cols, rows)
SQUARE_SIZE_MM = 29.0

# RGB images are center-cropped to 16:9 and resized to the event resolution
# before calibration; every consumer of the calibration must do the same.
CALIBRATION_IMAGE_SIZE = (1280, 720)

# Active calibration, written by RGB_Event_Calibration/calibrate.py
RGB_EVENT_CALIBRATION_FILE = REPO_ROOT / "CVLab_live_imgs" / "rgb_event_calibration.npz"

# ============================================================================
# PREVIEWS / STATUS
# ============================================================================

PREVIEW_PERIOD_S = 0.5
PREVIEW_WIDTH = 640
STATUS_PERIOD_S = 1.0

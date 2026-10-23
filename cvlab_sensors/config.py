"""Single source of truth for paths, sensor addresses and acquisition settings.

The sensor settings marked "(default: Settings page)" are only defaults: the values saved on the
Settings page (SETTINGS_FILE, see cvlab_sensors.settings.store) override them.
"""

import os
from pathlib import Path

import numpy as np

# ============================================================================
# PATHS
# ============================================================================

REPO_ROOT = Path(__file__).resolve().parent.parent
PYTHON = REPO_ROOT / "venv_3_12" / "bin" / "python"

# Everything the tools write (gitignored)
DATA_DIR = Path(os.environ.get("CVLAB_DATA_DIR", REPO_ROOT / "data")).resolve()

# Sessions are recorded into STAGING_DIR and moved (atomic rename, same
# filesystem) to SESSIONS_DIR when saved: <SESSIONS_DIR>/<YYYY_MM_DD__HH_MM_SS>/
STAGING_DIR = Path(os.environ.get("CVLAB_STAGING_DIR", DATA_DIR / "staging")).resolve()
SESSIONS_DIR = Path(os.environ.get("CVLAB_SESSIONS_DIR", DATA_DIR / "sessions")).resolve()
SESSION_NAME_FORMAT = "%Y_%m_%d__%H_%M_%S"

# Sensor settings saved on the Settings page (only the values that differ from the defaults here)
SETTINGS_FILE = DATA_DIR / "settings" / "active.json"

# Calibration captures and results: <CALIBRATION_DIR>/<tool>/{captures,results}/
CALIBRATION_DIR = DATA_DIR / "calibration"
# Outputs of the lidar recorders run by the live calibration tools, emptied as they are read
STREAM_DIR = DATA_DIR / "stream"
# Output of the web app (not the terminal: see cvlab_sensors.app.detach_from_terminal), moved to
# app.log.1 at start beyond APP_LOG_MAX_BYTES
APP_LOG = DATA_DIR / "logs" / "app.log"
APP_LOG_MAX_BYTES = 50_000_000

# Recorders: Python modules (run with python -m from REPO_ROOT) and native programs
RGB_RECORDER = "cvlab_sensors.recording.rgb_recorder"
EVENT_RECORDER = "cvlab_sensors.recording.event_recorder"
LIDAR_RECORDERS_DIR = REPO_ROOT / "lidar_recorders"
LIVOX_RECORDER = LIDAR_RECORDERS_DIR / "save_point_cloud_livox"
HESAI_RECORDER = LIDAR_RECORDERS_DIR / "save_point_cloud_hesai"
LIVOX_CONFIG = LIDAR_RECORDERS_DIR / "livox_hap.json"   # network only: the SDK does not send its lidar_configs (scan pattern... are Settings page values)
# Run as root through sudo -n: the root-owned installed copy (the repo copy is writable by the user,
# a sudoers rule on it would give root to anyone running as that user), the repo copy until installed
RESET_EVENT_CAMERAS_INSTALLED = Path("/usr/local/sbin/cvlab-reset-event-cameras")
RESET_EVENT_CAMERAS_SCRIPT = (RESET_EVENT_CAMERAS_INSTALLED if RESET_EVENT_CAMERAS_INSTALLED.exists()
                              else REPO_ROOT / "system" / "reset_event_cameras.sh")

# Recording stops automatically when the staging disk has less free space than this
MIN_FREE_DISK_GB = 2.0

# Sub-folders of a session
SESSION_SUBFOLDERS = [
    "rgb_left", "rgb_right",
    "event_left", "event_right",
    "lidar_livox", "lidar_hesai",
    "logs", "previews", "status", "settings",
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

RGB_FPS = 10   # (default: Settings page)
# Trigger every frame from the PTP clock (Basler Periodic Signal): with both cameras
# PTP-locked, left and right expose at the same instant, on multiples of 1/RGB_FPS s.
RGB_PTP_TRIGGER = True   # (default: Settings page)
# Region of interest, as a fraction of the full 2448x2048 sensor (centered)
RGB_SENSOR_SIZE = (2448, 2048)
RGB_ROI_FRACTION = (0.8, 0.6)   # (default: Settings page)
RGB_PIXEL_FORMAT = "BayerRG8"   # (default: Settings page) raw mosaic: 1/3 of RGB8 on the link and on disk

# The rig is on a moving car: keep exposures short and let the gain compensate.
RGB_EXPOSURE_LOWER_US = 20.0   # (default: Settings page)
RGB_EXPOSURE_UPPER_US = 4000.0   # (default: Settings page)
RGB_GAIN_UPPER_DB = 24.0   # (default: Settings page)
RGB_AUTO_TARGET_BRIGHTNESS = 0.5   # (default: Settings page) 0-1, mean level the auto exposure/gain aims for (camera default: 0.3)

# PTP: cameras must be PTP slaves with an offset below this before grabbing
RGB_REQUIRE_PTP = True
RGB_PTP_MAX_OFFSET_NS = 1_000_000
RGB_PTP_TIMEOUT_S = 60

# Saving
RGB_PNG_COMPRESSION = 1     # (default: Settings page) 0-9, low = fast
RGB_WRITER_THREADS = 6
# Frames waiting to be written; beyond this frames are dropped (and counted). 300 = 15 s of both cameras
# at 10 fps, to ride out disk stalls (~0.7 GB in BayerRG8, ~2.2 GB in RGB8)
RGB_QUEUE_SIZE = 300
RGB_GRAB_BUFFERS = 30       # pylon buffers per camera: frames received but not yet retrieved (pylon default 10)

# ============================================================================
# EVENT CAMERAS (Prophesee EVK4, USB)
# ============================================================================

EVENT_LEFT_SERIAL = "00051962"
EVENT_RIGHT_SERIAL = "00051961"
# Record the EVK4 trigger input (MAIN channel). Wire a Basler output line
# (LineSource = ExposureActive) into it to timestamp every RGB exposure in the event clock.
EVENT_TRIGGER_IN = False   # (default: Settings page)
# Hardware event rate cap (ERC) of each EVK4 in the live calibration tool, in events/s.
# Flickering scenes reach 60 Mev/s, more than the tool can render live. Beyond the cap the
# sensor drops events evenly in time (None = no cap).
EVENT_CALIBRATION_MAX_RATE = 20_000_000   # (default: Settings page)
# The same for recordings: uncapped, a flickering lab reached ~38 Mev/s per camera
# (~150 MB/s of raw data each, events delivered late) on 2026-10-09.
EVENT_RECORD_MAX_RATE = 20_000_000   # (default: Settings page)
# Pixels masked in hardware (x, y). Find them with python -m cvlab_sensors.tools.find_hot_pixels.
EVENT_HOT_PIXELS = {
    EVENT_LEFT_SERIAL: [(1186, 491), (397, 348), (798, 380)],  # 87% of all events on 2026-10-09
    EVENT_RIGHT_SERIAL: [(1240, 176)],  # 83% of all events on 2026-10-09
}

# ============================================================================
# LIDARS
# ============================================================================

HESAI_IP = "192.168.1.91"   # XT32
# Livox HAP: 192.168.1.100, set in LIVOX_CONFIG

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
# Default chessboard square sizes: each calibration uses its own board
RGB_EVENT_SQUARE_SIZE_MM = 34.0
RGB_LIVOX_SQUARE_SIZE_MM = 30.0

# Chessboard search: (gamma, blur kernel, CLAHE clip limit) variants tried in order until one
# finds the board. gamma < 1 brightens, > 1 darkens; blur 0 = none; CLAHE 0 = none (16x16 tiles).
# Measured on the 2026-10-09 recordings:
# RGB: brightening (or a light blur) finds 2.5x more boards than the plain image. CLAHE finds
# the faint board of a blinking screen caught while switching (11/16 frames, plain 2/16).
# gamma 10 is for a chessboard on an overexposed screen.
# Events are searched in several renderings of the same window (cvlab_sensors.events.calibration_frames),
# in the order of EVENT_RENDERINGS:
#   event_lockin    ON - OFF per pixel, phase-aligned over 100 ms (blinking board faster than
#                   the window: the squares blinking in opposite phase end up white / black)
#   event_polarity  ON - OFF events per pixel over 60 ms (blinking board, one blink in the window)
#   event_count     events per pixel, both polarities (moving board: its edges)
CHESSBOARD_VARIANTS = {
    "rgb": [(1.0, 0, 0), (0.5, 0, 0), (0.3, 0, 0), (1.0, 5, 0), (1.0, 5, 16), (1.0, 0, 8), (10.0, 5, 0)],
    "event_count": [(1.0, 0, 0), (2.0, 0, 0), (3.0, 5, 0), (0.5, 5, 0)],
    # Gaussian blur turns the speckle into flat squares: found 30/30 synthetic boards (raw: 26/30)
    "event_polarity": [(1.0, 13, 0), (1.0, 17, 0)],
    "event_lockin": [(1.0, 0, 0), (1.0, 5, 0), (1.0, 9, 0)],
}
EVENT_RENDERINGS = ["lockin", "polarity", "count"]

# RGB images are center-cropped to 16:9 and resized to the event resolution
# before calibration; every consumer of the calibration must do the same.
CALIBRATION_IMAGE_SIZE = (1280, 720)

# Installed calibrations, written by the solvers (cvlab_sensors/calibration/*/solve.py)
RGB_EVENT_CALIBRATION_FILE = CALIBRATION_DIR / "rgb_event_calibration.npz"
RGB_LIVOX_CALIBRATION_FILE = CALIBRATION_DIR / "rgb_livox_calibration.npz"

# RGB <-> Livox: the Livox points are projected by a virtual pinhole camera (camera-convention
# axes, no distortion) into a reflectivity image where the chessboard is searched. The HAP sees
# ~120 x 25 deg: a wide, short image.
LIVOX_IMAGE_SIZE = (1500, 400)
LIVOX_FOCAL_PX = 800.0
LIVOX_CAPTURE_S = 8          # default seconds of points per board (the HAP pattern fills in over time)

# Livox <-> Hesai (target-free, static scenes): P_livox = R P_hesai + t in the camera convention.
LIVOX_HESAI_CALIBRATION_FILE = CALIBRATION_DIR / "livox_hesai_calibration.npz"
LIVOX_HESAI_CAPTURE_S = 15   # default seconds per stop
# Initial guess: Hesai origin in the Livox frame (x right, y down, z forward), mm. A ruler is enough:
# the registration converges from +-10 deg / +-30 cm. Measured by registration on 2026-10-13.
LIVOX_HESAI_INITIAL_T_MM = (-18.0, -121.0, -62.0)

# ============================================================================
# PREVIEWS / STATUS
# ============================================================================

PREVIEW_PERIOD_S = 0.5
PREVIEW_WIDTH = 640
STATUS_PERIOD_S = 1.0

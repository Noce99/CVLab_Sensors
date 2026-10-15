"""Basler frames in either pixel format (Settings page): RGB8 (H×W×3) or BayerRG8 (H×W raw mosaic).

Bayer frames are saved as they come (1-channel PNG, a third of the size) and demosaiced when read.
OpenCV names Bayer patterns from the second row/column: the camera's RGGB (BayerRG8) is OpenCV's
"BG" (checked against RGB8 frames of the same scene on 2026-10-15).
"""

import cv2

BAYER_TO_RGB = cv2.COLOR_BayerBG2RGB
BAYER_TO_BGR = cv2.COLOR_BayerBG2BGR
PIXEL_FORMATS = {"BayerRG8": 1, "RGB8": 3}   # bytes per pixel


def to_rgb(frame):
    """RGB image of a camera frame in either format."""
    return cv2.cvtColor(frame, BAYER_TO_RGB) if frame.ndim == 2 else frame


def to_bgr(frame):
    """BGR image (OpenCV) of a camera frame in either format."""
    return cv2.cvtColor(frame, BAYER_TO_BGR if frame.ndim == 2 else cv2.COLOR_RGB2BGR)


def saved_image(frame):
    """What the recorder writes: the raw mosaic as is, RGB frames as BGR (cv2.imwrite)."""
    return frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)


def read_bgr(path):
    """BGR image of a recorded frame (PNG): 1-channel PNGs are Bayer mosaics. None if unreadable."""
    image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim == 3:
        return image
    return cv2.cvtColor(image, BAYER_TO_BGR)

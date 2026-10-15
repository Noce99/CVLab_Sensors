"""The Livox seen as a camera: a virtual pinhole (no distortion) projects the points, in the camera
convention of cvlab_sensors.lidar (x right, y down, z forward), into a reflectivity image of
config.LIVOX_IMAGE_SIZE where the chessboard is searched like in a camera image.

Projection, reflectivity image and the depth helpers follow
~/Downloads/LidarCameraCalibration_Solution.ipynb; the chessboard search is tuned on live data (see
find_lidar_corners).
"""

import cv2
import numpy as np

from cvlab_sensors import config
from cvlab_sensors.calibration.common import adjust_gamma, canonical_corners
from cvlab_sensors.sensors.lidar import MIN_DISTANCE_MM

# White levels tried, in order: intensities above are clipped. A printed board is dark for the Livox
# (white squares ~6-11 of 255 at 1-2 m) next to bright outliers, so the board is found by a low clip
LIDAR_WHITE_LEVELS = (6, 8, 10, 12, 15, 20, 30, 50, 80, 130, 255)
PREVIEW_GAMMA = 5


def lidar_K(size=config.LIVOX_IMAGE_SIZE, focal=config.LIVOX_FOCAL_PX):
    """Intrinsics of the virtual lidar camera; pixel centres at integer coordinates."""
    w, h = size
    return np.array([[focal, 0, (w - 1) / 2], [0, focal, (h - 1) / 2], [0, 0, 1]], dtype=np.float64)


LIDAR_D = np.zeros((5, 1))


def pixels(xyz, size=config.LIVOX_IMAGE_SIZE):
    """(row * W + col) of every point that falls in the lidar image, and the index of those points."""
    w, h = size
    K = lidar_K(size)
    idx = np.nonzero(xyz[:, 2] > MIN_DISTANCE_MM)[0]
    p = xyz[idx].astype(np.float64)
    u = np.rint(K[0, 0] * p[:, 0] / p[:, 2] + K[0, 2]).astype(np.int64)
    v = np.rint(K[1, 1] * p[:, 1] / p[:, 2] + K[1, 2]).astype(np.int64)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    return v[inside] * w + u[inside], idx[inside]


def reflectivity_image(xyz, intensity, size=config.LIVOX_IMAGE_SIZE):
    """uint8 (H, W) image, 0 where no point falls. Points are in time order: the last one of a pixel
    wins (the notebook compared nearest / mean / median...: no variant detects better)."""
    w, h = size
    flat, idx = pixels(xyz, size)
    image = np.zeros(h * w, np.uint8)
    last, first_in_reverse = np.unique(flat[::-1], return_index=True)
    image[last] = intensity[idx][len(flat) - 1 - first_in_reverse]
    return image.reshape(h, w)


def depth_image(xyz, size=config.LIVOX_IMAGE_SIZE, near_mm=500, far_mm=30000, dot_px=1):
    """BGR (H, W) image of the nearest point per pixel, magma colour map on a log scale from near_mm
    (bright) to far_mm (dark purple), black where no point falls. The scale is fixed so a colour is
    the same distance in every frame and for every lidar. dot_px > 1 draws every point as a
    dot_px square (the nearest one wins where dots overlap), for sparse lidars."""
    w, h = size
    flat, idx = pixels(xyz, size)
    zbuffer = np.full(h * w, np.inf, np.float32)
    np.minimum.at(zbuffer, flat, xyz[idx, 2].astype(np.float32))
    if dot_px > 1:
        zbuffer = cv2.erode(zbuffer.reshape(h, w), np.ones((dot_px, dot_px), np.uint8),
                            borderValue=np.inf).reshape(-1)
    valid = np.isfinite(zbuffer)
    level = np.zeros(h * w, np.uint8)
    # 255 near -> 32 far: the darkest end of magma is kept for "no point"
    t = np.log(np.clip(zbuffer[valid], near_mm, far_mm) / near_mm) / np.log(far_mm / near_mm)
    level[valid] = np.rint(255 - t * (255 - 32)).astype(np.uint8)
    colour = cv2.applyColorMap(level.reshape(h, w), cv2.COLORMAP_MAGMA)
    colour[~valid.reshape(h, w)] = 0
    return colour


def find_lidar_corners(image):
    """(corners (N, 2) canonically ordered, white level) of the chessboard in a reflectivity image,
    or (None, None).

    What the Livox sees of a board: the black squares return (almost) nothing, so they are the 0 of
    "no point", and look smaller than they are (a spot that touches white returns white); the white
    squares are very dark with scan holes. Tested on 20 s of a still board at ~1.5 m where the old
    search (gammas, default flags) never found it: the exhaustive search on the clipped image finds it
    in every window of >= 4 s (15/18 of 2 s), corners within 0.2 px (median) between windows."""
    for white in LIDAR_WHITE_LEVELS:
        clipped = np.sqrt(np.minimum(image.astype(np.float32), white) / white)
        found, corners = cv2.findChessboardCornersSB((clipped * 255).astype(np.uint8), config.PATTERN_SIZE,
                                                     flags=cv2.CALIB_CB_EXHAUSTIVE)
        if found:
            return canonical_corners(corners), white
    return None, None


def display(image, gamma=PREVIEW_GAMMA):
    """Brightened reflectivity image to look at."""
    return adjust_gamma(image, 1.0 / gamma)


# ============================================================================
# DEPTH ON A CAMERA IMAGE
# ============================================================================

def depth_map(xyz_lidar_cam, K, D, R, T, size, max_radius=None):
    """Project the lidar points into a camera and keep, for each pixel, the closest one (z-buffer).

    xyz_lidar_cam: (N, 3) points in the lidar camera frame (mm); R, T: P_lidar_cam = R P_cam + T.
    max_radius: normalized radius beyond which the camera distortion model is not valid (it folds
    back: far rays would land inside the image), see rgb_event.solve.monotonic_radius.
    Returns depth (H, W) float32 mm, 0 where there is no point.
    """
    w, h = size
    P = (xyz_lidar_cam.astype(np.float64) - T.reshape(1, 3)) @ R   # P_cam = R^T (P_lidar_cam - T)
    P = P[P[:, 2] > MIN_DISTANCE_MM]
    if max_radius is not None:
        P = P[(P[:, 0] ** 2 + P[:, 1] ** 2) / P[:, 2] ** 2 < max_radius ** 2]
    # Cheap pre-filter without distortion (with a margin), then the exact projection
    u0 = K[0, 0] * P[:, 0] / P[:, 2] + K[0, 2]
    v0 = K[1, 1] * P[:, 1] / P[:, 2] + K[1, 2]
    P = P[(u0 > -0.5 * w) & (u0 < 1.5 * w) & (v0 > -0.5 * h) & (v0 < 1.5 * h)]
    if not len(P):
        return np.zeros((h, w), np.float32)
    uv, _ = cv2.projectPoints(P.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
    uv = np.rint(uv.reshape(-1, 2)).astype(np.int64)
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    flat = uv[inside, 1] * w + uv[inside, 0]
    zbuffer = np.full(h * w, np.inf)
    np.minimum.at(zbuffer, flat, P[inside, 2])
    return np.where(np.isinf(zbuffer), 0, zbuffer).astype(np.float32).reshape(h, w)


def clean_depth(depth, window=5, rel_tol=0.1, min_neighbours=3):
    """Two cheap fixes for a sparse depth map (0 = no measurement):
      1. see-through: a far pixel with a much nearer one in its window (a background point that
         slipped between the samples of a foreground object) is removed,
      2. small holes: an empty pixel with at least min_neighbours valid neighbours at a similar depth
         gets their mean (it never bridges a depth discontinuity).
    rel_tol is a relative depth difference (0.1 = 10 %)."""
    depth = depth.astype(np.float32)
    kernel = np.ones((window, window), np.uint8)
    big = np.float32(1e9)

    valid = depth > 0
    local_min = cv2.erode(np.where(valid, depth, big), kernel)
    depth = np.where(valid & (depth > local_min * (1 + rel_tol)), 0, depth).astype(np.float32)

    valid = depth > 0
    count = cv2.boxFilter(valid.astype(np.float32), -1, (window, window), normalize=False)
    mean = cv2.boxFilter(depth * valid, -1, (window, window), normalize=False) / np.maximum(count, 1)
    local_min = cv2.erode(np.where(valid, depth, big), kernel)
    local_max = cv2.dilate(depth, kernel)
    fill = ~valid & (count >= min_neighbours) & ((local_max - local_min) < rel_tol * mean)
    return np.where(fill, mean, depth).astype(np.float32)


def colour_depth(depth, max_depth_mm=6000):
    """BGR image of a depth map, near = red, far = blue, black where there is no depth."""
    colour = cv2.applyColorMap(np.clip(255 - depth / max_depth_mm * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
    colour[depth <= 0] = 0
    return colour

"""Images of the sensors for the record and replay pages, and their reprojection on RGB-left.

Reprojection uses the installed calibrations (cvlab.config.*_CALIBRATION_FILE):
  - lidar points are 3D: they land exactly on RGB-left (Livox through RGB <-> Livox, Hesai through
    Livox <-> Hesai then RGB <-> Livox);
  - the other cameras' images have no depth: they are warped through a plane at a chosen distance
    in front of RGB-left (exact for things at that distance, parallax elsewhere).
Camera frames: x right, y down, z forward, mm. The distortion models are only used inside the
radius where they are valid (rgb_event.solve.monotonic_radius): beyond it they fold back.
"""

import os
import threading

import cv2
import numpy as np

from Calibration.rgb_event.solve import monotonic_radius
from Calibration.rgb_livox import lidar_image
from cvlab import config
from cvlab.calibration import full_frame_intrinsics, load_calibration
from cvlab.lidar import MIN_DISTANCE_MM

PANORAMA_WIDTH = 1440              # 0.25 deg per pixel
PANORAMA_ELEVATION = (-20.0, 20.0)  # deg, the XT32 sees -16..+15
DEPTH_RANGE_MM = (500.0, 30000.0)  # colour scale of the lidar depth (log)
LIDAR_DOT_PX = 2


def livox_view(xyz, intensity):
    """Reflectivity of camera-convention points in the Livox virtual camera (BGR)."""
    return cv2.cvtColor(lidar_image.display(lidar_image.reflectivity_image(xyz, intensity)), cv2.COLOR_GRAY2BGR)


def panorama(xyz, intensity, width=PANORAMA_WIDTH):
    """360 deg reflectivity of camera-convention points (azimuth 0 = forward, in the middle)."""
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    r = np.hypot(x, z)
    keep = np.hypot(r, y) > MIN_DISTANCE_MM
    azimuth = np.degrees(np.arctan2(x[keep], z[keep]))
    elevation = np.degrees(np.arctan2(-y[keep], r[keep]))
    e0, e1 = PANORAMA_ELEVATION
    height = int(width * (e1 - e0) / 360)
    u = ((azimuth + 180) / 360 * width).astype(int).clip(0, width - 1)
    v = ((e1 - elevation) / (e1 - e0) * height).astype(int)
    inside = (v >= 0) & (v < height)
    image = np.zeros((height, width), np.uint8)
    image[v[inside], u[inside]] = intensity[keep][inside]
    image = cv2.dilate(image, np.ones((3, 1), np.uint8))   # the rings are 1 deg (4 px) apart
    return cv2.cvtColor(lidar_image.display(image), cv2.COLOR_GRAY2BGR)


def depth_colours(depth_mm):
    """BGR colour of each depth: red near, blue far (log scale over DEPTH_RANGE_MM)."""
    lo, hi = np.log(DEPTH_RANGE_MM[0]), np.log(DEPTH_RANGE_MM[1])
    value = np.clip((np.log(np.maximum(depth_mm, 1)) - lo) / (hi - lo), 0, 1)
    return cv2.applyColorMap((255 * (1 - value)).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)


class Rig:
    """The installed calibrations, as transforms from each sensor to RGB-left."""

    def __init__(self):
        self.missing = {}     # sensor -> why it cannot be reprojected
        self.cameras = {}     # sensor -> (K, D at calibration size, R, T: x_sensor = R x_rgb_left + T)
        self.lidars = {}      # sensor -> (R, T): P_rgb_left = R P_sensor_camera_convention + T
        self.K_left = self.D_left = None
        self.maps = {}
        self.lock = threading.Lock()
        self.load()

    def load(self):
        try:
            c = load_calibration(config.RGB_EVENT_CALIBRATION_FILE)
        except (OSError, ValueError) as e:
            for s in ("rgb_left", "rgb_right", "event_left", "event_right", "livox", "hesai"):
                self.missing[s] = f"no RGB ↔ event calibration ({e})"
            return
        self.K_left, self.D_left = c["rgb_K_left"], c["rgb_D_left"]
        R_le, T_le = c["rgb_event_R"], c["rgb_event_T"].reshape(3)
        R_ee, T_ee = c["event_R"], c["event_T"].reshape(3)
        self.cameras = {
            "rgb_right": (c["rgb_K_right"], c["rgb_D_right"], c["rgb_R"], c["rgb_T"].reshape(3)),
            "event_left": (c["event_K_left"], c["event_D_left"], R_le, T_le),
            "event_right": (c["event_K_right"], c["event_D_right"], R_ee @ R_le, R_ee @ T_le + T_ee),
        }
        if not os.path.exists(config.RGB_LIVOX_CALIBRATION_FILE):
            for s in ("livox", "hesai"):
                self.missing[s] = "no RGB ↔ Livox calibration"
            return
        rl = np.load(config.RGB_LIVOX_CALIBRATION_FILE)
        R_rl, T_rl = rl["R"], rl["T"].reshape(3)        # P_livox = R_rl P_rgb + T_rl
        self.lidars["livox"] = (R_rl.T, -R_rl.T @ T_rl)
        if not os.path.exists(config.LIVOX_HESAI_CALIBRATION_FILE):
            self.missing["hesai"] = "no Livox ↔ Hesai calibration"
            return
        lh = np.load(config.LIVOX_HESAI_CALIBRATION_FILE)
        R_lh, t_lh = lh["R"], lh["t"].reshape(3)        # P_livox = R_lh P_hesai + t_lh
        self.lidars["hesai"] = (R_rl.T @ R_lh, R_rl.T @ (t_lh - T_rl))

    def base(self, size):
        """K, D of RGB-left for an image of `size` (the calibration size or the full frame)."""
        if tuple(size) == tuple(config.CALIBRATION_IMAGE_SIZE):
            return self.K_left, self.D_left
        return full_frame_intrinsics(self.K_left, size), self.D_left

    def project_lidar(self, sensor, xyz, size):
        """Pixels (N, 2) int and depths (N,) mm on RGB-left (image of `size`) of camera-convention points."""
        R, T = self.lidars[sensor]
        K, D = self.base(size)
        P = xyz.astype(np.float64) @ R.T + T
        P = P[P[:, 2] > MIN_DISTANCE_MM]
        r_max = monotonic_radius(D)
        P = P[(P[:, 0] ** 2 + P[:, 1] ** 2) / P[:, 2] ** 2 < r_max ** 2]
        if not len(P):
            return np.zeros((0, 2), int), np.zeros(0)
        uv, _ = cv2.projectPoints(P.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
        uv = np.rint(uv.reshape(-1, 2)).astype(int)
        w, h = size
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        return uv[inside], P[inside, 2]

    def warp_maps(self, sensor, size, depth_mm):
        """remap maps sampling `sensor` (calibration-size image) for each RGB-left pixel of an image
        of `size`, through the plane z = depth_mm in front of RGB-left; -1 outside its view."""
        key = (sensor, tuple(size), int(depth_mm))
        with self.lock:
            if key in self.maps:
                return self.maps[key]
        K, D = self.base(size)
        K_s, D_s, R, T = self.cameras[sensor]
        w, h = size
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        rays = cv2.undistortPoints(np.stack([u, v], -1).reshape(-1, 1, 2), K, D).reshape(-1, 2)
        X = np.hstack([rays, np.ones((len(rays), 1))]) * depth_mm
        Xs = X @ R.T + T
        z = Xs[:, 2]
        xn, yn = Xs[:, 0] / z, Xs[:, 1] / z
        valid = (z > 0) & (xn ** 2 + yn ** 2 < monotonic_radius(D_s) ** 2)
        uv, _ = cv2.projectPoints(Xs.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K_s, D_s)
        uv = uv.reshape(-1, 2)
        uv[~valid] = -1
        maps = (uv[:, 0].reshape(h, w).astype(np.float32), uv[:, 1].reshape(h, w).astype(np.float32))
        with self.lock:
            if len(self.maps) > 16:
                self.maps.clear()
            self.maps[key] = maps
        return maps

    def warp(self, sensor, image, size, depth_mm):
        """(warped image, mask) of a calibration-size image of `sensor` on RGB-left."""
        mx, my = self.warp_maps(sensor, size, depth_mm)
        out = cv2.remap(image, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return out, mx >= 0


def blend(base, layer, mask, alpha):
    """base where mask is False, alpha-blend of layer on base where it is True (in place)."""
    if alpha <= 0 or not mask.any():
        return base
    base[mask] = (base[mask].astype(np.float32) * (1 - alpha) + layer[mask].astype(np.float32) * alpha).astype(np.uint8)
    return base


def draw_points(shape, uv, colours, dot=LIDAR_DOT_PX):
    """(layer, mask) with square dots, the near points drawn last (on top)."""
    h, w = shape[:2]
    layer = np.zeros((h, w, 3), np.uint8)
    mask = np.zeros((h, w), bool)
    for dy in range(dot):
        for dx in range(dot):
            u = np.clip(uv[:, 0] + dx, 0, w - 1)
            v = np.clip(uv[:, 1] + dy, 0, h - 1)
            layer[v, u] = colours
            mask[v, u] = True
    return layer, mask

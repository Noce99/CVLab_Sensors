"""Reading lidar H5 files (format_version 2) and projecting them to images."""

import bisect
import os

import cv2
import h5py
import numpy as np

from cvlab import config
from cvlab.timesync import ClockMapping

MIN_DISTANCE_MM = 300
MAX_DISTANCE_MM = 20_000


class LidarChunk:
    """One H5 file written by save_point_cloud_{livox,hesai}."""

    def __init__(self, path):
        self.path = path
        with h5py.File(path, "r") as f:
            version = int(f.attrs.get("format_version", 1))
            if version != 2:
                raise ValueError(f"{path}: unsupported lidar format_version {version}")
            self.xyz_mm = f["xyz"][:]                  # (N, 3) native sensor frame
            self.timestamp_ns = f["timestamp_ns"][:]   # (N,) lidar clock
            self.intensity = f["intensity"][:]         # (N,)
            self.sync = f["sync"][:]                   # (M, 2) [lidar_ns, host_ns]
            self.lidar = _as_str(f.attrs["lidar"])
            self.time_source = _as_str(f.attrs["time_source"])

    def host_minus_lidar_ns(self):
        """Offset from the lidar clock to the host clock.

        Every sync pair is (lidar time, host receive time); the receive latency
        is always positive, so the minimum difference is the best estimate.
        """
        if len(self.sync) == 0:
            return None
        return int(np.min(self.sync[:, 1] - self.sync[:, 0]))

    def host_timestamps_ns(self, mapping=None):
        """Per-point host time. Pass the session-wide mapping (LidarFolder.clock())
        for best accuracy; without it the offset of this file alone is used."""
        if mapping is not None:
            return mapping.to_host_ns(self.timestamp_ns)
        offset = self.host_minus_lidar_ns()
        return None if offset is None else self.timestamp_ns + offset

    def xyz_camera_mm(self):
        """Points in the camera convention (x right, y down, z forward)."""
        return self.xyz_mm @ config.LIDAR_TO_CAMERA_AXES[self.lidar].T


def _as_str(value):
    return value.decode() if isinstance(value, bytes) else str(value)


def list_chunks(folder):
    """Sorted [(host_timestamp_us, path)] of the complete H5 files in a folder."""
    chunks = []
    for name in os.listdir(folder):
        if name.endswith(".h5"):  # *.h5.tmp files are still being written
            try:
                chunks.append((int(name[:-3]), os.path.join(folder, name)))
            except ValueError:
                pass
    chunks.sort()
    return chunks


def project_points(points_cam_mm, K, D=None, image_size=config.CALIBRATION_IMAGE_SIZE):
    """Project camera-frame points; returns (pixels (N, 2) int, depths_mm (N,)) of the visible points."""
    width, height = image_size
    in_front = points_cam_mm[:, 2] > MIN_DISTANCE_MM
    pts = points_cam_mm[in_front].astype(np.float64)
    if len(pts) == 0:
        return np.zeros((0, 2), dtype=np.int32), np.zeros(0)

    if D is None:
        uv = (pts[:, :2] / pts[:, 2:3]) * [K[0, 0], K[1, 1]] + [K[0, 2], K[1, 2]]
    else:
        uv, _ = cv2.projectPoints(pts.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), K, D)
        uv = uv.reshape(-1, 2)

    uv = np.round(uv).astype(np.int32)
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    return uv[inside], pts[inside, 2]


def default_K(image_size=config.CALIBRATION_IMAGE_SIZE, focal=500.0):
    width, height = image_size
    return np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]], dtype=np.float64)


def depth_image(points_cam_mm, K=None, D=None, image_size=config.CALIBRATION_IMAGE_SIZE,
                colormap=cv2.COLORMAP_MAGMA, dot_radius=1):
    """Colored depth rendering of the points (near = bright). Out-of-view points are dropped."""
    width, height = image_size
    if K is None:
        K = default_K(image_size)
    uv, depth = project_points(points_cam_mm, K, D, image_size)

    # Draw far points first so near points end up on top
    order = np.argsort(-depth)
    uv, depth = uv[order], depth[order]
    value = 255 * (1 - (np.clip(depth, MIN_DISTANCE_MM, MAX_DISTANCE_MM) - MIN_DISTANCE_MM)
                   / (MAX_DISTANCE_MM - MIN_DISTANCE_MM))

    img = np.zeros((height, width), dtype=np.uint8)
    for dy in range(-dot_radius + 1, dot_radius):
        for dx in range(-dot_radius + 1, dot_radius):
            u = np.clip(uv[:, 0] + dx, 0, width - 1)
            v = np.clip(uv[:, 1] + dy, 0, height - 1)
            img[v, u] = value.astype(np.uint8)

    colored = cv2.applyColorMap(img, colormap)
    colored[img == 0] = 0
    return colored


def render_chunk(path, K=None, D=None, image_size=config.CALIBRATION_IMAGE_SIZE, **kwargs):
    chunk = LidarChunk(path)
    return depth_image(chunk.xyz_camera_mm(), K, D, image_size, **kwargs)


class LidarFolder:
    """Points of a lidar sensor folder around given host times (chunks loaded lazily)."""

    def __init__(self, folder):
        self.chunks = list_chunks(folder)
        self.starts = [t for t, _ in self.chunks]
        self._cache = {}
        self._clock = None

    def clock(self):
        """Lidar -> host clock mapping fitted on the sync pairs of every file (drift included)."""
        if self._clock is None:
            sync = []
            for _, path in self.chunks:
                with h5py.File(path, "r") as f:
                    sync.append(f["sync"][:])
            sync = np.concatenate(sync)
            sync = sync[np.argsort(sync[:, 0])]
            self._clock = ClockMapping(sync[:, 0], sync[:, 1])
        return self._clock

    def _chunk(self, i):
        if i not in self._cache:
            if len(self._cache) > 4:
                self._cache.pop(next(iter(self._cache)))
            self._cache[i] = LidarChunk(self.chunks[i][1])
        return self._cache[i]

    def points_around(self, host_us, half_window_us=50_000):
        """Camera-convention points (mm) with host time in [t - w, t + w]."""
        lo = bisect.bisect_right(self.starts, host_us - half_window_us) - 1
        hi = bisect.bisect_right(self.starts, host_us + half_window_us)
        selected = []
        for i in range(max(lo, 0), hi):
            chunk = self._chunk(i)
            host_ns = chunk.host_timestamps_ns(self.clock())
            mask = np.abs(host_ns // 1000 - host_us) <= half_window_us
            if mask.any():
                selected.append(chunk.xyz_camera_mm()[mask])
        return np.concatenate(selected) if selected else np.zeros((0, 3), dtype=np.float32)

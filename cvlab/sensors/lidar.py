"""Reading lidar H5 files (format_version 2) written by lidar_recorders/."""

import os

import h5py
import numpy as np

from cvlab import config

MIN_DISTANCE_MM = 300


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
        """Per-point host time. Pass the session-wide mapping (cvlab.replay.session.Lidar)
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

"""Random access to a recorded session for the replay page.

RGB frames are PNG files named by host time, lidar chunks are H5 files with a clock mapping
(cvlab.lidar). Event recordings (.raw) cannot be read at a random time fast enough (a seek decodes
everything before it, ~0.4 s per recorded second, at up to ~40 Mev/s): they are rendered once into
frames of EVENT_FRAME_US (<session>/replay_cache/<camera>/<host_us>.jpg + index.npy) by prepare(),
one process per camera, a single pass over each file.
"""

import bisect
import multiprocessing
import os
import shutil
import threading
import time
from collections import OrderedDict

import cv2
import h5py
import numpy as np

from cvlab import config
from cvlab.lidar import LidarChunk, list_chunks
from cvlab.timesync import ClockMapping

RGB = ("rgb_left", "rgb_right")
EVENTS = ("event_left", "event_right")
LIDARS = {"livox": "lidar_livox", "hesai": "lidar_hesai"}
SENSORS = list(RGB) + list(EVENTS) + list(LIDARS)
EVENT_FRAME_US = 33_333           # one event frame per 1/30 s, events of that interval
CACHE_DIR = "replay_cache"
GAP_US = {"rgb": 1_000_000, "event": 200_000, "lidar": 2_000_000}   # longer holes split the timeline bars
MAX_RGB_OFFSET_US = 500_000       # an RGB frame further than this from the requested time is not shown
RGB_SAME_EXPOSURE_US = 5_000      # see Session.rgb_frame
MAX_EVENT_OFFSET_US = 100_000


def find_sessions():
    """[{name, path, place}] of the recorded sessions, newest first: saved ones and staging ones."""
    out = []
    for place, root in (("saved", config.SESSIONS_DIR), ("staging", config.STAGING_DIR)):
        if os.path.isdir(root):
            for name in os.listdir(root):
                path = os.path.join(root, name)
                if name[:4].isdigit() and os.path.isdir(path):
                    out.append({"name": name, "path": path, "place": place})
    return sorted(out, key=lambda s: s["name"], reverse=True)


def segments(times_us, gap_us):
    """[[start, end]] of sorted times, split where they are further apart than gap_us."""
    if not len(times_us):
        return []
    times_us = np.asarray(times_us, dtype=np.int64)
    breaks = np.nonzero(np.diff(times_us) > gap_us)[0]
    starts = np.r_[times_us[0], times_us[breaks + 1]]
    ends = np.r_[times_us[breaks], times_us[-1]]
    return [[int(a), int(b)] for a, b in zip(starts, ends)]


def png_times(folder):
    if not os.path.isdir(folder):
        return []
    return sorted(int(f[:-4]) for f in os.listdir(folder) if f.endswith(".png") and f[:-4].isdigit())


class Lidar:
    """The chunks of a lidar folder, with one clock mapping fitted on all their sync pairs."""

    def __init__(self, folder):
        self.chunks = list_chunks(folder) if os.path.isdir(folder) else []
        self.starts = [t for t, _ in self.chunks]
        self.cache = OrderedDict()
        self.clock = None
        if self.chunks:
            sync = []
            for _, path in self.chunks:
                with h5py.File(path, "r") as f:
                    sync.append(f["sync"][:])
            sync = np.concatenate(sync)
            if len(sync):
                sync = sync[np.argsort(sync[:, 0])]
                self.clock = ClockMapping(sync[:, 0], sync[:, 1])

    def chunk(self, i):
        """(host_us, xyz camera convention mm, intensity) of chunk i, cached."""
        if i not in self.cache:
            c = LidarChunk(self.chunks[i][1])
            host_ns = c.host_timestamps_ns(self.clock)
            self.cache[i] = ((host_ns // 1000).astype(np.int64), c.xyz_camera_mm().astype(np.float32), c.intensity)
            if len(self.cache) > 64:
                self.cache.popitem(last=False)
        return self.cache[i]

    def points(self, t0_us, t1_us):
        """(xyz, intensity) with host time in [t0, t1], in time order."""
        lo = max(bisect.bisect_right(self.starts, t0_us) - 2, 0)   # a chunk starts before its points
        hi = bisect.bisect_right(self.starts, t1_us)
        xyz, intensity = [], []
        for i in range(lo, hi):
            t, p, inten = self.chunk(i)
            keep = (t >= t0_us) & (t <= t1_us)
            if keep.any():
                xyz.append(p[keep])
                intensity.append(inten[keep])
        if not xyz:
            return np.zeros((0, 3), np.float32), np.zeros(0, np.uint8)
        return np.concatenate(xyz), np.concatenate(intensity)


class Session:
    """What the replay needs to know about a session, loaded once."""

    def __init__(self, path):
        self.path = path
        self.name = os.path.basename(path)
        self.rgb = {cam: png_times(os.path.join(path, cam)) for cam in RGB}
        self.lidar = {s: Lidar(os.path.join(path, folder)) for s, folder in LIDARS.items()}
        self.raw = {cam: sorted(f for f in os.listdir(os.path.join(path, cam)) if f.endswith(".raw"))
                    if os.path.isdir(os.path.join(path, cam)) else [] for cam in EVENTS}
        self.events = {}
        self.load_event_cache()
        self.images = OrderedDict()   # path -> image, a few full RGB frames
        self.lock = threading.Lock()

    def load_event_cache(self):
        for cam in EVENTS:
            index = os.path.join(self.path, CACHE_DIR, cam, "index.npy")
            self.events[cam] = np.load(index) if os.path.exists(index) else None

    def events_prepared(self):
        return all(self.events[cam] is not None or not self.raw[cam] for cam in EVENTS)

    def availability(self):
        """{sensor: [[start_us, end_us]]} of the data each sensor has."""
        out = {cam: segments(self.rgb[cam], GAP_US["rgb"]) for cam in RGB}
        for cam in EVENTS:
            out[cam] = segments(self.events[cam], GAP_US["event"]) if self.events[cam] is not None else []
        for s, lidar in self.lidar.items():
            out[s] = []
            if lidar.chunks:
                last = lidar.chunk(len(lidar.chunks) - 1)[0]
                out[s] = segments(lidar.starts + [int(last[-1])], GAP_US["lidar"])
        return out

    def time_range(self, availability):
        spans = [seg for segs in availability.values() for seg in segs]
        if not spans:
            return None
        return min(s[0] for s in spans), max(s[1] for s in spans)

    def summary(self):
        availability = self.availability()
        return {"name": self.name, "range": self.time_range(availability), "availability": availability,
                "events_prepared": self.events_prepared(), "has_raw": {cam: bool(self.raw[cam]) for cam in EVENTS},
                "frames": {cam: len(self.rgb[cam]) for cam in RGB}}

    # -- frames ---------------------------------------------------------------

    def rgb_frame(self, cam, t_us):
        """(host_us, BGR full frame) of the newest frame at or before t (else the nearest), or None.
        Frames up to RGB_SAME_EXPOSURE_US after t count as at t: left and right expose together but
        their timestamps differ by tens of us, and the previous frame is 100 ms older."""
        times = self.rgb[cam]
        if not times:
            return None
        i = bisect.bisect_right(times, t_us + RGB_SAME_EXPOSURE_US) - 1
        if i < 0:
            i = 0
        if abs(times[i] - t_us) > MAX_RGB_OFFSET_US:
            return None
        path = os.path.join(self.path, cam, f"{times[i]}.png")
        with self.lock:
            image = self.images.get(path)
        if image is None:
            image = cv2.imread(path)
            with self.lock:
                self.images[path] = image
                if len(self.images) > 8:
                    self.images.popitem(last=False)
        return times[i], image

    def rgb_neighbours(self, t_us):
        """(previous, next) RGB-left times strictly before / after t, None at the ends (frame stepping)."""
        times = self.rgb["rgb_left"]
        i = bisect.bisect_left(times, t_us)
        j = bisect.bisect_right(times, t_us)
        return (times[i - 1] if i > 0 else None), (times[j] if j < len(times) else None)

    def event_frame(self, cam, t_us):
        """(host_us, BGR frame) of the events of the EVENT_FRAME_US ending at or before t, or None."""
        index = self.events[cam]
        if index is None or not len(index):
            return None
        i = int(np.searchsorted(index, t_us, side="right")) - 1
        if i < 0 or t_us - index[i] > MAX_EVENT_OFFSET_US:
            return None
        return int(index[i]), cv2.imread(os.path.join(self.path, CACHE_DIR, cam, f"{index[i]}.jpg"))

    def lidar_points(self, sensor, t_us, window_us):
        """(xyz camera convention mm, intensity) of the window_us before t."""
        return self.lidar[sensor].points(t_us - window_us, t_us)


# ============================================================================
# EVENT CACHE
# ============================================================================

def render_events(session_path, cam, progress):
    """Child process: render every EVENT_FRAME_US of the camera's raw files into JPEG frames."""
    from cvlab.events import colored_frame, find_recordings, open_raw

    out = os.path.join(session_path, CACHE_DIR, cam)
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    recordings = find_recordings(os.path.join(session_path, cam))
    total = sum(max(r.last_host_us - r.first_host_us, 1) for r in recordings)
    done, times = 0, []
    for rec in recordings:
        iterator = open_raw(rec.raw_path, EVENT_FRAME_US)
        height, width = iterator.get_size()
        span = max(rec.last_host_us - rec.first_host_us, 1)
        for k, evs in enumerate(iterator):
            # Intervals without events are not frames; the end of an interval comes from its events
            if not len(evs):
                continue
            camera_end = (int(evs["t"][-1]) // EVENT_FRAME_US + 1) * EVENT_FRAME_US
            host_us = rec.to_host_us(camera_end)
            cv2.imwrite(os.path.join(out, f"{host_us}.jpg"), colored_frame(evs, width, height),
                        [cv2.IMWRITE_JPEG_QUALITY, 85])
            times.append(host_us)
            if k % 15 == 0:
                progress.value = min((done + min(host_us - rec.first_host_us, span)) / total, 0.999)
        done += span
    np.save(os.path.join(out, "index.npy"), np.array(sorted(times), dtype=np.int64))
    progress.value = 1.0


class Preparation:
    """Renders the event cache of one session in the background, both cameras in parallel."""

    def __init__(self):
        self.lock = threading.Lock()
        self.session = None
        self.progress = {}
        self.state = "idle"
        self.error = None

    def start(self, session):
        with self.lock:
            if self.state == "running":
                return False
            self.session, self.state, self.error = session.name, "running", None
            cams = [cam for cam in EVENTS if session.raw[cam]]
            context = multiprocessing.get_context("spawn")
            self.progress = {cam: context.Value("d", 0.0) for cam in cams}
            processes = [context.Process(target=render_events, args=(session.path, cam, self.progress[cam]), daemon=True)
                         for cam in cams]
        for p in processes:
            p.start()
        threading.Thread(target=self.wait, args=(session, processes), daemon=True).start()
        return True

    def wait(self, session, processes):
        for p in processes:
            p.join()
        failed = [p for p in processes if p.exitcode != 0]
        session.load_event_cache()
        with self.lock:
            self.state = "failed" if failed else "done"
            if failed:
                self.error = f"{len(failed)} event camera(s) could not be rendered (see the server log)"

    def status(self):
        with self.lock:
            return {"session": self.session, "state": self.state, "error": self.error,
                    "progress": {cam: round(v.value, 3) for cam, v in self.progress.items()}}

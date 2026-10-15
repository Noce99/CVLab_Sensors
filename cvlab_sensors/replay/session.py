"""Random access to a recorded session for the replay page.

RGB frames are PNG files named by host time, lidar chunks are H5 files with a clock mapping
(cvlab_sensors.sensors.lidar). Event recordings (.raw) cannot be read at a random time fast enough (a seek
decodes everything before it, ~0.4 s per recorded second, at up to ~40 Mev/s): they are rendered once
into frames, one per accumulation time (<session>/replay_cache/<representation>_<ms>ms/<camera>/
<host_us>.png + index.npy) by Preparation, one process per camera, a single pass over each file.
Every event is in exactly one frame: the frame at host time t shows the events of (t - accumulation, t].
"""

import bisect
import multiprocessing
import os
import shutil
import threading
from collections import OrderedDict

import cv2
import h5py
import numpy as np

from cvlab_sensors import config
from cvlab_sensors.sensors.lidar import LidarChunk, list_chunks
from cvlab_sensors.sensors.timesync import ClockMapping

RGB = ("rgb_left", "rgb_right")
EVENTS = ("event_left", "event_right")
LIDARS = {"livox": "lidar_livox", "hesai": "lidar_hesai"}
SENSORS = list(RGB) + list(EVENTS) + list(LIDARS)
CACHE_DIR = "replay_cache"
DEFAULT_EVENT_REPRESENTATION = "tencode"   # see cvlab_sensors.sensors.events.EVENT_REPRESENTATIONS
DEFAULT_EVENT_ACCUMULATION_MS = 20
MIN_EVENT_ACCUMULATION_MS, MAX_EVENT_ACCUMULATION_MS = 1, 1000
GAP_US = {"rgb": 1_000_000, "event": 200_000, "lidar": 2_000_000}   # longer holes split the timeline bars
MAX_RGB_OFFSET_US = 500_000       # an RGB frame further than this from the requested time is not shown
RGB_SAME_EXPOSURE_US = 5_000      # see Session.rgb_frame
MAX_EVENT_OFFSET_US = 100_000    # an event frame further than this (or its accumulation time) is not shown


def event_cache_name(representation, accumulation_ms):
    """Folder of a cache in <session>/replay_cache/, e.g. tencode_20ms."""
    from cvlab_sensors.sensors.events import EVENT_REPRESENTATIONS
    if representation not in EVENT_REPRESENTATIONS:
        raise ValueError(f"unknown event representation {representation!r}")
    if not MIN_EVENT_ACCUMULATION_MS <= accumulation_ms <= MAX_EVENT_ACCUMULATION_MS:
        raise ValueError(f"accumulation {accumulation_ms} ms: expected "
                         f"{MIN_EVENT_ACCUMULATION_MS}-{MAX_EVENT_ACCUMULATION_MS} ms")
    return f"{representation}_{int(accumulation_ms)}ms"


def event_sync_times(folder):
    """Sorted host times (us) of the clock sync pairs of every recording of an event folder: the
    stretches with events, ~10 per second, without decoding the events."""
    times = []
    for name in sorted(os.listdir(folder)) if os.path.isdir(folder) else []:
        if name.endswith(".sync.csv"):
            sync = np.loadtxt(os.path.join(folder, name), delimiter=",", skiprows=1, dtype=np.int64, ndmin=2)
            if len(sync):
                times.append(sync[:, 1] // 1000)
    return np.sort(np.concatenate(times)) if times else np.zeros(0, np.int64)


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
        self.event_times = {cam: event_sync_times(os.path.join(path, cam)) for cam in EVENTS}
        self.event_indexes = {}   # (cache name, camera) -> sorted frame host times, or None
        self.images = OrderedDict()   # path -> image, a few full RGB frames
        self.lock = threading.Lock()

    def event_index(self, cache, cam):
        """Frame host times of an event cache (see event_cache_name), None when not rendered."""
        key = (cache, cam)
        if self.event_indexes.get(key) is None:
            index = os.path.join(self.path, CACHE_DIR, cache, cam, "index.npy")
            self.event_indexes[key] = np.load(index) if os.path.exists(index) else None
        return self.event_indexes[key]

    def forget_event_cache(self, cache):
        """Reload the cache from disk next time (after it is rendered again)."""
        for cam in EVENTS:
            self.event_indexes.pop((cache, cam), None)

    def events_prepared(self, cache):
        return all(self.event_index(cache, cam) is not None or not self.raw[cam] for cam in EVENTS)

    def availability(self):
        """{sensor: [[start_us, end_us]]} of the data each sensor has."""
        out = {cam: segments(self.rgb[cam], GAP_US["rgb"]) for cam in RGB}
        for cam in EVENTS:
            out[cam] = segments(self.event_times[cam], GAP_US["event"])
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

    def summary(self, cache):
        availability = self.availability()
        return {"name": self.name, "range": self.time_range(availability), "availability": availability,
                "events_prepared": self.events_prepared(cache), "has_raw": {cam: bool(self.raw[cam]) for cam in EVENTS},
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

    def event_frame(self, cache, cam, t_us):
        """(host_us, BGR frame) of the newest frame of the cache ending at or before t, or None."""
        index = self.event_index(cache, cam)
        if index is None or not len(index):
            return None
        i = int(np.searchsorted(index, t_us, side="right")) - 1
        accumulation_us = int(cache.rsplit("_", 1)[1][:-2]) * 1000
        if i < 0 or t_us - index[i] > max(MAX_EVENT_OFFSET_US, accumulation_us):
            return None
        return int(index[i]), cv2.imread(os.path.join(self.path, CACHE_DIR, cache, cam, f"{index[i]}.png"))

    def lidar_points(self, sensor, t_us, window_us):
        """(xyz camera convention mm, intensity) of the window_us before t."""
        return self.lidar[sensor].points(t_us - window_us, t_us)


# ============================================================================
# EVENT CACHE
# ============================================================================

def render_events(session_path, cam, representation, accumulation_ms, progress):
    """Child process: render the camera's raw files into one frame per accumulation time."""
    from cvlab_sensors.sensors.events import find_recordings, open_raw, render_window

    accumulation_us = int(accumulation_ms) * 1000
    out = os.path.join(session_path, CACHE_DIR, event_cache_name(representation, accumulation_ms), cam)
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    recordings = find_recordings(os.path.join(session_path, cam))
    total = sum(max(r.last_host_us - r.first_host_us, 1) for r in recordings)
    done, times = 0, []
    for rec in recordings:
        # The iterator's slices are aligned on multiples of delta_t in camera time: one slice, one frame
        iterator = open_raw(rec.raw_path, accumulation_us)
        height, width = iterator.get_size()
        span = max(rec.last_host_us - rec.first_host_us, 1)
        for k, evs in enumerate(iterator):
            # Intervals without events are not frames; the end of an interval comes from its events
            if not len(evs):
                continue
            camera_end = (int(evs["t"][-1]) // accumulation_us + 1) * accumulation_us
            frame = render_window(representation, evs, width, height, camera_end - accumulation_us, camera_end)
            host_us = rec.to_host_us(camera_end)
            cv2.imwrite(os.path.join(out, f"{host_us}.png"), frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            times.append(host_us)
            if k % 15 == 0:
                progress.value = min((done + min(host_us - rec.first_host_us, span)) / total, 0.999)
        done += span
    np.save(os.path.join(out, "index.npy"), np.array(sorted(times), dtype=np.int64))
    progress.value = 1.0


class Preparation:
    """Renders one event cache of one session in the background, both cameras in parallel."""

    def __init__(self):
        self.lock = threading.Lock()
        self.session = None
        self.cache = None
        self.progress = {}
        self.state = "idle"
        self.error = None

    def start(self, session, representation, accumulation_ms):
        cache = event_cache_name(representation, accumulation_ms)
        with self.lock:
            if self.state == "running":
                return False
            self.session, self.cache, self.state, self.error = session.name, cache, "running", None
            cams = [cam for cam in EVENTS if session.raw[cam]]
            context = multiprocessing.get_context("spawn")
            self.progress = {cam: context.Value("d", 0.0) for cam in cams}
            processes = [context.Process(target=render_events, daemon=True,
                                         args=(session.path, cam, representation, accumulation_ms, self.progress[cam]))
                         for cam in cams]
        for p in processes:
            p.start()
        threading.Thread(target=self.wait, args=(session, cache, processes), daemon=True).start()
        return True

    def wait(self, session, cache, processes):
        for p in processes:
            p.join()
        failed = [p for p in processes if p.exitcode != 0]
        session.forget_event_cache(cache)
        with self.lock:
            self.state = "failed" if failed else "done"
            if failed:
                self.error = f"{len(failed)} event camera(s) could not be rendered (see the server log)"

    def status(self):
        with self.lock:
            return {"session": self.session, "cache": self.cache, "state": self.state, "error": self.error,
                    "progress": {cam: round(v.value, 3) for cam, v in self.progress.items()}}

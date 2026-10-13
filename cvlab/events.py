"""Reading event recordings (get_data/event.py) and turning them into frames."""

import glob
import os

import cv2
import numpy as np
from metavision_core.event_io import EventsIterator

from cvlab.timesync import ClockMapping

# Calibration-style frames: per pixel event count (both polarities), saturated,
# inverted (static background white, chessboard edges black) and closed.
CALIBRATION_WINDOW_US = 60_000
# Lock-in frame (blinking chessboard): slices of LOCKIN_SLICE_US over the last LOCKIN_WINDOW_US
LOCKIN_SLICE_US = 10_000
LOCKIN_WINDOW_US = 100_000
# Events needed to render calibration_frames
CALIBRATION_FRAMES_WINDOW_US = max(CALIBRATION_WINDOW_US, LOCKIN_WINDOW_US)
CALIBRATION_SATURATION = 3
MIN_ACTIVATION = 0.1   # fraction of active pixels to consider a frame usable


def open_raw(path, delta_t):
    """EventsIterator over a recorded .raw file, with the camera's own timestamps.

    By default Metavision shifts a file's timestamps so that its first event is at t = 0, but the
    .sync.csv pairs are in camera time: a camera opened 13 s before the recording started (buffered
    events of that time open the file) put every event 13 s early on 2026-10-13."""
    return EventsIterator(path, delta_t=delta_t, do_time_shifting=False)


class EventRecording:
    """One <start_us>.raw file with its <start_us>.sync.csv clock mapping."""

    def __init__(self, raw_path):
        self.raw_path = raw_path
        base = raw_path[:-len(".raw")]
        sync = np.loadtxt(base + ".sync.csv", delimiter=",", skiprows=1, dtype=np.int64, ndmin=2)
        if len(sync) == 0:
            raise ValueError(f"{base}.sync.csv has no sync pairs")
        # Lower envelope of the offsets (USB latency >= 0), clock drift included
        self.clock = ClockMapping(sync[:, 0] * 1000, sync[:, 1])
        self.first_host_us = int(sync[0, 1] // 1000)
        self.last_host_us = int(sync[-1, 1] // 1000)

    def to_camera_us(self, host_us):
        return int(self.clock.to_device_ns(int(host_us) * 1000) // 1000)

    def to_host_us(self, camera_us):
        return int(self.clock.to_host_ns(int(camera_us) * 1000) // 1000)

    def frames_at(self, host_times_us, window_us=CALIBRATION_WINDOW_US, render=None):
        """Yield (host_us, frame) for each requested time (sorted), in a single pass.

        Each frame accumulates the events in [t - window_us, t].
        """
        render = render or calibration_frame
        targets = [(int(t), self.to_camera_us(int(t))) for t in sorted(host_times_us)]
        targets = [(h, c) for h, c in targets if c - window_us >= 0]
        if not targets:
            return

        iterator = open_raw(self.raw_path, delta_t=10_000)
        height, width = iterator.get_size()
        buffer = []
        i = 0
        for evs in iterator:
            if len(evs):
                buffer.append(evs)
            if not buffer:
                continue
            end = buffer[-1]["t"][-1]
            while i < len(targets) and targets[i][1] <= end:
                host_us, cam_us = targets[i]
                window = np.concatenate(buffer)
                window = window[(window["t"] > cam_us - window_us) & (window["t"] <= cam_us)]
                yield host_us, render(window, width, height)
                i += 1
            if i >= len(targets):
                return
            # Keep only what the next target can still need
            if i < len(targets):
                keep_from = targets[i][1] - window_us
                buffer = [b for b in buffer if len(b) and b["t"][-1] > keep_from]


def calibration_frame(evs, width, height, saturation=CALIBRATION_SATURATION):
    # One bincount over (pixel, polarity): about twice as fast as one per polarity
    idx = (evs["y"].astype(np.int64) * width + evs["x"]) * 2 + (evs["p"] > 0)
    counts = np.minimum(np.bincount(idx, minlength=2 * height * width), saturation)
    counts = counts.reshape(height, width, 2).sum(axis=2)
    img = (counts * (255.0 / (2 * saturation))).astype(np.uint8)
    img = 255 - img
    return cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def polarity_frame(evs, width, height, saturation=CALIBRATION_SATURATION):
    """ON minus OFF events per pixel, saturated: gray = no change (or both), white = ON, black = OFF.

    A blinking chessboard (alternating with its inverse) is a clean chessboard here.
    """
    idx = (evs["y"].astype(np.int64) * width + evs["x"]) * 2 + (evs["p"] > 0)
    counts = np.bincount(idx, minlength=2 * height * width).reshape(height, width, 2)
    diff = np.clip(counts[..., 1] - counts[..., 0], -saturation, saturation)
    return (128 + diff * (127 / saturation)).astype(np.uint8)


def lockin_frame(evs, width, height, slice_us=LOCKIN_SLICE_US, window_us=LOCKIN_WINDOW_US):
    """Blinking chessboard, whatever its blink frequency (the board must be still).

    Within a window longer than a blink, every square gets both ON and OFF events and the
    polarity frame cancels out. Here the window is cut into short slices, each slice's
    ON - OFF image is flipped to the blink phase of the strongest slice (sign of their
    correlation) and the slices are summed: the two sets of squares, which blink in
    opposite phase, end up white and black, while the noise averages out.
    Found the board in 15/19 real captures where polarity_frame found none.
    """
    if not len(evs):
        return np.full((height, width), 128, np.uint8)
    start = int(evs["t"][-1]) - window_us
    evs = evs[evs["t"] > start]
    slices = ((evs["t"] - start - 1) // slice_us).astype(np.int64)
    sign = np.where(evs["p"] > 0, 1.0, -1.0)
    idx = evs["y"].astype(np.int64) * width + evs["x"]
    order = np.argsort(slices, kind="stable")
    bounds = np.searchsorted(slices[order], np.arange(int(slices.max()) + 2))
    diff = np.stack([np.bincount(idx[order[a:b]], weights=sign[order[a:b]], minlength=width * height)
                     for a, b in zip(bounds[:-1], bounds[1:])]).astype(np.float32)
    reference = diff[np.argmax(np.abs(diff).sum(axis=1))]
    phase = np.sign(diff @ reference)
    phase[phase == 0] = 1
    total = (phase[:, None] * diff).sum(axis=0).reshape(height, width)
    limit = np.percentile(np.abs(total), 99) or 1.0
    return (128 + np.clip(total / limit, -1, 1) * 127).astype(np.uint8)


def calibration_frames(evs, width, height):
    """Renderings of the same events for find_event_corners: {"count", "polarity", "lockin"}.

    evs: the CALIBRATION_FRAMES_WINDOW_US of events ending at the frame time.
    """
    recent = evs[evs["t"] > evs["t"][-1] - CALIBRATION_WINDOW_US] if len(evs) else evs
    return {"count": calibration_frame(recent, width, height),
            "polarity": polarity_frame(recent, width, height),
            "lockin": lockin_frame(evs, width, height)}


def activation(frame):
    """Fraction of pixels with events in a calibration frame (background is 255)."""
    return np.count_nonzero(frame < 255) / frame.size


def colored_frame(evs, width, height):
    img = np.full((height, width, 3), 64, dtype=np.uint8)
    pos = evs["p"] > 0
    img[evs["y"][pos], evs["x"][pos]] = (255, 255, 255)
    img[evs["y"][~pos], evs["x"][~pos]] = (255, 120, 0)
    return img


def find_recordings(folder):
    """EventRecording for every raw file of a session sensor folder, sorted by start time."""
    return [EventRecording(p) for p in sorted(glob.glob(os.path.join(folder, "*.raw")))]


def frames_at(folder, host_times_us, window_us=CALIBRATION_WINDOW_US, render=None):
    """Like EventRecording.frames_at, across every raw file of a sensor folder."""
    recordings = find_recordings(folder)
    host_times_us = sorted(host_times_us)
    for i, rec in enumerate(recordings):
        end = recordings[i + 1].first_host_us if i + 1 < len(recordings) else float("inf")
        times = [t for t in host_times_us if rec.first_host_us <= t < end]
        yield from rec.frames_at(times, window_us, render)


def stereo_frames_at(session, host_times_us, window_us=CALIBRATION_WINDOW_US, render=None):
    """Yield (host_us, left_frame, right_frame) for the times both event cameras can render."""
    left = frames_at(os.path.join(session, "event_left"), host_times_us, window_us, render)
    right = frames_at(os.path.join(session, "event_right"), host_times_us, window_us, render)
    r = next(right, None)
    for t_left, frame_left in left:
        while r is not None and r[0] < t_left:
            r = next(right, None)
        if r is None:
            return
        if r[0] == t_left:
            yield t_left, frame_left, r[1]

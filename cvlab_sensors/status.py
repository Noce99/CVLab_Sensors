"""Status JSON and preview images written by the recorders, read by the server / CLI."""

import json
import os
import time

import cv2

from cvlab_sensors import config


def write_json_atomic(path, data):
    tmp = f"{path}.tmp"
    with open(tmp, "w") as f:
        json.dump(data, f)
    os.replace(tmp, path)


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def write_preview_atomic(path, image, width=config.PREVIEW_WIDTH):
    if image.shape[1] > width:
        height = int(image.shape[0] * width / image.shape[1])
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    tmp = f"{path}.tmp.jpg"
    cv2.imwrite(tmp, image, [cv2.IMWRITE_JPEG_QUALITY, 80])
    os.replace(tmp, path)


class RateCounter:
    """Counts events and reports the rate over the last reporting period."""

    def __init__(self):
        self.total = 0
        self._last_total = 0
        self._last_time = time.monotonic()

    def add(self, n=1):
        self.total += n

    def rate(self):
        now = time.monotonic()
        dt = now - self._last_time
        rate = (self.total - self._last_total) / dt if dt > 0 else 0.0
        self._last_total, self._last_time = self.total, now
        return rate


class Throttle:
    """True at most once every `period_s` seconds."""

    def __init__(self, period_s):
        self.period_s = period_s
        self._next = 0.0

    def ready(self):
        now = time.monotonic()
        if now >= self._next:
            self._next = now + self.period_s
            return True
        return False

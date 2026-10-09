"""Mapping a device clock onto the host clock from [device_time, host_receive_time] pairs.

host_receive_time = device_time + offset + drift * t + latency, with latency >= 0 and
occasionally large (late frames/packets). We fit a line to the lower envelope of the
offsets: iteratively drop the points above the fit and refit.
"""

import numpy as np


class ClockMapping:
    def __init__(self, device_ns, host_ns, iterations=5, keep_fraction=0.3):
        device_ns = np.asarray(device_ns, dtype=np.int64)
        host_ns = np.asarray(host_ns, dtype=np.int64)
        if len(device_ns) == 0:
            raise ValueError("no sync pairs")
        self.t0 = int(device_ns[0])
        x = (device_ns - self.t0) / 1e9                       # s since first pair
        y = (host_ns - device_ns).astype(np.float64)          # offset, ns
        y0 = float(np.min(y))
        y = y - y0
        mask = np.ones(len(x), dtype=bool)
        slope, intercept = 0.0, float(np.min(y))
        if len(x) >= 2 and x.max() - x.min() > 1.0:
            for _ in range(iterations):
                slope, intercept = np.polyfit(x[mask], y[mask], 1)
                residual = y - (slope * x + intercept)
                mask = residual <= np.quantile(residual, keep_fraction)
                if mask.sum() < 2:
                    break
            # Shift down so the line is the lower envelope (latency >= 0)
            intercept += float(np.min(y - (slope * x + intercept)))
        self.slope_ns_per_s = float(slope)
        self.intercept_ns = float(intercept) + y0
        residual = y + y0 - (self.slope_ns_per_s * x + self.intercept_ns)
        self.latency_median_ms = float(np.median(residual)) / 1e6

    def to_host_ns(self, device_ns):
        x = (np.asarray(device_ns, dtype=np.int64) - self.t0) / 1e9
        return np.asarray(device_ns, dtype=np.int64) + np.round(self.slope_ns_per_s * x + self.intercept_ns).astype(np.int64)

    def to_device_ns(self, host_ns):
        # Invert host = d + s*(d - t0)/1e9 + c  (one fixed-point step is plenty for ppm drifts)
        host_ns = np.asarray(host_ns, dtype=np.int64)
        d = host_ns - np.round(self.intercept_ns).astype(np.int64)
        for _ in range(2):
            d = host_ns - np.round(self.slope_ns_per_s * (d - self.t0) / 1e9 + self.intercept_ns).astype(np.int64)
        return d

    def __repr__(self):
        return (f"ClockMapping(drift {self.slope_ns_per_s / 1e3:.1f} ppm, "
                f"median latency {self.latency_median_ms:.2f} ms)")

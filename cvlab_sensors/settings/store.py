"""Sensor settings edited on the Settings page, saved in config.SETTINGS_FILE over the config.py defaults.

A sensor reads them when it opens: a saved change applies the next time the sensor opens (next
recording, calibration capture or settings preview), never to a running sensor. Whatever opens a
sensor reports what it applied (see write_applied), so the pages can tell running from saved values.

The file keeps only the values that differ from the defaults, in one profile ("active"): presets
can be added later as more profiles.
"""

import os
from datetime import datetime

import numpy as np

from cvlab_sensors import config
from cvlab_sensors.status import read_json, write_json_atomic

FORMAT_VERSION = 1
PROFILE = "active"
GIGE_MAX_MB_S = 110   # usable payload of one 1 GbE camera link

GROUPS = {"rgb": "Frame cameras (Basler)", "event": "Event cameras (EVK4)"}
# Every camera of a group uses the group's settings (a stereo pair must match)
GROUP_OUTPUTS = {"rgb": ["rgb_left", "rgb_right"], "event": ["event_left", "event_right"]}
OUTPUT_LABELS = {"rgb_left": "RGB left", "rgb_right": "RGB right", "event_left": "Event left", "event_right": "Event right"}
EVENT_BIASES = ["bias_diff_on", "bias_diff_off", "bias_fo", "bias_hpf", "bias_refr"]


def field(key, label, kind, default, section, unit="", min=None, max=None, step=None, choices=None,
          optional=False, when=None, help="", geometry=False):
    """kind: number, int, bool or choice ([[value, label]]). optional: empty = None (no cap, camera
    default...). when: [key, value], the field only matters when that other field has that value.
    geometry: changes the image geometry, which the installed calibrations depend on."""
    return dict(key=key, label=label, kind=kind, default=default, section=section, unit=unit, min=min, max=max,
                step=step, choices=choices, optional=optional, when=when, help=help, geometry=geometry)


def _mev(rate):
    return None if rate is None else rate / 1e6


AUTO_CHOICES = [["Continuous", "Auto"], ["Off", "Manual"]]

FIELDS = {
    "rgb": [
        field("roi_width", "ROI width", "number", config.RGB_ROI_FRACTION[0], "Image", "× sensor", 0.1, 1.0, 0.01,
              geometry=True, help=f"Fraction of the {config.RGB_SENSOR_SIZE[0]} px sensor width, centered "
                                  "(rounded up to a multiple of 4 px)"),
        field("roi_height", "ROI height", "number", config.RGB_ROI_FRACTION[1], "Image", "× sensor", 0.1, 1.0, 0.01,
              geometry=True, help=f"Fraction of the {config.RGB_SENSOR_SIZE[1]} px sensor height, centered"),
        field("fps", "Frame rate", "number", config.RGB_FPS, "Timing", "fps", 1, 60, 0.5),
        field("ptp_trigger", "PTP trigger", "bool", config.RGB_PTP_TRIGGER, "Timing",
              help="Trigger every frame from the PTP clock: left and right expose at the same instant. "
                   "Off: free-running cameras, not synchronized"),
        field("exposure_auto", "Exposure", "choice", "Continuous", "Exposure", choices=AUTO_CHOICES),
        field("exposure_lower_us", "Auto exposure min", "number", config.RGB_EXPOSURE_LOWER_US, "Exposure", "µs",
              1, 1_000_000, 1, when=["exposure_auto", "Continuous"]),
        field("exposure_upper_us", "Auto exposure max", "number", config.RGB_EXPOSURE_UPPER_US, "Exposure", "µs",
              1, 1_000_000, 1, when=["exposure_auto", "Continuous"],
              help="The rig is on a moving car: keep it short, the gain compensates in low light"),
        field("exposure_us", "Exposure time", "number", 1000.0, "Exposure", "µs", 1, 1_000_000, 1,
              when=["exposure_auto", "Off"]),
        field("gain_auto", "Gain", "choice", "Continuous", "Gain", choices=AUTO_CHOICES),
        field("gain_upper_db", "Auto gain max", "number", config.RGB_GAIN_UPPER_DB, "Gain", "dB", 0, 48, 0.5,
              when=["gain_auto", "Continuous"]),
        field("gain_db", "Gain", "number", 0.0, "Gain", "dB", 0, 48, 0.5, when=["gain_auto", "Off"]),
        field("target_brightness", "Auto target brightness", "number", config.RGB_AUTO_TARGET_BRIGHTNESS,
              "Auto functions", "", 0.01, 0.99, 0.01,
              help="Mean level the auto exposure / gain aim for, 0–1 (camera default: 0.3)"),
        field("auto_profile", "Auto profile", "choice", "MinimizeExposureTime", "Auto functions",
              choices=[["MinimizeExposureTime", "Minimize exposure (raise gain first)"],
                       ["MinimizeGain", "Minimize gain (raise exposure first)"]]),
        field("white_balance", "White balance", "choice", "unchanged", "Color",
              choices=[["unchanged", "Camera setting (not changed)"], ["Continuous", "Auto"],
                       ["Once", "Once at start"], ["Off", "Off"]]),
        field("png_compression", "PNG compression", "int", config.RGB_PNG_COMPRESSION, "Saving", "", 0, 9, 1,
              help="Recorder, 0–9: higher = smaller files but slower writing (frames are dropped when the writers fall behind)"),
    ],
    "event": [
        field("record_max_rate_mev", "Rate cap, recording", "number", _mev(config.EVENT_RECORD_MAX_RATE),
              "Event rate", "Mev/s", 1, 200, 1, optional=True,
              help="Hardware event rate control (ERC): beyond the cap the sensor drops events evenly in time. "
                   "Empty = no cap"),
        field("calibration_max_rate_mev", "Rate cap, live tools", "number", _mev(config.EVENT_CALIBRATION_MAX_RATE),
              "Event rate", "Mev/s", 1, 200, 1, optional=True,
              help="The same for the calibration captures and the preview on this page, which cannot "
                   "render more than ~20 Mev/s live. Empty = no cap"),
        *[field(name, name, "int", None, "Biases", "", -1000, 1000, 1, optional=True,
                help=help + ". Empty = camera default")
          for name, help in [("bias_diff_on", "ON contrast threshold: higher = fewer ON events"),
                             ("bias_diff_off", "OFF contrast threshold: higher = fewer OFF events"),
                             ("bias_fo", "Photoreceptor low-pass filter"),
                             ("bias_hpf", "High-pass filter"),
                             ("bias_refr", "Refractory period (dead time after an event)")]],
        field("antiflicker", "Anti-flicker", "bool", False, "Filters",
              help="Hardware filter of flickering lights (100 Hz under 50 Hz mains)"),
        field("antiflicker_mode", "Anti-flicker mode", "choice", "BandStop", "Filters",
              choices=[["BandStop", "Remove the band"], ["BandPass", "Keep only the band"]], when=["antiflicker", True]),
        field("antiflicker_low_hz", "Band low", "int", 100, "Filters", "Hz", 1, 2000, 1, when=["antiflicker", True]),
        field("antiflicker_high_hz", "Band high", "int", 120, "Filters", "Hz", 1, 2000, 1, when=["antiflicker", True]),
        field("trail_filter", "Trail filter", "bool", False, "Filters",
              help="Hardware event trail / spatio-temporal contrast (STC) filter"),
        field("trail_type", "Trail filter type", "choice", "TRAIL", "Filters",
              choices=[["TRAIL", "Trail"], ["STC_CUT_TRAIL", "STC, cut trail"], ["STC_KEEP_TRAIL", "STC, keep trail"]],
              when=["trail_filter", True]),
        field("trail_threshold_us", "Trail threshold", "int", 10_000, "Filters", "µs", 1, 1_000_000, 1,
              when=["trail_filter", True]),
        field("roi", "Hardware ROI", "bool", False, "Region of interest",
              help="Keep only the events in a window of the 1280 × 720 sensor (coordinates stay sensor pixels)"),
        field("roi_x", "ROI x", "int", 0, "Region of interest", "px", 0, 1279, 1, when=["roi", True]),
        field("roi_y", "ROI y", "int", 0, "Region of interest", "px", 0, 719, 1, when=["roi", True]),
        field("roi_width", "ROI width", "int", 1280, "Region of interest", "px", 1, 1280, 1, when=["roi", True]),
        field("roi_height", "ROI height", "int", 720, "Region of interest", "px", 1, 720, 1, when=["roi", True]),
        field("trigger_in", "Trigger input", "bool", config.EVENT_TRIGGER_IN, "Trigger",
              help="Record the EVK4 trigger input (MAIN channel), e.g. wired to a Basler ExposureActive line"),
    ],
}
FIELD_BY_KEY = {group: {f["key"]: f for f in fields} for group, fields in FIELDS.items()}


# ============================================================================
# LOAD / SAVE
# ============================================================================

def defaults(group):
    return {f["key"]: f["default"] for f in FIELDS[group]}


def _read_file():
    data = read_json(config.SETTINGS_FILE)
    if not isinstance(data, dict) or data.get("format_version") != FORMAT_VERSION:
        return {"format_version": FORMAT_VERSION, "profile": PROFILE, "groups": {}}
    return data


def load(group):
    """The values a sensor of this group must open with: the defaults, overridden by the saved ones."""
    values = defaults(group)
    saved = _read_file()["groups"].get(group, {}).get("values", {})
    for key, value in saved.items():
        if key in values:
            try:
                values[key] = _coerce(FIELD_BY_KEY[group][key], value)
            except ValueError as e:
                print(f"WARNING: ignoring saved setting {group}.{key}: {e}")
    return values


def saved_at(group):
    return _read_file()["groups"].get(group, {}).get("saved_at")


def save(group, values):
    """Validate and save the values of a group; returns (values, errors, warnings), nothing saved
    when there are errors."""
    values, errors, warnings = validate(group, values)
    if errors:
        return values, errors, warnings
    data = _read_file()
    data["groups"][group] = {"saved_at": datetime.now().isoformat(timespec="seconds"),
                             "values": {k: v for k, v in values.items() if v != FIELD_BY_KEY[group][k]["default"]}}
    os.makedirs(os.path.dirname(config.SETTINGS_FILE), exist_ok=True)
    write_json_atomic(config.SETTINGS_FILE, data)
    return values, errors, warnings


def reset(group):
    data = _read_file()
    data["groups"].pop(group, None)
    os.makedirs(os.path.dirname(config.SETTINGS_FILE), exist_ok=True)
    write_json_atomic(config.SETTINGS_FILE, data)


def changed_from_defaults(group, values=None):
    """[(label, value, unit)] of the values that differ from the defaults."""
    values = load(group) if values is None else values
    return [(f["label"], values[f["key"]], f["unit"]) for f in FIELDS[group]
            if values[f["key"]] != f["default"] and applies(f, values)]


def applies(f, values):
    return f["when"] is None or values.get(f["when"][0]) == f["when"][1]


# ============================================================================
# VALIDATION
# ============================================================================

def _coerce(f, value):
    if value is None or value == "":
        if f["optional"]:
            return None
        raise ValueError("required")
    if f["kind"] == "bool":
        if not isinstance(value, bool):
            raise ValueError("must be true or false")
        return value
    if f["kind"] == "choice":
        if value not in [c[0] for c in f["choices"]]:
            raise ValueError(f"must be one of {', '.join(c[0] for c in f['choices'])}")
        return value
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError("must be a number")
    if not np.isfinite(number):
        raise ValueError("must be a number")
    if f["kind"] == "int":
        if number != int(number):
            raise ValueError("must be a whole number")
        number = int(number)
    if f["min"] is not None and number < f["min"]:
        raise ValueError(f"must be at least {f['min']:g}")
    if f["max"] is not None and number > f["max"]:
        raise ValueError(f"must be at most {f['max']:g}")
    return number


def validate(group, values):
    """(values, {key: error}, [warning]): each field checked alone, then the combinations."""
    if group not in FIELDS:
        raise ValueError(f"unknown settings group {group!r}")
    clean, errors = defaults(group), {}
    for f in FIELDS[group]:
        if f["key"] in values:
            try:
                clean[f["key"]] = _coerce(f, values[f["key"]])
            except ValueError as e:
                errors[f["key"]] = str(e)
    warnings = []
    if not errors:
        (_validate_rgb if group == "rgb" else _validate_event)(clean, errors, warnings)
    return clean, errors, warnings


def rgb_size(values):
    """(width, height) px of the Basler ROI, as basler.setup_stereo_cameras sets it."""
    def multiple_of_4(n):   # rounded up
        return -(-n // 4) * 4
    w = multiple_of_4(int(config.RGB_SENSOR_SIZE[0] * values["roi_width"]))
    h = multiple_of_4(int(config.RGB_SENSOR_SIZE[1] * values["roi_height"]))
    return min(w, config.RGB_SENSOR_SIZE[0]), min(h, config.RGB_SENSOR_SIZE[1])


def _validate_rgb(v, errors, warnings):
    period_us = 1e6 / v["fps"]
    if v["exposure_auto"] == "Continuous":
        if v["exposure_lower_us"] > v["exposure_upper_us"]:
            errors["exposure_lower_us"] = "must not exceed the auto exposure max"
        longest, key = v["exposure_upper_us"], "exposure_upper_us"
    else:
        longest, key = v["exposure_us"], "exposure_us"
    if longest >= period_us:
        errors[key] = f"{longest:g} µs does not fit in a frame at {v['fps']:g} fps ({period_us:.0f} µs)"
    w, h = rgb_size(v)
    mb_s = w * h * 3 * v["fps"] / 1e6
    if mb_s > GIGE_MAX_MB_S:
        warnings.append(f"{w} × {h} RGB8 at {v['fps']:g} fps is {mb_s:.0f} MB/s per camera, more than a GigE link "
                        f"carries (~{GIGE_MAX_MB_S} MB/s): frames will be missed. Lower the frame rate or the ROI.")
    if not v["ptp_trigger"]:
        warnings.append("PTP trigger off: left and right expose at different instants (not synchronized).")
    roi = (v["roi_width"], v["roi_height"])
    for name, path in [("RGB ↔ event", config.RGB_EVENT_CALIBRATION_FILE), ("RGB ↔ Livox", config.RGB_LIVOX_CALIBRATION_FILE)]:
        calibrated = calibrated_rgb_roi(path)
        if calibrated is not None and not same_roi(calibrated, roi):
            warnings.append(f"The installed {name} calibration was made with the RGB ROI "
                            f"{calibrated[0]:g} × {calibrated[1]:g}, these settings use {roi[0]:g} × {roi[1]:g}: "
                            "replay overlays and the tools that use it will refuse it until you recalibrate.")


def _validate_event(v, errors, warnings):
    if v["antiflicker"] and v["antiflicker_low_hz"] >= v["antiflicker_high_hz"]:
        errors["antiflicker_high_hz"] = "must be above the band low"
    if v["roi"]:
        if v["roi_x"] + v["roi_width"] > 1280:
            errors["roi_width"] = f"x + width must not exceed 1280 (is {v['roi_x'] + v['roi_width']})"
        if v["roi_y"] + v["roi_height"] > 720:
            errors["roi_height"] = f"y + height must not exceed 720 (is {v['roi_y'] + v['roi_height']})"
    if v["record_max_rate_mev"] is None:
        warnings.append("No rate cap while recording: a flickering scene reached ~38 Mev/s per camera "
                        "(~150 MB/s of raw data each, events delivered late).")
    live = v["calibration_max_rate_mev"]
    if live is None or live > 30:
        warnings.append("Live tools above ~20 Mev/s: events may arrive too late for the calibration captures.")


# ============================================================================
# CALIBRATION GEOMETRY
# ============================================================================

def rgb_roi():
    values = load("rgb")
    return values["roi_width"], values["roi_height"]


def same_roi(a, b):
    return all(abs(float(x) - float(y)) < 1e-6 for x, y in zip(a, b))


def calibrated_rgb_roi(path):
    """RGB ROI fraction an installed calibration was made with, or None."""
    if not os.path.exists(path):
        return None
    try:
        with np.load(path) as data:
            return tuple(float(x) for x in data["rgb_roi_fraction"]) if "rgb_roi_fraction" in data.files else None
    except (OSError, ValueError):
        return None


# ============================================================================
# APPLIED SETTINGS
# ============================================================================

def applied_report(group, values, readback=None, notes=None):
    """What a sensor opened with: the settings values, what the device reports back, and the
    adjustments it made (clamped values...)."""
    return {"group": group, "values": values, "readback": readback or {}, "notes": notes or [],
            "saved_at": saved_at(group), "opened_at": datetime.now().isoformat(timespec="seconds")}


def write_applied(session, output, report):
    """Record in a session the settings an output was recorded with: <session>/settings/<output>.json."""
    folder = os.path.join(session, "settings")
    os.makedirs(folder, exist_ok=True)
    write_json_atomic(os.path.join(folder, f"{output}.json"), report)


def read_applied(session, output):
    return read_json(os.path.join(session, "settings", f"{output}.json"))


def pending(group, report):
    """Labels of the saved values a sensor opened with this report does not use yet."""
    if report is None:
        return []
    saved = load(group)
    return [f["label"] for f in FIELDS[group] if report["values"].get(f["key"]) != saved[f["key"]]]

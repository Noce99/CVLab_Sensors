"""Basler stereo pair (GigE): camera settings (from the Settings page), opening both cameras, clocks."""

import time

from pypylon import pylon

from cvlab_sensors import config
from cvlab_sensors.settings import store
from cvlab_sensors.settings.store import rgb_size

CAMERA_NAMES = ["rgb_left", "rgb_right"]
CAMERA_IPS = [config.RGB_LEFT_IP, config.RGB_RIGHT_IP]
CLOCK_SYNC_SAMPLES = 5


def set_clamped(node, value, label, unit, notes):
    """Set a float node to value clamped to its range; a note when it had to be clamped."""
    clamped = min(max(value, node.Min), node.Max)
    if clamped != value:
        notes.append(f"{label} {value:g} {unit} is outside the camera range, set to {clamped:g} {unit}")
    node.Value = clamped


def setup_stereo_cameras(camera, values):
    """Configure one camera with the "rgb" settings (cvlab_sensors.settings.store); returns
    (readback, notes): the values the camera reports, and what it could not set as asked."""
    notes = []

    #######
    # PTP #
    #######
    ptp_settings = {
        # Lowest priority: the cameras must never win the master election against ptp4l
        "BslPtpPriority1": 255,
        # Enable end-to-end delay measurement
        "BslPtpProfile": "DelayRequestResponseDefaultProfile",
        "BslPtpNetworkMode": "Multicast",
        # Enable PTP Management Protocol
        "BslPtpManagementEnable": True,
        # Disable two-step operation
        "BslPtpTwoStep": False,
    }
    # Reconfiguring PTP needs disabling it, which loses the lock (and re-locking
    # takes a while): only do it when a setting is different.
    if not camera.PtpEnable.Value or any(getattr(camera, k).Value != v for k, v in ptp_settings.items()):
        camera.PtpEnable.Value = False
        for k, v in ptp_settings.items():
            getattr(camera, k).Value = v
        camera.PtpEnable.Value = True

    #######
    # FPS #
    #######
    if values["ptp_trigger"]:
        # Frame start on every tick of a periodic signal derived from the PTP clock:
        # all PTP-locked cameras fire at the same instants
        camera.AcquisitionFrameRateEnable.Value = False
        camera.BslPeriodicSignalSelector.Value = "PeriodicSignal1"
        # Read-only (fixed to PtpClock) while PTP is enabled
        if camera.BslPeriodicSignalSource.Value != "PtpClock":
            raise RuntimeError("Periodic signal is not driven by the PTP clock: is PTP enabled?")
        camera.BslPeriodicSignalPeriod.Value = 1e6 / values["fps"]
        camera.BslPeriodicSignalDelay.Value = 0
        camera.TriggerSelector.Value = "FrameStart"
        camera.TriggerSource.Value = "PeriodicSignal1"
        camera.TriggerMode.Value = "On"
    else:
        camera.TriggerSelector.Value = "FrameStart"
        camera.TriggerMode.Value = "Off"
        camera.AcquisitionFrameRateEnable.Value = True
        set_clamped(camera.AcquisitionFrameRate, values["fps"], "Frame rate", "fps", notes)

    ############
    # Exposure #
    ############
    # The rig is on a moving car: short exposures, gain compensates in low light
    if values["exposure_auto"] == "Continuous":
        camera.ExposureAuto.Value = "Continuous"
        set_clamped(camera.AutoExposureTimeLowerLimit, values["exposure_lower_us"], "Auto exposure min", "µs", notes)
        set_clamped(camera.AutoExposureTimeUpperLimit, values["exposure_upper_us"], "Auto exposure max", "µs", notes)
    else:
        camera.ExposureAuto.Value = "Off"
        set_clamped(camera.ExposureTime, values["exposure_us"], "Exposure time", "µs", notes)

    ########
    # Gain #
    ########
    if values["gain_auto"] == "Continuous":
        camera.GainAuto.Value = "Continuous"
        camera.AutoGainLowerLimit.Value = camera.AutoGainLowerLimit.Min
        set_clamped(camera.AutoGainUpperLimit, values["gain_upper_db"], "Auto gain max", "dB", notes)
    else:
        camera.GainAuto.Value = "Off"
        set_clamped(camera.Gain, values["gain_db"], "Gain", "dB", notes)

    #################
    # Auto Function #
    #################
    camera.AutoFunctionProfile.Value = values["auto_profile"]
    set_clamped(camera.AutoTargetBrightness, values["target_brightness"], "Auto target brightness", "", notes)

    ##########
    # Colors #
    ##########
    camera.PixelFormat.Value = values["pixel_format"]
    if values["white_balance"] != "unchanged":
        camera.BalanceWhiteAuto.Value = values["white_balance"]

    ################
    # Spatial Size #
    ################
    # Offsets first: the largest width allowed is the sensor width minus the current offset
    camera.OffsetX.Value = 0
    camera.OffsetY.Value = 0
    w, h = rgb_size(values)
    camera.Width.Value = w
    camera.Height.Value = h
    camera.BslCenterX.Execute()
    camera.BslCenterY.Execute()

    def read(get):
        try:
            return get()
        except Exception:   # a node this camera model does not have: not worth failing the open
            return None

    width, height = read(lambda: camera.Width.Value), read(lambda: camera.Height.Value)
    readback = {
        "size": f"{width} × {height} px",   # the ROI fractions, rounded up to multiples of 4 px
        "fps": read(lambda: round(1e6 / camera.BslPeriodicSignalPeriod.Value, 3) if values["ptp_trigger"]
                    else round(camera.AcquisitionFrameRate.Value, 3)),
        "max_fps": read(lambda: round(camera.ResultingFrameRate.Value, 1)),
        "ptp_trigger": read(lambda: camera.TriggerMode.Value == "On" and camera.TriggerSource.Value == "PeriodicSignal1"),
        "exposure_auto": read(lambda: camera.ExposureAuto.Value),
        "exposure_lower_us": read(lambda: camera.AutoExposureTimeLowerLimit.Value),
        "exposure_upper_us": read(lambda: camera.AutoExposureTimeUpperLimit.Value),
        "exposure_us": read(lambda: round(camera.ExposureTime.Value, 1)),
        "gain_auto": read(lambda: camera.GainAuto.Value),
        "gain_upper_db": read(lambda: round(camera.AutoGainUpperLimit.Value, 2)),
        "gain_db": read(lambda: round(camera.Gain.Value, 2)),
        "target_brightness": read(lambda: round(camera.AutoTargetBrightness.Value, 3)),
        "auto_profile": read(lambda: camera.AutoFunctionProfile.Value),
        "white_balance": read(lambda: camera.BalanceWhiteAuto.Value),
        "pixel_format": read(lambda: camera.PixelFormat.Value),
    }
    # Without the trigger the camera runs at most at the rate its exposure and readout allow
    if not values["ptp_trigger"] and readback["max_fps"] is not None and readback["max_fps"] < values["fps"]:
        notes.append(f"The camera can only reach {readback['max_fps']:g} fps with these settings")
    return readback, notes


def open_cameras(values):
    """Open and configure both cameras with the "rgb" settings; returns (cameras, [report per camera])
    (cvlab_sensors.settings.store.applied_report)."""
    cameras = pylon.InstantCameraArray(2)
    tlf = pylon.TlFactory.GetInstance()
    tl = tlf.CreateTl("BaslerGigE")

    reports = []
    for idx, ip in enumerate(CAMERA_IPS):
        info = tl.CreateDeviceInfo()
        info.SetIpAddress(ip)
        cameras[idx].Attach(tl.CreateDevice(info))
        cameras[idx].Open()
        readback, notes = setup_stereo_cameras(cameras[idx], values)
        reports.append(store.applied_report("rgb", values, readback, notes))
        for note in notes:
            print(f"WARNING {CAMERA_NAMES[idx]}: {note}")
        cameras[idx].SetCameraContext(idx)
        cameras[idx].MaxNumBuffer.Value = config.RGB_GRAB_BUFFERS
        cameras[idx].StreamGrabber.TransmissionType.Value = "Multicast"
        cameras[idx].StreamGrabber.DestinationAddr.Value = config.RGB_STREAM[idx]["destination"]
        cameras[idx].StreamGrabber.DestinationPort.Value = config.RGB_STREAM[idx]["port"]
    return cameras, reports


def measure_clock_offset(camera):
    """host_ns - camera_ns, from the latch sample with the shortest round trip.

    Returns (offset_ns, uncertainty_ns).
    """
    best = None
    for _ in range(CLOCK_SYNC_SAMPLES):
        t0 = time.time_ns()
        camera.TimestampLatch.Execute()
        t1 = time.time_ns()
        camera_ns = camera.TimestampLatchValue.Value
        half_rtt = (t1 - t0) // 2
        if best is None or half_rtt < best[1]:
            best = ((t0 + t1) // 2 - camera_ns, half_rtt)
    return best


def ptp_state(camera):
    camera.PtpDataSetLatch.Execute()
    return camera.PtpStatus.Value, camera.PtpServoStatus.Value, camera.PtpOffsetFromMaster.Value

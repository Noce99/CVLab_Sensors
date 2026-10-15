"""Basler stereo pair (GigE): camera settings, opening both cameras, clocks."""

import time

from pypylon import pylon

from cvlab_sensors import config

CAMERA_NAMES = ["rgb_left", "rgb_right"]
CAMERA_IPS = [config.RGB_LEFT_IP, config.RGB_RIGHT_IP]
CLOCK_SYNC_SAMPLES = 5


def setup_stereo_cameras(camera):
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
    if config.RGB_PTP_TRIGGER:
        # Frame start on every tick of a periodic signal derived from the PTP clock:
        # all PTP-locked cameras fire at the same instants
        camera.AcquisitionFrameRateEnable.Value = False
        camera.BslPeriodicSignalSelector.Value = "PeriodicSignal1"
        # Read-only (fixed to PtpClock) while PTP is enabled
        if camera.BslPeriodicSignalSource.Value != "PtpClock":
            raise RuntimeError("Periodic signal is not driven by the PTP clock: is PTP enabled?")
        camera.BslPeriodicSignalPeriod.Value = 1e6 / config.RGB_FPS
        camera.BslPeriodicSignalDelay.Value = 0
        camera.TriggerSelector.Value = "FrameStart"
        camera.TriggerSource.Value = "PeriodicSignal1"
        camera.TriggerMode.Value = "On"
    else:
        camera.TriggerSelector.Value = "FrameStart"
        camera.TriggerMode.Value = "Off"
        camera.AcquisitionFrameRateEnable.Value = True
        camera.AcquisitionFrameRate.Value = config.RGB_FPS

    ############
    # Exposure #
    ############
    # The rig is on a moving car: short exposures, gain compensates in low light
    camera.ExposureAuto.Value = "Continuous"
    camera.AutoExposureTimeLowerLimit.Value = config.RGB_EXPOSURE_LOWER_US
    camera.AutoExposureTimeUpperLimit.Value = config.RGB_EXPOSURE_UPPER_US

    ########
    # Gain #
    ########
    camera.GainAuto.Value = "Continuous"
    camera.AutoGainLowerLimit.Value = camera.AutoGainLowerLimit.Min
    camera.AutoGainUpperLimit.Value = min(config.RGB_GAIN_UPPER_DB, camera.AutoGainUpperLimit.Max)

    #################
    # Auto Function #
    #################
    camera.AutoFunctionProfile.Value = "MinimizeExposureTime"
    camera.AutoTargetBrightness.Value = config.RGB_AUTO_TARGET_BRIGHTNESS

    ##########
    # Colors #
    ##########
    camera.PixelFormat.Value = config.RGB_PIXEL_FORMAT

    ################
    # Spatial Size #
    ################
    w = int(config.RGB_SENSOR_SIZE[0] * config.RGB_ROI_FRACTION[0])
    h = int(config.RGB_SENSOR_SIZE[1] * config.RGB_ROI_FRACTION[1])

    while w % 4 != 0:
        w = w + 1
    camera.Width.Value = w

    while h % 4 != 0:
        h = h + 1
    camera.Height.Value = h

    camera.BslCenterX.Execute()
    camera.BslCenterY.Execute()


def open_cameras():
    cameras = pylon.InstantCameraArray(2)
    tlf = pylon.TlFactory.GetInstance()
    tl = tlf.CreateTl("BaslerGigE")

    for idx, ip in enumerate(CAMERA_IPS):
        info = tl.CreateDeviceInfo()
        info.SetIpAddress(ip)
        cameras[idx].Attach(tl.CreateDevice(info))
        cameras[idx].Open()
        setup_stereo_cameras(cameras[idx])
        cameras[idx].SetCameraContext(idx)
        cameras[idx].StreamGrabber.TransmissionType.Value = "Multicast"
        cameras[idx].StreamGrabber.DestinationAddr.Value = config.RGB_STREAM[idx]["destination"]
        cameras[idx].StreamGrabber.DestinationPort.Value = config.RGB_STREAM[idx]["port"]
    return cameras


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

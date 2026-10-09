import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from cvlab import config


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

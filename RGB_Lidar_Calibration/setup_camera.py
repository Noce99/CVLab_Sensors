def setup_camera(camera, random=False):
    #######
    # PTP #
    #######
    # First let's disable PTP to change its parameters
    camera.PtpEnable.Value = False
    camera.BslPtpPriority1.Value = 255
    # Enable end-to-end delay measurement
    camera.BslPtpProfile.Value = "DelayRequestResponseDefaultProfile"
    # Set the network mode to unicast
    camera.BslPtpNetworkMode.Value = "Multicast"
    # Enable PTP Management Protocol
    camera.BslPtpManagementEnable.Value = True
    # Disable two-step operation
    camera.BslPtpTwoStep.Value = False
    # Enable PTP on the current device
    camera.PtpEnable.Value = True

    #######
    # FPS #
    #######
    camera.AcquisitionFrameRateEnable.Value = True
    camera.AcquisitionFrameRate.Value = 1

    ############
    # Exposure #
    ############
    camera.ExposureAuto.Value = "Continuous"
    camera.AutoExposureTimeLowerLimit.Value = 1000.0
    camera.AutoExposureTimeUpperLimit.Value = 20000.0

    ########
    # Gain #
    ########
    camera.GainAuto.Value = "Continuous"
    # camera.AutoGainLowerLimit.Value = 0.0
    # camera.AutoGainUpperLimit.Value = 48.0

    #################
    # Auto Function #
    #################
    camera.AutoFunctionProfile.Value = "MinimizeExposureTime"
    # MinimizeExposureTime          MinimizeGain
    
    ##########
    # Colors #
    ##########
    camera.PixelFormat.Value = "Mono8"
    """
    Mono8, Mono10, Mono10p, Mono12, Mono12p, RGB8, BGR8 ...
    """

    ################
    # Spatial Size #
    ################
    w = 2472
    h = 1500

    camera.BslCenterX.Execute()
    camera.BslCenterY.Execute()


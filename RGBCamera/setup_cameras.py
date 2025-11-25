def setup_camera(camera):
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
    camera.AcquisitionFrameRate.Value = 20

    ############
    # Exposure #
    ############
    camera.ExposureAuto.Value = "Continuous"
    camera.AutoExposureTimeLowerLimit.Value = 1.0
    camera.AutoExposureTimeUpperLimit.Value = 100000.0

    ########
    # Gain #
    ########
    camera.GainAuto.Value = "Continuous"
    # camera.AutoGainLowerLimit.Value = 0.0
    # camera.AutoGainUpperLimit.Value = 48.0

    #################
    # Auto Function #
    #################
    camera.AutoFunctionProfile.Value = "MinimizeGain"
    # MinimizeExposureTime          MinimizeGain
    
    ##########
    # Colors #
    ##########
    camera.PixelFormat.Value = "RGB8"
    """
    Mono8, Mono10, Mono10p, Mono12, Mono12p, RGB8, BGR8 ...
    """

    ################
    # Spatial Size #
    ################
    w = int(2448 / 2.5)      # 2448
    h = int(2048 / 2.5)     # 2048

    if w % 2 != 0:
        camera.Width.Value = w + 1
    else:
        camera.Width.Value = w 

    if h % 2 != 0:
        camera.Height.Value = h + 1
    else:
        camera.Height.Value = h

    camera.BslCenterX.Execute()
    camera.BslCenterY.Execute()


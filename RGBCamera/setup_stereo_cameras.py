def setup_stereo_cameras(camera, random=False):
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
    camera.AcquisitionFrameRate.Value = 10

    ############
    # Exposure #
    ############
    camera.ExposureAuto.Value = "Continuous"
    camera.AutoExposureTimeLowerLimit.Value = 50000.0  # us
    camera.AutoExposureTimeUpperLimit.Value = 100000.0 # us

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
    camera.PixelFormat.Value = "RGB8"
    """
    Mono8, Mono10, Mono10p, Mono12, Mono12p, RGB8, BGR8 ...
    """

    ################
    # Spatial Size #
    ################
    w = int(2448 * 0.8)     # 2448
    h = int(2048  * 0.6)     # 2048


    while w % 4 != 0:
        w = w + 1
    
    camera.Width.Value = w 

    while h %  4 != 0:
        h = h + 1

    camera.Height.Value = h

    
    # camera.OffsetX.Value = 500

    camera.BslCenterX.Execute()
    camera.BslCenterY.Execute()


"""Shared code for the CVLab sensor rig (config, recorders, lidar I/O, calibration helpers).

Scripts outside this package add the repo root to sys.path:

    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from cvlab import config
"""

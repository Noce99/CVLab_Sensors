"""python -m cvlab_sensors: the rig's web app (see cvlab_sensors/app.py)."""

from cvlab_sensors.app import main

# Guarded: the sensor and event-cache processes are spawned, and a spawned process imports this module again
if __name__ == "__main__":
    main()

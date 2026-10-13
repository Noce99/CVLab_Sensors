"""python -m cvlab: the rig's web app (see cvlab/app.py)."""

from cvlab.app import main

# Guarded: the sensor and event-cache processes are spawned, and a spawned process imports this module again
if __name__ == "__main__":
    main()

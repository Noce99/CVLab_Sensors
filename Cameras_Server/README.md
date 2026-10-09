## Camera Monitoring Server

Web dashboard to start, stop and monitor the sensor recorders from a phone or a laptop.
It runs the same recorders as `get_data/get_data.py`:

| Group | Recorder | Outputs |
|-------|----------|---------|
| `rgb` | `get_data/rgb.py` | `rgb_left`, `rgb_right` |
| `event` | `get_data/event.py` (x2) | `event_left`, `event_right` |
| `lidar_livox` | `get_data/cpp/save_point_cloud_livox` | `lidar_livox` |
| `lidar_hesai` | `get_data/cpp/save_point_cloud_hesai` | `lidar_hesai` |

### Start

```bash
./start_server.sh                 # from any directory
../venv_3_12/bin/python camera_server.py --allow-unsynced   # record RGB without PTP
```

Open `http://<NUC_IP>:5000`.

### Sessions

- The first **Start** opens a session in `get_data/data/<date>/`.
- **Salva sessione** moves it to `CVLab_live_imgs/<date>/`.
- **Cancella sessione** deletes it (asks for confirmation).
- Both are refused while a sensor is recording: stop everything first.
- Recording stops by itself when the disk has less than `MIN_FREE_DISK_GB` free.

### Status

Each sensor card shows whether it is active, its rate (frames/s, Mev/s for event
cameras, files/s for lidars), the last update and any error from the recorder. The
recorder logs are in `<session>/logs/`. There is no dummy/simulation mode: a sensor
that fails shows its error and the recorder keeps trying to reconnect.

All settings are in `cvlab/config.py`.

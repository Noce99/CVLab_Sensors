# CVLab Sensors

One web app drives the sensor rig: record sessions, replay them, and calibrate the sensors.

```
venv_3_12/bin/python -m cvlab_sensors          # from the repo root, then open http://<NUC_IP>:5001
tail -f data/logs/app.log                      # its output
```
The server writes its output to `data/logs/app.log`, not to the terminal, and survives the terminal
closing: the NUC is reached over Wi-Fi, which drops during runs, and an SSH session that stalls or
closes would otherwise block or kill the server (the recorders kept running, without the disk
guard). Ctrl+C stops it; a dropped session leaves it running (`pgrep -af cvlab_sensors`, stop it
with `kill -INT <pid>`). The Record page shows when it lost the connection and does not stop the
recording: reload it when the connection is back.

To start it at boot instead (then do not start it by hand: the port is taken), install the service:
```
sudo cp system/cvlab-sensors.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now cvlab-sensors
sudo systemctl restart cvlab-sensors           # after changing the code
sudo systemctl stop cvlab-sensors              # the recorders flush their data first
journalctl -u cvlab-sensors                    # start / stop errors (the rest is in data/logs/app.log)
```

| Computers | ssh                      |
| --------  | -------                  |
| Orin      | `ssh cvlab@192.168.1.49` |
| Nuc       | `ssh ubm@192.168.1.50`   |

| Sensor Type | Sensor Name | SDK | Connection | IP | Used by the app |
| --------    | -------     | -------   | ------- | ------- | ------- |
| Lidar  | XT32  | [HesaiLidar SDK](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0) | Ethernet | 192.168.1.91 | yes |
| Lidar  | Livox HAP  | [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2) | Ethernet | 192.168.1.100 | yes |
| Event Camera  | 2x EVK4 - HD | [Metavision SDK](https://propheseeai.jfrog.io/ui/repos/tree/General/metavision-sdk-5-debian) | USB | | yes |
| RGB Camera | 2x Basler | [PyPylon](https://github.com/basler/pypylon/tree/master) | Ethernet | 192.168.1.70-71 | yes |
| Lidar  | OT128  | [HesaiLidar SDK](https://github.com/HesaiTechnology/HesaiLidar_SDK_2.0) | Ethernet | 192.168.1.90 | no |
| Lidar  | Livox Mid360  | [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)  | Ethernet | 192.168.1.151 | no |
| Depth Camera  | RealSense | [RealSense SDK](https://github.com/IntelRealSense/librealsense) | USB | | no |

## Repository layout
```
cvlab_sensors/             Python package: the web app and everything it uses
├── __main__.py, app.py    python -m cvlab_sensors: Flask app, home page, tool registry
├── config.py              IPs, serials, paths, calibration parameters, and the defaults of the sensor settings
├── settings/              Settings page (page.py) and the saved sensor settings (store.py, data/settings/active.json)
├── web.py                 shared page code: image helpers, background jobs, capture / solve routes
├── status.py              status JSON and preview files written by the recorders
├── sensors/               sensor I/O
│   ├── basler.py, evk4.py     camera settings (Basler pair, EVK4)
│   ├── hesai.py               Hesai settings, read over its PTC protocol (never written)
│   ├── events.py, lidar.py    readers of the recorded event (.raw) and lidar (.h5) files
│   ├── timesync.py            device clock -> host clock mapping
│   └── live.py                SensorManager: live sensors for the calibration capture pages
├── recording/             Record page (page.py), recorder processes (recorders.py),
│                          RGB and event recorders (rgb_recorder.py, event_recorder.py)
├── replay/                Replay page (page.py), session reader (session.py), reprojection (views.py)
├── calibration/           common.py (chessboard, captured boards), rig.py (installed rig)
│   ├── rgb_event/             capture.py + solve.py: 2 Basler + 2 EVK4, intrinsics and extrinsics
│   ├── rgb_livox/             capture.py + solve.py (+ lidar_image.py): RGB-left <-> Livox
│   └── livox_hesai/           capture.py + solve.py (+ registration.py): Livox <-> Hesai, target-free
├── tools/                 check_ptp.py, find_hot_pixels.py
└── templates/, static/    pages
lidar_recorders/           C++ Livox / Hesai recorders (built in the SDK trees by compile.sh), livox_hap.json
system/                    ptp4l config + systemd services (ptp4l, web app), EVK4 reset + USB3 check script (sudoers)
data/                      everything the app writes (not in git)
├── staging/               sessions being recorded
├── sessions/              saved sessions
├── calibration/           <tool>/captures/, <tool>/results/, installed *_calibration.npz
├── stream/                lidar chunks of the live capture pages (emptied as read)
└── legacy/                data of the old scripts, kept for reference
```
Every Python entry point runs as a module from the repo root (`python -m cvlab_sensors...`).

## Recording
Web app → **Record**: start / stop each sensor group, save or discard the session. The live images are
the recorders' small previews (`<session>/previews/`), not the recorded data.

Sessions are recorded in `data/staging/<date>/` and moved to `data/sessions/<date>/` when saved.
Recording stops automatically when less than `MIN_FREE_DISK_GB` is free.
```
<session>/
├── rgb_left/, rgb_right/        <host_time_us>.png
├── event_left/, event_right/    <start_us>.raw (+ .sync.csv clock mapping, .json metadata)
├── lidar_livox/, lidar_hesai/   <host_time_us>.h5: xyz (mm, float32, native sensor frame),
│                                timestamp_ns (lidar clock), intensity, sync [lidar_ns, host_ns]
├── rgb_clock_sync.csv           Basler clock <-> host clock
├── logs/, status/, previews/    logs/host.log: host events during the recording
├── replay_cache/                event frames rendered by the Replay page
```
Readers are in `cvlab_sensors/sensors/` (`lidar.py`, `events.py`). Read `.raw` files with
`cvlab_sensors.sensors.events.open_raw`: Metavision otherwise shifts a file's timestamps so its first event is at 0,
which breaks the `.sync.csv` mapping when a camera was opened before the recording started.

The event rate is capped in hardware at `EVENT_RECORD_MAX_RATE` (20 Mev/s per camera): uncapped, a
flickering lab reached ~38 Mev/s and the events reached the host up to 5 s late.
**Event cameras must be on USB3**: on USB2 (480 Mbps) their stream gets corrupted (`Evt3 protocol
violation` in the logs, holes of ~80 ms every 100 ms, the recorder can crash) even at 2 Mev/s. At a
cold boot they power up together with the hub and miss the USB3 link training: they show up on the
hub's USB2 side (`lsusb -t`: under a 480M bus) and stay there. **After each boot, re-plug both
EVK4s**: the Record page warns when one is on USB2. Software cannot power-cycle them on this hub
(tried: its power switching is ganged and turning off every port did not cut the power); a hub with
per-port power switching (`uhubctl`) would allow it. Hot pixels are masked in hardware
(`EVENT_HOT_PIXELS` in `cvlab_sensors/config.py`, find them with `python -m cvlab_sensors.tools.find_hot_pixels <file.raw>`).

Before recording the event cameras, the app resets them (and checks they are on USB3) with
`system/reset_event_cameras.sh` through `sudo -n`, from a root-owned copy: a passwordless sudo rule
on a file the user can edit would give root to anyone running as that user. Install it (again after
each change of the script) and allow it without password (`sudo visudo -f /etc/sudoers.d/cvlab-sensors`):
```
sudo install -o root -g root -m 755 system/reset_event_cameras.sh /usr/local/sbin/cvlab-reset-event-cameras
ubm ALL=(ALL) NOPASSWD: /usr/local/sbin/cvlab-reset-event-cameras      # the sudoers rule
```
Each recording also keeps the host events (kernel/USB, Wi-Fi, NetworkManager, tailscale, ptp4l) in
`<session>/logs/host.log`, to check holes in the data against.

## Replay
Web app → **Replay**: any session of `data/sessions/` (or not yet saved, in `data/staging/`) along a
timeline (play / pause, ×0.25–×4, frame steps, go to a time; Space and ←/→).
Each sensor alone at its recorded resolution, or all of them on RGB left with the installed calibrations,
one checkbox and alpha per sensor: lidar points land exactly (needs RGB ↔ Livox, and Livox ↔ Hesai for
the Hesai). The other cameras (RGB right, events) are warped through the lidar depth of each RGB-left pixel
(Warp = Lidar depth: the lidar points of the window, densified) and through a plane at the chosen distance
where the lidars see nothing (above / below their field of view, screens and glass that return no points);
Warp = Plane puts everything at that distance. Without lidar depth, objects off the plane are shifted by
their parallax (the event camera is ~4 cm from RGB left: ~70 px for something at 0.75 m with the plane at 5 m).
Events are drawn red (ON) and blue (OFF) on black, in one of two representations, over an accumulation
time (20 ms by default, 1–1000 ms):
- **TEncode** (default): the newest event of each pixel, its age in green (0 at the start of the window,
  255 at its end: recent ON events look yellow, recent OFF events cyan);
- **Histogram**: the number of events of each polarity per pixel (full intensity at 3 events).

Event recordings are rendered once into frames, one per accumulation time, so every event is in exactly one
frame ("Prepare events", `<session>/replay_cache/<representation>_<ms>ms/`, about the recording's duration);
each representation and accumulation time is its own cache. `/replay?session=<name>&t=<s>&view=<sensor>`
(plus `&events=<tencode|histogram>&acc_ms=<ms>`) opens a moment directly.
Sessions marked "(not saved)" are still in `data/staging/` (recorded before the server restarted, so the
Record page no longer knows them): the Replay page can **Save** them to `data/sessions/` or **Discard** them.

## Calibration (RGB <-> event, RGB <-> Livox, Livox <-> Hesai)
Stop the recordings first: a sensor can be opened by one program only, and the capture pages refuse
while the Record page records. Each capture page opens the sensors it needs and releases the others.

1. **Get RGB-event calibration data**: the 4 cameras live; **Capture** (or Space, or automatic capture)
   looks for the chessboard (29 mm squares by default, set per session on the page and confirmed on the calibrate page) in all four and saves
   the board to `data/calibration/rgb_event/captures/<date>/` when all four see it and it did not move.
   Each view shows the boards saved so far, the areas still uncovered (red cells) and how old its image is.
   The event cameras only see changes: use the **Blinking pattern** (on a screen, frequency on the page) or
   move the board. The session selector adds boards to an existing folder. The event rate is capped in
   hardware at `EVENT_CALIBRATION_MAX_RATE` (20 Mev/s) so the tool keeps up with flickering scenes.
2. **Calibrate RGB <-> event**: intrinsics of the 4 cameras, both stereo pairs and RGB-left <-> event-left,
   installed as `data/calibration/rgb_event_calibration.npz`.
3. **Get RGB-Livox calibration data**: RGB-left and the Livox reflectivity image live. The HAP pattern
   needs seconds to cover the board: each capture collects the points of `Hold` seconds (8 by default,
   `LIVOX_CAPTURE_S`) while the board stays still (checked on RGB-left), and saves the board to
   `data/calibration/rgb_livox/captures/<date>/` when both see it.
4. **Calibrate RGB <-> Livox**: extrinsics only, with the RGB-left intrinsics of the installed RGB-event
   calibration (do step 2 first) and the Livox as a virtual camera (`LIVOX_IMAGE_SIZE`, `LIVOX_FOCAL_PX`);
   checks the Livox points on the board against the board plane seen by RGB-left. Installed as
   `data/calibration/rgb_livox_calibration.npz` (`rgb_from_livox`: native Livox point -> RGB-left camera, mm).
5. **Get Livox-Hesai calibration data**: no target. Stop the car in a place with structure in front of it
   (walls, kerbs, steps, a slope, parked cars at 3–15 m), keep the rig and the scene still and record a
   stop (15 s by default). Each stop is checked at once: the rig must not have moved during it, and the
   page shows how well the scene constrains each direction (surfaces facing only one way leave the
   translation along them free). Record 3–6 stops in different places and headings.
6. **Calibrate Livox <-> Hesai**: registers the Hesai points on the Livox surfaces (point-to-plane ICP)
   over all stops at once, from an initial guess of the Hesai position in the Livox frame (a ruler is
   enough: it converges from ±30 cm / ±10°; default `LIVOX_HESAI_INITIAL_T_MM`). Every stop is also solved
   alone: their spread is the uncertainty. Installed as `data/calibration/livox_hesai_calibration.npz`
   (`livox_from_hesai`: native Hesai point -> native Livox frame, mm).

The solvers also run from the terminal:
`python -m cvlab_sensors.calibration.rgb_event.solve [folder] [--no-install]`,
`python -m cvlab_sensors.calibration.rgb_livox.solve [folder] [--no-install]`,
`python -m cvlab_sensors.calibration.livox_hesai.solve [folder] [--initial X Y Z] [--no-install]`.

RGB images are center-cropped to 16:9 and resized to 1280x720 before calibration
(`cvlab_sensors.calibration.common.to_calibration_size`): do the same before using the intrinsics, or use
`cvlab_sensors.calibration.common.full_frame_intrinsics` for the full frame.

## Time synchronization (PTP)
Every Ethernet sensor sits on its own USB network adapter (r8152), which has no
hardware timestamping. A single `ptp4l` with software timestamping serves the
NUC system clock to the Basler cameras and the Livox (accuracy: tens of microseconds),
installed as a service (starts at boot):
```
sudo cp system/ptp4l-cvlab.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now ptp4l-cvlab
journalctl -u ptp4l-cvlab -f
```
- **Basler**: the RGB recorder refuses to record until both cameras are PTP `Slave` + `Locked`.
  Frames are triggered by the PTP clock (`RGB_PTP_TRIGGER`), so left and right expose at the same
  instant (measured: < 0.2 ms apart).
- **Livox HAP**: needs PTP over L2 (set in `system/ptp4l.conf`); each H5 file stores `time_source`
  (`livox_ptp` when synced).
- Check everything with `python -m cvlab_sensors.tools.check_ptp`.
- **Hesai XT32**: keep its clock source on GPS (free running). It cannot follow a two-step PTP
  master (software timestamping is always two-step): its clock jumps by seconds. Its H5 files
  store `sync` pairs, and `cvlab_sensors.sensors.lidar.LidarChunk.host_timestamps_ns()` maps it to host time.
- **Event cameras** (USB) are not on PTP: their clock is mapped to the host clock through the
  `*.sync.csv` file written next to each recording. For exact RGB-event sync, wire a Basler output
  line (LineSource = ExposureActive) into the EVK4 trigger input and turn on "Trigger input" on the
  Settings page.

## SDKs
### Lidar recorders
`lidar_recorders/compile.sh` links the sources into the SDK trees (`~/SDKs/Livox-SDK2`,
`~/SDKs/HesaiLidar_SDK_2.0`) and builds `save_point_cloud_livox` and `save_point_cloud_hesai`;
`lidar_recorders/save_point_cloud_*` are symlinks to the built programs.

Livox SDK2 installation:
```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j && sudo make install
```
Documentation about lidar packets [here](https://github.com/Livox-SDK/Livox-SDK2/wiki/Livox-SDK-Communication-Protocol-HAP(English)#2-point-cloud--imu-data-protocol).

### Event cameras
- [SDK 5.0 deb](https://propheseeai.jfrog.io/ui/repos/tree/General/metavision-sdk-5-debian)
- [Metavision Studio](https://propheseeai.jfrog.io/ui/packages/deb:%2F%2Fmetavision-studio)
- [Creating a Python Venv](https://docs.prophesee.ai/stable/installation/linux.html#installing-dependencies)

### RGB cameras
Change the IP using `/opt/pylon/bin/ipconfigurator`, inspect the camera settings using
`/opt/pylon/bin/pylonviewer` (the app sets them itself, see `cvlab_sensors/sensors/basler.py`).

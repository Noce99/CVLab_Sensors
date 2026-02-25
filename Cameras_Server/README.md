## System Overview

The server provides:
- Real-time web-based monitoring dashboard
- Automatic frame capture and storage with microsecond timestamps
- Support for 6 sensors across 3 sensor types
- Thread-safe concurrent acquisition
- Graceful fallback to simulation mode when hardware unavailable

## Supported Sensors

| Sensor Type | Models | Quantity | Frame Rate | Interface |
|-------------|--------|----------|------------|-----------|
| RGB Stereo | Basler (pypylon) | 2 | 25-30 FPS | USB3 |
| Event Stereo | Prophesee (Metavision SDK) | 2 | 10-15 FPS | USB3 |
| LiDAR | Livox | 1 | 1 FPS | Ethernet |
| LiDAR | Hesai | 1 | 20 FPS | Ethernet |

## Quick Start

### Installation

```bash
# Activate virtual environment
source ~/venv/bin/activate

# Install required packages
pip install flask flask-cors opencv-python numpy pypylon

# Start server
python camera_server.py
```

### Access

- Local: `http://localhost:5000`
- Remote: `http://<server_ip>:5000`

To find server IP:
```bash
hostname -I
```

## Data Storage

### Directory Structure

All frames are automatically saved with microsecond Unix timestamps:

```
./live_imgs/
├── rgb_left/
│   ├── 1708445844123456.png
│   ├── 1708445844156789.png
│   └── ...
├── rgb_right/
├── event_left/
├── event_right/
├── lidar_livox/
└── lidar_hesai/
```


## Configuration

### RGB Cameras (Basler)

Pre-configured with serial numbers:
- Left: `40663113`
- Right: `40665216`

**Verification:**
```bash
python test_rgb_cameras.py
```

If cameras not detected, update serial numbers in `camera_server.py` lines 33-34.

### Event Cameras (Prophesee)

Requires:
1. Metavision SDK installed
2. Acquisition script accessible

**Setup:**
```bash
# Option A: Create symbolic link
ln -s /path/to/save_images_no_sign_distintion.py ./

# Option B: Update path in camera_server.py 
```


### LiDAR Sensors

Requires:
1. Compiled C++ acquisition programs
2. Programs accessible to server

**Livox Setup:**
```bash
ln -s /path/to/save_intensity ./save_intensity
```

**Hesai Setup:**
```bash
ln -s /path/to/hesai_program ./hesai_lidar_example
```

## Usage

### Starting Sensors

1. Launch server: `python camera_server.py`
2. Open dashboard: `http://localhost:5000`
3. Click "Start" button for desired sensors
4. Monitor real-time frames and FPS
5. Frames automatically saved to `./live_imgs/`

### Dashboard Controls

**Per-Sensor Controls:**
- Start: Begin acquisition for sensor pair
- Stop: Halt acquisition
- Refresh: Manually update frame display

**Global Controls:**
- Refresh All: Update all frame displays
- Start All: Begin all sensor acquisitions
- Stop All: Halt all acquisitions

### Status Indicators

- Green: Sensor active and streaming
- Red: Sensor inactive
- FPS Counter: Current frame rate
- Timestamp: Last frame update time

## Performance Specifications

### Frame Rates

| Sensor | Target FPS | Actual FPS | Notes |
|--------|-----------|------------|-------|
| RGB | 25-30 | 25-30 | Hardware synchronized via PTP |
| Event | 10-15 | 10-15 | Event accumulation dependent |
| Livox | 1 | 1 | As configured in C++ program |
| Hesai | 20 | 20 | As configured in C++ program |


## Maintenance

**Remove all frames:**
```bash
rm -rf ./live_imgs/*
```

### Temporary Directories

The following directories are used for frame buffering:
- `./temp_event_frames/` - Event camera buffer
- `./temp_livox_frames/` - Livox LiDAR buffer
- `./temp_hesai_frames/` - Hesai LiDAR buffer

These directories have automatic cleanup (maintain last 5 files) and do not require manual maintenance.


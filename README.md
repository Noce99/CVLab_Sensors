# CVLab Sensors

| Computers | ssh                    | Password |
| --------  | -------                | -------   |
| Orin      | `ssh cvlab@192.168.1.49` | cvlabsensors |
| Nuc       | `ssh ubm@192.168.1.50`   | ubm12345 |

| Sensor Type | Sensor Name | Ros2 Node | Connection | IP | Instructions |
| --------    | -------     | -------   | ------- | ------- | ------- |
| Lidar  | OT128  | [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) | Ethernet | 192.168.1.90 |[build](#hesai_lidar_build) - [run](#hesai_lidar_run)|
| Lidar  | XT32  | [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) | Ethernet | 192.168.1.91 |[build](#hesai_lidar_build) - [run](#hesai_lidar_run)|
| Lidar  | Livox Hap  | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Ethernet | 192.168.1.100 | [build](#livox_lidar_build) - [run](#livox_lidar_run)|
| Lidar  | Livox Mid360  | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Ethernet | 192.168.1.151 |[build](#livox_lidar_build) - [run](#livox_lidar_run)|
| Event Camera  | EVKV4 - HD | [official](https://github.com/prophesee-ai/prophesee_ros_wrapper) / [alternative](https://github.com/ros-event-camera/metavision_driver) | USB | | [build](#eventcamera_build) - [run](eventcamera_run) |
| Depth Camera  | RealSense | [realsense-ros](https://github.com/IntelRealSense/realsense-ros) | USB | | [build](#realsense_build) - [run](#realsense_run) |
| RGB Camera | Basler | [pylon-ros-camera](https://github.com/basler/pylon-ros-camera) | Ethernet |192.168.1.70-77 | [build](#basler_build) - [run](#basler_run) |

## Start VNC
From the orin:
```
sudo systemctl restart vnc.service
```
From your computer with a VNC client connect to:
```
192.168.1.84:5900
```
## View Data
From your computer:
```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml send_buffer_limit:=10000000000
```

## HesaiLidar_ROS_2.0
<a name="hesai_lidar_build"></a>
### Build
From `/home/cvlab/ros_ws`
```
colcon build --symlink-install --packages-select hesai_ros_driver
```
<a name="hesai_lidar_run"></a>
### Run
```
source install/setup.bash
```
```
ros2 launch hesai_ros_driver start.py
```

## Livox Lidars
<a name="livox_lidar_build"></a>
### Build
#### SDK Installation
From `/home/cvlab/CVLab_Sensors`
```
git clone https://github.com/Livox-SDK/Livox-SDK2.git
```
```
cd ./Livox-SDK2/
```
```
mkdir build
```
```
cd build
```
```
cmake .. && make -j
```
```
sudo make install
```
#### Ros 2 Node Installation
From `/home/cvlab/CVLab_Sensors`
```
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```
```
cd livox_ros_driver2
```
```
cp -f package_ROS2.xml package.xml
```
```
cp -rf launch_ROS2/ launch/
```
Set receiving ip and lidar ip in:
```
/home/cvlab/ros_ws/src/livox_ros_driver2/config/MID360_config.json
/home/cvlab/ros_ws/src/livox_ros_driver2/config/HAP_config.json
```
From `/home/cvlab/ros_ws`
```
colcon build --symlink-install --packages-select livox_ros_driver2
```
### Run
<a name="livox_lidar_run"></a>
MID360:
```
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
HAP:
```
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```
with cpp sdk:
```
/home/ubm/CVLab_Sensors/Livox-SDK2/build/samples/livox_lidar_quick_start/livox_lidar_quick_start /home/ubm/CVLab_Sensors/Livox-SDK2/samples/livox_lidar_quick_start/hap_noce.json
```
## RealSensor
<a name="realsense_build"></a>
### Build
#### SDK Installation
```
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt-get install librealsense2-utils
sudo apt-get install librealsense2-dev
```
You can try the correct installation with: `realsense-viewer`
#### Ros 2 Node Installation
From `/home/cvlab/CVLab_Sensors`
```
sudo apt install ros-foxy-realsense2-*
```
<a name="realsense_run"></a>
### Run
```
ros2 launch realsense2_camera rs_launch.py
```
## Event Camera
<a name="eventcamera_build"></a>
### Build
From `/home/cvlab/ros_ws`
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --packages-select event_camera_codecs event_camera_renderer metavision_driver event_camera_msgs
```
<a name="eventcamera_run"></a>
### Run
```
ros2 launch metavision_driver driver_node.launch.py
```
To visualise:
```
ros2 launch event_camera_renderer renderer.launch.py
```
Lunch `rqt`
Plugins > Visualization > Image View
## RGB Camera
Change the ip using:
```
/opt/pylon/bin/ipconfigurator
```
Change camera settings using:
```
/opt/pylon/bin/pylonviewer
```
or using ros:
SET RGB (other values: `mono8`, `mono16`, `bgr8`, `rgb8`, `bayer_bggr8`, `bayer_gbrg8`, `bayer_rggb8`, `bayer_grbg8`, `bayer_rggb16`, `bayer_bggr16`, `bayer_gbrg16`, `bayer_grbg16`):
```
ros2 service call /my_camera/pylon_ros2_camera_node/set_image_encoding pylon_ros2_camera_interfaces/srv/SetStringValue "{value: rgb8}"
```
Set Brightness:
```
ros2 service call /my_camera/pylon_ros2_camera_node/set_brightness pylon_ros2_camera_interfaces/srv/SetBrightness "{target_brightness: 200, brightness_continuous: true, exposure_auto: true, gain_auto: true}"
```
<a name="basler_build"></a>
### Build
From `/home/cvlab/ros_ws`
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DPYLON_ROOT=/opt/pylon --packages-select pylon_ros2_camera_component pylon_ros2_camera_interfaces pylon_ros2_camera_wrapper
```
<a name="basler_run"></a>
### Run
```
ros2 launch pylon_ros2_camera_wrapper pylon_ros2_camera.launch.py
```

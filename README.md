# CVLab Sensors
All the ros2 nodes to receive data from all the CVLab sensors.
| Sensor Type | Sensor Name | Ros2 Node | Connection | IP | Instructions |
| --------    | -------     | -------   | ------- | ------- | ------- |
| Lidar  | PandarSX  | [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) | Ethernet | 192.168.1.201 |[build](#hesai_lidar_build) - [run](#hesai_lidar_run)|
| Lidar  | Livox Hap  | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Ethernet | 192.168.1.100 | [build](#livox_lidar_build) - [run](#livox_lidar_run)|
| Lidar  | Livox Mid360  | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Ethernet | 192.168.1.151 |[build](#livox_lidar_build) - [run](#livox_lidar_run)|
| Event Camera  | EVKV4 - HD | [official](https://github.com/prophesee-ai/prophesee_ros_wrapper) / [alternative](https://github.com/ros-event-camera/metavision_driver) | USB | | |
| Depth Camera  | RealSense | [realsense-ros](https://github.com/IntelRealSense/realsense-ros) | USB | | [build](#realsense_build) - [run](#realsense_run) |

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
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
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

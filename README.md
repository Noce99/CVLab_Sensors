# CVLab Sensors
All the ros2 nodes to receive data from all the CVLab sensors.
| Sensor Type | Sensor Name | Ros2 Node | IP |
| --------    | -------     | -------   | ------- |
| Lidar  | PandarSX  | [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) | 192.168.1.201 |
| Lidar  | Livox Hap  | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | |
| Lidar  | Livox Mid360  | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | 192.168.1.151 |
|   |   | []() |  |

## View Data
From your computer:
```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## HesaiLidar_ROS_2.0
### Build
From `/home/cvlab/ros_ws`
```
colcon build --symlink-install --packages-select hesai_ros_driver
```
### Run
```
source install/setup.bash
```
```
ros2 launch hesai_ros_driver start.py
```

## Livox Lidars
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
MID360:
```
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```
HAP:
```
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
```

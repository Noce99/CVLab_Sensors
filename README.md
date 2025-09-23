# CVLab Sensors
All the ros2 nodes to receive data from all the CVLab sensors.
| Sensor Type | Sensor Name | Ros2 Node | IP |
| --------    | -------     | -------   | ------- |
| Lidar  | PandarSX  | [HesaiLidar_ROS_2.0](https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0) | 192.168.1.201 |
| Lidar  | Livox Hap  | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | |
| Lidar  | Livox Mid360  | [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | 192.168.1.151 |
|   |   | []() |  |

## Vuiew Data
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

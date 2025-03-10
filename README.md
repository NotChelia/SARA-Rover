# SARA Rover (Search and Rescue Autonomous Rover)

![](https://github.com/NotChelia/SARA-Rover/blob/master/demo.gif)

This repository contains a complete system for LiDAR-based exploration and mapping, built on ROS Noetic and ROS Foxy using the following:
## Components

### 1. catkin_point_lio_unilidar
A LiDAR-inertial odometry system based on Point-LIO, adapted for use with Unitree LiDAR sensors. This provides accurate pose estimation and mapping capabilities.

### 2. unilidar_sdk
The official SDK for Unitree LiDAR sensors, including:
- Core SDK libraries
- ROS1 integration
- ROS2 integration

### 3. exploration_ws
Custom autonomous exploration implementation, including:
- Path planning algorithms (Dubins paths, Reeds-Shepp curves, cubic splines)
- Navigation integration with Nav2
- Exploration node for autonomous mapping

### Usage

The repository includes several shell scripts to simplify operation:
- `run_exploration_system_new.sh`: Launches the complete exploration system
- `check_exploration_topics.sh` and `check_topics.sh`: Utility scripts for debugging

### Prerequisites
- Ubuntu 20.04
- ROS Noetic (```sudo apt-get install ros-noetic-desktop-full```)
- ROS2 Foxy (```sudo apt-get install ros-foxy-desktop```)
- PCL (```sudo apt-get install ros-noetic-pcl-conversions```)
- Eigen (```sudo apt-get install libeigen3-dev```)
- ROS1-ROS2 Bridge (```sudo apt-get install ros-foxy-ros1-bridge```)

### Building
```
#build unitree lidar sdk and ros2 driver
cd unilidar_sdk/unitree_lidar_sdk
mkdir -p build && cd build
cmake ..
make -j4

cd ../../unitree_lidar_ros2
colcon build
```
```
#Build Point LIO
cd catkin_point_lio_unilidar
catkin_make
```
```
# Build Explorer
cd exploration_ws
colcon build --symlink-install --packages-select autonomous_explorer
```


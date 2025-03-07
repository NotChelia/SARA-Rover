# Lidar Exploration System

This repository contains a complete system for LiDAR-based exploration and mapping, built on ROS. It integrates several key components:

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

## Usage

The repository includes several shell scripts to simplify operation:
- `run_exploration_system_new.sh`: Launches the complete exploration system
- `robust_bridge.sh`: Provides communication between different ROS components
- `check_exploration_topics.sh` and `check_topics.sh`: Utility scripts for debugging

## Setup

Please refer to the individual component READMEs for detailed setup instructions.

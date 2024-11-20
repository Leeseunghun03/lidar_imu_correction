# ROS IMU-Based LiDAR Point Cloud Tilt Compensation

## Overview
This ROS 1 package compensates for tilt in LiDAR point clouds using IMU sensor data.

---

## Features
- Real-time tilt correction for LiDAR point clouds using IMU data.
- Subscribes to IMU and LiDAR point cloud topics.
- Publishes tilt-corrected point clouds for downstream navigation and mapping.

---

## Requirements
- ROS 1 (tested with Noetic)
- IMU sensor publishing `/imu` topic (sensor_msgs/Imu)
- LiDAR sensor publishing `/scan` or `/points` topic (sensor_msgs/PointCloud2)

---

## Installation
Clone the repository into your ROS 1 workspace and build:
```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/imu_lidar_tilt_compensation.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash

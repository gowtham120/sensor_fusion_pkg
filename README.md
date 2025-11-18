# sensor_fusion_pkg

[![License](https://img.shields.io/badge/license-Apache%202.0-blue.svg)](LICENSE)

A ROS2 package for fusing IMU and depth sensor data to estimate vertical velocity and publish it as a ROS2 topic at 200Hz.

---

# 1 Features

- Subscribes to `/imu/data` (IMU) and `/depth` (depth sensor) topics.
- Estimates vertical velocity using:
  - IMU linear acceleration integration
  - Depth change over time
- Fuses the velocities and publishes `/vertical_velocity`.
- Simple and modular ROS2 node structure.

---

# 2 System Requirements

- ROS2 Humble Hawksbill
- Python 3.10
- `sensor_msgs`, `std_msgs`
- `colcon` build system

---

# 3-a Source Installation Setup

Clone the package into your ROS2 workspace:

```bash
cd ~/oo_ws/src
git clone https://github.com/gowtham120/sensor_fusion_pkg.git
cd ~/oo_ws
colcon build --symlink-install
source install/setup.bash
```
# 3-b Docker Setup

This guide explains how to run the ROS2 `sensor_fusion_pkg` inside a Docker container.

---

### Prerequisites

- Install Docker on your system: [Install Docker](https://docs.docker.com/get-docker/)

---

## Steps to Run Using Docker

### a) Download the Dockerfile

Clone the repository or download the Dockerfile to a local folder:

```bash
git clone https://github.com/gowtham120/sensor_fusion_pkg.git
cd sensor_fusion_pkg
```

### b) Build the Docker Image

Docker build creates a new Docker image from the Dockerfile in the current directory.

```bash
sudo docker build -t sensor_fusion_pkg_img .
```

### c) Run the Docker Container

docker run starts a new container from the image.

```bash
sudo docker run -it --name sensor_fusion_container sensor_fusion_pkg_img
```
### 5) To stop the container

```bash
sudo docker stop sensor_fusion_container
```
---

# Launch the sensor fusion node using Launch file

```bash
ros2 launch sensor_fusion_pkg fuse_data.launch.py
```
# Publish `/imu/data` data at 200 Hz

The IMU measures proper acceleration which includes gravity. assuming the sensor’s +Z axis points upward a stationary IMU reads approximately +9.81 m/s² on the Z-axis.

To Experience change velocty pub above or below +9.8 m/s²
```bash
ros2 topic pub /imu_data sensor_msgs/msg/Imu "header:
  stamp: now
  frame_id: 'base_link'
linear_acceleration:
  x: 0.0
  y: 0.0
  z: 9.81  # adjust here for change in vel
angular_velocity:
  x: 0.0
  y: 0.0
  z: 0.0
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0" -r 200
```
# Publish `/depth` at 20 Hz
```bash
ros2 topic pub /depth std_msgs/msg/Float32 "{data: 2.5}" -r 200
```
# To view fused velocity echo `/vertical_velocity`
```bash
ros2 topic echo /vertical_velocity
```






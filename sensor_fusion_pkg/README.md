# sensor_fusion_pkg

A ROS2 package that subscribes to IMU and depth data, estimates vertical velocity from depth changes, and publishes the result.

---

## Build Instructions

```bash
cd ~/oo_ws
colcon build --symlink-install
source install/local_setup.bash

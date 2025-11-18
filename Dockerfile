# base image: Ros2 humble on Ubuntu 22.04
FROM ros:humble

# Installing dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create orpheus ocean workspace
WORKDIR /oo_ws
RUN mkdir -p src

# cloning sensor fusion package from github
RUN git clone https://github.com/gowtham120/sensor_fusion_pkg.git src/sensor_fusion_pkg

# Source the ROS2 underlay and build package
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /oo_ws && \
    colcon build --symlink-install --parallel-workers 1"

# Source workspace overlay on container start
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /oo_ws/install/setup.bash" >> ~/.bashrc


ENTRYPOINT ["/bin/bash"]




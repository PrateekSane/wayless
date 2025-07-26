# 0) Base: ROS2 Rolling on Ubuntu/arm64 or x86_64
FROM ros:rolling-ros-base

# 1) Install system tooling + ROS2 core packages
RUN apt-get update && apt-get install -y \
      wget git build-essential \
      python3-colcon-common-extensions \
      python3-rosdep \
      ros-rolling-ros-bridge-server \
      ros-rolling-image-transport \
      ros-rolling-cv-bridge \
      ros-rolling-rosbag2 \
      ros-rolling-rosbag2-transport \
    && rm -rf /var/lib/apt/lists/*

# 2) Install Basler Pylon SDK (Linux .deb)
#    👉 Grab the real .deb link from Basler’s site under “Linux → Debian Installer”
RUN wget https://www.baslerweb.com/en/downloads/software/3520605482/ \
      -O /tmp/pylon.deb && \
    dpkg -i /tmp/pylon.deb || true && \
    apt-get update && apt-get install -y -f && \
    rm /tmp/pylon.deb

# 3) Initialize rosdep for source builds
RUN rosdep init || true && rosdep update

# 4) Create a ROS2 workspace
RUN mkdir -p /root/ros2_ws/src

# 5) Clone the ROS2 drivers
RUN cd /root/ros2_ws/src && \
    git clone https://github.com/basler/pylon-ros-camera.git && \    # Basler ROS2 driver :contentReference[oaicite:0]{index=0}
    git clone https://github.com/ros-drivers/velodyne.git             # Velodyne ROS2 driver & pointcloud :contentReference[oaicite:1]{index=1}

# 6) Install source dependencies & build everything
RUN /bin/bash -c "\
    source /opt/ros/rolling/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro rolling -y && \
    colcon build --symlink-install \
  "

# 7) Auto‑source on shell start
RUN echo 'source /opt/ros/rolling/setup.bash' >> /root/.bashrc && \
    echo 'source /root/ros2_ws/install/setup.bash' >> /root/.bashrc

CMD ["bash"]

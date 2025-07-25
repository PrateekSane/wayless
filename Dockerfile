FROM ros:noetic-perception

# 1) Update package lists and install basic dependencies first
RUN apt-get update && apt-get install -y \
    wget \
    lsb-release \
    gnupg2 \
    git \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
  && rm -rf /var/lib/apt/lists/*

# 2) Initialize rosdep (this needs to happen early)
RUN rosdep init || true && rosdep update

# 3) Install ROS packages and build dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-velodyne \
    ros-noetic-velodyne-pointcloud \
    ros-noetic-rosbag \
    ros-noetic-rosbridge-server \
    ros-noetic-pcl-ros \
    ros-noetic-sensor-msgs \
    ros-noetic-roslint \
    ros-noetic-camera-info-manager \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-diagnostic-updater \
    ros-noetic-actionlib \
    libopencv-dev \
    cmake \
    pkg-config \
  && rm -rf /var/lib/apt/lists/*

# 4) Install Basler pylon SDK from local tar file
COPY pylon.tar.gz /tmp/
RUN cd /tmp && \
    tar -xzf pylon.tar.gz && \
    cd pylon_*_linux_x86_64 && \
    tar -C /opt -xzf pylonSDK*.tar.gz && \
    rm -rf /tmp/pylon*

# 4a) Verify Pylon installation and set up proper paths
RUN echo "=== Pylon Installation Check ===" && \
    find /opt -name "*pylon*" -type d 2>/dev/null && \
    find /opt -name "PylonIncludes.h" 2>/dev/null && \
    ls -la /opt/ && \
    # Create symlink to the actual pylon installation
    PYLON_DIR=$(find /opt -name "*pylon*" -type d | head -1) && \
    if [ -n "$PYLON_DIR" ]; then \
        echo "Found Pylon at: $PYLON_DIR" && \
        ln -sf "$PYLON_DIR" /opt/pylon && \
        ls -la /opt/pylon/include/ || echo "No include directory found"; \
    else \
        echo "ERROR: Pylon installation not found!"; \
        exit 1; \
    fi

# 5) Create catkin workspace and clone repositories first
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws/src && \
    git clone https://github.com/magazino/pylon_camera.git && \
    git clone https://github.com/magazino/camera_control_msgs.git

# 6) Create custom rosdep rule for pylon
RUN mkdir -p /etc/ros/rosdep/sources.list.d && \
    echo "pylon:" > /etc/ros/rosdep/sources.list.d/50-pylon.yaml && \
    echo "  ubuntu:" >> /etc/ros/rosdep/sources.list.d/50-pylon.yaml && \
    echo "    focal: [pylon]" >> /etc/ros/rosdep/sources.list.d/50-pylon.yaml && \
    echo "    bionic: [pylon]" >> /etc/ros/rosdep/sources.list.d/50-pylon.yaml && \
    echo "    xenial: [pylon]" >> /etc/ros/rosdep/sources.list.d/50-pylon.yaml

# 7) Add the magazino pylon_sdk rosdep rule as well
RUN echo "yaml https://raw.githubusercontent.com/magazino/pylon_camera/indigo-devel/rosdep/pylon_sdk.yaml" \
      > /etc/ros/rosdep/sources.list.d/15-pylon_sdk.yaml

# 8) Update rosdep with new rules
RUN rosdep update

# 9) Install dependencies manually (skip problematic packages)
RUN /bin/bash -c "cd /root/catkin_ws && \
    source /opt/ros/noetic/setup.bash && \
    rosdep install --from-paths src --ignore-src --rosdistro noetic -y --skip-keys='pylon pylon_sdk' || true"

# 10) Build the workspace with dynamically found Pylon paths
RUN /bin/bash -c "cd /root/catkin_ws && \
    source /opt/ros/noetic/setup.bash && \
    PYLON_DIR=\$(readlink -f /opt/pylon) && \
    echo \"Using Pylon directory: \$PYLON_DIR\" && \
    export PYLON_ROOT=\$PYLON_DIR && \
    export CMAKE_PREFIX_PATH=\$PYLON_DIR:\$CMAKE_PREFIX_PATH && \
    export CMAKE_INCLUDE_PATH=\$PYLON_DIR/include:\$CMAKE_INCLUDE_PATH && \
    export CMAKE_LIBRARY_PATH=\$PYLON_DIR/lib64:\$PYLON_DIR/lib:\$CMAKE_LIBRARY_PATH && \
    export LD_LIBRARY_PATH=\$PYLON_DIR/lib64:\$PYLON_DIR/lib:\$LD_LIBRARY_PATH && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -j2 \
      -DCMAKE_CXX_FLAGS=\"-I\$PYLON_DIR/include\" \
      -DCMAKE_EXE_LINKER_FLAGS=\"-L\$PYLON_DIR/lib64 -L\$PYLON_DIR/lib\""

# 11) Set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# 12) Copy in your existing launch scripts
COPY ros_launch /root/wayless/ros_launch
RUN chmod +x /root/wayless/ros_launch/*.sh

# 13) Set environment variables for pylon
ENV PYLON_ROOT=/opt/pylon

# 14) Default to bash
CMD ["bash"]
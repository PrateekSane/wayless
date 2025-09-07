# Use the full ROS image that definitely has everything
FROM ros:noetic
ENV DEBIAN_FRONTEND=noninteractive

# Install additional tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    vim nano \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-diagnostic-updater \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-image-geometry \
    ros-noetic-roslint \
    && rm -rf /var/lib/apt/lists/*

# Install Pylon SDK
COPY pylon.tar.gz /tmp/
RUN cd /tmp && \
    tar -xzf pylon.tar.gz && \
    tar -xzf pylon-*.tar.gz && \
    mkdir -p /opt/pylon && \
    cp -r include lib /opt/pylon/ && \
    echo "/opt/pylon/lib" > /etc/ld.so.conf.d/pylon.conf && \
    ldconfig && \
    rm -rf /tmp/pylon*

# Setup workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Clone repos
RUN git clone https://github.com/magazino/pylon_camera.git && \
    git clone https://github.com/magazino/camera_control_msgs.git

WORKDIR /catkin_ws

# The ros:noetic image has an entrypoint that sources ROS
CMD ["/bin/bash"]
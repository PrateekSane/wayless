FROM --platform=linux/amd64 ros:noetic-ros-core-focal

# Install system dependencies
RUN apt-get update && apt-get install -y \
    wget \
    git \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    cmake \
    python3-catkin-tools \
    python3-osrf-pycommon \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Install additional ROS packages
RUN apt-get update && apt-get install -y \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-camera-info-manager \
    ros-noetic-diagnostic-updater \
    ros-noetic-rosbag \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Install Aravis build dependencies
RUN apt-get update && apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libxml2-dev \
    libglib2.0-dev \
    libusb-1.0-0-dev \
    pkg-config \
    meson \
    ninja-build \
    libgtk-3-dev \
    gobject-introspection \
    libgirepository1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Build and install Aravis from source
WORKDIR /tmp
RUN git clone https://github.com/AravisProject/aravis.git && \
    cd aravis && \
    git checkout 0.6.4 && \
    meson setup build && \
    meson compile -C build && \
    meson install -C build && \
    ldconfig && \
    cd / && \
    rm -rf /tmp/aravis

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Initialize rosdep
RUN rosdep init && rosdep update

# Clone camera_aravis ROS driver
WORKDIR /catkin_ws/src
RUN git clone https://github.com/ipa-bnm/camera_aravis.git

# Install dependencies
WORKDIR /catkin_ws
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Create launch directory and copy launch files
RUN mkdir -p /catkin_ws/launch

# Set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /catkin_ws

# Default command
CMD ["/bin/bash"]
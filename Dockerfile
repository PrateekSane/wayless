# Dockerfile

FROM ros:noetic-ros-base

# Install rosbag tools
RUN apt-get update && apt-get install -y python3-rosbag

# Set working directory
WORKDIR /bags

# Default command: open a shell
CMD ["/bin/bash"]


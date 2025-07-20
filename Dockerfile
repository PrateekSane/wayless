# Dockerfile

FROM ros:noetic-perception

# 1) Install Velodyne drivers, rosbag, and rosbridge
RUN apt-get update && apt-get install -y \
      ros-noetic-velodyne \
      ros-noetic-velodyne-pointcloud \
      ros-noetic-rosbag \
      ros-noetic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# 2) Auto‑source ROS on shell start
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# 3) (Optional) Copy in your project with record scripts
#    Assumes you have a ros_launch/ directory next to this Dockerfile
COPY ros_launch /root/wayless/ros_launch
RUN chmod +x /root/wayless/ros_launch/*.sh

# 4) Default to bash so the container stays alive
CMD ["bash"]

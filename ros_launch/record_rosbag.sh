#!/bin/bash
# ~/wayless/ros_launch/record_rosbag.sh

# 1) load ROS environment
source /opt/ros/noetic/setup.bash

# 2) wait until the driver is actually publishing scans
until rostopic list | grep -q /velodyne_points; do
  sleep 0.5
done

# 3) make sure your bag directory exists
mkdir -p "${HOME}/bags"

# 4) start recording, new file every 10 s
rosbag record /velodyne_points \
    --split --duration=1 \
    -O "${HOME}/bags/velodyne" 

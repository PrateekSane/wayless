#!/bin/bash
# ~/wayless/ros_launch/record_rosbag.sh

# 1) Load ROS environment
source /opt/ros/noetic/setup.bash

# 2) Wait until the Velodyne driver is actually publishing scans
until rostopic list | grep -q /velodyne_points; do
  sleep 0.5
done

# 3) Build a timestamped session directory: mm-dd-HH-MM
BASE_DIR="${HOME}/bags"
TIMESTAMP=$(date +'%m-%d-%H-%M')
SESSION_DIR="${BASE_DIR}/${TIMESTAMP}"

mkdir -p "${SESSION_DIR}"

# 4) Start recording (splitting every 10 s) into that folder
rosbag record /velodyne_points /tf /tf_static /clock \
    --split --duration=10s \
    -O "${SESSION_DIR}/velo" 2>&1 | tee "${SESSION_DIR}/rosbag.log" 

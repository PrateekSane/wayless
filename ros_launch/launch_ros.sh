#!/usr/bin/env bash
set -e

# 1) source global ROS
source /opt/ros/noetic/setup.bash

# 2) source your workspace
source ~/catkin_ws/devel/setup.bash

# 3) optional: cd into your workspace if you need to
cd ~/catkin_ws

# 4) launch
exec roslaunch velodyne_pointcloud VLP16_points.launch \
  2>&1 | tee "${HOME}/roslaunch.log"

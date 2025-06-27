#!/bin/bash
# 1) load your ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 2) if roscore isn’t already running, start it in the background
if ! pgrep -x roscore > /dev/null; then
  roscore > /home/pi/roscore.log 2>&1 &
  sleep 2    # give it a moment to spin up
fi

# 3) replace this shell with your Velodyne launch
exec roslaunch velodyne_pointcloud VLP16_points.launch

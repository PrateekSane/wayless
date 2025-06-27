#!/bin/bash
# 1) load your ROS environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

if pgrep -x roscore > /dev/null; then
  echo "roscore is already up. Don't need to start"
fi


# 3) replace this shell with your Velodyne launch
exec roslaunch velodyne_pointcloud VLP16_points.launch

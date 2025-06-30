##!/bin/bash

source /opt/ros/noetic/setup.bash

# 2) hand off to roslaunch (it will also start roscore)
exec roslaunch velodyne_pointcloud VLP16_points.launch \
    > "${HOME}/roslaunch.log" 2>&1

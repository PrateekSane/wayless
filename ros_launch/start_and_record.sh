#!/bin/bash
# ~/wayless/ros_launch/start_ros_and_record.sh

# 1) Load ROS environment
source /opt/ros/noetic/setup.bash
# 2) If you rebuilt drivers in a catkin overlay, source that too:
source /home/prateeksane/catkin_ws/install_isolated/setup.bash

# 3) Launch the LiDAR driver in the background
/home/prateeksane/wayless/ros_launch/launch_ros.sh &
DRIVER_PID=$!

# 4) Ensure the driver is killed when this script exits
trap "kill ${DRIVER_PID}" EXIT

# 5) Start the recorder (this will block until you stop it)
/home/prateeksane/wayless/ros_launch/record_rosbag.sh

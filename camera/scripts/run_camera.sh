#!/bin/bash

# Script to run Basler camera with ROS bag recording

# Set timestamp for unique bag file names
export TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

echo "Starting Basler camera with ROS bag recording..."
echo "Timestamp: $TIMESTAMP"

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Create bags directory if it doesn't exist
mkdir -p /catkin_ws/bags

# Check if camera is connected
echo "Checking for GigE Vision cameras with Aravis..."
if command -v arv-tool-0.8 >/dev/null 2>&1; then
    arv-tool-0.8 device-list
elif command -v arv-tool >/dev/null 2>&1; then
    arv-tool device-list
else
    echo "Aravis tools not found, trying to list GigE devices..."
    echo "You can manually check if your camera is accessible at 169.254.x.x"
fi

echo "Starting camera node and recording..."
echo "Recording will be saved to: /catkin_ws/bags/camera_${TIMESTAMP}.bag"

# Start the camera and recording
roslaunch /catkin_ws/launch/basler_camera.launch &

# Wait a moment for camera to initialize
sleep 3

# Start recording
rosbag record -O /catkin_ws/bags/camera_${TIMESTAMP}.bag \
    /aravis_camera_node/image_raw \
    /aravis_camera_node/camera_info \
    /tf \
    /tf_static &

echo "Camera and recording started. Press Ctrl+C to stop."
echo "To view the camera feed, run: rosrun image_view image_view image:=/aravis_camera_node/image_raw"

# Keep script running
wait
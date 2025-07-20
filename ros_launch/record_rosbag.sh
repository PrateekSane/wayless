#!/bin/bash
# ~/wayless/ros_launch/record_rosbag.sh

set -e

# 1) Load ROS environment
echo "[*] Sourcing ROS environment..."
source /opt/ros/noetic/setup.bash

# 2) When we exit for any reason, kill the background TF publisher if it’s still alive
cleanup(){
  if [[ -n "$TF_PID" ]] && kill -0 "$TF_PID" &>/dev/null; then
    echo "[*] Shutting down static transform publisher (pid $TF_PID)..."
    kill "$TF_PID"
  fi
}
trap cleanup EXIT INT TERM

# 3) Wait until the Velodyne driver is up
echo "[*] Waiting for /velodyne_points topic to appear..."
until rostopic list | grep -q /velodyne_points; do
  sleep 0.5
done
echo "[*] Detected /velodyne_points."

# 4) Publish a repeating static transform into /tf
echo "[*] Starting static transform publisher (base_link→velodyne)..."
rosrun tf static_transform_publisher \
  0 0 1 0 0 0 base_link velodyne 100 \
  &> /dev/null &
TF_PID=$!
echo "[*] static_transform_publisher running with pid $TF_PID."

# 5) Build a timestamped session directory
BASE_DIR="${HOME}/wayless/bags"
TIMESTAMP=$(date +'%m-%d-%H-%M')
SESSION_DIR="${BASE_DIR}/${TIMESTAMP}"
echo "[*] Creating session directory: ${SESSION_DIR}"
mkdir -p "${SESSION_DIR}"

# 6) Record LiDAR + live TF + clock
echo "[*] Recording /velodyne_points, /tf, /clock → ${SESSION_DIR}/velo_*.bag"
rosbag record \
    /velodyne_points \
    /tf \
    /clock \
  --split --duration=10s \
  -O "${SESSION_DIR}/velo" \
  2>&1 | tee "${SESSION_DIR}/rosbag.log"

# (when rosbag record finishes or you hit Ctrl-C, the EXIT trap will fire and kill $TF_PID)


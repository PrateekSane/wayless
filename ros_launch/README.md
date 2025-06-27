## Setup for LiDAR launching

ros_all.service needs to be in /etc/systemd/system/ros_all.service

### First time 
sudo chmod +x /home/pi/start_ros.sh
sudo systemctl daemon-reload
sudo systemctl enable ros_all.service
sudo reboot

### Check status:

systemctl status ros_all.service

### View logs:

journalctl -u ros_all.service -f
tail -n 20 /home/pi/roscore.log

### ROS topics:

rosnode list
rostopic list | grep velodyne

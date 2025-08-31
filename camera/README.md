# Basler acA640-120gm Camera Docker Setup

This setup provides a Docker container with ROS Noetic and Basler Pylon SDK for controlling your acA640-120gm GigE camera and recording data with ROS bags.

## Prerequisites

1. **Camera Setup**: Connect your Basler acA640-120gm camera to an Ethernet switch, then connect the switch to your laptop via USB-C to Ethernet adapter.

2. **Network Configuration**: Your camera should be discoverable on the network. The camera typically uses link-local addressing (169.254.x.x).

3. **Docker**: Ensure Docker and docker-compose are installed on your system.

## Quick Start

### 1. Build and Start the Container

```bash
# Build the Docker image
docker-compose build

# Start the container
docker-compose up -d

# Enter the container
docker-compose exec basler_camera bash
```

### 2. Test Camera Connection

Inside the container, check if your camera is detected:

```bash
# List available cameras
pylonviewer --list-devices

# Or use the Pylon IP configurator to find cameras
/usr/bin/PylonViewerApp
```

### 3. Run Camera Node

```bash
# Option 1: Use the provided script
./scripts/run_camera.sh

# Option 2: Manual launch
roscore &
roslaunch /catkin_ws/launch/basler_camera.launch
```

### 4. Record Data

```bash
# Record camera data to ROS bag
rosbag record -O camera_recording.bag \
    /pylon_camera_node/image_raw \
    /pylon_camera_node/camera_info \
    /tf \
    /tf_static
```

## Configuration

### Camera Settings

Edit `config/camera_config.yaml` to adjust:
- Frame rate
- Exposure time
- Gain settings
- Image resolution
- Network parameters (MTU size, packet delay)

### Network Optimization

For better performance with GigE cameras:

1. **MTU Size**: Increase to 3000-9000 if your network supports jumbo frames
2. **Packet Delay**: Adjust `inter_pkg_delay` if you experience packet loss
3. **Receive Buffer**: Ensure your network interface has sufficient buffer size

## Troubleshooting

### Camera Not Detected

1. Check network connection:
   ```bash
   ping 169.254.x.x  # Replace with your camera's IP
   ```

2. Verify camera power and Ethernet connection

3. Check if camera is in use by another application

### Performance Issues

1. Increase MTU size in launch file
2. Adjust network buffer sizes on host system
3. Reduce frame rate or image size
4. Check CPU usage and available bandwidth

### Permission Issues

Ensure the container runs with `privileged: true` for device access.

## File Structure

```
.
├── Dockerfile              # Container definition
├── docker-compose.yml      # Service configuration
├── launch/
│   ├── basler_camera.launch    # Basic camera launch
│   └── record_camera.launch    # Camera + recording
├── config/
│   └── camera_config.yaml      # Camera parameters
├── scripts/
│   └── run_camera.sh           # Convenience script
└── bags/                       # ROS bag recordings
```

## Future LiDAR Integration

This setup is designed to accommodate future LiDAR integration on the same Ethernet switch. The network configuration and ROS setup will support multiple sensors.

## Commands Reference

### Inside Container

```bash
# Start camera node
roslaunch /catkin_ws/launch/basler_camera.launch

# View camera feed
rosrun image_view image_view image:=/pylon_camera_node/image_raw

# Record specific topics
rosbag record /pylon_camera_node/image_raw /pylon_camera_node/camera_info

# List available cameras
pylonviewer --list-devices

# Camera configuration tool
PylonViewerApp
```

### Host System

```bash
# Build container
docker-compose build

# Start services
docker-compose up -d

# View logs
docker-compose logs -f

# Stop services
docker-compose down
```
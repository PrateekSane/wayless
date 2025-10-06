# Rig: AV Data Collection platform
This repo contains my work when building this rig. It includes ROS scripts, data handling scripts, segmentation experimenting, labelling experimenting, and the src for a web page to play with a point cloud

[Youtube video](https://www.youtube.com/watch?v=H7yCXRUB-rc)

Play with a [Point Cloud Here](rig-dusky.vercel.app/)

```
av-launchpad-hub/: website to interact with PointClouds.

camera/: docker container and scripts to handle Basler ACA camera.

data_handling/: useful data scripts

ros_launch/: ROS scripts to launch the VLP16 LiDAR

segmentation/: learning classical segmentation techniques
```

## Velodyne VLP-16 to Raspberry Pi Streaming Guide

This guide walks you through the complete steps to stream 3D pointclouds from a Velodyne VLP-16 LiDAR to a Raspberry Pi running **Ubuntu 20.04 LTS (Focal Fossa)** and **ROS Noetic**.

### starting local docker image for using ros
On mac using any ros package need this docker
```
docker run -it --rm -v "$PWD":/bags rosbag-combiner
```

### Building Local docker image
```
docker build -t rosbag-combiner .
```
---

## 1. Hardware & Networking

1. **Power your VLP-16** via its PoE injector (12 V into the `PWR+DATA` port).
2. **Connect the injector’s** `DATA OUT` RJ-45 → **Pi’s Ethernet** (`eth0`).
3. **Configure static IP** on the Pi so it can reach the LiDAR (default IP `192.168.1.201`):

   ```yaml
   # /etc/netplan/50-cloud-init.yaml
   network:
     version: 2
     renderer: networkd
     ethernets:
       eth0:
         dhcp4: no
         addresses:
           - 192.168.1.100/24
   ```
4. **Apply and verify**:

   ```bash
   sudo netplan apply
   ip a show eth0    # should show 192.168.1.100/24
   ping 192.168.1.201  # test LiDAR connectivity
   ```

---

## 2. Install Ubuntu 20.04 LTS on Raspberry Pi

### A. Download the image

* Ubuntu Server 20.04.6 LTS (64-bit for Raspberry Pi 3/4/5):

  > [https://cdimage.ubuntu.com/releases/20.04.6/release/ubuntu-20.04.6-preinstalled-server-arm64+raspi.img.xz](https://cdimage.ubuntu.com/releases/20.04.6/release/ubuntu-20.04.6-preinstalled-server-arm64+raspi.img.xz)

### B. Flash with Raspberry Pi Imager (recommended)

1. Install Raspberry Pi Imager on your computer:

   * **macOS**: `brew install --cask raspberry-pi-imager`
   * **Windows/Linux**: Download from [https://www.raspberrypi.com/software/](https://www.raspberrypi.com/software/)
2. Launch Imager → **Choose OS** → scroll down → **Use custom** → select the `.img.xz` file.
3. **Choose Storage** → select your microSD card.
4. (Optional) Click ⚙️ to configure SSH, hostname, Wi-Fi credentials, etc.
5. Click **Write**, wait for completion, then **Eject**.

### C. Flash with `dd` (alternative advanced)

```bash
# On macOS, determine device (e.g. /dev/rdisk2)
diskutil list
diskutil unmountDisk /dev/diskN
xzcat ubuntu-20.04.6-preinstalled-server-arm64+raspi.img.xz \
  | sudo dd of=/dev/rdiskN bs=4M conv=fsync status=progress
diskutil eject /dev/diskN
```

### D. First boot

1. Insert the SD into the Pi and power on.
2. Find its IP on your router or by scanning your network.
3. SSH in:

   ```bash
   ssh ubuntu@<pi-ip>
   # default password: ubuntu  (you'll be prompted to change it)
   ```

---

## 3. Install ROS Noetic Core & Build Tools

1. **Add the ROS apt repository and key**:

   ```bash
   sudo apt update
   sudo apt install curl gnupg lsb-release -y
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
     | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros/ubuntu focal main" \
     | sudo tee /etc/apt/sources.list.d/ros-latest.list
   sudo apt update
   ```
2. **Install ROS base and catkin tools**:

   ```bash
   sudo apt install ros-noetic-ros-base \
                    ros-noetic-catkin \
                    python3-catkin-pkg python3-rospkg \
                    build-essential cmake -y
   ```
3. **Install diagnostics & lint dependencies**:

   ```bash
   sudo apt install ros-noetic-diagnostic-updater \
                    ros-noetic-diagnostics \
                    ros-noetic-roslint -y
   ```
4. **Source ROS** in your shell and initialize rosdep:

   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   sudo apt install python3-rosdep -y
   sudo rosdep init
   rosdep update
   ```

---

## 4. Create & Build Your Catkin Workspace

1. **Make the workspace** and enter it:

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   ```
2. **Clone the ROS 1 Velodyne driver (melodic-devel branch)**:

   ```bash
   cd src
   git clone -b melodic-devel https://github.com/ros-drivers/velodyne.git
   cd ..
   ```
3. **Install missing dependencies and build**:

   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   ```
4. **Source your workspace** on startup:

   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

---

## 5. Stream Your LiDAR Data

1. **Start ROS core (if needed):**

   ```bash
   roscore
   ```
2. **Launch the pointcloud driver:**

   ```bash
   roslaunch velodyne_pointcloud VLP16_points.launch
   ```

   You should see:

   ```
   [ INFO] … Velodyne UDP driver listening on port 2368
   ```
3. **Verify topics:**

   ```bash
   rostopic list | grep velodyne
   # → /velodyne_packets, /velodyne_points
   ```
4. **Inspect a message:**

   ```bash
   rostopic echo /velodyne_points/header -n1
   ```

   If you see valid header fields, your Pi is now streaming live 3D scans!

---

You can now visualize in RViz (via X11 forwarding), record with:

```bash
rosbag record /velodyne_points
```

or use Foxglove Studio on your Mac to inspect bag files.


# EE2-Balance-Robot-LiDAR
Raspberry Pi OS: Raspberry Pi 64 bit Lite OS

WSL distribution: Ubuntu 22.04

LiDAR datasheet: [LD19 datasheet](./LDROBOT_LD19_Development_Manual_EN_v2.5.pdf)

## Terminal visualiser - `lidar.py`
Parsed raw data from the lidar can be visualised in the terminal. The graph plot in the terminal will have 360 points of what can be seen by the LiDAR. The code currently is set show detections within a 2m radius with intensity of 30 or greater.

### Dependencies
``` bash
# setup python local environment
sudo apt update
sudo install python3-venv
python3 -m venv .venv

# enter local environment
source .venv/bin/activate

# download dependencies
pip install pyserial
pip install plotext
```

### Running the program
``` bash
python lidar.py
```

### Expected output
![Terminal output of LiDAR reading](./images/LiDAR_terminal.png)

## ROS2
`lidar_udp_sender.py`: Run on Raspberry Pi

Data is parsed using the same logic as `lidar.py`, it is packaged into JSON and sent over UDP to your laptop's IP address

`lidar_udp_receiver.py`: Run on laptop (WSL)

Laptop receives UDP packets and translates the data packets from LiDAR into data appropriate for visualisation in ROS2 using Rviz2.

### Dependencies
```bash
# Raspberry Pi in python local environment
pip install pyserial
pip install socket
pip install json

# Laptop WSL in python local environment
pip install socket
pip install json
```

Install ROS2 Humble Hawksbill - Follow documentation for installation process: https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

### Running the program
```bash
# On Raspberry Pi and in python local environment
python lidar_udp_sender.py

#WSL terminal and in python local environment
export ROS_LOCALHOST_ONLY=0
source ~/ros2_ws/install/setup.bash
python lidar_udp_receiver.py

# in another separate WSL terminal
export ROS_LOCALHOST_ONLY=0
source /opt/ros/humble/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
rviz2
```

### Expected output
Same real time LiDAR output in ROS2 as the terminal output above

Make sure to select /scan as the topic for LaserScan node
![LiDAR output in rviz2](./images/LiDAR_rviz2.png)

## SLAM on ROS2
`lidar_udp_sender`: Run on Raspberry Pi, same file sending data out via UDP

`launcher.py`: Able to manually map an environment using SLAM with the LiDAR

### Dependencies
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
sudo apt install python3-colcon-common-extensions

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/MAPIRlab/rf2o_laser_odometry.git
cd ~/ros2_ws
colcon build --symlink-install
```

### Running the program
```bash
# Raspberry Pi in python local environment
python lidar_udp_sender.py

# WSL in python local environment
ros2 launch launcher.pys
```

### Expected output
Current setup is reliant on the LiDAR itself for odometry, which is not ideal as flat walls look the same at different distances. To have good results, map a room with some interesting features (e.g. corners) which SLAM can lock onto. When mapping, avoid rotating the LiDAR to have a cleaner map of the environment.

![LiDAR SLAM in rviz2](./images/LiDAR_SLAM.png)

## Further development
* Integrate with robot IMU for odometry (forward and backward)
* Integrate with robot motor data for odometry (turning)
* Integrate with robot camera for obstacle detection (outside of planar FOV of LiDAR)

## Sensor Fusion with IMU

```bash
# to run launcher on WSL terminal
cd ~/Documents/EE2-Balance-Robot-LiDAR-ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ./launcher.py

```
# EE2-Balance-Robot-LiDAR

ROS 2 **mapping** and **autonomous face-seeking** stack for the self-balancing robot.
An LD19 2-D LiDAR plus the robot's IMU and wheel (stepper) odometry are fused into a
SLAM map; an optional face seeker then drives the robot around that map to find and
identify people.

This repo runs on the **laptop** under WSL (Ubuntu 22.04, ROS 2 Humble). The robot's
Raspberry Pi (Raspberry Pi OS 64-bit Lite) and the laptop's web/vision server
(`main.py`, separate repo) are prerequisites

LiDAR datasheet: [LD19 Development Manual](./LDROBOT_LD19_Development_Manual_EN_v2.5.pdf)

## System overview


| Machine | Role |
|---|---|
| **Pi Client** | Streams LiDAR scans + ESP32 `ODOM:` lines (step counts, gyro yaw-rate, body pitch) to the laptop over **UDP 31415**, and accepts velocity commands over **UDP 31416**, relaying them to the ESP32 as `V:`/`A:` serial. (Runs the Pi client from the controller repo.) |
| **Laptop - server** | `main.py`: web dashboard + camera face detection. Broadcasts detections over `ws://<laptop>:8000/ws/ui`. |
| **Laptop - ROS (this repo)** | ROS 2 mapping (`launcher.py`) and the autonomous face seeker (`face_seek.py`). |

```
Robot Pi  --(UDP 31415: scan + ODOM)-->  laptop WSL  --(UDP 31416: cmd_vel)-->  Robot Pi
Pi camera --------------------------->  laptop server (main.py) --(/ws/ui detections)--> face_seeker
```

## Prerequisites

ROS 2 Humble (`ros-humble-desktop`) installed and sourced. Then:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox ros-humble-robot-localization

pip install websockets        
```

`rclpy`, `tf2_ros`, `rviz2`, `sensor_msgs`, `geometry_msgs`, `std_srvs` and
`visualization_msgs` all ship with `ros-humble-desktop`. No colcon workspace is
required - `launcher.py` and `face_seek.py` run the Python nodes directly.

## Mapping - `launcher.py`

Receives the Pi's sensor stream and builds a live map:

* `sensors_udp_receiver.py`: publishes `/scan`, `/imu/data` (gyro **yaw-rate only**) and
  `/wheel/joint_states` (raw stepper counts). Drops `/scan` whenever the body pitch
  exceeds `PITCH_GATE` so balancing wobble doesn't smear the map.
* `wheel_odometry_node.py`: converts step counts to `/wheel/odom` (forward velocity
  trusted; wheel heading distrusted).
* `robot_localization`: EKF fuses wheel `vx` + gyro yaw-rate into `odom -> base_link`.
* `slam_toolbox` - scan-matching + loop closure, publishes `map -> odom` and `/map`.
* RViz2 opens automatically.

TF tree: `map -> odom -> base_link -> {laser_frame, imu_link}`.

### Run

```bash
# 1. On the Pi

# 2. On the laptop (WSL):
cd ~/Documents/EE2-Balance-Robot-LiDAR
source /opt/ros/humble/setup.bash
ros2 launch ./launcher.py
```

In RViz set **Fixed Frame = `map`** and add **Map** (`/map`) and **LaserScan** (`/scan`)
displays.

![LiDAR SLAM in RViz2](./images/LiDAR_SLAM.png)

### Tips & tuning

* Map rooms with **corners / features** - featureless straight walls look identical at
  different distances and cause "doubled" walls. Drive slowly.


## Autonomous face seeker - `face_seek.py`

A state machine that roams the map and greets people:

`SEARCH_ROTATE` (scan in place) → `SEARCH_WANDER` (drive with obstacle avoidance) →
`APPROACH` (steer toward the largest detected face by bounding-box error) →
`IDENTIFY` (hold still, tally identity votes from the server's recogniser) →
`MOVE_ON` (back off, turn away, resume searching).

Each greeted person is marked in RViz (`/people_markers`): **green** = recognised,
**red** = unknown. It consumes detections from the server's `/ws/ui` and publishes
`/cmd_vel`, which `cmd_vel_bridge.py` relays to the Pi over UDP.

### Requirements

* `launcher.py` running (provides `/scan`, `/map`, TF).
* The laptop **server `main.py`** running with the Pi camera detection active.
* The robot Pi client running (to receive velocity commands).

### Run

```bash
# Terminal 1 - mapping (see above)
ros2 launch ./launcher.py

# Terminal 2 - face seeker + cmd_vel bridge
source /opt/ros/humble/setup.bash
ros2 launch ./face_seek.py pi_ip:=<robot-Pi-IP> server:=<laptop-IP>
```

The seeker starts **disabled**. Enable / disable it with a service call:

```bash
ros2 service call /seek/enable std_srvs/srv/SetBool "{data: true}"    # start seeking
ros2 service call /seek/enable std_srvs/srv/SetBool "{data: false}"   # stop
```

### Tuning

All tunables are plain attributes at the top of `face_seeker.py`. The ones you are most
likely to touch:

| Variable | Purpose |
|---|---|
| `v_fwd` / `w_search` | Forward creep speed / search-rotate speed. |
| `min_face_frac` | Ignore faces shorter than this fraction of the frame (skips far/background faces). |
| `approach_face_frac` / `approach_stop_dist` | When to consider a face "reached". |
| `identify_time` | Seconds held still while confirming identity. |
| `visit_radius` | Don't re-greet within this distance of an already-marked spot. |


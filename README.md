# UGV01 Room Explore

ROS 2 Jazzy package for autonomous indoor exploration and 2D SLAM on a Waveshare UGV01-X3 mobile robot based on Raspberry Pi 5.

This project combines wheel odometry, LiDAR-based autonomous exploration, SLAM Toolbox mapping, a simple URDF model and ready-to-use launch files.

The main goal is to enable a small tracked robot to autonomously explore an indoor environment (classroom, lab, corridor) and build a 2D map in real time.

---

![Waveshare UGV01-X3 Robot](ugv01-room-explore/REALUGV01.jpg)
![map](ugv01-room-explore/UGVRVIZMAP.png)

## Platform

### Robot
- Waveshare UGV01-X3 (tracked)
- Raspberry Pi 5 (onboard computer)

### Sensors
- LDROBOT LD19 / STL-19P 2D LiDAR
- Motor feedback from robot controller

### Software
- Ubuntu 24.04
- ROS 2 Jazzy Jalisco
- slam_toolbox
- robot_state_publisher
- rviz2

---

## Main Features

- Wheel odometry node (`ugv_odom`)
- Autonomous exploration node (`auto_explore`)
- Integration with SLAM Toolbox
- Simple URDF model for TF and visualization
- Two launch files:
  - Mapping only
  - Mapping + autonomous exploration
- Map saving support

---

## Repository Structure

```text
ugv01_room_explore/
├── config/
│   └── slam_async.yaml
├── launch/
│   ├── autonomy_mapping.launch.py
│   └── mapping.launch.py
├── resource/
│   └── ugv01_room_explore
├── ugv01_room_explore/
│   ├── __init__.py
│   ├── auto_explore.py
│   └── ugv_odom.py
├── urdf/
│   └── ugv01_box.urdf
├── package.xml
├── setup.cfg
└── setup.py
```

## Nodes

### 1. ugv_odom
- Reads feedback from the robot controller
- Computes and publishes wheel odometry to `/odom`
- Publishes TF transform `odom → base_link`

### 2. auto_explore
- Processes LiDAR scans from `/scan`
- Implements hybrid exploration logic
- Publishes velocity commands to `/cmd_vel`
- Includes wall-following, free exploration and stuck recovery behaviors

### 3. robot_state_publisher
- Publishes robot TF frames based on the URDF model
- Provides correct transformation for the LiDAR frame (`base_link → laser`)

### 4. slam_toolbox
- Builds 2D occupancy grid map in real time
- Fuses LiDAR scans and odometry data

---

## Exploration Logic

The robot uses a simple state machine that does not require a pre-built map:

- **START_ASSESS** — initial scene observation at startup
- **START_WALL_FOLLOW** — follows a wall on the right side while maintaining a safe distance
- **WALL_CORNER_TURN** — handles corners and attempts to reacquire the wall
- **FREE_EXPLORE** — moves toward open space when wall following is not reliable
- **STUCK_RECOVERY** — performs a recovery maneuver when the robot is physically stuck

This hybrid approach makes the system more robust in real indoor environments with furniture and irregular walls.


## Build Instructions

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ugv01_room_explore
source ~/ros2_ws/install/setup.bash
```
## Launch Commands
# Mapping only:
```bash
ros2 launch ugv01_room_explore mapping.launch.py
```
# Mapping + Autonomous Exploration:
```bash
ros2 launch ugv01_room_explore autonomy_mapping.launch.py
```
## Save Map
```bash
mkdir -p ~/ros2_ws/maps
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map
```

#This will create two files:

~/ros2_ws/maps/my_map.pgm
~/ros2_ws/maps/my_map.yaml

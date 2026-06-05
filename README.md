# Teleoperation + SLAM

ROS2 package for simultaneous teleoperation and SLAM (Simultaneous Localization and Mapping). Drive a robot manually via keyboard while building a real-time map of the environment.

## Overview

Combines keyboard teleoperation with Cartographer SLAM to map an unknown Gazebo environment in real time. Visualize the evolving occupancy grid in RViz as you drive.

## Stack

- **ROS2 Humble**
- **Cartographer SLAM** — real-time map building
- **teleop_twist_keyboard** — keyboard control
- **Gazebo** — simulation
- **RViz** — map visualization

## Usage

```bash
colcon build
source install/setup.bash

# Launch robot + Gazebo + SLAM + RViz
ros2 launch teleop_slam sim_launch.py

# In a new terminal — keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Keyboard controls:** `w/s` = forward/back · `a/d` = rotate · `q/e` = diagonal

## Robotronics Club — IIT Mandi

Part of the Robotronics Club recruitment task series (Task 1).

## Author

Rohit Jangra · [github.com/Rohitjangra7370](https://github.com/Rohitjangra7370)

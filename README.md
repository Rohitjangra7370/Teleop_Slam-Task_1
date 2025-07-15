# Teleop SLAM Task 1 Repository

This repository, `rohitjangra7370/teleop_slam-task_1`, contains a ROS2-based project focused on teleoperation, SLAM (Simultaneous Localization and Mapping), and navigation using the TurtleBot3 robot in a Gazebo simulation environment. It integrates packages like `tod_nav2` for launching Gazebo worlds, spawning robots, and handling navigation tasks. The project appears to be part of a larger setup (e.g., "task_3" as noted in the original README), possibly for robotics simulation and mapping exercises.

## Project Overview

The main package here is `tod_nav2`, which sets up a Gazebo simulation with a TurtleBot3 Waffle model in a custom world (e.g., `office_cpr.world`). It includes launch files for displaying the simulation, teleoperating the robot, saving maps, and running navigation stacks. Other directories like `DynamixelSDK`, `turtlebot3`, and `turtlebot3_msgs` suggest dependencies for motor control and robot messaging, though they aren't fully detailed in the analyzed files.

Key features include:
- Launching Gazebo with a specific world and robot model.
- Teleoperation using keyboard controls.
- Map saving with Nav2 tools.
- Integration with TurtleBot3 navigation for autonomous movement using a pre-saved map.

## Prerequisites

To run this project, ensure you have:
- ROS2 Humble distribution installed.
- Gazebo Classic (not Garden, as per the launch files).
- TurtleBot3 packages: `turtlebot3_gazebo`, `turtlebot3_navigation2`, `turtlebot3_teleop`.
- Additional dependencies: `ros_gz_sim`, `ros_gz_bridge`, `nav2_map_server`.

Install them via:
```
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-gazebo-ros ros-humble-ros-gz
```

The project is built using `ament_cmake` and assumes a workspace like `/home/olympusforge/mars_rover` based on VSCode settings.

## Installation

1. Clone the repository:
   ```
   git clone https://github.com/rohitjangra7370/teleop_slam-task_1.git
   cd teleop_slam-task_1
   ```

2. Source your ROS2 environment:
   ```
   source /opt/ros/humble/setup.bash
   ```

3. Build the workspace (assuming it's part of a larger colcon workspace):
   ```
   colcon build --packages-select tod_nav2
   source install/setup.bash
   ```

4. Set environment variables as needed (e.g., `export TURTLEBOT3_MODEL=waffle`).

Note: The `DynamixelSDK` directory may require additional setup for hardware integration if using physical motors.

## Usage

### 1. Launching the Simulation
To start the Gazebo simulation with the robot:
```
ros2 launch tod_nav2 display.launch.py
```
This includes `gazebo_launch.py`, which loads the `office_cpr.world` file, sets the TurtleBot3 model to "waffle", and spawns it at position (-2.0, -0.5, 0.01).

### 2. Teleoperation
Once the simulation is running, control the robot using:
```
ros2 run turtlebot3_teleop teleop_keyboard
```
Use keyboard keys to move the TurtleBot3 around the environment.

### 3. Saving a Map
After exploring and mapping the environment via teleoperation:
```
ros2 run nav2_map_server map_saver_cli -f my_map
```
This saves the map as `my_map.yaml` and `my_map.pgm` in your current directory.

### 4. Running Navigation
To launch navigation with the saved map:
```
ros2 launch tod_nav2 gazebo_launch.py
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=src/tod_nav2/maps/my_map.yaml
```
This enables autonomous navigation using Nav2, with simulation time synchronization.

### Configuration Notes
- **World File**: Located in `gazebo_models_worlds_collection/worlds/office_cpr.world`.
- **Models Path**: Set via `GAZEBO_MODEL_PATH` to include custom models.
- **Launch Arguments**:
  - `world`: Path to the Gazebo world file (default: office_cpr.world).
  - `gui`: Set to "false" for headless mode.
  - Robot spawn position: Defaults to x=-2.0, y=-0.5.
- VSCode settings are provided for ROS development, including include paths and Python autocomplete for the Humble distro.

## Directory Structure

- **README.md**: This file (updated version).
- **DynamixelSDK/**: SDK for Dynamixel motors (likely for hardware extension).
- **tod_nav2/**: Core package with CMakeLists.txt, package.xml, launch files, Gazebo worlds/models, logs, and VSCode configs.
  - **launch/**: Contains `display.launch.py` and `gazebo_launch.py`.
  - **gazebo_models_worlds_collection/**: Custom worlds and models for simulation.
  - **log/**: Build logs and symlinks (e.g., COLCON_IGNORE).
  - **.vscode/**: Editor settings for C++ and Python.
- **turtlebot3/**: TurtleBot3 core files.
- **turtlebot3_msgs/**: Message definitions for TurtleBot3.

## Troubleshooting

- If Gazebo fails to launch, ensure `GAZEBO_MODEL_PATH` is correctly set and the world file exists.
- For spawn errors, verify the SDF model path: `/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf`.
- Check dependencies in `package.xml`; some (like `ros_gz_interfaces`) might need manual installation.
- The log directory has symlinks pointing to a future date (2025-06-21); this could be a timestamp artifact—ignore unless debugging builds.

## Video Demonstration

https://drive.google.com/drive/folders/1AE7zTh7yydI7f0yBz0ihsfpwbSRC41ob?usp=sharing

For contributions or issues, open a pull request or contact the maintainer at rohitjangra7370@gmail.com.

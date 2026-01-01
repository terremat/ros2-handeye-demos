# ROS 2 Hand-Eye Calibration Demos

This package provides a simulated environment using a UR robot (Universal Robots) and Gazebo, enabling users to test and understand the workflow of hand-eye calibration without real hardware.



## Overview

This repository contains:

- **Gazebo simulation** of a UR robot (UR5/UR10)
- **MoveIt 2** motion planning setup
- **Hand-eye calibration setups** (eye-in-hand and eye-to-hand)
- Example calibration patterns and configuration files

You can use this simulation to experiment with different calibration setups, camera placements, and patterns.



## 🛠️ Prerequisites

### Supported Versions
- **ROS 2 Distribution:** Jazzy Jalisco  
- **Gazebo Version:** Harmonic  
- **Ubuntu:** 24.04 (recommended)

### Install Dependencies

Make sure the following dependencies are installed:

```bash
sudo apt update
sudo apt install ros-jazzy-moveit \
                 ros-jazzy-ros-gz \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-ur \
                 ros-jazzy-ur-simulation-gz \
                 ros-jazzy-moveit-py
```

Then build and source your workspace:
```
colcon build
source install/setup.bash
```

Note:
The ros-rolling-ur package provides the Universal Robots (UR) description, control, and simulation interfaces required for this demo.

---

## 🚀 Running the Simulation

To start the hand-eye calibration demo in simulation, use:
```bash
ros2 launch ur_handeye_simulation sim_moveit.launch.py handeye_setup:=eye_in_hand
```

Once launched, you should see:
- A Gazebo window with the UR robot and calibration pattern.
- A MoveIt RViz interface for planning and visualizing motion.
  
### Parameters

handeye_setup — Defines the calibration configuration.
- eye_in_hand: The camera is attached to the robot’s end-effector.
- eye_to_hand: The camera is fixed in the world frame, observing the robot.



## 🧩 Customizing the Calibration Pattern

You can customize the calibration pattern used in simulation to match your preferred marker board (e.g., checkerboard, Charuco, AprilTag).

1. Prepare a pattern PNG file by converting from PDF to PNG a pattern generated with [calib.io](https://calib.io/pages/camera-calibration-pattern-generator)

2. Copy the pattern PNG file in the `ur_handeye_simulation/models/textures` folder

3. Modify the pattern xacro properties in `ur_handeye_simulation/urdf/ur_handeye_workcell.urdf.xacro` with the name of the desired pattern file
   
```xml
  <xacro:property name="pattern_width"   value="0.200"/>
  <xacro:property name="pattern_height"  value="0.200"/>
  <xacro:property name="pattern_texture" value="checker_200x200_8x9_20.png"/>
```

   

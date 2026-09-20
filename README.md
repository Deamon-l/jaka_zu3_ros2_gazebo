# JAKA ZU3 ROS2 Gazebo Simulation Project

## 1. Introduction

This project is a ROS 2 based simulation platform for the JAKA ZU3 collaborative robot.

The project integrates robot description, Gazebo simulation, MoveIt2 motion planning, and RGB-D vision processing.

The main purpose is to build a robotic manipulation platform with visual perception and automatic motion control capabilities.

## 2. Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Simulation
- MoveIt2
- Python 3

Robot:

- JAKA ZU3 collaborative robot

## 3. Features

### 3.1 Robot Simulation

- JAKA ZU3 robot URDF/Xacro description
- Robot model visualization in Gazebo
- Joint state publishing
- Robot motion control through ROS 2

### 3.2 MoveIt2 Integration

- Motion planning with MoveIt2
- Robot arm trajectory execution
- Joint and pose control

### 3.3 RGB-D Camera Integration

- Depth camera model added to the robot end-effector
- RGB image and depth image publishing
- Camera coordinate transformation using TF2

### 3.4 Guarded Visual Picking

- Timestamp-matched RGB and depth frames
- Red-object segmentation with robust median depth estimation
- Stable-target filtering and configurable workspace limits
- MoveIt pre-grasp, descend and lift sequence
- Symmetric two-finger commands with joint-position synchronization checks
- Gazebo-only object attachment after a confirmed close, so the simulated
  object follows the gripper during lift instead of relying on contact
  friction alone

## 4. Build

```bash
cd ~/jaka_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5. Run the visual pick

First run in plan-only mode. This validates the observation-pose plan without
moving the simulated robot:

```bash
ros2 launch jaka_zu3_moveit_config demo_gazebo.launch.py \
  enable_camera:=true run_grasp:=true execute_grasp:=false
```

To execute one complete pick in Gazebo:

```bash
ros2 launch jaka_zu3_moveit_config demo_gazebo.launch.py \
  enable_camera:=true run_grasp:=true execute_grasp:=true
```

For a server-only run, append `headless:=true use_rviz:=false`. Motion is
disabled by default and is enabled only when `execute_grasp:=true` is passed.

Detection and grasp tuning parameters are kept in
`src/jaka_zu3_vision/config/vision_grasp.yaml`. The camera mounting transform
and the camera-to-optical-frame transform are not altered by this file. The
most commonly adjusted values are:

- `min_area`, `max_area`, `min_depth`, and `max_depth` for detection
- `approach_offset_z`, `grasp_offset_z`, and `lift_offset_z` for motion
- `open_width` and `closed_width` for the inward-moving finger joints
- `finger_sync_tolerance` and `finger_goal_tolerance` for close verification
- `workspace_min_*` and `workspace_max_*` for target safety limits

The left and right prismatic joints use opposite axes and receive the same
positive position, so both fingers move inward by the same amount. During the
Gazebo demo, `use_sim_attachment` is enabled by the launch file only when
`run_grasp:=true`; it is a simulation aid and is not part of real-robot grasp
control.

The detector publishes `/detected_target_point` and
`/detected_target_marker`. A pick starts only after the configured number of
fresh detections agree within `target_stability_tolerance`.

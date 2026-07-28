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


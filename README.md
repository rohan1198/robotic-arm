# Robotic Arm

A standalone ROS 2 package for a 6-DOF robotic arm with control functionality.

## Overview

This package provides a complete simulation of a 6-DOF robotic arm using ros2_control. It includes:

* Hardware interface for the robot
* Controller for trajectory following
* Robot description (URDF)
* Launch files for visualization and control
* Reference trajectory generator

## Usage

### To visualize the robot:

```bash
ros2 launch robotic_arm view_robot.launch.py

# Autonomous Pick and Place with Aruco Marker and Universal Robot UR5e

## Description

This project aims to implement an autonomous pick-and-place system using the Aruco marker for object identification and the Universal Robot UR5e for manipulation. The system is designed to identify Aruco markers on objects, plan a trajectory for the UR5e to pick up the object, and place it at a predefined location.

## Table of Contents

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Features](#features)
- [Result](#Result)
- [Contributing](#contributing)
- [License](#license)

## Installation

To get started with the project, follow these installation steps. Include any dependencies, hardware requirements, or configuration instructions.

**Dependencies**

To run this project, make sure you have the following dependencies installed:

- [Python Downloads](https://www.python.org/downloads/)
- [ROS Noetic Installation on Ubuntu](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [Universal Robot GitHub Repository](https://github.com/ros-industrial/universal_robot)
- [MoveIt! Source Installation](https://moveit.ros.org/install/source/)
- [ROS tf (Transform) Documentation](http://wiki.ros.org/tf)

**Hardware Requirements**
To run this project, you will need the following hardware:

- Universal Robot UR5e
- Intel RealSense Depth Camera with a 3m USB 3 cable
- PC with Ubuntu 20 and ROS Noetic
- OnRobot RG2 Gripper with Compute Box

Ensure that all hardware components are properly connected and configured before running the system.

## Usage

python3 setup.py

## Features

- Aruco Marker Recognition: Identify Aruco markers on objects for precise localization.
- Trajectory Planning: Plan a trajectory for the UR5e to pick up and place objects autonomously.
- Object Detection: Detect and recognize objects in the workspace for efficient manipulation.
- User Interface: Include a user interface for easy interaction and monitoring.

## Result

![Universal Robot](https://github.com/Raviteja-T/test_auto/raw/master/Universal_robot.gif)

## Contact Information

For any questions or further information about this project, feel free to contact:

- **Name:** Raviteja Tirumalapudi
- **Email:** [t.raviteja@gmail.com](mailto:t.raviteja@gmail.com)

Feel free to reach out with any inquiries or feedback.


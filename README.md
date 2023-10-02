# mach1_hardware ROS Package

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

The `mach1_hardware` ROS package is tailored for the Mach1 robot, which utilizes ROS Noetic and is designed to control the Yahboom 4WD expansion board. This package serves as a communication and control interface to manage the hardware components of the robot.

For more detailed information about the **Mach1** robot, including hardware specifications and project documentation, please visit the official [Mach1 GitHub repository](https://github.com/jrendon102/mach1.git).

## Prerequisites

Before using this ROS package, ensure that you have the following prerequisites installed:

- [ROS Noetic](http://wiki.ros.org/noetic/Installation)
- **Python 3.8**: This package is compatible with Python 3.8. Please make sure you have Python 3.8 installed (Python 3.6+ may also work but is not officially supported).

## Dependencies

The `mach1_hardware` package relies on the following external dependencies:

- **camera_driver**: This C++ library simplifies the process of capturing frames, displaying video feeds, and obtaining camera information. It is used for camera-related functionality on the robot.

- **gpiozero_plus**: A Python library that enhances the control and interaction with GPIO pins. It provides a convenient interface for managing various hardware components when working with Raspberry Pi. This module is used for GPIO-related tasks on the robot.

## Installation

1. Clone the `mach1_hardware` package to your ROS workspace:

   ```bash
   cd ~/your_ros_workspace/src
   git clone https://github.com/yourusername/mach1_hardware.git
   ```

2. Build the package using either `catkin build` or `catkin_make`:
    - **catkin build**
    ```bash
    cd ~/your_ros_workspace
    catkin build
    ```
    - **catkin_make**
    ```bash
    cd ~/your_ros_workspace
    catkin_make
    ```

3. Source your ROS workspace to make the package available:
    ```bash
    source devel/setup.bash
    ```

## Author & Maintainer

Julian Rendon (julianrendon514@gmail.com)

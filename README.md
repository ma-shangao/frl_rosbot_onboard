# frl_rosbot_onboard

This package is developed to be implemented on the physical robot Husarian ROSbot Pro.

It is a CMake ROS 2 package, tested on foxy.

## Dependencies
This package relies on below packages:
    - packages integrated in the system file at https://robot-os-images.s3.eu-central-1.amazonaws.com/ros-foxy-upboard-2022-08-23.iso
    - `ros-$ROS_DISTRO-xarco`
    - nav2, `ros-$ROS_DISTRO-navigation2`, `ros-$ROS_DISTRO-nav2-bringup`

## Features
### Robot state publisher
This package rewrapped the robot description for Husarian ROSbot Pro in `launch/robot_state_publisher_launch.py`.

### Bringing up the whole robot
We provided the `launch/frl_rosbot_bringup_launch.py` to bringup everything you need from the robot, such as sensors, actuators and robot desciption.

### Basic navigation functions
We also provided an example for robot localisation and waypoint following in an mapped environment based on nav2.

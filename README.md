# frl_rosbot_onboard

This package is developed to be implemented on the physical robot Husarian ROSbot Pro.

It is a CMake ROS 2 package, tested on foxy.

## Installation
### Prepare the ROSbot 2 Pro
Install the system files following the instructions at https://husarion.com/manuals/rosbot/operating-system-reinstallation/.

### Install the package from source
Clone the package into your workspace and build it.
```bash
cd $YOUR_WORKSPACE/src
git clone https://github.com/ma-shangao/frl_rosbot_onboard.git
cd ..
colcon build --symlink-install
```


## Dependencies
This package relies on the packages integrated in the system files at https://robot-os-images.s3.eu-central-1.amazonaws.com/ros-foxy-upboard-2022-08-23.iso

Other dependecies are specified in `package.xml`. You can install them by running
 ```bash
 cd $YOUR_WORKSPACE
 rosdep update
 rosdep install --from-paths . --ignore-src -y -r
 ```
If this is your first time to run this command, you may need to run `sudo rosdep init` first.

## Features
### Robot state publisher
This package rewrapped the robot description for Husarian ROSbot Pro in `launch/robot_state_publisher_launch.py`.

### Bringing up the whole robot
We provided the `launch/frl_rosbot_bringup_launch.py` to bringup everything you need from the robot, such as sensors, actuators and robot desciption.

Simply launch the robot by:
```bash
ros2 launch frl_rosbot_onboard frl_rosbot_bringup_launch.py
```

### Basic navigation functions
We also provided an example for robot localisation and waypoint following in an mapped environment based on nav2.

## Known limitations
* The depth camera pointcloud registration is currently not available on ROS 2 for Husarian ROSbot Pro.
* The multi-robot support has yet been finished. (WIP)

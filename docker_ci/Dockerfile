# From: https://github.com/clydemcqueen/tello_ros
# To build:
# docker build --pull --no-cache --tag frl_rosbot_onboard:foxy .

# To run:
# docker run -it tello_ros:frl_rosbot_onboard bash

# I'm using this for smoke tests
# To run the pakckage in a docker container you will need to set up ports, x-windows, etc.

FROM osrf/ros:foxy-desktop

RUN apt-get update
RUN apt-get upgrade -y

RUN apt-get install -y python3-pip

WORKDIR /work/ros2_ws/src

# RUN git clone https://github.com/ma-shangao/frl_rosbot_onboard.git
COPY /home/runner/work/frl_rosbot_onboard/frl_rosbot_onboard/ frl_rosbot_onboard/

RUN git clone https://github.com/husarion/rosbot_description.git

WORKDIR /work/ros2_ws/src/frl_rosbot_onboard
RUN git checkout foxy

WORKDIR /work/ros2_ws/src/rosbot_description
RUN git checkout foxy

WORKDIR /work/ros2_ws

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

FROM osrf/ros:noetic-desktop-full AS ros_desktop

FROM dorowu/ubuntu-desktop-lxde-vnc:latest
COPY --from=ros_desktop / /
WORKDIR /

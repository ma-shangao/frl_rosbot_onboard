# Filename: Dockerfile 
# $ docker build -t song6991/rosbot_gazebo_vnc:noetic .
FROM husarion/rosbot:noetic-simulation AS sim

FROM dorowu/ubuntu-desktop-lxde-vnc:latest
COPY --from=sim / /
WORKDIR /

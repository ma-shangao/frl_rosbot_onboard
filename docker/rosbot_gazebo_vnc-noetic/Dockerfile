FROM husarion/rosbot:noetic-simulation AS sim

FROM dorowu/ubuntu-desktop-lxde-vnc:latest
COPY --from=sim / /
WORKDIR /

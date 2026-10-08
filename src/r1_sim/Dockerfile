# ROS 2 Jazzy on Ubuntu Noble. ros-jazzy-ros-gz installs Gazebo Sim 8
# (Harmonic), the same major version as gz sim 8.15.0 on the dev machine.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        python3-colcon-common-extensions \
        python3-numpy \
        python3-opencv \
        python3-yaml \
        ros-jazzy-nav2-bringup \
        ros-jazzy-rmf-building-sim-gz-plugins \
        ros-jazzy-ros-gz \
        ros-jazzy-rviz2 \
        libegl1 \
        libegl-mesa0 \
        libgl1 \
        libgl1-mesa-dri \
        libglx-mesa0 \
        libx11-6 \
        libxext6 \
        libxrender1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY package.xml CMakeLists.txt /ws/src/r1_sim/
COPY launch /ws/src/r1_sim/launch
COPY config /ws/src/r1_sim/config
COPY models /ws/src/r1_sim/models
COPY worlds /ws/src/r1_sim/worlds
COPY maps /ws/src/r1_sim/maps
COPY src /ws/src/r1_sim/src
COPY rviz /ws/src/r1_sim/rviz

RUN . /opt/ros/jazzy/setup.sh \
    && colcon build --packages-select r1_sim --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

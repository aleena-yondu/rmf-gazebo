# Base Image: ROS 2 Humble Desktop pre-installed for L4T R36.3.0
ARG VARIANT=# Base Image: ROS 2 Humble Desktop pre-installed for L4T R36.3.0
ARG VARIANT=humble-ros-base

FROM dustynv/ros:${VARIANT}-l4t-r36.3.0

# Set environment variables for NVIDIA capabilities
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV DEBIAN_FRONTEND=noninteractive

# === Setup ROS apt repo and base tools ===
RUN set -e; \
    rm -f /etc/apt/sources.list.d/ros2.list || true; \
    apt-get update; \
    apt-get install -y --no-install-recommends curl ca-certificates gnupg lsb-release equivs; \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list; \
    apt-get update

# === Dummy OpenCV provides to avoid conflicts with L4T's OpenCV ===
RUN set -e; \
    mkdir -p /tmp/dummy-opencv && cd /tmp/dummy-opencv; \
    printf "Package: dummy-opencv-dev\nVersion: 1.0\nArchitecture: all\nSection: misc\nPriority: optional\nMaintainer: dummy <dummy@example.com>\nProvides: libopencv-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-core-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-flann-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-imgproc-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-imgcodecs-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-videoio-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-highgui-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-ml-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-features2d-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-calib3d-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-dnn-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-objdetect-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-photo-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-video-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-shape-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-stitching-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-superres-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-videostab-dev (= 4.5.4+dfsg-9ubuntu4), libopencv-contrib-dev (= 4.5.4+dfsg-9ubuntu4)\nConflicts: libopencv-dev, libopencv-core-dev, libopencv-flann-dev, libopencv-imgproc-dev, libopencv-imgcodecs-dev, libopencv-videoio-dev, libopencv-highgui-dev, libopencv-ml-dev, libopencv-features2d-dev, libopencv-calib3d-dev, libopencv-dnn-dev, libopencv-objdetect-dev, libopencv-photo-dev, libopencv-video-dev, libopencv-shape-dev, libopencv-stitching-dev, libopencv-superres-dev, libopencv-videostab-dev, libopencv-contrib-dev\nReplaces: libopencv-dev, libopencv-core-dev, libopencv-flann-dev, libopencv-imgproc-dev, libopencv-imgcodecs-dev, libopencv-videoio-dev, libopencv-highgui-dev, libopencv-ml-dev, libopencv-features2d-dev, libopencv-calib3d-dev, libopencv-dnn-dev, libopencv-objdetect-dev, libopencv-photo-dev, libopencv-video-dev, libopencv-shape-dev, libopencv-stitching-dev, libopencv-superres-dev, libopencv-videostab-dev, libopencv-contrib-dev\nDescription: Dummy package to satisfy OpenCV dependencies on L4T.\n" > control; \
    equivs-build control; \
    dpkg -i dummy-opencv-dev_1.0_all.deb; \
    cd /; rm -rf /tmp/dummy-opencv

# === Install ROS / Nav2 / Dev packages ===
RUN set -e; \
    apt-get install -y --no-install-recommends \
    build-essential \
    git \
    vim \
    nano \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-behavior-tree \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-waypoint-follower \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-slam-toolbox \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-laser-geometry \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-robot-localization \
    ros-humble-foxglove-bridge \
    ros-humble-nav2-smac-planner \
    ros-humble-nav2-mppi-controller \
    ros-humble-laser-filters \
    libpcl-dev \
    libeigen3-dev \
    libtbb-dev \
    x11-apps \
    xauth \
    htop \
    tmux \
    iputils-ping \
    net-tools \
    libboost-all-dev \
    libsuitesparse-dev; \
    pip3 install trimesh --index-url https://pypi.org/simple; \
    rm -rf /var/lib/apt/lists/*

# === Install GTSAM from source ===
RUN set -e; \
    git clone https://github.com/borglab/gtsam.git /tmp/gtsam; \
    cd /tmp/gtsam; \
    mkdir build; \
    cd build; \
    cmake ..; \
    make -j$(nproc); \
    make install; \
    ldconfig; \
    cd /; \
    rm -rf /tmp/gtsam

# Install Python packages
RUN pip3 install numpy

# Ensure nvidia user exists with passwordless sudo and known password
RUN set -e; \
    if ! id -u nvidia >/dev/null 2>&1; then \
        useradd -m -s /bin/bash nvidia; \
    fi; \
    echo "nvidia:nvidia" | chpasswd; \
    echo "nvidia ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/nvidia; \
    chmod 0440 /etc/sudoers.d/nvidia

# Setup environment sourcing in the 'nvidia' user's .bashrc
USER nvidia
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc && \
    echo "if [ -f /r1_navigation/driver_ws/install/setup.bash ]; then source /r1_navigation/driver_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "if [ -f /r1_navigation/install/setup.bash ]; then source /r1_navigation/install/setup.bash; fi" >> ~/.bashrc

# Define a generic working directory. The devcontainer.json will specify the final workspace folder.
WORKDIR /workspaces
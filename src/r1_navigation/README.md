# R1 Navigation ROS 2 Workspace

Navigation stack for the R1 robot platform: SLAM, localization, path planning, and warehouse-specific behavior tree plugins.

## 1. Workspace Structure

```
r1_navigation/
├── r1_nav_cpp/            # Primary navigation package (localization, Nav2, BT plugins, maps)
├── driver_ws/             # Stable Livox LiDAR driver (built separately)
│   └── src/
│       └── livox_ros_driver2/
├── Livox-SDK2/            # Livox LiDAR SDK (submodule)
├── scripts/               # Shell helpers
└── .devcontainer/         # Docker / devcontainer configs
```

### Why Chain Workspaces?

The `driver_ws` MUST be sourced before building `r1_nav_cpp` because FAST-LIO depends on `livox_ros_driver2` custom message types.

## 2. Build and Usage

### Building the Workspaces

```bash
# 1. Build the driver workspace
cd /navigation/driver_ws
colcon build --symlink-install

# 2. Source the driver, then build navigation
source /navigation/driver_ws/install/setup.bash
cd /navigation
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Environment Setup

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /navigation/driver_ws/install/setup.bash" >> ~/.bashrc
echo "source /navigation/install/setup.bash" >> ~/.bashrc
```

## 3. Dependencies

### System Dependencies

- ROS 2 Humble Desktop
- `ros-dev-tools`, `colcon-common-extensions`
- `libgtsam-dev` (build from source — see below)

### GTSAM (manual)

```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake .. && make -j8 && sudo make install
```

### ROS Dependencies

```bash
cd /navigation
rosdep install -i --from-path r1_nav_cpp --rosdistro humble -y
```

## 4. Environment Variables

| Variable | Description | Allowed Values | Default |
|----------|-------------|----------------|---------|
| `LIDAR` | LiDAR sensor hardware type | `LIVOX`, `OUSTER` | `LIVOX` |
| `LOCATION` | Deployment warehouse location | `YONDU`, `SHIPBOTS` | `YONDU` |
| `ROS_NAMESPACE` | Robot identifier | any string | (none) |
| `ROS_DOMAIN_ID` | ROS 2 domain ID | integer | `0` |

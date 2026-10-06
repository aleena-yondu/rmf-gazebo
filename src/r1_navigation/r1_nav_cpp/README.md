# r1_nav_cpp - C++ Navigation Components for R1 Robot

This package contains C++ implementations of navigation components for the R1 robot, including odometry, sensor processing, behavior tree nodes, and navigation utilities.

## Overview

The C++ implementation provides better performance, lower latency, and reduced overhead compared to Python for real-time navigation tasks. All nodes maintain API compatibility with their Python counterparts where applicable.

## Package Structure

```
r1_nav_cpp/
├── src/
│   ├── bt_nodes/                    # Behavior Tree nodes for Nav2
│   │   ├── debug_print.cpp
│   │   ├── get_chassis_target_service.cpp
│   │   ├── pan_scanner.cpp
│   │   ├── publish_pose.cpp
│   │   ├── run_inference_bt_node.cpp
│   │   ├── scan_for_barcode_action.cpp
│   │   ├── servo_to_target_action.cpp
│   │   ├── set_blackboard_string.cpp
│   │   ├── set_planner_tolerance.cpp
│   │   └── update_order_state.cpp
│   ├── cmd_vel_relay.cpp            # Velocity command relay
│   ├── dynamic_footprint.cpp        # Dynamic footprint publisher
│   ├── online_imu_bias_estimator.cpp
│   ├── pointcloud_timestamp_fixer.cpp
│   ├── scan_timestamp_fixer.cpp
│   └── synchro_drive_odometry.cpp
├── config/                          # Navigation parameters
├── launch/                          # Launch files
└── maps/                            # Map files
```

---

## Standalone Nodes

### 1. synchro_drive_odometry

Computes accurate odometry for the R1's 3-wheel synchrodrive system.

**Published Topics:**
- `/synchro_odom` (nav_msgs/Odometry) - Robot odometry

**Subscribed Topics:**
- `/hdas/feedback_chassis` (sensor_msgs/JointState) - Wheel states

**Parameters:**
- `wheel_radius` (double, default: 0.076) - Wheel radius in meters
- `wheel_base_radius` (double, default: 0.327) - Distance from center to wheel
- `odom_frame` (string, default: "odom") - Odometry frame
- `base_frame` (string, default: "base_link") - Robot base frame
- `publish_tf` (bool, default: false) - Publish odom->base_link transform
- `publish_rate` (double, default: 20.0) - Publishing frequency in Hz

**Features:**
- Proper 3-wheel synchrodrive kinematics using least-squares solver
- Thread-safe velocity computation
- Data freshness monitoring with timeout
- Accurate pose integration with proper covariances

### 2. online_imu_bias_estimator

Estimates and corrects IMU gyroscope and accelerometer biases online during operation.

**Published Topics:**
- `/imu/data_bias_corrected` (sensor_msgs/Imu) - Bias-corrected IMU data

**Subscribed Topics:**
- `/livox/imu` (sensor_msgs/Imu) - Raw IMU data
- `/synchro_odom` (nav_msgs/Odometry) - Odometry for stationarity detection

**Parameters:**
- `input_topic` (string, default: "/livox/imu") - Raw IMU topic
- `output_topic` (string, default: "/imu/data_bias_corrected") - Corrected IMU topic
- `odom_topic` (string, default: "/synchro_odom") - Odometry topic
- `lin_stationary_thresh` (double, default: 0.02) - Linear velocity threshold (m/s)
- `ang_stationary_thresh` (double, default: 0.02) - Angular velocity threshold (rad/s)
- `alpha` (double, default: 0.01) - EMA filter gain for bias estimation
- `warmup_seconds` (double, default: 3.0) - Warmup period for initial bias estimation

**Features:**
- Online bias estimation using exponential moving average
- Warmup period for rapid initial convergence
- Stationarity detection using odometry feedback
- Gravity-aware accelerometer bias estimation

### 3. cmd_vel_relay

Bridges Nav2 velocity commands to HDAS chassis controller format.

**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Nav2 velocity commands

**Published Topics:**
- `/motion_target/target_speed_chassis` (geometry_msgs/TwistStamped) - HDAS format velocity

### 4. dynamic_footprint

Dynamically updates robot footprint based on arm/torso positions for collision avoidance.

**Published Topics:**
- `/dynamic_footprint` (geometry_msgs/PolygonStamped) - Current robot footprint

**Features:**
- Monitors torso and gripper link transforms
- Updates both local and global costmap footprints via service calls
- Extends footprint to encompass arm positions

### 5. scan_timestamp_fixer / pointcloud_timestamp_fixer

Corrects timestamps on sensor data from sources with clock drift.

---

## Behavior Tree Nodes

Custom BT nodes for Nav2's behavior tree navigator, implementing warehouse pick operations.

### Action Nodes (StatefulActionNode)

| Node | Description |
|------|-------------|
| `ServoToTargetAction` | Moves robot arms/torso to target configurations for bin or tote access. Sequences through arms, gripper, and torso movements based on current order. |
| `ScanForBarcodeAction` | Triggers barcode scanner action to find a specific barcode. Generates expected barcode from current order or accepts explicit target. |
| `RunInferenceBtNode` | Calls the `/run_inference` action server to execute learned pick/place motions via policy inference. |
| `PanScanner` | Pans the arm holding the barcode scanner in a sinusoidal pattern to scan for barcodes on shelving. |

### Service Nodes (BtServiceNode)

| Node | Description |
|------|-------------|
| `GetChassisTargetService` | Calls `GetChassisTarget` service to get navigation poses for BIN, TOTE, or CENTER_AISLE targets. Outputs best_estimate_pose, detected_pose, and scan_pose. |

### Sync Action Nodes (SyncActionNode)

| Node | Description |
|------|-------------|
| `UpdateOrderState` | Monitors `/teleop/order` topic and updates blackboard variables `active_order` and `current_target` based on incoming pick tickets. |
| `SetBlackboardString` | Simple utility to set a string value on the blackboard. |
| `SetPlannerTolerance` | Adjusts Nav2 planner's goal tolerance parameter dynamically (set absolute or increment). |
| `PublishPose` | Publishes a PoseStamped from blackboard to `/current_goal` for visualization. |
| `DebugPrint` | Logs debug messages and optionally pose information for BT debugging. |

### BT Node Ports

#### ServoToTargetAction
```xml
<ServoToTargetAction target_type="{current_target}" />
```
- `target_type` (input): "BIN" or "TOTE"

#### ScanForBarcodeAction
```xml
<ScanForBarcodeAction target_barcode="" timeout="10.0" 
                      scanned_barcode="{barcode}" barcode_found="{found}" />
```
- `target_barcode` (input): Explicit barcode or empty to generate from order
- `timeout` (input): Scan timeout in seconds
- `scanned_barcode` (output): The barcode that was searched for
- `barcode_found` (output): Boolean indicating if barcode was found

#### GetChassisTargetService
```xml
<GetChassisTargetService target_type="BIN" 
                         best_estimate_pose="{goal_pose}"
                         detected_pose="{detected_pose}"
                         target_detected="{detected}"
                         target_barcode="{barcode}"
                         scan_pose="{scan_pose}" />
```

#### RunInferenceBtNode
```xml
<RunInferenceBtNode steps="50" sleep="20" action_type="pick" message="{msg}" />
```
- `steps` (input): Number of inference timesteps
- `sleep` (input): Milliseconds between timesteps
- `action_type` (input): "pick" or "place"

---

## Building

```bash
cd ~/yondu_ws
colcon build --packages-select r1_nav_cpp
source install/setup.bash
```

## Usage

### Complete Navigation System (Recommended) ⭐

**Launch everything at once:**
```bash
ros2 launch r1_nav_cpp nav_complete.launch.py
```

This launches the complete navigation and localization system:
- Wheel odometry (C++)
- IMU bias correction (C++)
- EKF sensor fusion
- Scan processing
- SLAM Toolbox localization
- Nav2 navigation stack
- Footprint visualization

**With custom map:**
```bash
ros2 launch r1_nav_cpp nav_complete.launch.py map_file:=/path/to/your/map
```

### Launch Specific Subsystems

**Localization only (no navigation):**
```bash
ros2 launch r1_nav_cpp localization.launch.py
```

**SLAM mapping (create new map):**
```bash
ros2 launch r1_nav_cpp mapping.launch.py
```

**Navigation only (requires localization running):**
```bash
ros2 launch r1_nav_cpp navigation.launch.py
```

**Odometry pipeline only:**
```bash
ros2 launch r1_nav_cpp wheel_imu_localization.launch.py
```

### Run Individual Nodes

```bash
# Odometry
ros2 run r1_nav_cpp synchro_drive_odometry

# IMU Bias Estimator
ros2 run r1_nav_cpp online_imu_bias_estimator

# Cmd Vel Relay
ros2 run r1_nav_cpp cmd_vel_relay

# Dynamic Footprint
ros2 run r1_nav_cpp dynamic_footprint
```

**See `launch/README.md` for detailed launch file documentation.**

---

## Performance Comparison

| Metric | Python | C++ | Improvement |
|--------|--------|-----|-------------|
| CPU Usage | ~15% | ~3% | 5x |
| Memory | ~80MB | ~12MB | 6.7x |
| Latency (avg) | 8ms | 1.5ms | 5.3x |
| Latency (p99) | 25ms | 3ms | 8.3x |

*Measurements on R1 onboard computer (ARM64)*

---

## Dependencies

- ROS 2 (Humble or later)
- Eigen3
- TF2
- BehaviorTree.CPP v3
- sensor_msgs, nav_msgs, geometry_msgs, visualization_msgs
- robot_localization (for EKF fusion)
- r1_interfaces (custom messages/services/actions)
- yaml-cpp

---

## Architecture

### Odometry Pipeline

```
Wheel Encoders → synchro_drive_odometry → /synchro_odom
                                              ↓
Raw IMU → online_imu_bias_estimator → Corrected IMU → EKF → /odom
```

### Pick Operation Flow (Behavior Tree)

```
UpdateOrderState → GetChassisTargetService(BIN) → NavigateToPose
       ↓                                               ↓
  [active_order]                              ServoToTargetAction(BIN)
       ↓                                               ↓
ScanForBarcodeAction ← PanScanner              RunInference(pick)
       ↓                                               ↓
GetChassisTargetService(TOTE) → NavigateToPose → ServoToTargetAction(TOTE)
                                                       ↓
                                               RunInference(place)
```

---

## Migration from Python

To switch from Python to C++ nodes:

1. Replace `r1_nav` with `r1_nav_cpp` in launch files
2. Replace executable names:
   - `synchro_drive_odometry` (same name)
   - `online_imu_bias_estimator` (same name)
   - `footprint_node` → `footprint_visualizer`

All topic names and parameters remain identical for compatibility.

---

## Troubleshooting

**No odometry published:**
- Check that `/hdas/feedback_chassis` is publishing
- Verify wheel parameters match your robot

**IMU bias not converging:**
- Ensure robot is stationary during warmup
- Check `ang_stationary_thresh` is above noise level
- Verify odometry topic is publishing

**BT nodes not loading:**
- Ensure `r1_nav_cpp` is built and sourced
- Check that Nav2 bt_navigator has the plugin path configured
- Verify blackboard has "node" entry from bt_navigator

**High CPU usage:**
- Reduce `publish_rate` parameter
- Check for TF lookup issues

---

## License

Apache-2.0

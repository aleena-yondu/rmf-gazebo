# URDF Tuning for Rotation Skew Fix

**Date:** 2026-03-11
**Context:** R1 robot SLAM mapping — robot rotation causes scan skew/lateral shift

---

## Problem Description

During in-place rotation, the SLAM map shows a skew or lateral shift. The `imu_bias_estimator` corrects yaw drift while stationary (confirmed working), but the skew persists during rotation.

Mentor's suggestion: tune URDF parameters using a caliper.

---

## Root Cause Analysis

### Primary Cause — LiDAR Static TF Error (biggest impact)

Current `ouster_static_tf` in `mapping.launch.py`:
```python
arguments=["0.2", "-0.2", "0.2", "0", "0", "0", "base_link", "os_sensor"]
#           x      y      z     r    p    yaw
```

The **y = -0.2 m** (20 cm off-center) is the largest amplifier of rotation error.

`pointcloud_to_laserscan` transforms all points into `base_link` using this TF. If the TF has any positional error — especially in x/y — rotation magnifies it:

> Apparent scan error ≈ `TF_position_error × rotation_angle`

A 5 mm error in the LiDAR y-offset causes ~5 mm of apparent lateral shift per radian of rotation. SLAM sees this as the environment sliding sideways.

### Secondary Cause — `wheel_base_radius` Error

```python
# mapping.launch.py synchro_drive_odometry parameters
"wheel_radius": 0.076,
"wheel_base_radius": 0.327,
```

For a 3-wheel synchro drive, yaw rate is computed as `v / wheel_base_radius`. An error here causes the odometry to systematically over- or under-rotate, compounding the scan mismatch.

### Tertiary Cause — IMU Gyro During Rotation

The `imu_bias_estimator` only updates bias estimates during stationary periods. During rotation:
- The frozen bias estimate is applied (partially corrects drift)
- **Gyro scale factor error** is not correctable by bias estimation
- This is a smaller effect compared to TF and odometry errors

The EKF fuses both wheel `vyaw` and IMU `vyaw`, so fixing the wheel odometry reduces reliance on the IMU during rotation.

---

## Parameters to Measure with Caliper

### 1. LiDAR Position (Priority 1)

Measure from the `base_link` origin (robot geometric center) to the **Ouster sensor center**:

| Parameter | Current Value | What to Measure |
|-----------|--------------|-----------------|
| `x`       | 0.2 m        | Forward distance from robot center to Ouster |
| `y`       | -0.2 m       | Lateral offset (negative = right side) |
| `z`       | 0.2 m        | Height from base_link origin to Ouster |
| `yaw (rz)`| 0.0 rad      | Angle if sensor is not perfectly forward-facing |

Update in `mapping.launch.py`:
```python
ouster_static_tf = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="ouster_base_tf",
    arguments=["<x>", "<y>", "<z>", "0", "0", "<yaw>", "base_link", "os_sensor"],
    ...
)
```

### 2. Wheel Base Radius (Priority 2)

Measure from the robot center (base_link origin projected to floor) to each wheel axle center. Check all 3 wheels for symmetry.

Update in `mapping.launch.py`:
```python
"wheel_base_radius": <measured_value>,
```

### 3. Wheel Radius (Priority 3)

Measure from wheel axle center to ground contact point on a flat surface.

Update in `mapping.launch.py`:
```python
"wheel_radius": <measured_value>,
```

---

## Tuning Validation Procedure

1. **Rotation test**: Rotate robot 360° in place and stop. In RViz, the map scan should realign perfectly with no lateral residual offset.
2. **Full rotation test**: Rotate 360° and return to start — cumulative map drift should be near zero.
3. **Linear test**: Drive straight forward and back — validate `wheel_radius` by checking position returns to origin.
4. **Combined test**: Drive a rectangular path and check closure error.

---

## Pipeline Reference

```
/ouster/imu
    │
    ▼
imu_bias_estimator  (EMA, stationary-gated gyro bias correction)
    │
    │  /imu/data_bias_corrected
    ▼
EKF (fuses vyaw from /imu/data_bias_corrected + vx,vy from /synchro_odom)
    │
    │  /odom
    ▼
SLAM Toolbox

/ouster/points
    │
    ▼  (transform via base_link→os_sensor TF)
pointcloud_to_laserscan  →  /scan_raw  →  laser_filter  →  /scan
    │
    ▼
SLAM Toolbox
```

---

## Key Files

| File | Purpose |
|------|---------|
| `src/r1_navigation/r1_nav_cpp/launch/mapping.launch.py` | Static TF, wheel odometry params, full pipeline |
| `src/r1_startup/src/brs_description_cpp/urdf/latest_urdf/final_robot.urdf` | Robot URDF (no os_sensor frame — defined via static TF in launch) |
| `src/r1_nav_cpp/config/nav_configs/ekf_with_ouster_imu.yaml` | EKF sensor fusion config |
| `src/r1_nav_cpp/config/nav_configs/imu_bias_estimator_params.yaml` | Bias estimator config |

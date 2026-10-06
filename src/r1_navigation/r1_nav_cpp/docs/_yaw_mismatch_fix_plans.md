# Rotational Map Drift Issue

## Overview
The map drifts during **in-place rotation**, with the drift **growing with rotation speed**. Unlike stationary yaw drift (fixed by IMU bias estimation), this occurs specifically during motion.

### Observations
- Clockwise rotation → smaller drift  
- Counter-clockwise rotation → larger drift (Drift happened even with a slow angular velocity)
- Linear motion after rotation helps **SLAM Toolbox** correct the drift  
- Continuous rotation at origin → drift accumulates; scan matching has limited effect  

### Edge Case
- Robot rotates in place without translation → worst-case map misalignment  

### Systems Involved
- LiDAR (Ouster) + SLAM Toolbox  
- IMU  
- EKF-based state estimation  
- TF-based sensor extrinsics



ros2 bag record /ouster/points /ouster/imu /tf /tf_static --qos-profile-overrides-path /dev/stdin /synchro_odom -o /home/nvidia/calib_bag_straight << 'EOF'
  /synchro_odom:
    reliability: best_effort  
    durability:volatile
    history: keep_last
    depth: 10
  EOF

## Plans to fix.
Typical sources of directional drift include:

- [x] Sensor Validation: (IMU reading, scan correctness)
- [ ] Sensor Extrinsics & URDF
    - [ ] IMU gyro bias
    - [x] IMU axis sign errors (which Ouster handles it with 180 offset between os_sensor and os_lidar, scan matched frame in pointcloud_to_laserscan_node checked)
    - ~~wheel odometry asymmetry~~
    - [ ] IMU-LiDAR extrinsic rotation misalignment → even a small roll/pitch offset can accumulate in yaw during rotation.
    - [ ] TF frame inconsistencies -> CW vs CCW asymmetry, mounting?

- [ ] State Estimator / EKF
- [ ] SLAM
    - [ ] scan-matching parameters

1. Need to **calibrate the IMU extrinsics**. With ros2 bag, tune it offline.

    - slow rotation
    - fast rotation
    - ~~straight-line motion~~
    - square trajectory

    - Offline calibration to refine:
        - IMU-LiDAR extrinsics
        - IMU gyro bias
        - Axis sign alignment

    Other methods to tune:  
    - Ground Plane Fitting: z and pitch/roll alignment. eliminating tilt-induced errors (maybe not related)
    - Target-Based Calibration: yaw misalignment and horizontal offsets.
    - fast-GICP (Generalized Iterative Closest Point): Align a LiDAR scan to a reference 3D map or known structure [refine the LiDAR → base_link transform] 
    _Scan Matching as a Diagnostic_


2. We have to assume sync timing is good, as now we use PTP 1580.

3. SLAM Toolbox:
    - scan-matching parameters for high rotation rates _Scan Matching as a Diagnostic_

4. Really do not want to spend time on but in the end:
    - Two EKF arch
    - Fast LIO





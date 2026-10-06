# Resolving Yaw Drift in LiDAR-Inertial SLAM: A Systematic Debug Writeup

**Platform**: R1 Robot (3-wheel synchro drive)
**Stack**: Ouster OS1, slam_toolbox, robot_localization (EKF), ROS 2
**Date**: 2026-03-18


## Overview

Persistent yaw drift during SLAM on the R1 robot (map misalignment during in-place rotation) had multiple overlapping root causes, addressed sequentially across four phases: extrinsic calibration, EKF audit, IMU dynamic bias modelling, and LiDAR deskewing.

---

## Phase 1: Extrinsic Calibration via KISS-ICP

An inaccurate static transform between `base_link` and `os_sensor` geometrically amplifies rotational map error:

$$\text{$Err_{scan}$} \approx \delta_{TF} \times \Delta\theta$$

With the Ouster at ~20 cm offset, a 5 mm lateral TF error produces 5 mm/rad of map shift. Without a calibration target, KISS-ICP was run on ROS 2 bags to produce a reference trajectory. The 6-DoF LiDAR ↔ base\_link transform was solved iteratively, using bag sequences that included pure in-place rotation, which isolates the angular component of the transform and make rotational errors clearly visible. The result was committed to both `mapping.launch.py` and `localization.launch.py`.

**Reference**: `docs/_extrinsic_calibration.md`, `launch/mapping.launch.py`

---

## Phase 2: EKF Sensor Fusion Audit

The EKF was audited for misconfiguration that could cause good sensor data to be ignored or poor data over-weighted:

- `imu0` confirmed pointing to `/imu/data_bias_corrected`, not the raw topic.
- Covariance values reviewed against actual sensor quality. IMU only provided yaw rate; wheel odometry only provided vx and vy.
- `vx_scale_factor` and minimum velocity thresholds verified consistent across mapping and localization launches.

**Reference**: `config/nav_configs/ekf_with_ouster_imu.yaml`, `launch/localization.launch.py`

---

## Phase 3: IMU Bias Estimation and the Dynamic Bias Problem

The most analytically involved phase. The Ouster IMU gyroscope z-axis exhibited drift that a single fixed offset could not explain, leading to the discovery of velocity-dependent dynamic bias.

### 3.1 The Sensor Error Model

$$\omega_{\text{measured}} = \omega_{\text{true}} + b_{\text{static}} + b_{\text{dynamic}}(\omega) + \eta$$

- **$b_{static}$**: Temperature-dependent constant offset; measured at −0.009294 rad/s on z-axis (mean of 1605 stationary samples).
- **$b_{dynamic}(\omega)$**: Velocity-dependent bias, the key discovery in this phase.
- **$\eta$**: White noise; cannot be removed.

### 3.2 Observing the CW/CCW Asymmetry

CW and CCW rotations at the same speed produced *different* drift magnitudes. Static bias and ω² terms are both direction-symmetric; asymmetry implies a k·ω term that flips sign with rotation direction.

This led to the working model:

$$b_{\text{dynamic}}(\omega) = b_0 + k\,\omega \;(+\; k_2\,\omega^2)$$

| Term | Characteristic | Hypothesized Source | Relation to Radius *r* |
|------|---------------|---------------------|------------------------|
| k₂·ω² | Speed-Symmetric | Centripetal g-Sensitivity: Mechanical stress from inward *aᶜ*. | Zero if *r* = 0 |
| k·ω | Speed-Asymmetric | **Scale Factor** (imperfect gain calibration); **Torque Deformation** (motor micro-tilts amplified by lever arm); **VRE** (asymmetric CW/CCW vibration). | Weak (Reduced by centering) |
| b₀ | Constant Offset | Intrinsic Bias: Zero-rate offset affected by ambient temperature, power-on state, and long-term sensor aging. | None |

### 3.3 Why Off-Center Mounting Creates Velocity-Dependent Bias (and Why Static TF Cannot Fix It)

Two distinct mechanisms are at play:

- **Static TF**: corrects where the sensor *is* in space. Fixes the geometric lever-arm effect on odometry and scan alignment. Applied once, offline.
- **Centripetal g-sensitivity**: a measurement artifact from the IMU being physically subjected to centripetal acceleration $a_c = \omega^2 r$ during rotation. This force is sensed by the accelerometers and, via MEMS g-sensitivity, couples into the gyroscope output as an ω²-proportional bias. No coordinate transform can subtract a force that was actually experienced.
- **Scale Factor / mechanical coupling**: imperfect gain calibration and torque-induced micro-tilts are similarly unresolvable by any TF adjustment.

Phase 1 calibration was therefore necessary but insufficient on its own.

![IMU off-center mounting: accelerometer and gyroscope model](haha.svg)

### 3.4 Two-Regime Correction Strategy

An `online_imu_bias_estimator` node applies two mutually exclusive regimes:

- **Stationary** (`|v| < 0.01 m/s` and `|ω| < 0.02 rad/s`):
  - **Warmup** (first 500 frames, ~5 s): computes arithmetic mean to seed EMA near the correct initial value.
  - **EMA updates**: begin only after 150 *consecutive* stationary frames (~1.5 s), to let post-braking vibrations settle. These two phases run in sequence: warmup first, then guarded EMA.
  - A 2nd-order Butterworth LPF (20 Hz) is applied during this regime to suppress noise that would otherwise reset the consecutive-frame counter.

- **Moving**:
  - Applies the frozen EMA continuously (*not* zeroed).
  - Adds the dynamic model correction as a function of instantaneous ω.

> **On b₀ vs $b_{static}$**: The calibration script subtracts `b_static` *before* fitting, so the model intercept `b₀ ≈ 0` and does not re-encode the static offset. The EMA tracks `b_static`; the dynamic model handles `b_dynamic`.

**Reference**: `src/online_imu_bias_estimator.cpp`, `docs/imu_bias_estimator_notes_en.md`, `config/nav_configs/imu_bias_estimator_params.yaml`

### 3.5 Multi-Speed Calibration

Bags recorded at ±0.3, ±0.5, ±0.7, ±1.0, ±1.2 rad/s (10 bags total). KISS-ICP extracted ground-truth ω from each bag. After static bias subtraction, residuals were pooled and fit via least-squares to produce two model options:

- `linear`: b(ω) = b₀ + k·ω
- `quadratic`: b(ω) = b₀ + k·ω + k₂·ω²

> **Model selection guidance**: The quadratic term is not guaranteed to be meaningful; at typical R1 operating speeds, IMU noise may dominate the k₂·ω² signal, and the fitted k₂ could be absorbing noise rather than capturing real physics. **The linear model is the safer default.** Only consider switching to quadratic if operating at consistently high ω (e.g. a faster platform) and re-calibrating at those speeds shows a clear, repeatable quadratic residual. MAE < 0.4 mrad/s is the acceptance threshold in either case.

**Reference**: `scripts/calibrate_rotation_bias_multi.py`, `docs/imu_bias_estimator_notes_en.md`

### 3.6 EMA Batch Update (Stability Fix)

At high rotation speeds, brief stops do not give the EMA enough time to fully converge before the robot moves again. Accumulated across multiple stops, this causes bias wander. Fixed by switching from sample-by-sample EMA updates to a batch strategy: 200 consecutive stationary samples are accumulated first, then applied as a single weighted update (`bias_update_alpha = 0.5`).

**Reference**: `docs/imu_bias_estimator_notes.md`

---

## Phase 4: LiDAR Point Cloud Deskewing

After Phase 3 resolved near-range (<3 m) drift, distant objects (>5 m) still showed range-proportional drift, consistent with intra-scan point cloud distortion (rolling shutter effect).

### 4.1 The Physics of Scan Distortion

A 10 Hz LiDAR takes 100 ms to complete one full scan. If the robot rotates at ω during that period, its heading changes by Δθ = ω × 100 ms. Points captured at the start and end of the scan were taken from different robot orientations, but the naive implementation treats all 1024 columns as if they were simultaneous.

The resulting position error for an object at range d is:

$$\epsilon_{\text{pos}} \approx d \cdot \Delta\theta = d \cdot \omega \cdot T_{\text{scan}}$$

At 1.0 rad/s and 10 m range: ~1.0 m error. At 2 m range: ~0.2 m. This explains why the error is range-proportional and why IMU bias correction alone cannot fix it, since bias correction only improves the inter-scan heading estimate fed to the EKF, while the geometric distortion is already baked into each individual point cloud frame.

The fix is **motion undistortion** (deskew): each point is rotated back from its actual acquisition time to a common reference (`max_t` = scan end), using IMU-interpolated ω at that moment. The Ouster provides a per-point `t` field (uint32, nanoseconds from scan start) enabling per-point correction.

### 4.2 Deskew Implementation Bugs

Two bugs in `pointcloud_deskew.cpp` compounded each other during rotation.

**Bug 1 — Wrong IMU lookup time** (corrupts per-point geometric correction): The Ouster's `header.stamp` marks the scan *start*, which is the moment the first point was captured. Each point's `t` field is the offset in nanoseconds from that start. The correct absolute timestamp for an IMU lookup is:

$$t_{\text{point}} = t_{\text{header}} + t_{\text{field}}$$

The original code incorrectly assumed `header.stamp` was the scan *end*, computing:

$$t_{\text{point}} = t_{\text{header}} - (t_{\text{max}} - t_{\text{field}})$$

This is equivalent to `header.stamp + t_field - 100 ms`, effectively shifting every IMU lookup one full scan period into the past and fetching angular velocity from the *previous* scan's time window. During straight motion this had no effect (ω ≈ 0 in both frames), but during rotation it fetched the wrong ω, or even an ω of opposite sign, making per-point deskew actively harmful.

**Bug 2 — Wrong output timestamp** (causes slam_toolbox scan matching snap): Deskew brings all points to the scan *end* geometry (`max_t`), but the output `header.stamp` was left at scan start. slam_toolbox queries `/odom` at `header.stamp` to seed scan matching, so it used the robot's pose at `t_start` against a cloud that geometrically represents `t_end`. The resulting yaw mismatch per scan is:

$$\epsilon_{\text{yaw}} = \omega \times (t_{\text{end}} - t_{\text{start}}) = \omega \times 100\,\text{ms}$$

This appeared as a fixed-magnitude snap during rotation that vanished when the robot stopped. Fix: set output `header.stamp` = scan end time.

Bug 1 corrupted the per-point correction itself; Bug 2 misaligned the scan's initial pose for matching. Neither fix alone was sufficient.

**Reference**: `docs/ouster_deskew_timestamp_convention.md`, `docs/deskew_header_stamp_fix_zh.md`, `src/pointcloud_deskew.cpp`

---

## Summary of Root Causes and Fixes

| Phase | Root Cause | Fix | Scope |
|-------|-----------|-----|-------|
| 1 | Inaccurate LiDAR↔base_link TF amplifies rotation error geometrically | KISS-ICP offline extrinsic calibration | All ranges |
| 2 | EKF misconfiguration; corrected IMU topic not confirmed | Audit EKF yaml; verify `/imu/data_bias_corrected` routing | All motion |
| 3a | Static gyro z-axis bias (~−0.009 rad/s) | Stationary-gated EMA estimator | Low-speed / stationary |
| 3b | Velocity-dependent dynamic bias (CW/CCW asymmetry) | b(ω) = b₀ + k·ω; multi-speed calibration | In-place rotation |
| 3c | EMA frozen bias zeroed during motion (code bug) | Always apply frozen EMA regardless of motion state | Translation |
| 4a | Intra-scan distortion: points captured at different headings | Per-point deskew using IMU-interpolated ω | Far-range objects |
| 4b | IMU lookup used wrong time (previous-frame ω), corrupting per-point correction | `t_point = header.stamp + t_field` | Rotation |
| 4c | Output `header.stamp` left at scan start; slam_toolbox used wrong initial pose | Set output `header.stamp` = scan end time | Rotation |

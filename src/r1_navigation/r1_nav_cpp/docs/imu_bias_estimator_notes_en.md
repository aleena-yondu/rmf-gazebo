# IMU Bias Estimator — Design Notes and Debug Guide

> Source: `src/r1_nav_cpp/src/online_imu_bias_estimator.cpp`
> Header: `src/r1_nav_cpp/include/r1_nav_cpp/online_imu_bias_estimator.hpp`
> Parameters: `config/nav_configs/imu_bias_estimator_params.yaml`
> Calibration scripts: `scripts/calibrate_rotation_bias.py` (single bag), `scripts/calibrate_rotation_bias_multi.py` (multi-speed)
> Last updated: 2026-03-17

---

## Current Status (Checkpoint 2026-03-17)

### Resolved
- Static gyro z bias (EMA, -0.009294 rad/s) correctly estimated
- Dynamic rotation bias corrected via quadratic model (10-bag × 5-speed multi calibration)
- EKF confirmed using `/imu/data_bias_corrected` (ekf_with_ouster_imu.yaml imu0 field)
- Nearby objects (< ~3 m) no longer drift in map during in-place rotation ✓

### Remaining Issue
- Distant objects (> ~5 m) still drift during in-place rotation

### Root Cause Analysis (2026-03-17)
**Most likely cause: missing within-scan LiDAR deskewing.**

Current pipeline:
```
/ouster/points → pointcloud_to_laserscan (target_frame: base_link) → /scan → slam_toolbox
```
`pointcloud_to_laserscan` applies a **single TF lookup** to the entire 100 ms point cloud — it is NOT per-point motion compensation.
At 1.0 rad/s rotation, the robot rotates ~0.1 rad over 100 ms. For a point at 10 m, this creates ~1 m positional error due to the time difference between early and late points in the same scan. At 2 m the same error is only ~0.2 m — directly explaining the distance-dependent drift pattern.

**IMU bias correction fixes inter-scan angular estimation (EKF); it cannot fix intra-scan point cloud distortion.**

### New Finding: Yaw Drift During Translation (2026-03-17)

**Symptom**: Rotational drift fixed, but translational motion now causes yaw drift that slam_toolbox cannot correct.

**Root Cause**: Mismatch between calibration methodology and code assumption.

The calibration script pre-subtracts the static bias before fitting:
```python
error = (gyro_z - stationary_bias_z) - ref_interp  # static bias removed first
```
So `quad_b0 = 0.001815 ≈ 0` — it does **NOT** contain the static bias component.

But the C++ code assumes "b0 already encodes static component" and sets static_bias_z to 0 during motion:
```cpp
double static_bias_z = stationary ? gyro_bias_[2] : 0.0;  // ← 0 during motion — wrong
```

Result: during translation (wz≈0), the static bias (≈-0.007 rad/s) is never corrected and flows entirely into the EKF, accumulating yaw drift.

At 0.5 m/s for 10 m (~20 s): accumulated yaw error ≈ 0.007 × 20 = **0.14 rad = 8°**, beyond slam_toolbox's scan match search window.

**Fix**: Always apply the frozen EMA, even during motion:
```cpp
// Change in onImu():
double static_bias_z = gyro_bias_[2];  // frozen during motion, always applied
```
All three regimes then work correctly:
- Stationary: gyro_bias_[2] updates via EMA; dynamic terms off → total = EMA ✓
- Translation (wz≈0): gyro_bias_[2] frozen + quad_b0 ≈ 0 → total ≈ static bias ✓
- Rotation (wz=±1.0): gyro_bias_[2] frozen + b_dynamic(wz) → full correction ✓

> **Important**: This fix is valid only if the calibration script continues to pre-subtract stationary_bias_z. The two must be used together.

### Next Steps (by priority)
1. **Fix the static bias bug** (one-line code change above)
2. Verify that translation drift disappears after fix
3. Check whether Ouster driver has IMU-based deskewing enabled (separate cause of far-object drift)
4. If deskewing not enabled: add a deskewing node before `pointcloud_to_laserscan`

---

## 1. Problem Background

**Goal**: Eliminate map drift in SLAM (slam_toolbox) during in-place rotation.

**Symptom**: Asymmetric accumulated angular error between CW (clockwise) and CCW (counter-clockwise) in-place rotation, causing SLAM position drift. Root cause: the Ouster IMU gyro z-axis has a velocity-dependent dynamic bias.

---

## 2. Sensor Error Model

```
wz_measured = wz_true + bias_static + bias_dynamic(wz) + noise
```

| Error Component | Description | Handled by |
|---|---|---|
| `bias_static` | Fixed offset, independent of motion | EMA average while stationary |
| `bias_dynamic(wz)` | Extra bias that varies with angular velocity (nonlinear) | Rotation calibration script, fits b(ω) model |
| `noise` | IMU random white noise (magnitude not yet measured; unrelated to stationary bias) | Cannot be removed; part of sensor spec |

### 2.1 Physical Origin of Dynamic Bias

- **Asymmetric term (k·ω)**: IMU mounting offset from robot rotation center causes g-sensitivity to centripetal acceleration; or CW/CCW motor current asymmetry induces vibration differences.
- **Symmetric term (k2·ω²)**: Centripetal acceleration magnitude is proportional to ω², independent of rotation direction.

---

## 3. Two-Regime Correction Strategy

The code uses two mutually exclusive regimes — stationary and moving — with different corrections:

```cpp
// online_imu_bias_estimator.cpp, onImu()

double static_bias_z  = stationary ? gyro_bias_[2] : 0.0;   // EMA, stationary only
double fixed_offset_z = 0.0;
double linear_corr_z  = 0.0;
double quad_corr_z    = 0.0;

if (!stationary && dynamic_bias_model_ != "none") {
    if (dynamic_bias_model_ == "linear") {
        fixed_offset_z = linear_b0_;
        linear_corr_z  = linear_k_ * wz;
    } else if (dynamic_bias_model_ == "quadratic") {
        fixed_offset_z = quad_b0_;
        linear_corr_z  = quad_k_ * wz;
        quad_corr_z    = quad_k2_ * wz * wz;
    }
}

double total_corr_z = static_bias_z + fixed_offset_z + linear_corr_z + quad_corr_z;
double wz_corrected = wz - total_corr_z;
```

| State | Correction used | Reason |
|---|---|---|
| Stationary | EMA bias (`gyro_bias_[2]`) | Zero-velocity average is most accurate; b0 already encodes the static component |
| Moving | b(ω) model (b0 + k·wz [+ k2·wz²]) | b0 encodes static component; cannot add EMA on top |

**Why not add both?** `b0` is the intercept of a fit on *moving* data, which inherently includes the static bias contribution. Adding the EMA on top during motion double-counts the static component, which makes CW/CCW asymmetry worse, not better.

---

## 4. Dynamic Bias Models

Controlled by the `dynamic_bias_model` parameter (choose one):

| Model | Formula | Parameters | Use when |
|---|---|---|---|
| `"none"` | No dynamic correction | — | Initial testing, rotation correction not needed |
| `"linear"` | b(ω) = linear_b0 + linear_k·ω | `linear_b0`, `linear_k` | R² is high (>0.80) |
| `"quadratic"` | b(ω) = quad_b0 + quad_k·ω + quad_k2·ω² | `quad_b0`, `quad_k`, `quad_k2` | **Recommended**; better when ω² term is significant |

> **Important**: The `linear` and `quadratic` models use **completely independent parameter sets**. Both come from fits on the same data but with different model forms — the values are NOT interchangeable.

### 4.1 Current Calibration Results (2026-03-17)

Calibration method: `calibrate_rotation_bias_multi.py`, 10 bags × 5 speeds (±0.3/0.5/0.7/1.0/1.2 rad/s), KISS-ICP reference, 11,106 steady-state samples total.

```yaml
# Linear model  (R²=0.11 — poor, not recommended)
linear_b0: -0.007262
linear_k:  -0.007444

# Quadratic model  (R²=0.34, quad_res MAE < 0.4 mrad/s — currently active)
quad_b0:  0.001815    # offset at ω=0  (rad/s)
quad_k:  -0.007437    # linear term    (CW-CCW asymmetry)
quad_k2: -0.016103    # quadratic term (centripetal g-sensitivity)
```

> R²=0.34 appears low, but the absolute MAE < 0.4 mrad/s is sufficient for SLAM. The low R² is mainly driven by the IMU random noise floor (magnitude to be measured from std of stationary data), not model quality.

---

## 5. Stationarity Detection and EMA

### 5.1 Stationarity Logic

```
stationary = (odom_v < lin_stationary_thresh) AND (|wz_raw| < ang_stationary_thresh)
```

```yaml
lin_stationary_thresh: 0.01    # m/s, linear velocity threshold
ang_stationary_thresh: 0.02    # rad/s, z-axis angular velocity threshold (tightened from 0.05 to exclude braking vibration)
use_z_only_stationary: true    # true = use only wz; false = use ||(wx,wy,wz)||
min_stationary_count:  150     # consecutive stationary frames required before EMA updates (~1.5s @ 100Hz)
```

**Why 150 frames (1.5 s)?** After stopping, the IMU still vibrates from mechanical settling. Waiting avoids including the braking transient in bias estimation.

### 5.2 EMA Update

```cpp
// Only when consec_stationary_ >= min_stationary_count_
gyro_bias_[2] = (1 - alpha) * gyro_bias_[2] + alpha * wz_raw;
```

- `alpha = 0.01` (yaml default), time constant ~100 frames ≈ 1 s
- Initial value: `initial_gyro_bias = [0.0, 0.0, -0.009294]` (measured 2026-03-16, mean of 1605 samples)

### 5.3 Warmup Phase

The first `warmup_samples=500` frames (~5 s @ 100 Hz) after startup compute a running arithmetic mean to seed the EMA. This avoids large initial error when the node starts cold (bias initially 0).

---

## 6. Calibration Workflow

### 6.1 Single-bag Calibration (`calibrate_rotation_bias.py`)

Suitable for a quick check or when only 1–2 speed levels are available.

```bash
# Record a bag (both CW and CCW in one bag, or separate bags)
ros2 bag record /ouster/imu /ouster/points -o ~/calib/bag_rotation

# Run with KISS-ICP reference (recommended)
python3 scripts/calibrate_rotation_bias.py \
    --bag ~/calib/bag_rotation \
    --stationary-bias-z -0.009294

# Separate bags per direction
python3 scripts/calibrate_rotation_bias.py \
    --bag-cw  ~/calib/bag_cw \
    --bag-ccw ~/calib/bag_ccw

# Fall back to wheel odom (less accurate during in-place rotation)
python3 scripts/calibrate_rotation_bias.py \
    --bag ~/calib/bag_rotation --use-odom
```

**Outputs**:
- `b_cw`, `b_ccw`: mean residual bias for each direction
- `b0`, `k`: linear fit `b(ω) = b0 + k·ω`
- `c_cent`: centripetal-only fit `b(ω) = c·ω²` (useful if CW/CCW are symmetric)
- Ready-to-paste YAML snippet

**Filter parameters**:
```
--w-min     0.20   # min |ω| to accept as steady rotation (rad/s)
--w-dot-max 0.10   # max |dω/dt| to accept as steady (rad/s²)
```

### 6.2 Multi-Speed Calibration (`calibrate_rotation_bias_multi.py`) — Recommended

Pools data from multiple speed bags for a global least-squares fit. Outputs both linear and quadratic models with R², and automatically recommends one.

```bash
# Record one bag per speed/direction (naming convention: rot_{speed}_{cw|ccw})
# Speeds: 0.3, 0.5, 0.7, 1.0, 1.2 rad/s — one bag each for CW and CCW
ros2 bag record /ouster/imu /ouster/points -o ~/calib_bags/rot_0_3_cw &
# Command the robot to rotate at 0.3 rad/s CW for 15-20 s
# ... repeat for other speeds ...

# Auto-discover bags (directory contains rot_*_cw / rot_*_ccw)
python3 scripts/calibrate_rotation_bias_multi.py \
    --bag-dir ~/calib_bags/20260317 \
    --stationary-bias-z -0.009294 \
    --save result.json

# Or specify bags and speeds explicitly
python3 scripts/calibrate_rotation_bias_multi.py \
    --bags rot_0_3_cw rot_0_5_cw rot_1_0_ccw rot_0_5_ccw \
    --speeds -0.3 -0.5 1.0 0.5 \
    --save result.json
```

**R² interpretation**:

| R² | Quality | Action |
|---|---|---|
| ≥ 0.90 | Good | Use the model directly |
| 0.70–0.89 | Moderate | Inspect per-bag residuals for outliers |
| < 0.70 | Poor | Possible temperature dependency or hardware issue; investigate |

The script recommends `quadratic` over `linear` when `R²(quadratic) - R²(linear) > 0.05`.

**Example YAML output**:
```
# Linear model (dynamic_bias_model: linear)
linear_b0: -0.007262
linear_k:  -0.007444
# R²=0.11

# Quadratic model (dynamic_bias_model: quadratic)  ← RECOMMENDED
quad_b0:  0.001815
quad_k:  -0.007437
quad_k2: -0.016103
# R²=0.34
```

### 6.3 Prerequisites

- `--stationary-bias-z` must match the current `initial_gyro_bias[2]` in the yaml (the known static bias). The calibration script subtracts this first, then fits only the dynamic residual.
- If the static bias is not measured accurately, the dynamic model's `b0` will absorb the error and become less interpretable.

---

## 7. Debug Log Reference

### 7.1 Log Format

Throttled to one message every 500 ms (`RCLCPP_INFO_THROTTLE`):

```
[z-bias|quadratic] raw_wz=X | ema=A b0=B k*wz=C k2*wz²=D total=E | corrected=F | STA/MOV consec=M v=V
```

| Field | Meaning |
|---|---|
| `raw_wz` | Raw IMU wz (uncorrected) |
| `ema` | EMA bias used when stationary (0.0 while moving) |
| `b0` | Dynamic model offset (0.0 while stationary) |
| `k*wz` | Dynamic model linear term (0.0 while stationary) |
| `k2*wz²` | Dynamic model quadratic term (0.0 while stationary or in linear mode) |
| `total` | Total correction = ema + b0 + k·wz + k2·wz² |
| `corrected` | Corrected wz = raw\_wz − total |
| `STA/MOV` | Current stationarity state |
| `consec` | Consecutive stationary frames (0 while moving) |
| `v` | Odometry linear velocity (m/s) |

### 7.2 Normal Stationary Segment

```
[z-bias|quadratic] raw_wz=-0.00692 | ema=-0.00686 b0=0.00000 k*wz=0.00000 k2*wz²=0.00000 total=-0.00686 | corrected=-0.00007 | STA consec=150 v=0.000
```

- `b0=0, k*wz=0, k2*wz²=0`: dynamic terms fully off ✓
- `corrected ≈ 0`: EMA has converged ✓
- `consec=150`: meets `min_stationary_count`, EMA is updating ✓

**FAQ: Why is `corrected` not exactly zero when stationary?**
This is normal. The EMA is a weighted average of past samples, while each new sample contains random noise. The difference between the current raw sample and the EMA mean is the residual in `corrected`. Once the EMA converges, `corrected` should fluctuate randomly near zero — not a bug. (Note: -0.009294 is the *stationary bias mean*, not the white noise amplitude — these are entirely different quantities.)

### 7.3 Normal Rotation Segment (quadratic model)

```
# CW (~-1.0 rad/s)
[z-bias|quadratic] raw_wz=-1.007 | ema=0.00000 b0=0.001815 k*wz=0.007437 k2*wz²=-0.016103 total=-0.006851 | corrected=-1.000

# CCW (~+1.0 rad/s)
[z-bias|quadratic] raw_wz=+0.997 | ema=0.00000 b0=0.001815 k*wz=-0.007437 k2*wz²=-0.016103 total=-0.021725 | corrected=+1.019
```

- `ema=0`: EMA term disabled while moving ✓
- `b0`, `k*wz`, `k2*wz²` all non-zero: dynamic model active ✓

### 7.4 Status Log (every 3 seconds)

```
stationary=true consec=150/150 | gbias=[0.0102, -0.0319, -0.0073]
```

- `consec=150/150`: stationarity condition met, EMA updating
- `gbias`: current EMA bias values for x/y/z axes

### 7.5 Common Abnormal Patterns

| Symptom | Cause | Action |
|---|---|---|
| `consec` resets frequently while robot is stopped | IMU noise spike exceeds 0.02 rad/s threshold | Relax `ang_stationary_thresh` to 0.025 |
| Transient MOV frames when `odom_v≈0, raw_wz≈0` | Odom update rate slower than IMU, brief misjudgement | Normal; `min_stationary_count` absorbs this |
| EMA unchanged for a long time | Not reaching 150 consecutive stationary frames | Confirm robot is truly still; check for residual odom noise |
| Large `corrected` values during rotation | Calibration data insufficient, or actual speed outside calibrated range | Re-run multi-speed calibration with more speed levels |

---

## 8. Parameter Reference

```yaml
# imu_bias_estimator_params.yaml — annotated

# ── Topic configuration ───────────────────────────────────────────
input_topic:  "/ouster/imu"
output_topic: "/imu/data_bias_corrected"
odom_topic:   "/synchro_odom"    # set to "" to disable odometry gating

# ── Stationarity detection ────────────────────────────────────────
lin_stationary_thresh: 0.01    # m/s, linear velocity threshold
ang_stationary_thresh: 0.02    # rad/s, z-axis angular velocity threshold
use_z_only_stationary: true    # true = use only wz; false = use ||(wx,wy,wz)||
min_stationary_count:  150     # consecutive frames required before EMA updates (~1.5s @ 100Hz)

# ── EMA parameters ────────────────────────────────────────────────
alpha:        0.01             # EMA learning rate (time constant ≈ 1/alpha frames)
warmup_samples: 500            # cold-start accumulation frames (~5s @ 100Hz)
initial_gyro_bias: [0.0, 0.0, -0.009294]  # seed value (measured 2026-03-16, 1605 samples)

# ── Dynamic bias model ────────────────────────────────────────────
dynamic_bias_model: "quadratic"  # "none" | "linear" | "quadratic"

# Linear parameters (R²=0.11, not recommended)
linear_b0: -0.007262
linear_k:  -0.007444

# Quadratic parameters (R²=0.34, currently active)
quad_b0:  0.001815
quad_k:  -0.007437
quad_k2: -0.016103
```

> `leak_rate` appears in the yaml comments but is **not implemented** in C++ — it is currently ignored.

---

## 9. Multi-Robot Deployment

### Strategy A — Independent calibration per robot (most accurate)

1. After setup, robot automatically rotates through 5 speed levels in an open area
2. Script computes b0/k/k2 and writes to each robot's yaml
3. Best for robots with individual sensor installation differences

### Strategy B — Shared dynamic params, per-robot EMA static bias (simpler)

- `quad_k` / `quad_k2` fixed at fleet-wide values (averaged across multiple robots)
- `quad_b0` set to a typical value or near 0 (let the EMA absorb per-robot offset)
- Each robot only needs to remain stationary for ~5 s at startup; EMA converges automatically

> Empirical rule: `k` and `k2` are hardware-determined and vary little across same-model units. `b0` varies more (mounting angle + individual temperature drift). Prefer sharing k/k2 and letting EMA handle the b0-level variation.

---

## 10. Change History

| Date | Change | Reason |
|---|---|---|
| 2026-03-16 | Set `initial_gyro_bias[2] = -0.009294` | 1605-sample stationary mean measurement |
| 2026-03-17 | Added `calibrate_rotation_bias.py` (single bag) | Quick two-point linear fit |
| 2026-03-17 | Added `calibrate_rotation_bias_multi.py` (multi-speed) | Global least-squares fit; supports linear + quadratic with R² |
| 2026-03-17 | C++ implements `dynamic_bias_model` (none/linear/quadratic) | Replaces old single `rotation_bias_b0/k` parameter group |
| 2026-03-17 | Parameters split into `linear_b0/k` and `quad_b0/k/k2` — two independent sets | The two model parameter sets are not interchangeable |
| 2026-03-17 | Switched to `dynamic_bias_model: quadratic` using 10-bag × 5-speed multi calibration | Quadratic R² superior to linear; MAE < 0.4 mrad/s |
| 2026-03-17 | Fixed: moving phase uses b(ω) only; stationary phase uses EMA only | Eliminates double-counting of static bias; fixes CW/CCW asymmetry |
| 2026-03-17 | Added `RCLCPP_INFO_THROTTLE` decomposing each correction term (ema/b0/k·wz/k2·wz²) | Visual verification of correction components |

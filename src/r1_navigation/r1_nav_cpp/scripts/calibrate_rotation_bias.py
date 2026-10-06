#!/usr/bin/env python3
"""
Gyro Rotation Bias Calibration
================================
Estimates the gyro z-axis bias that appears *during rotation*, caused by:
  - IMU not being at the robot's rotation center (centripetal g-sensitivity)
  - Motor current / vibration asymmetry between CW and CCW

Reference source: KISS-ICP on /ouster/points  (default, recommended)
  - Accurate even when wheels slip
  - Directly measures LiDAR frame rotation
Fallback:        wheel odom /synchro_odom  (--use-odom flag)
  - Faster to compute, but unreliable during in-place rotation

Outputs
-------
  b_cw   : mean bias during clockwise rotation      (rad/s)
  b_ccw  : mean bias during counter-clockwise rotation (rad/s)
  b0, k  : linear model  b(ω) = b0 + k·ω  (least-squares fit)

Usage
-----
  # 1. Record a bag during in-place rotation (~0.3-0.8 rad/s, smooth)
  ros2 bag record /ouster/imu /ouster/points -o ~/calib/bag_rotation

  # 2. Run (KISS-ICP reference, recommended)
  python3 calibrate_rotation_bias.py --bag ~/calib/bag_rotation

  # 3. Separate bags per direction (optional)
  python3 calibrate_rotation_bias.py --bag-cw ~/calib/bag_cw --bag-ccw ~/calib/bag_ccw

  # 4. Fall back to wheel odom if LiDAR not available
  python3 calibrate_rotation_bias.py --bag ~/calib/bag_rotation --use-odom
"""

import argparse
import json
import math
import sys

import numpy as np
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry

# ─────────────────────────────────────────────
# Bag helpers
# ─────────────────────────────────────────────

_CDR_OPTS = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr",
)


def _open_bag(bag_path: str, topics: list) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        _CDR_OPTS,
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def _check_topics(bag_path: str, required: list):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        _CDR_OPTS,
    )
    available = {t.name for t in reader.get_all_topics_and_types()}
    missing = [t for t in required if t not in available]
    if missing:
        print(f"[error] missing topics in bag: {missing}")
        print(f"        available: {sorted(available)}")
        sys.exit(1)


# ─────────────────────────────────────────────
# Data loading
# ─────────────────────────────────────────────

def _load_imu(bag_path: str, imu_topic: str):
    imu_t, gyro_z = [], []
    reader = _open_bag(bag_path, [imu_topic])
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg: Imu = deserialize_message(raw, Imu)
        imu_t.append(t_ns * 1e-9)
        gyro_z.append(msg.angular_velocity.z)
    return np.array(imu_t), np.array(gyro_z)


def _load_lidar_frames(bag_path: str, lidar_topic: str, max_range: float):
    """Returns list of (timestamp_sec, Nx3 float64 array, N float64 timestamps_norm).

    timestamps_norm: per-point acquisition time normalized to [0, 1] within the scan,
    derived from the Ouster 't' field (uint32, ns from scan start).
    Falls back to zeros if 't' field is absent.
    """
    frames = []
    reader = _open_bag(bag_path, [lidar_topic])
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg: PointCloud2 = deserialize_message(raw, PointCloud2)

        # Check whether the 't' field (Ouster per-point timestamp) is available
        has_t = any(f.name == "t" for f in msg.fields)
        if has_t:
            raw_pts = list(pc2.read_points(msg, field_names=("x", "y", "z", "t"),
                                           skip_nans=True))
            if not raw_pts:
                continue
            pts  = np.array([[p[0], p[1], p[2]] for p in raw_pts], dtype=np.float64)
            t_field = np.array([p[3] for p in raw_pts], dtype=np.float64)
        else:
            raw_pts = list(pc2.read_points(msg, field_names=("x", "y", "z"),
                                           skip_nans=True))
            if not raw_pts:
                continue
            pts     = np.array(raw_pts, dtype=np.float64)
            t_field = np.zeros(len(pts), dtype=np.float64)

        if len(pts) < 100:
            continue
        r    = np.linalg.norm(pts, axis=1)
        mask = (r > 0.3) & (r < max_range)
        pts     = pts[mask]
        t_field = t_field[mask]

        if len(pts) < 100:
            continue

        # Normalize per-point timestamps to [0, 1] for KISS-ICP deskew
        t_max = t_field.max()
        ts_norm = (t_field / t_max) if t_max > 0 else np.zeros(len(pts))

        frames.append((t_ns * 1e-9, pts, ts_norm))
    return frames


def _load_odom(bag_path: str, odom_topic: str):
    odom_t, odom_w = [], []
    reader = _open_bag(bag_path, [odom_topic])
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        msg: Odometry = deserialize_message(raw, Odometry)
        odom_t.append(t_ns * 1e-9)
        odom_w.append(msg.twist.twist.angular.z)
    return np.array(odom_t), np.array(odom_w)


# ─────────────────────────────────────────────
# KISS-ICP reference
# ─────────────────────────────────────────────

def _run_kiss_icp(frames_np: list, timestamps_np: list, max_range: float) -> list:
    try:
        from kiss_icp.kiss_icp import KissICP
        from kiss_icp.config import KISSConfig
    except ImportError:
        print("[error] kiss-icp not installed.  pip install kiss-icp")
        sys.exit(1)

    has_timestamps = any(ts.max() > 0 for ts in timestamps_np if len(ts) > 0)
    cfg = KISSConfig()
    cfg.data.max_range     = max_range
    cfg.data.min_range     = 0.3
    cfg.data.deskew        = has_timestamps  # enable only when per-point timestamps exist
    cfg.mapping.voxel_size = 0.5

    if has_timestamps:
        print("[icp] deskew=True (Ouster 't' field found)")
    else:
        print("[icp] deskew=False (no per-point timestamps)")

    odometry = KissICP(config=cfg)
    poses = [np.eye(4)]
    for pts, ts in zip(frames_np, timestamps_np):
        odometry.register_frame(pts, timestamps=ts)
        poses.append(odometry.last_pose.copy())
    return poses


def _yaw_from_pose(T: np.ndarray) -> float:
    return math.atan2(T[1, 0], T[0, 0])


def _icp_yaw_rate(frames: list, max_range: float):
    """
    Run KISS-ICP on frames, differentiate yaw to get angular rate.

    Returns:
        ref_t : (M,) timestamps (seconds)  — one per LiDAR frame
        ref_w : (M,) yaw rate  (rad/s)
    """
    timestamps = np.array([f[0] for f in frames])
    pts_list   = [f[1] for f in frames]
    ts_list    = [f[2] for f in frames]

    print(f"[icp] running KISS-ICP on {len(pts_list)} frames ...")
    poses = _run_kiss_icp(pts_list, ts_list, max_range)
    print(f"[icp] done")

    yaws = np.array([_yaw_from_pose(p) for p in poses[1:]])

    # Unwrap to avoid ±π jumps
    yaws = np.unwrap(yaws)

    # Central-difference derivative; dt between consecutive frames
    dt = np.diff(timestamps)
    dt = np.concatenate([[dt[0]], dt])          # pad to same length
    dyaw = np.gradient(yaws, timestamps)        # uses central diff internally

    # Outlier removal: ICP jump → huge yaw step → huge rate
    median_rate = np.median(np.abs(dyaw))
    good = np.abs(dyaw) < max(median_rate * 10.0, 5.0)
    if (~good).sum() > 0:
        print(f"[icp] removed {(~good).sum()} outlier frames (yaw rate spike)")
        dyaw[~good] = np.nan

    return timestamps, dyaw


# ─────────────────────────────────────────────
# Alignment + filtering
# ─────────────────────────────────────────────

def _align(imu_t, gyro_z, ref_t, ref_w):
    """Interpolate reference yaw rate onto IMU timestamps."""
    # Drop NaN reference points before interpolation
    valid = np.isfinite(ref_w)
    ref_t_v = ref_t[valid]
    ref_w_v = ref_w[valid]

    t_start = max(imu_t[0], ref_t_v[0])
    t_end   = min(imu_t[-1], ref_t_v[-1])
    mask = (imu_t >= t_start) & (imu_t <= t_end)

    imu_t  = imu_t[mask]
    gyro_z = gyro_z[mask]
    ref_interp = np.interp(imu_t, ref_t_v, ref_w_v)
    return imu_t, gyro_z, ref_interp


def _steady_mask(ref_w, w_min: float, w_dot_max: float, imu_t):
    w_dot = np.abs(np.gradient(ref_w, imu_t))
    return (np.abs(ref_w) > w_min) & (w_dot < w_dot_max)


def _fit_linear(w_ref, error):
    """Fit  error = b0 + k * w_ref  via least squares."""
    A = np.column_stack([np.ones_like(w_ref), w_ref])
    result, _, _, _ = np.linalg.lstsq(A, error, rcond=None)
    return float(result[0]), float(result[1])


# ─────────────────────────────────────────────
# Core calibration
# ─────────────────────────────────────────────

def calibrate(bag_path: str,
              imu_topic: str,
              lidar_topic: str,
              odom_topic: str,
              use_odom: bool,
              max_range: float,
              w_min: float,
              w_dot_max: float,
              stationary_bias_z: float) -> dict:

    print(f"\n[load] bag: {bag_path}")

    if use_odom:
        _check_topics(bag_path, [imu_topic, odom_topic])
        imu_t,  gyro_z = _load_imu(bag_path, imu_topic)
        ref_t, ref_w   = _load_odom(bag_path, odom_topic)
        ref_label = "wheel odom"
        print(f"       IMU samples  : {len(imu_t)}")
        print(f"       Odom samples : {len(ref_t)}")
    else:
        _check_topics(bag_path, [imu_topic, lidar_topic])
        imu_t,  gyro_z = _load_imu(bag_path, imu_topic)
        frames         = _load_lidar_frames(bag_path, lidar_topic, max_range)
        ref_label = "KISS-ICP"
        print(f"       IMU samples    : {len(imu_t)}")
        print(f"       LiDAR frames   : {len(frames)}")
        if len(frames) < 10:
            print("[error] too few LiDAR frames")
            sys.exit(1)
        ref_t, ref_w = _icp_yaw_rate(frames, max_range)

    imu_t, gyro_z, ref_interp = _align(imu_t, gyro_z, ref_t, ref_w)

    # Residual after removing known stationary bias
    error = (gyro_z - stationary_bias_z) - ref_interp

    mask = _steady_mask(ref_interp, w_min, w_dot_max, imu_t)
    n_total  = len(imu_t)
    n_steady = int(mask.sum())
    print(f"\n[filter] reference: {ref_label}")
    print(f"         steady samples: {n_steady}/{n_total} "
          f"({100*n_steady/n_total:.1f}%)  "
          f"|ω|>{w_min:.2f} rad/s, |dω/dt|<{w_dot_max:.2f} rad/s²")

    if n_steady < 50:
        print("[error] too few steady-rotation samples — "
              "try slower --w-dot-max or record more rotation")
        sys.exit(1)

    e_steady = error[mask]
    w_steady = ref_interp[mask]

    cw_mask  = w_steady < 0
    ccw_mask = w_steady > 0
    b_cw  = float(np.mean(e_steady[cw_mask]))  if cw_mask.sum()  > 10 else float("nan")
    b_ccw = float(np.mean(e_steady[ccw_mask])) if ccw_mask.sum() > 10 else float("nan")

    b0, k = _fit_linear(w_steady, e_steady)
    c_fit = float(np.mean(e_steady / (w_steady**2 + 1e-9)))

    return {
        "reference":          ref_label,
        "stationary_bias_z":  stationary_bias_z,
        "b_cw":               b_cw,
        "b_ccw":              b_ccw,
        "b0":                 b0,
        "k":                  k,
        "c_cent":             c_fit,
        "n_cw":               int(cw_mask.sum()),
        "n_ccw":              int(ccw_mask.sum()),
    }


# ─────────────────────────────────────────────
# Report
# ─────────────────────────────────────────────

def _print_report(r: dict):
    print("\n" + "="*58)
    print("  Gyro Rotation Bias Results")
    print("="*58)
    print(f"  Reference source          : {r['reference']}")
    print(f"  Stationary bias  (input)  : {r['stationary_bias_z']:+.6f} rad/s")
    print(f"  b_cw   (CW  residual)     : {r['b_cw']:+.6f} rad/s  (n={r['n_cw']})")
    print(f"  b_ccw  (CCW residual)     : {r['b_ccw']:+.6f} rad/s  (n={r['n_ccw']})")
    print()
    print("  Linear model   b(ω) = b0 + k·ω")
    print(f"    b0 = {r['b0']:+.6f} rad/s   (symmetric offset)")
    print(f"    k  = {r['k']:+.6f}          (asymmetric, g-sensitivity scale)")
    print()
    print("  Centripetal-only  b(ω) = c·ω²")
    print(f"    c  = {r['c_cent']:+.6f}          (if CW/CCW symmetric, prefer this)")
    print("="*58)

    asym = (abs(r['b_cw'] - r['b_ccw'])
            if not (np.isnan(r['b_cw']) or np.isnan(r['b_ccw'])) else float("nan"))
    print(f"\n  CW/CCW asymmetry : {asym:.6f} rad/s")
    if not np.isnan(asym):
        if asym > 0.003:
            print("  → significant asymmetry: use the linear model (b0 + k·ω)")
        else:
            print("  → small asymmetry: a single b0 offset is sufficient")

    print("\n  ── YAML snippet for imu_bias_estimator_params.yaml ─")
    print(f"  rotation_bias_b0: {r['b0']:.6f}")
    print(f"  rotation_bias_k:  {r['k']:.6f}")
    print(f"  # b_cw:  {r['b_cw']:.6f}")
    print(f"  # b_ccw: {r['b_ccw']:.6f}")


# ─────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Estimate gyro z rotation bias from a rosbag.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # KISS-ICP reference (recommended) — single bag, both directions
  python3 calibrate_rotation_bias.py --bag ~/calib/bag_rotation

  # KISS-ICP reference — separate bags per direction
  python3 calibrate_rotation_bias.py --bag-cw ~/calib/bag_cw --bag-ccw ~/calib/bag_ccw

  # Wheel odom fallback
  python3 calibrate_rotation_bias.py --bag ~/calib/bag_rotation --use-odom
        """)

    p.add_argument("--bag",       default=None, help="Bag with both CW and CCW")
    p.add_argument("--bag-cw",    default=None, help="CW-only bag")
    p.add_argument("--bag-ccw",   default=None, help="CCW-only bag")

    p.add_argument("--imu-topic",   default="/ouster/imu")
    p.add_argument("--lidar-topic", default="/ouster/points")
    p.add_argument("--odom-topic",  default="/synchro_odom")
    p.add_argument("--use-odom",    action="store_true",
                   help="Use wheel odom instead of KISS-ICP as reference")
    p.add_argument("--max-range",   type=float, default=30.0,
                   help="Max LiDAR range for ICP (m)")
    p.add_argument("--stationary-bias-z", type=float, default=-0.009294,
                   help="Known stationary z-bias from imu_bias_estimator_params.yaml")
    p.add_argument("--w-min",     type=float, default=0.20,
                   help="Min |yaw rate| to accept as steady rotation (rad/s)")
    p.add_argument("--w-dot-max", type=float, default=0.10,
                   help="Max |dω/dt| to accept as steady (rad/s²)")
    p.add_argument("--save",      default=None, help="Save JSON results to file")
    args = p.parse_args()

    if args.bag is None and args.bag_cw is None and args.bag_ccw is None:
        p.error("Provide --bag (combined) or --bag-cw / --bag-ccw (separate)")

    common = dict(
        imu_topic=args.imu_topic,
        lidar_topic=args.lidar_topic,
        odom_topic=args.odom_topic,
        use_odom=args.use_odom,
        max_range=args.max_range,
        w_min=args.w_min,
        w_dot_max=args.w_dot_max,
        stationary_bias_z=args.stationary_bias_z,
    )

    if args.bag:
        result = calibrate(bag_path=args.bag, **common)
    else:
        result_cw  = calibrate(bag_path=args.bag_cw,  **common) if args.bag_cw  else None
        result_ccw = calibrate(bag_path=args.bag_ccw, **common) if args.bag_ccw else None

        b_cw  = result_cw["b_cw"]   if result_cw  else float("nan")
        b_ccw = result_ccw["b_ccw"] if result_ccw else float("nan")
        n_cw  = result_cw["n_cw"]   if result_cw  else 0
        n_ccw = result_ccw["n_ccw"] if result_ccw else 0

        b0 = (b_cw + b_ccw) / 2 if not (np.isnan(b_cw) or np.isnan(b_ccw)) else (
             b_cw if not np.isnan(b_cw) else b_ccw)
        k  = (b_ccw - b_cw) / 2 if not (np.isnan(b_cw) or np.isnan(b_ccw)) else 0.0

        result = {
            "reference":         result_cw["reference"] if result_cw else result_ccw["reference"],
            "stationary_bias_z": args.stationary_bias_z,
            "b_cw":   b_cw,  "b_ccw":  b_ccw,
            "b0":     b0,    "k":       k,
            "c_cent": float("nan"),
            "n_cw":   n_cw,  "n_ccw":  n_ccw,
        }

    _print_report(result)

    if args.save:
        with open(args.save, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\n[saved] {args.save}")


if __name__ == "__main__":
    main()

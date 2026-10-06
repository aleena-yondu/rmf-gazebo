#!/usr/bin/env python3

"""
! DOES NOT MAKE SENSE
 KISS-ICP reference would fail.
"""

"""
Transient vs Steady-State Gyro Error Analysis
==============================================
Quantifies how much gyro error occurs during rotation START and STOP transients
compared to steady-state rotation, using KISS-ICP as ground truth.

Complements calibrate_rotation_bias_multi.py — does NOT modify steady-state
calibration; uses the same bag format and helper functions.

Usage
-----
  # Analyze all bags in a calibration run
  python3 analyze_transient_error.py \\
      --bag-dir ~/yondu_ws/calib/calib_bags/20260318_024012 \\
      --stationary-bias-z -0.00844

  # With current quad calibration applied (shows residuals after correction)
  python3 analyze_transient_error.py \\
      --bag-dir ~/yondu_ws/calib/calib_bags/20260318_024012 \\
      --stationary-bias-z -0.00844 \\
      --quad-b0 0.000935 --quad-k -0.006787 --quad-k2 -0.014978

  # Single bag
  python3 analyze_transient_error.py \\
      --bag ~/yondu_ws/calib/calib_bags/20260318_024012/rot_1_2_cw \\
      --stationary-bias-z -0.00844

Output
------
  Per-bag table:
    steady_mean  : mean error during constant-velocity rotation   [rad/s]
    steady_std   : std  error during constant-velocity rotation   [rad/s]
    transient_mean: mean error during acceleration/deceleration   [rad/s]
    transient_std : std  error during acceleration/deceleration   [rad/s]
    transient_integral: ∫ error dt over all transient windows     [rad]
                        — this is the yaw error per start+stop pair

  If transient_integral >> steady_mean * steady_duration, transients are
  the dominant error source and need separate modelling.
"""

import argparse
import os
import re
import sys

import numpy as np

# ── Import helpers from sibling script ───────────────────────────────────────

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

try:
    from calibrate_rotation_bias import (
        _load_imu,
        _load_lidar_frames,
        _icp_yaw_rate,
        _align,
        _steady_mask,
        _check_topics,
    )
except ImportError as e:
    print(f"[error] Cannot import calibrate_rotation_bias.py: {e}")
    print("        Make sure both scripts are in the same directory.")
    sys.exit(1)


# ── Bag discovery (same as multi script) ─────────────────────────────────────

def _discover_bags(bag_dir: str) -> list:
    pattern = re.compile(r"rot_(\d+_\d+)_(cw|ccw)$")
    entries = []
    for name in sorted(os.listdir(bag_dir)):
        m = pattern.match(name)
        if not m:
            continue
        speed_str = m.group(1).replace("_", ".")
        direction = m.group(2)
        omega = -float(speed_str) if direction == "cw" else float(speed_str)
        bag_path = os.path.join(bag_dir, name)
        if os.path.isdir(bag_path):
            entries.append((bag_path, omega, name))
    entries.sort(key=lambda x: (abs(x[1]), x[1]))
    return entries


# ── Transient mask ────────────────────────────────────────────────────────────

def _transient_mask(ref_w, imu_t, w_min: float, w_dot_max: float,
                    w_dot_min_transient: float = 0.3):
    """
    Returns a boolean mask selecting transient samples.

    Transient = robot IS rotating (|w| > w_min) AND angular rate is changing
    quickly (|dw/dt| >= w_dot_min_transient rad/s²).

    This is the complement of _steady_mask within the rotation window.

    Parameters
    ----------
    w_dot_min_transient : rad/s²
        Minimum |dω/dt| to be considered a transient.
        Default 0.3 rad/s² (empirical; ~0.3s to reach 1 rad/s → peak ~3 rad/s²).
    """
    w_dot = np.abs(np.gradient(ref_w, imu_t))
    rotating   = np.abs(ref_w) > w_min
    accelerating = w_dot >= w_dot_min_transient
    return rotating & accelerating


# ── Per-bag analysis ──────────────────────────────────────────────────────────

def _analyze_bag(bag_path, imu_topic, lidar_topic, max_range,
                 stationary_bias_z, w_min, w_dot_max,
                 quad_b0, quad_k, quad_k2,
                 w_dot_min_transient):
    """
    Returns dict with steady and transient error stats, or None on failure.
    """
    _check_topics(bag_path, [imu_topic, lidar_topic])

    imu_t, gyro_z = _load_imu(bag_path, imu_topic)
    frames = _load_lidar_frames(bag_path, lidar_topic, max_range)
    if len(frames) < 10:
        print(f"  [skip] too few LiDAR frames ({len(frames)})")
        return None

    ref_t, ref_w = _icp_yaw_rate(frames, max_range)
    imu_t, gyro_z, ref_interp = _align(imu_t, gyro_z, ref_t, ref_w)

    # Raw error after removing static bias
    error_raw = (gyro_z - stationary_bias_z) - ref_interp

    # Error after applying quad dynamic model
    if quad_b0 is not None:
        dyn_corr  = quad_b0 + quad_k * gyro_z + quad_k2 * gyro_z ** 2
        error_corrected = error_raw - dyn_corr
    else:
        error_corrected = None

    # Masks
    s_mask = _steady_mask(ref_interp, w_min, w_dot_max, imu_t)
    t_mask = _transient_mask(ref_interp, imu_t, w_min, w_dot_max,
                             w_dot_min_transient)
    stationary_mask = np.abs(ref_interp) <= w_min

    dt = np.median(np.diff(imu_t))  # nominal sample period

    def _stats(mask, err):
        if mask.sum() == 0:
            return dict(n=0, mean=np.nan, std=np.nan, integral=np.nan)
        e = err[mask]
        return dict(
            n=int(mask.sum()),
            mean=float(np.mean(e)),
            std=float(np.std(e)),
            integral=float(np.sum(e) * dt),   # ∫ error dt  [rad]
        )

    result = dict(
        steady_raw   = _stats(s_mask, error_raw),
        transient_raw= _stats(t_mask, error_raw),
        n_stationary = int(stationary_mask.sum()),
        n_total      = len(imu_t),
        dt           = float(dt),
    )

    if error_corrected is not None:
        result['steady_corrected']    = _stats(s_mask, error_corrected)
        result['transient_corrected'] = _stats(t_mask, error_corrected)

    # Duration breakdown
    result['duration_total']      = float(len(imu_t) * dt)
    result['duration_steady']     = float(s_mask.sum() * dt)
    result['duration_transient']  = float(t_mask.sum() * dt)
    result['duration_stationary'] = float(stationary_mask.sum() * dt)

    return result


# ── Printing ──────────────────────────────────────────────────────────────────

def _print_bag_result(name, omega, r, has_corrected):
    print(f"\n  {'─'*68}")
    print(f"  Bag: {name:30s}  (nominal ω = {omega:+.2f} rad/s)")
    print(f"  Duration: {r['duration_total']:.1f}s total | "
          f"{r['duration_steady']:.1f}s steady | "
          f"{r['duration_transient']:.1f}s transient | "
          f"{r['duration_stationary']:.1f}s stationary")

    sr = r['steady_raw']
    tr = r['transient_raw']
    print(f"\n  {'Phase':<12}  {'n':>5}  {'mean error':>12}  {'±std':>10}  {'∫error dt':>12}")
    print(f"  {'─'*60}")
    print(f"  {'steady':<12}  {sr['n']:>5}  {sr['mean']:>+12.5f}  "
          f"{sr['std']:>10.5f}  {sr['integral']:>+12.5f} rad")
    print(f"  {'transient':<12}  {tr['n']:>5}  {tr['mean']:>+12.5f}  "
          f"{tr['std']:>10.5f}  {tr['integral']:>+12.5f} rad")

    if has_corrected and 'steady_corrected' in r:
        sc = r['steady_corrected']
        tc = r['transient_corrected']
        print(f"\n  After quad model correction:")
        print(f"  {'steady':<12}  {sc['n']:>5}  {sc['mean']:>+12.5f}  "
              f"{sc['std']:>10.5f}  {sc['integral']:>+12.5f} rad")
        print(f"  {'transient':<12}  {tc['n']:>5}  {tc['mean']:>+12.5f}  "
              f"{tc['std']:>10.5f}  {tc['integral']:>+12.5f} rad")

    # Ratio: is transient error larger than steady?
    if abs(sr['mean']) > 1e-6 and not np.isnan(tr['mean']):
        ratio = abs(tr['mean']) / abs(sr['mean'])
        print(f"\n  |transient mean| / |steady mean| = {ratio:.1f}×")

    # Yaw error per rotation (assuming transient occurs at start+stop)
    if not np.isnan(tr['integral']):
        print(f"  Estimated yaw error per start+stop: "
              f"{np.degrees(abs(tr['integral']) * 2):.3f}°")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    grp = ap.add_mutually_exclusive_group(required=True)
    grp.add_argument("--bag-dir", help="Directory containing rot_*_cw/ccw subdirs")
    grp.add_argument("--bag",     help="Single bag path")

    ap.add_argument("--stationary-bias-z", type=float, required=True,
                    help="Static gyro z bias [rad/s] (from statusTick at rest)")
    ap.add_argument("--imu-topic",   default="/ouster/imu")
    ap.add_argument("--lidar-topic", default="/ouster/points")
    ap.add_argument("--max-range",   type=float, default=30.0)
    ap.add_argument("--w-min",       type=float, default=0.1,
                    help="Min |wz| to count as rotating [rad/s]")
    ap.add_argument("--w-dot-max",   type=float, default=0.5,
                    help="Max |dw/dt| for steady-state [rad/s²]")
    ap.add_argument("--w-dot-min-transient", type=float, default=0.3,
                    help="Min |dw/dt| to count as transient [rad/s²]")

    # Optional: apply current quad model to see corrected residuals
    ap.add_argument("--quad-b0",  type=float, default=None)
    ap.add_argument("--quad-k",   type=float, default=None)
    ap.add_argument("--quad-k2",  type=float, default=None)

    args = ap.parse_args()

    has_quad = (args.quad_b0 is not None and
                args.quad_k  is not None and
                args.quad_k2 is not None)

    # Collect bags
    if args.bag_dir:
        bags = _discover_bags(args.bag_dir)
        if not bags:
            print(f"[error] No rot_*_cw/ccw bags found in {args.bag_dir}")
            sys.exit(1)
        print(f"[discover] {len(bags)} bags in {args.bag_dir}")
    else:
        name = os.path.basename(args.bag.rstrip("/"))
        bags = [(args.bag, 0.0, name)]

    print(f"  stationary_bias_z = {args.stationary_bias_z}")
    print(f"  w_dot_min_transient = {args.w_dot_min_transient} rad/s²  "
          f"(samples with |dω/dt| ≥ this are transient)")
    if has_quad:
        print(f"  quad model: b0={args.quad_b0} k={args.quad_k} k2={args.quad_k2}")

    all_steady_integrals    = []
    all_transient_integrals = []

    for bag_path, omega, name in bags:
        print(f"\n[bag] {name}  (nominal ω = {omega:+.2f} rad/s)")
        r = _analyze_bag(
            bag_path,
            imu_topic=args.imu_topic,
            lidar_topic=args.lidar_topic,
            max_range=args.max_range,
            stationary_bias_z=args.stationary_bias_z,
            w_min=args.w_min,
            w_dot_max=args.w_dot_max,
            quad_b0=args.quad_b0 if has_quad else None,
            quad_k=args.quad_k  if has_quad else None,
            quad_k2=args.quad_k2 if has_quad else None,
            w_dot_min_transient=args.w_dot_min_transient,
        )
        if r is None:
            continue
        _print_bag_result(name, omega, r, has_quad)
        if not np.isnan(r['steady_raw']['integral']):
            all_steady_integrals.append(r['steady_raw']['integral'])
        if not np.isnan(r['transient_raw']['integral']):
            all_transient_integrals.append(r['transient_raw']['integral'])

    # Summary
    if len(bags) > 1 and all_transient_integrals:
        print(f"\n{'='*68}")
        print(f"  SUMMARY across {len(bags)} bags")
        print(f"{'='*68}")
        st = np.array(all_steady_integrals)
        tr = np.array(all_transient_integrals)
        print(f"  Steady    ∫error dt : mean={np.mean(st):+.4f} rad  "
              f"std={np.std(st):.4f} rad")
        print(f"  Transient ∫error dt : mean={np.mean(tr):+.4f} rad  "
              f"std={np.std(tr):.4f} rad")
        print(f"  Estimated yaw error/rotation (start+stop): "
              f"{np.degrees(abs(np.mean(tr)) * 2):.3f}° ± "
              f"{np.degrees(np.std(tr) * 2):.3f}°")
        print()


if __name__ == "__main__":
    main()

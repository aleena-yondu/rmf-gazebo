#!/usr/bin/env python3
"""
Multi-Speed Gyro Rotation Bias Calibration
============================================
Improves b0/k estimation by pooling data from multiple bags recorded at
different rotation speeds, then fitting a global least-squares linear model.

Companion to calibrate_rotation_bias.py — imports its helpers directly.
Does NOT modify that script.

Usage
-----
  # Auto-discover bags from record_rotation_calib.sh output
  python3 calibrate_rotation_bias_multi.py \\
      --bag-dir ~/calib_bags/20260317_103000

  # Explicit bags with nominal speeds (rad/s, negative = CW)
  python3 calibrate_rotation_bias_multi.py \\
      --bags rot_0_3_cw rot_0_5_cw rot_1_0_ccw rot_0_5_ccw \\
      --speeds -0.3 -0.5 1.0 0.5

  # Save results to JSON
  python3 calibrate_rotation_bias_multi.py \\
      --bag-dir ~/calib_bags/... --save result.json

Interpretation
--------------
  R² >= 0.90 : linear model b(ω) = b0 + k·ω is a good fit → use it
  R² >= 0.70 : moderate fit → watch per-speed residuals for outliers
  R² <  0.70 : linear model is poor → may need higher-order or temperature term

  If k ≈ 0 and per-speed residuals are consistent, a single b0 is enough.
"""

import argparse
import json
import os
import re
import sys

import numpy as np

# ── Import helpers from sibling script (no code duplication) ─────────────────

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)

try:
    from calibrate_rotation_bias import (
        _load_imu,
        _load_lidar_frames,
        _load_odom,
        _icp_yaw_rate,
        _align,
        _steady_mask,
        _fit_linear,
        _check_topics,
    )
except ImportError as e:
    print(f"[error] Cannot import calibrate_rotation_bias.py: {e}")
    print("        Make sure both scripts are in the same directory.")
    sys.exit(1)


# ── Bag discovery ─────────────────────────────────────────────────────────────

def _discover_bags(bag_dir: str) -> list:
    """
    Auto-discover bags named rot_{speed_str}_{cw|ccw} in bag_dir.
    Returns list of (bag_path, nominal_omega, display_name) sorted by speed.
    """
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
    # Sort: CW (negative) first per speed, then CCW
    entries.sort(key=lambda x: (abs(x[1]), x[1]))
    return entries


# ── Per-bag data extraction ───────────────────────────────────────────────────

def _extract_steady(bag_path, imu_topic, lidar_topic, odom_topic,
                    use_odom, max_range, w_min, w_dot_max, stationary_bias_z):
    """
    Extract (w_ref, error) arrays for steady-state rotation samples from one bag.

    error = (gyro_z - stationary_bias_z) - w_ref
          = the residual dynamic bias at this angular velocity

    Returns:
        w_steady : (N,) array of reference angular rates  [rad/s]
        e_steady : (N,) array of residual errors          [rad/s]
        n        : number of steady samples
        ref_label: 'KISS-ICP' or 'odom'
    """
    if use_odom:
        _check_topics(bag_path, [imu_topic, odom_topic])
        imu_t, gyro_z = _load_imu(bag_path, imu_topic)
        ref_t, ref_w  = _load_odom(bag_path, odom_topic)
        ref_label = "odom"
    else:
        _check_topics(bag_path, [imu_topic, lidar_topic])
        imu_t, gyro_z = _load_imu(bag_path, imu_topic)
        frames = _load_lidar_frames(bag_path, lidar_topic, max_range)
        if len(frames) < 10:
            return None, None, 0, "KISS-ICP"
        ref_t, ref_w = _icp_yaw_rate(frames, max_range)
        ref_label = "KISS-ICP"

    imu_t, gyro_z, ref_interp = _align(imu_t, gyro_z, ref_t, ref_w)

    # Residual: remove static bias, then subtract reference rate
    error = (gyro_z - stationary_bias_z) - ref_interp

    mask     = _steady_mask(ref_interp, w_min, w_dot_max, imu_t)
    w_steady = ref_interp[mask]
    e_steady = error[mask]

    return w_steady, e_steady, int(mask.sum()), ref_label


# ── Statistics ────────────────────────────────────────────────────────────────

def _r_squared(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = np.sum((y_true - y_pred) ** 2)
    ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
    return float(1.0 - ss_res / ss_tot) if ss_tot > 0 else 0.0


def _fit_quadratic(w: np.ndarray, e: np.ndarray):
    """
    Fit  e = b0 + k1·ω + k2·ω²  via least squares.
    Returns (b0, k1, k2).
    """
    A = np.column_stack([np.ones_like(w), w, w ** 2])
    result, _, _, _ = np.linalg.lstsq(A, e, rcond=None)
    return float(result[0]), float(result[1]), float(result[2])


def _confidence_intervals(X: np.ndarray, y: np.ndarray,
                           b0: float, k: float, confidence: float = 0.95):
    """
    OLS 95% confidence intervals on b0 and k.
    Requires scipy. Returns (b0_lo, b0_hi, k_lo, k_hi) or None if unavailable.
    """
    try:
        from scipy import stats
    except ImportError:
        return None

    n = len(y)
    if n < 3:
        return None

    y_pred    = b0 + k * X
    residuals = y - y_pred
    s2        = np.sum(residuals ** 2) / (n - 2)
    Xbar      = np.mean(X)
    Sxx       = np.sum((X - Xbar) ** 2)

    if Sxx == 0:
        return None

    se_k  = np.sqrt(s2 / Sxx)
    se_b0 = np.sqrt(s2 * (1.0 / n + Xbar ** 2 / Sxx))
    t     = stats.t.ppf((1.0 + confidence) / 2.0, df=n - 2)

    return (b0 - t * se_b0, b0 + t * se_b0,
            k  - t * se_k,  k  + t * se_k)


# ── Multi-bag fit ─────────────────────────────────────────────────────────────

def run_multi(bags_and_speeds: list,
              imu_topic: str, lidar_topic: str, odom_topic: str,
              use_odom: bool, max_range: float,
              w_min: float, w_dot_max: float,
              stationary_bias_z: float) -> dict:
    """
    Pool steady-state data from all bags and fit a global b(ω) = b0 + k·ω.

    Parameters
    ----------
    bags_and_speeds : list of (bag_path, nominal_omega, display_name)

    Returns
    -------
    result dict with global fit, per-bag stats, R², confidence intervals.
    """
    all_w  = []
    all_e  = []
    per_bag_stats = []

    for bag_path, nominal_omega, label in bags_and_speeds:
        print(f"\n[bag] {label}  (nominal ω = {nominal_omega:+.2f} rad/s)")

        w_s, e_s, n, ref = _extract_steady(
            bag_path, imu_topic, lidar_topic, odom_topic,
            use_odom, max_range, w_min, w_dot_max, stationary_bias_z,
        )

        if w_s is None or n < 20:
            print(f"  [skip] too few steady samples ({n})")
            per_bag_stats.append({
                "label":         label,
                "nominal_omega": nominal_omega,
                "n":             n,
                "mean_w":        None,
                "mean_error":    None,
                "std_error":     None,
                "skipped":       True,
            })
            continue

        mean_w = float(np.mean(w_s))
        mean_e = float(np.mean(e_s))
        std_e  = float(np.std(e_s))
        print(f"  reference      : {ref}")
        print(f"  steady samples : {n}")
        print(f"  mean ω         : {mean_w:+.4f} rad/s")
        print(f"  mean error     : {mean_e:+.6f} ± {std_e:.6f} rad/s")

        per_bag_stats.append({
            "label":         label,
            "nominal_omega": nominal_omega,
            "n":             n,
            "mean_w":        mean_w,
            "mean_error":    mean_e,
            "std_error":     std_e,
            "skipped":       False,
        })

        all_w.extend(w_s.tolist())
        all_e.extend(e_s.tolist())

    if len(all_w) < 50:
        print("\n[error] too few total steady samples across all bags.")
        print("        Check that bags contain actual rotation data.")
        sys.exit(1)

    all_w = np.array(all_w)
    all_e = np.array(all_e)

    # ── Linear fit ────────────────────────────────────────────────────────────
    b0_lin, k_lin   = _fit_linear(all_w, all_e)
    y_pred_lin      = b0_lin + k_lin * all_w
    r2_lin          = _r_squared(all_e, y_pred_lin)

    # ── Quadratic fit ─────────────────────────────────────────────────────────
    b0_q, k1_q, k2_q = _fit_quadratic(all_w, all_e)
    y_pred_q          = b0_q + k1_q * all_w + k2_q * all_w ** 2
    r2_q              = _r_squared(all_e, y_pred_q)

    # Pick recommended model
    recommended = "quadratic" if (r2_q - r2_lin) > 0.05 else "linear"

    # Per-bag residuals for recommended model
    for s in per_bag_stats:
        if not s["skipped"] and s["mean_w"] is not None:
            mw = s["mean_w"]
            s["lin_prediction"]  = b0_lin + k_lin * mw
            s["quad_prediction"] = b0_q + k1_q * mw + k2_q * mw ** 2
            s["lin_residual"]    = s["mean_error"] - s["lin_prediction"]
            s["quad_residual"]   = s["mean_error"] - s["quad_prediction"]
        else:
            s["lin_prediction"] = s["quad_prediction"] = None
            s["lin_residual"]   = s["quad_residual"]   = None

    ci_lin  = _confidence_intervals(all_w, all_e, b0_lin, k_lin)

    return {
        "n_total":           int(len(all_w)),
        "n_bags":            int(sum(1 for s in per_bag_stats if not s["skipped"])),
        "recommended":       recommended,
        # linear
        "b0":                float(b0_lin),
        "k":                 float(k_lin),
        "r2_linear":         float(r2_lin),
        "b0_ci_95":          list(ci_lin[:2]) if ci_lin else None,
        "k_ci_95":           list(ci_lin[2:]) if ci_lin else None,
        # quadratic
        "b0_q":              float(b0_q),
        "k1_q":              float(k1_q),
        "k2_q":              float(k2_q),
        "r2_quadratic":      float(r2_q),
        "stationary_bias_z": float(stationary_bias_z),
        "per_bag":           per_bag_stats,
    }


# ── Report ────────────────────────────────────────────────────────────────────

def _r2_label(r2: float) -> str:
    if r2 >= 0.90:
        return "✓ good"
    elif r2 >= 0.70:
        return "⚠ moderate"
    else:
        return "✗ poor"


def _print_report(r: dict):
    print("\n" + "=" * 66)
    print("  Multi-Speed Gyro Rotation Bias — Global Fit Results")
    print("=" * 66)
    print(f"  Bags used         : {r['n_bags']}  ({r['n_total']} steady samples)")
    print(f"  Stationary bias   : {r['stationary_bias_z']:+.6f} rad/s  (input)")
    rec = r["recommended"]
    print(f"  Recommended model : {rec.upper()}  ← use this one")
    print()

    # ── Linear ────────────────────────────────────────────────────────────────
    r2_lin = r["r2_linear"]
    print(f"  [LINEAR]  b(ω) = b0 + k·ω                   R² = {r2_lin:.4f}  {_r2_label(r2_lin)}")
    print(f"    b0 = {r['b0']:+.6f} rad/s")
    if r["b0_ci_95"]:
        print(f"         95% CI: [{r['b0_ci_95'][0]:+.6f}, {r['b0_ci_95'][1]:+.6f}]")
    print(f"    k  = {r['k']:+.6f}")
    if r["k_ci_95"]:
        print(f"         95% CI: [{r['k_ci_95'][0]:+.6f}, {r['k_ci_95'][1]:+.6f}]")
    print()

    # ── Quadratic ─────────────────────────────────────────────────────────────
    r2_q = r["r2_quadratic"]
    print(f"  [QUADRATIC]  b(ω) = b0 + k·ω + k2·ω²       R² = {r2_q:.4f}  {_r2_label(r2_q)}")
    print(f"    b0 = {r['b0_q']:+.6f} rad/s  (offset @ ω=0)")
    print(f"    k  = {r['k1_q']:+.6f}         (linear / CW-CCW asymmetry)")
    print(f"    k2 = {r['k2_q']:+.6f}         (quadratic / centripetal g-sensitivity)")
    delta_r2 = r2_q - r2_lin
    print(f"    ΔR² vs linear: {delta_r2:+.4f}")
    print()

    # ── Per-bag table ─────────────────────────────────────────────────────────
    print("  Per-bag statistics:")
    hdr = (f"  {'Bag':<24} {'ω':>7} {'error':>9} {'±std':>8}"
           f" {'lin_res':>9} {'quad_res':>9} {'n':>5}")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    for s in r["per_bag"]:
        if s["skipped"]:
            print(f"  {s['label']:<24}  SKIPPED")
            continue
        lr = f"{s['lin_residual']:+.5f}"  if s["lin_residual"]  is not None else "—"
        qr = f"{s['quad_residual']:+.5f}" if s["quad_residual"] is not None else "—"
        print(
            f"  {s['label']:<24}"
            f" {s['mean_w']:>+7.4f}"
            f" {s['mean_error']:>+9.5f}"
            f" {s['std_error']:>8.5f}"
            f" {lr:>9}"
            f" {qr:>9}"
            f" {s['n']:>5}"
        )

    print()
    print("  ── YAML snippets ─────────────────────────────────────────────────")
    print()
    print("  # Linear model (dynamic_bias_model: linear)")
    print(f"  rotation_bias_b0: {r['b0']:.6f}")
    print(f"  rotation_bias_k:  {r['k']:.6f}")
    print(f"  rotation_bias_k2: 0.0")
    print(f"  # R²={r2_lin:.4f}")
    print()
    print("  # Quadratic model (dynamic_bias_model: quadratic)  ← RECOMMENDED")
    print(f"  rotation_bias_b0: {r['b0_q']:.6f}")
    print(f"  rotation_bias_k:  {r['k1_q']:.6f}")
    print(f"  rotation_bias_k2: {r['k2_q']:.6f}")
    print(f"  # R²={r2_q:.4f}")
    print("=" * 66)


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Multi-speed gyro rotation bias calibration (global fit).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Auto-discover bags from record_rotation_calib.sh
  python3 calibrate_rotation_bias_multi.py \\
      --bag-dir ~/calib_bags/20260317_103000

  # Explicit bags (negative speed = CW)
  python3 calibrate_rotation_bias_multi.py \\
      --bags rot_0_3_cw rot_0_5_cw rot_1_0_cw rot_0_3_ccw rot_0_5_ccw rot_1_0_ccw \\
      --speeds -0.3 -0.5 -1.0 0.3 0.5 1.0

  # Save results
  python3 calibrate_rotation_bias_multi.py \\
      --bag-dir ~/calib_bags/... --save ~/calib_bags/result.json
        """,
    )

    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--bag-dir", default=None,
                     help="Directory with bags named rot_{speed}_{cw|ccw}/")
    src.add_argument("--bags", nargs="+", default=None,
                     help="Explicit bag paths")

    p.add_argument("--speeds", nargs="+", type=float, default=None,
                   help="Nominal ω (rad/s) for each --bags entry. Negative = CW.")

    p.add_argument("--imu-topic",   default="/ouster/imu")
    p.add_argument("--lidar-topic", default="/ouster/points")
    p.add_argument("--odom-topic",  default="/synchro_odom")
    p.add_argument("--use-odom",    action="store_true",
                   help="Use wheel odom instead of KISS-ICP")
    p.add_argument("--max-range",   type=float, default=30.0)
    p.add_argument("--stationary-bias-z", type=float, default=-0.009294,
                   help="Static gyro z bias from imu_bias_estimator_params.yaml")
    p.add_argument("--w-min",     type=float, default=0.20,
                   help="Min |ω| to accept as steady rotation (rad/s)")
    p.add_argument("--w-dot-max", type=float, default=0.10,
                   help="Max |dω/dt| to accept as steady (rad/s²)")
    p.add_argument("--save", default=None,
                   help="Save JSON results to this path")
    args = p.parse_args()

    # Build (bag_path, nominal_omega, label) list
    if args.bag_dir:
        bags_and_speeds = _discover_bags(args.bag_dir)
        if not bags_and_speeds:
            print(f"[error] No bags matching rot_*_{{cw,ccw}} found in: {args.bag_dir}")
            sys.exit(1)
        print(f"[discover] {len(bags_and_speeds)} bags found in {args.bag_dir}:")
        for bp, omega, name in bags_and_speeds:
            print(f"  {name:<30}  nominal ω = {omega:+.2f} rad/s")
    else:
        if args.speeds is None or len(args.speeds) != len(args.bags):
            p.error("--speeds must have the same number of entries as --bags")
        bags_and_speeds = [
            (b, w, os.path.basename(b.rstrip("/")))
            for b, w in zip(args.bags, args.speeds)
        ]

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

    result = run_multi(bags_and_speeds, **common)
    _print_report(result)

    if args.save:
        with open(args.save, "w") as f:
            json.dump(result, f, indent=2)
        print(f"\n[saved] {args.save}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Live IMU Yaw Rate Monitor
==========================
Subscribes to bias-corrected IMU and (optionally) wheel odom, groups samples
into speed buckets, and prints rolling statistics.

Drive the robot at different speeds to populate each bucket.
The "error vs odom" column shows how much the corrected gyro deviates from
wheel odometry — useful for spotting speed-dependent model errors.

Usage
-----
  ROS_DOMAIN_ID=40 python3 imu_yaw_monitor.py
  ROS_DOMAIN_ID=40 python3 imu_yaw_monitor.py --imu /imu/data_bias_corrected
  ROS_DOMAIN_ID=40 python3 imu_yaw_monitor.py --no-odom   # IMU only
  ROS_DOMAIN_ID=40 python3 imu_yaw_monitor.py --window 300

Press Ctrl+C to quit.  Press Enter to reset all accumulators.
"""

import argparse
import collections
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


# ── Speed buckets (rad/s, by |wz|) ───────────────────────────────────────────

BUCKETS = [
    (0.00, 0.05,  "static   "),
    (0.05, 0.30,  "slow     "),
    (0.30, 0.55,  "med-slow "),
    (0.55, 0.80,  "med-fast "),
    (0.80, 1.05,  "fast     "),
    (1.05, 1.35,  "v-fast   "),
    (1.35, 99.0,  "max      "),
]


def bucket_for(wz_abs):
    for lo, hi, label in BUCKETS:
        if lo <= wz_abs < hi:
            return label
    return BUCKETS[-1][2]


# ── Rolling statistics ────────────────────────────────────────────────────────

class RollingStats:
    def __init__(self, maxlen):
        self._wz   = collections.deque(maxlen=maxlen)
        self._err  = collections.deque(maxlen=maxlen)  # wz_imu - wz_ref

    def add(self, wz_imu, wz_ref=None):
        self._wz.append(wz_imu)
        if wz_ref is not None:
            self._err.append(wz_imu - wz_ref)

    def clear(self):
        self._wz.clear()
        self._err.clear()

    @property
    def n(self):
        return len(self._wz)

    def stats_wz(self):
        if not self._wz:
            return float('nan'), float('nan')
        d = list(self._wz)
        mean = sum(d) / len(d)
        std  = math.sqrt(sum((x - mean)**2 for x in d) / len(d)) if len(d) > 1 else 0.0
        return mean, std

    def stats_err(self):
        if not self._err:
            return float('nan'), float('nan')
        d = list(self._err)
        mean = sum(d) / len(d)
        std  = math.sqrt(sum((x - mean)**2 for x in d) / len(d)) if len(d) > 1 else 0.0
        return mean, std


# ── ROS2 node ────────────────────────────────────────────────────────────────

class YawMonitor(Node):
    def __init__(self, imu_topic, odom_topic, window):
        super().__init__("imu_yaw_monitor")

        self._lock      = threading.Lock()
        self._stats     = {b[2]: RollingStats(window) for b in BUCKETS}
        self._last_wz   = 0.0
        self._last_ref  = None
        self._last_time = time.time()
        self._has_odom  = odom_topic is not None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Imu, imu_topic, self._on_imu, sensor_qos)
        if odom_topic:
            self.create_subscription(Odometry, odom_topic, self._on_odom, sensor_qos)

        self.get_logger().info(
            f"Monitoring {imu_topic}"
            + (f" vs {odom_topic}" if odom_topic else " (no reference)")
        )

    def _on_odom(self, msg: Odometry):
        with self._lock:
            self._last_ref = msg.twist.twist.angular.z

    def _on_imu(self, msg: Imu):
        wz = msg.angular_velocity.z
        with self._lock:
            ref = self._last_ref
            self._last_wz = wz
            self._last_time = time.time()
            key = bucket_for(abs(wz))
            self._stats[key].add(wz, ref)

    def reset(self):
        with self._lock:
            for s in self._stats.values():
                s.clear()

    def snapshot(self):
        with self._lock:
            return {
                k: {
                    "n":        s.n,
                    "wz":       s.stats_wz(),
                    "err":      s.stats_err(),
                }
                for k, s in self._stats.items()
            }, self._last_wz, self._last_ref, self._has_odom


# ── Display ───────────────────────────────────────────────────────────────────

def _fmt(v, decimals=4):
    return f"{v:+.{decimals}f}" if math.isfinite(v) else "     —    "


def print_table(snap, last_wz, last_ref, has_odom):
    lines = []
    lines.append("\033[2J\033[H")  # clear screen

    lines.append("═" * 78)
    lines.append("  IMU Yaw Rate Monitor  (Ctrl+C quit | Enter reset)")
    lines.append(f"  live: wz_imu={last_wz:+.4f} rad/s"
                 + (f"  wz_ref={last_ref:+.4f} rad/s" if last_ref is not None else "  (no ref)"))
    lines.append("─" * 78)

    if has_odom:
        hdr = (f"  {'bucket':<12} {'n':>5}  {'mean_wz':>9}  {'±std':>8}"
               f"  {'err_mean':>9}  {'err_std':>8}  {'quality'}")
    else:
        hdr = (f"  {'bucket':<12} {'n':>5}  {'mean_wz':>9}  {'±std':>8}  {'quality'}")
    lines.append(hdr)
    lines.append("─" * 78)

    for lo, hi, label in BUCKETS:
        d = snap[label]
        n = d["n"]
        wz_mean, wz_std = d["wz"]
        err_mean, err_std = d["err"]

        # Quality indicator based on std
        if n == 0:
            qual = "  (no data)"
        elif wz_std < 0.005:
            qual = "  ✓ stable"
        elif wz_std < 0.015:
            qual = "  ~ ok"
        elif wz_std < 0.040:
            qual = "  ! noisy"
        else:
            qual = "  ✗ very noisy"

        # Highlight current bucket
        cur_label = bucket_for(abs(last_wz))
        prefix = "► " if label == cur_label else "  "

        if has_odom:
            row = (f"{prefix}{label:<12} {n:>5}  {_fmt(wz_mean):>9}  {_fmt(wz_std, 4):>8}"
                   f"  {_fmt(err_mean):>9}  {_fmt(err_std, 4):>8}{qual}")
        else:
            row = (f"{prefix}{label:<12} {n:>5}  {_fmt(wz_mean):>9}  {_fmt(wz_std, 4):>8}{qual}")
        lines.append(row)

    lines.append("─" * 78)
    if has_odom:
        lines.append("  err = wz_imu_corrected − wz_odom  (negative = imu under-reads)")
    lines.append("  ±std: <0.005 stable | <0.015 ok | <0.040 noisy | ≥0.040 unreliable")
    lines.append("═" * 78)

    print("\n".join(lines), end="", flush=True)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--imu",    default="/imu/data_bias_corrected")
    ap.add_argument("--odom",   default="/synchro_odom")
    ap.add_argument("--no-odom", action="store_true",
                    help="Disable odometry reference (IMU stats only)")
    ap.add_argument("--window", type=int, default=500,
                    help="Rolling window size per bucket (default 500 samples)")
    ap.add_argument("--rate",   type=float, default=2.0,
                    help="Display refresh rate Hz (default 2)")
    args = ap.parse_args()

    odom_topic = None if args.no_odom else args.odom

    rclpy.init()
    node = YawMonitor(args.imu, odom_topic, args.window)

    # Keyboard listener thread (Enter = reset)
    def _kbd():
        while True:
            try:
                input()
                node.reset()
                print("  [reset]", flush=True)
            except EOFError:
                break

    t = threading.Thread(target=_kbd, daemon=True)
    t.start()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    period = 1.0 / max(args.rate, 0.1)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=period)
            snap, last_wz, last_ref, has_odom = node.snapshot()
            print_table(snap, last_wz, last_ref, has_odom)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

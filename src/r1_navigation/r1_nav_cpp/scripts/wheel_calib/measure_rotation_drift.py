#!/usr/bin/env python3
"""
Measure CW/CCW yaw drift asymmetry.

Usage:
  1. Run script, keep robot stationary until "READY" appears
  2. Rotate robot CW (or CCW) N full circles at constant speed
  3. Stop and keep still — script prints accumulated yaw error
  4. Press Ctrl+C, repeat for the other direction

  ros2 run r1_nav_cpp measure_rotation_drift.py
  # or directly:
  ROS_DOMAIN_ID=40 python3 measure_rotation_drift.py --rotations 3
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


def euler_from_quaternion(x, y, z, w):
    """Return (roll, pitch, yaw) in radians."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)   # yaw only


def angle_diff(a, b):
    """Signed shortest difference a - b, wrapped to (-π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


class DriftMeter(Node):
    def __init__(self, target_rotations: int):
        super().__init__('rotation_drift_meter')
        self.target_rotations = target_rotations
        self.target_rad = target_rotations * 2 * math.pi

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        self.sub = self.create_subscription(Odometry, '/odom', self.cb, qos)

        self.yaw0         = None   # start yaw
        self.yaw_prev     = None   # previous sample (for unwrapping)
        self.yaw_accum    = 0.0    # unwrapped cumulative yaw
        self.done         = False

        self.get_logger().info("Waiting for /odom ... (keep robot STATIONARY)")

    # ------------------------------------------------------------------
    def cb(self, msg: Odometry):
        if self.done:
            return

        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        if self.yaw0 is None:
            self.yaw0      = yaw
            self.yaw_prev  = yaw
            self.get_logger().info(
                f"READY — start yaw = {math.degrees(yaw):.2f}°  "
                f"| target = {self.target_rotations} full rotation(s)")
            return

        # Unwrap: accumulate incremental changes
        delta = angle_diff(yaw, self.yaw_prev)
        self.yaw_accum += delta
        self.yaw_prev   = yaw

        rotations_done = abs(self.yaw_accum) / (2 * math.pi)

        # Progress every ~30° of accumulated rotation
        if int(abs(self.yaw_accum) / math.radians(30)) > \
           int((abs(self.yaw_accum) - abs(delta)) / math.radians(30)):
            direction = "CCW" if self.yaw_accum > 0 else "CW"
            self.get_logger().info(
                f"  {direction}  {math.degrees(self.yaw_accum):+.1f}°  "
                f"({rotations_done:.2f} / {self.target_rotations} rotations)")

        # Check completion
        if abs(self.yaw_accum) >= self.target_rad:
            self._report()

    # ------------------------------------------------------------------
    def _report(self):
        self.done = True
        direction  = "CCW" if self.yaw_accum > 0 else "CW"
        ideal_deg  = math.degrees(self.target_rad)
        actual_deg = math.degrees(abs(self.yaw_accum))
        error_deg  = actual_deg - ideal_deg          # positive = over-rotated
        error_rad  = math.radians(error_deg)

        # Estimate average wz bias during rotation
        # (we don't know duration here, but user can compare CW vs CCW error sign)
        print("\n" + "="*55)
        print(f"  Direction  : {direction}")
        print(f"  Rotations  : {self.target_rotations}")
        print(f"  Ideal      : {ideal_deg:.2f}°  ({math.radians(ideal_deg):.4f} rad)")
        print(f"  Actual     : {actual_deg:.2f}°  ({abs(self.yaw_accum):.4f} rad)")
        print(f"  Error      : {error_deg:+.2f}°  ({error_rad:+.4f} rad)")
        print(f"  (+ = over-rotated / - = under-rotated)")
        print("="*55)
        print("Rotate back to start or Ctrl+C to exit.\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rotations', type=int, default=3,
                        help='Number of full rotations to measure (default: 3)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = DriftMeter(args.rotations)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

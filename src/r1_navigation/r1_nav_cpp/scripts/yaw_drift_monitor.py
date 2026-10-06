#!/usr/bin/env python3
"""
Yaw drift monitor - logs map->base_link and odom->base_link yaw over time.
Outputs a CSV and prints a summary report at the end.

Usage:
    python3 yaw_drift_monitor.py [duration_seconds]
    python3 yaw_drift_monitor.py 300   # 5 minutes
    python3 yaw_drift_monitor.py 600   # 10 minutes
"""

import sys
import math
import time
import csv
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def quat_to_yaw_deg(q):
    """Convert quaternion to yaw in degrees."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


class YawDriftMonitor(Node):
    def __init__(self, duration_sec):
        super().__init__("yaw_drift_monitor")
        self.duration = duration_sec
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.records = []          # (t, map_yaw, odom_yaw, map_odom_diff)
        self.start_time = None
        self.map_yaw_start = None
        self.odom_yaw_start = None

        self.output_csv = f"/tmp/yaw_drift_{int(time.time())}.csv"

        print(f"\n{'='*60}")
        print(f"  Yaw Drift Monitor — {duration_sec}s test")
        print(f"  Output CSV: {self.output_csv}")
        print(f"  Keep robot STATIONARY for a clean baseline test")
        print(f"{'='*60}\n")
        print(f"{'Time(s)':>8}  {'map yaw(°)':>12}  {'odom yaw(°)':>12}  "
              f"{'map drift(°)':>13}  {'odom drift(°)':>14}")
        print("-" * 70)

        self.timer = self.create_timer(1.0, self.tick)

    def get_yaw(self, parent, child):
        try:
            t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return quat_to_yaw_deg(t.transform.rotation)
        except Exception:
            return None

    def tick(self):
        now = time.time()
        if self.start_time is None:
            self.start_time = now

        elapsed = now - self.start_time
        if elapsed > self.duration:
            self.finish()
            return

        map_yaw = self.get_yaw("map", "base_link")
        odom_yaw = self.get_yaw("odom", "base_link")

        if map_yaw is None or odom_yaw is None:
            print(f"  [{elapsed:5.0f}s] Waiting for transforms...")
            return

        if self.map_yaw_start is None:
            self.map_yaw_start = map_yaw
            self.odom_yaw_start = odom_yaw

        map_drift = map_yaw - self.map_yaw_start
        odom_drift = odom_yaw - self.odom_yaw_start
        diff = map_yaw - odom_yaw

        self.records.append((elapsed, map_yaw, odom_yaw, map_drift, odom_drift, diff))

        print(f"{elapsed:>8.0f}  {map_yaw:>12.4f}  {odom_yaw:>12.4f}  "
              f"{map_drift:>+13.4f}  {odom_drift:>+14.4f}")

    def finish(self):
        self.timer.cancel()

        if len(self.records) < 2:
            print("\nNot enough data collected.")
            rclpy.shutdown()
            return

        # Save CSV
        with open(self.output_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_s", "map_yaw_deg", "odom_yaw_deg",
                        "map_drift_deg", "odom_drift_deg", "map_odom_diff_deg"])
            w.writerows(self.records)

        # Summary
        duration = self.records[-1][0]
        map_drifts  = [r[3] for r in self.records]
        odom_drifts = [r[4] for r in self.records]
        diffs       = [r[5] for r in self.records]

        total_map_drift  = map_drifts[-1]
        total_odom_drift = odom_drifts[-1]
        map_rate  = total_map_drift  / (duration / 60.0)
        odom_rate = total_odom_drift / (duration / 60.0)
        max_map_excursion  = max(map_drifts,  key=abs)
        max_odom_excursion = max(odom_drifts, key=abs)

        print(f"\n{'='*60}")
        print(f"  SUMMARY — {duration:.0f}s ({duration/60:.1f} min)")
        print(f"{'='*60}")
        print(f"  map->base_link  (AMCL corrected)")
        print(f"    Total drift : {total_map_drift:+.4f}°")
        print(f"    Rate        : {map_rate:+.4f}°/min")
        print(f"    Max excursion: {max_map_excursion:+.4f}°")
        print(f"")
        print(f"  odom->base_link (raw EKF, no AMCL)")
        print(f"    Total drift : {total_odom_drift:+.4f}°")
        print(f"    Rate        : {odom_rate:+.4f}°/min")
        print(f"    Max excursion: {max_odom_excursion:+.4f}°")
        print(f"")
        print(f"  map-odom gap")
        print(f"    Initial     : {diffs[0]:+.4f}°")
        print(f"    Final       : {diffs[-1]:+.4f}°")
        print(f"    Max gap     : {max(diffs, key=abs):+.4f}°")
        print(f"")
        print(f"  VERDICT")
        if abs(map_rate) < 0.5:
            print(f"    map yaw: STABLE  ({abs(map_rate):.3f}°/min < 0.5 threshold)")
        else:
            print(f"    map yaw: DRIFTING ({abs(map_rate):.3f}°/min >= 0.5 threshold)")
        if abs(odom_rate) < 5.0:
            print(f"    odom yaw: ACCEPTABLE ({abs(odom_rate):.3f}°/min < 5.0 threshold)")
        else:
            print(f"    odom yaw: HIGH DRIFT ({abs(odom_rate):.3f}°/min >= 5.0 threshold)")
        print(f"{'='*60}")
        print(f"  CSV saved: {self.output_csv}")
        print(f"{'='*60}\n")

        rclpy.shutdown()


def main():
    duration = int(sys.argv[1]) if len(sys.argv) > 1 else 300
    rclpy.init()
    node = YawDriftMonitor(duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass


if __name__ == "__main__":
    main()

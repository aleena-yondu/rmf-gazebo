#!/usr/bin/env python3
"""
Publish synthetic PickTicket orders and trigger the custom Nav2 BT.

This is meant for diagnosing full-system navigation load. It publishes an order
on /teleop/order, sends a dummy NavigateToPose goal, and prints CPU/load plus
Nav2/BT warnings while the goal is active.

Examples:
  ros2 run r1_nav_cpp nav_order_stress_test.py
  ros2 run r1_nav_cpp nav_order_stress_test.py --cycle-totes
  ros2 run r1_nav_cpp nav_order_stress_test.py --cycle-totes --rows 4 --cols 4
"""

import argparse
import os
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from r1_interfaces.msg import PickTicket
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


ROSOUT_NAMES = (
    "bt_navigator",
    "controller_server",
    "planner_server",
    "global_costmap",
    "local_costmap",
    "slam_toolbox",
    "tag_detections_node",
)

ROSOUT_KEYWORDS = (
    "missed its desired rate",
    "tick rate",
    "failed",
    "fail",
    "aborting",
    "lethal",
    "optimizer",
    "transform",
    "localiz",
    "costmap",
)


@dataclass
class BinSpec:
    side: str
    row: int
    bay: int
    shelf: str
    bin_number: int

    @property
    def barcode(self) -> str:
        return (
            f"{self.side}{self.row}-{self.bay:02d}-"
            f"{self.shelf}-{self.bin_number:02d}"
        )


@dataclass
class ToteSpec:
    row: int
    col: int

    @property
    def barcode(self) -> str:
        return f"TOTE-R{self.row}-C{self.col}"


class CpuSampler:
    def __init__(self) -> None:
        self._last_total: Optional[int] = None
        self._last_idle: Optional[int] = None

    def sample(self) -> Tuple[Optional[float], str]:
        cpu_pct = self._sample_cpu_pct()
        load = " ".join(f"{v:.2f}" for v in os.getloadavg())
        return cpu_pct, load

    def _sample_cpu_pct(self) -> Optional[float]:
        try:
            with open("/proc/stat", "r", encoding="utf-8") as f:
                parts = f.readline().split()
        except OSError:
            return None

        if not parts or parts[0] != "cpu":
            return None

        values = [int(v) for v in parts[1:]]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        total = sum(values)

        if self._last_total is None:
            self._last_total = total
            self._last_idle = idle
            return None

        delta_total = total - self._last_total
        delta_idle = idle - (self._last_idle or 0)
        self._last_total = total
        self._last_idle = idle

        if delta_total <= 0:
            return None
        return 100.0 * (delta_total - delta_idle) / delta_total


class NavOrderStressTest(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("nav_order_stress_test")
        self.args = args
        self.cpu = CpuSampler()
        self.latest_rosout: List[str] = []

        order_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        rosout_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.order_pub = self.create_publisher(PickTicket, "/teleop/order", order_qos)
        self.rosout_sub = self.create_subscription(
            Log, "/rosout", self._on_rosout, rosout_qos
        )
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_rosout(self, msg: Log) -> None:
        name_match = any(name in msg.name for name in ROSOUT_NAMES)
        text = msg.msg.lower()
        keyword_match = any(keyword in text for keyword in ROSOUT_KEYWORDS)
        # ROS log levels: DEBUG=10, INFO=20, WARN=30, ERROR=40, FATAL=50.
        if int(msg.level) >= 30 or (name_match and keyword_match):
            line = f"{self._stamp(msg)} {msg.name}: {msg.msg}"
            self.latest_rosout.append(line)

    @staticmethod
    def _stamp(msg: Log) -> str:
        stamp = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        return f"{stamp:.3f}"

    def wait_for_nav_server(self) -> bool:
        print("Waiting for /navigate_to_pose action server...")
        return self.nav_client.wait_for_server(timeout_sec=10.0)

    def publish_order(self, bin_spec: BinSpec, tote_spec: ToteSpec, seq: int) -> None:
        msg = PickTicket()
        msg.robot_name = self.args.robot_name
        msg.order_id = f"stress-{int(time.time())}-{seq}"
        msg.order_number = msg.order_id
        msg.item_name = f"Stress Test {bin_spec.barcode} -> {tote_spec.barcode}"
        msg.sku = "STRESS-TEST"
        msg.quantity = 1
        msg.line_item_id = str(seq)
        msg.pick_token = msg.order_id

        msg.pick_arm = self.args.pick_arm
        msg.bin.side = bin_spec.side
        msg.bin.row = bin_spec.row
        msg.bin.bay = bin_spec.bay
        msg.bin.shelf = bin_spec.shelf
        msg.bin.bin_number = bin_spec.bin_number
        msg.bin_tag_txt = bin_spec.barcode

        msg.tote.row = tote_spec.row
        msg.tote.col = tote_spec.col
        msg.tote_name = tote_spec.barcode
        msg.tote_barcode = tote_spec.barcode
        msg.tote_tag_txt = tote_spec.barcode
        msg.cart_barcode = self.args.cart_barcode

        msg.batch_id = "nav-stress"
        msg.pick_number = seq
        msg.total_picks_in_order = 1
        msg.remaining_picks_in_batch = 0
        msg.total_orders_in_batch = 1
        msg.remaining_orders_in_batch = 0

        self.order_pub.publish(msg)
        print(f"Published order: bin={bin_spec.barcode} tote=({tote_spec.row},{tote_spec.col})")

    def clear_order(self) -> None:
        msg = PickTicket()
        msg.item_name = "NONE"
        self.order_pub.publish(msg)
        print("Published clear order: item_name=NONE")

    def send_nav_goal(self, seq: int) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.orientation.w = 1.0

        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            print(f"[{seq}] Timed out sending NavigateToPose goal")
            return False

        handle = send_future.result()
        if not handle.accepted:
            print(f"[{seq}] NavigateToPose goal rejected")
            return False

        print(f"[{seq}] NavigateToPose accepted; monitoring until result...")
        result_future = handle.get_result_async()
        start = time.monotonic()
        last_print = 0.0
        last_rosout_idx = len(self.latest_rosout)

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = time.monotonic() - start

            if elapsed - last_print >= self.args.status_period:
                last_print = elapsed
                cpu_pct, load = self.cpu.sample()
                cpu_text = "warming" if cpu_pct is None else f"{cpu_pct:5.1f}%"
                print(f"[{seq}] t={elapsed:6.1f}s cpu={cpu_text} load={load}")

                for line in self.latest_rosout[last_rosout_idx:]:
                    print(f"  rosout: {line}")
                last_rosout_idx = len(self.latest_rosout)

            if self.args.goal_timeout > 0.0 and elapsed > self.args.goal_timeout:
                print(f"[{seq}] Goal timeout; canceling active NavigateToPose")
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                return False

        result = result_future.result()
        elapsed = time.monotonic() - start
        print(f"[{seq}] result status={result.status} elapsed={elapsed:.1f}s")
        return result.status == 4


def prompt_bin() -> BinSpec:
    raw = input("Bin barcode [e.g. Y50-01-B-02]: ").strip().upper()
    if raw:
        try:
            side_row, bay, shelf, bin_number = raw.split("-")
            return BinSpec(
                side=side_row[0],
                row=int(side_row[1:]),
                bay=int(bay),
                shelf=shelf,
                bin_number=int(bin_number),
            )
        except (ValueError, IndexError) as exc:
            raise SystemExit(f"Invalid bin barcode: {raw}") from exc

    side = input("Bin side [X/Y] (default Y): ").strip().upper() or "Y"
    row = int(input("Bin row number (default 50): ").strip() or "50")
    bay = int(input("Bin bay (default 1): ").strip() or "1")
    shelf = input("Bin shelf [A/B/C] (default B): ").strip().upper() or "B"
    bin_number = int(input("Bin number (default 2): ").strip() or "2")
    return BinSpec(side=side, row=row, bay=bay, shelf=shelf, bin_number=bin_number)


def prompt_tote() -> ToteSpec:
    row = int(input("Tote row 0-3 (default 1): ").strip() or "1")
    col = int(input("Tote col 0-3 (default 1): ").strip() or "1")
    return ToteSpec(row=row, col=col)


def build_totes(args: argparse.Namespace) -> List[ToteSpec]:
    return [
        ToteSpec(row=row, col=col)
        for row in range(args.start_row, args.start_row + args.rows)
        for col in range(args.start_col, args.start_col + args.cols)
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycle-totes", action="store_true",
                        help="Cycle through a rows x cols tote grid.")
    parser.add_argument("--rows", type=int, default=4,
                        help="Number of tote rows to cycle.")
    parser.add_argument("--cols", type=int, default=4,
                        help="Number of tote columns to cycle.")
    parser.add_argument("--start-row", type=int, default=0)
    parser.add_argument("--start-col", type=int, default=0)
    parser.add_argument("--bin", dest="bin_barcode", default="Y50-01-B-02",
                        help="Bin barcode used for cycle mode.")
    parser.add_argument("--repeat", type=int, default=1,
                        help="Repeat the selected/cycled orders this many times.")
    parser.add_argument("--pause", type=float, default=2.0,
                        help="Seconds to pause between orders.")
    parser.add_argument("--goal-timeout", type=float, default=180.0,
                        help="Cancel any goal that runs longer than this. 0 disables.")
    parser.add_argument("--status-period", type=float, default=1.0,
                        help="Seconds between CPU/load status prints.")
    parser.add_argument("--robot-name", default="juan")
    parser.add_argument("--pick-arm", default="right")
    parser.add_argument("--cart-barcode", default="STRESS-CART")
    parser.add_argument("--frame", default="map",
                        help="Frame for dummy NavigateToPose goal.")
    parser.add_argument("--clear-at-end", action="store_true",
                        help="Publish item_name=NONE after the test.")
    return parser.parse_args()


def parse_bin_barcode(value: str) -> BinSpec:
    side_row, bay, shelf, bin_number = value.strip().upper().split("-")
    return BinSpec(
        side=side_row[0],
        row=int(side_row[1:]),
        bay=int(bay),
        shelf=shelf,
        bin_number=int(bin_number),
    )


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = NavOrderStressTest(args)

    if not node.wait_for_nav_server():
        print("ERROR: /navigate_to_pose action server not available")
        rclpy.shutdown()
        return 2

    if args.cycle_totes:
        bin_spec = parse_bin_barcode(args.bin_barcode)
        totes = build_totes(args)
    else:
        bin_spec = prompt_bin()
        totes = [prompt_tote()]

    print(f"Using bin {bin_spec.barcode}")
    print(f"Testing {len(totes)} tote target(s), repeat={args.repeat}")
    print("Press Ctrl-C to stop.")

    seq = 0
    try:
        for _ in range(args.repeat):
            for tote in totes:
                seq += 1
                node.publish_order(bin_spec, tote, seq)
                # Give the transient-local order a moment to reach BT subscribers.
                end = time.monotonic() + 0.5
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                node.send_nav_goal(seq)
                if args.pause > 0.0:
                    time.sleep(args.pause)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if args.clear_at_end:
            node.clear_order()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())

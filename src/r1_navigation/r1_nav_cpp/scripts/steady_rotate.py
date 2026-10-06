#!/usr/bin/env python3
"""
steady_rotate.py
================
Commands the robot to rotate at a precise, steady angular velocity with
smooth ramp-up and ramp-down. Designed for gyro bias calibration recording.

Modes
-----
  Interactive (no args):
      python3 steady_rotate.py
      → menu: pick speed, hold until ENTER, ramp down

  CLI / shell integration:
      python3 steady_rotate.py --speed -0.5 --duration 20
      → blocks for 20 s then exits cleanly; use with ros2 bag record

Examples
--------
  # Interactive
  python3 steady_rotate.py

  # CW at 0.5 rad/s for 20 s (negative = CW)
  python3 steady_rotate.py --speed -0.5 --duration 20

  # CCW at 1.0 rad/s, slow ramp (3 s)
  python3 steady_rotate.py --speed 1.0 --duration 20 --ramp-time 3.0
"""

import argparse
import math
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class SteadyRotator(Node):
    def __init__(self):
        super().__init__("steady_rotator")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/synchro_odom", self._odom_cb, 10
        )
        self.odom_w = 0.0       # latest angular.z from odom
        self.odom_x = 0.0       # latest x from odom (metres)
        self.odom_y = 0.0       # latest y from odom (metres)
        self._stop_flag = False  # set True to interrupt a running rotate()

    # ── internals ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.odom_w = msg.twist.twist.angular.z
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def _pub(self, w: float):
        t = Twist()
        t.angular.z = w
        self.cmd_pub.publish(t)

    def _spin(self, dt: float = 0.05):
        """Spin ROS2 callbacks for one timestep."""
        rclpy.spin_once(self, timeout_sec=dt)

    # ── public API ────────────────────────────────────────────────────────────

    def stop(self):
        """Interrupt any ongoing rotate() and send zero velocity."""
        self._stop_flag = True

    def hard_stop(self):
        """Send zero velocity immediately (no ramp)."""
        for _ in range(5):
            self._pub(0.0)
            self._spin(0.02)

    def rotate(
        self,
        speed: float,
        duration: float = 0.0,
        ramp_time: float = 2.0,
        on_steady=None,
        progress_cb=None,
    ) -> bool:
        """
        Ramp up → hold → ramp down.

        Parameters
        ----------
        speed      : target angular velocity (rad/s, negative = CW)
        duration   : hold time at steady speed (s); 0 = hold until stop() called
        ramp_time  : linear ramp duration (s)
        on_steady  : called once steady speed is reached (no args)
        progress_cb: called each control step (elapsed_s, duration_s)

        Returns True if completed normally, False if interrupted.
        """
        self._stop_flag = False
        dt = 0.05
        steps = max(1, int(ramp_time / dt))

        # ── ramp up ───────────────────────────────────────────────────────────
        for i in range(1, steps + 1):
            if self._stop_flag:
                self._ramp_down(speed * i / steps, steps - i, dt)
                return False
            self._pub(speed * i / steps)
            self._spin(dt)

        if on_steady:
            on_steady()

        # ── hold ──────────────────────────────────────────────────────────────
        t0 = time.time()
        while not self._stop_flag:
            self._pub(speed)
            self._spin(dt)
            elapsed = time.time() - t0
            if progress_cb:
                progress_cb(elapsed, duration)
            if duration > 0 and elapsed >= duration:
                break

        interrupted = self._stop_flag

        # ── ramp down ─────────────────────────────────────────────────────────
        self._ramp_down(speed, steps, dt)
        return not interrupted

    def _ramp_down(self, speed: float, steps: int, dt: float):
        for i in range(steps, -1, -1):
            self._pub(speed * i / max(steps, 1))
            self._spin(dt)
        self._pub(0.0)
        self._spin(dt)

    def translate(
        self,
        distance: float,
        speed: float = 0.15,
        ramp_time: float = 1.0,
    ) -> bool:
        """
        Move forward (+) or backward (-) by |distance| metres using odometry.

        Parameters
        ----------
        distance  : metres; positive = forward, negative = backward
        speed     : max linear speed (m/s)
        ramp_time : ramp-up / ramp-down duration (s)

        Returns True if target reached, False if interrupted.
        """
        self._stop_flag = False
        dt = 0.05
        steps = max(1, int(ramp_time / dt))
        sign = 1.0 if distance >= 0 else -1.0
        target = abs(distance)
        linear_speed = sign * abs(speed)

        # Wait up to 0.5 s for at least one odom message
        t_wait = time.time()
        while time.time() - t_wait < 0.5:
            self._spin(dt)

        x0, y0 = self.odom_x, self.odom_y

        def _pub_linear(v: float):
            t = Twist()
            t.linear.x = v
            self.cmd_pub.publish(t)

        # ramp up
        for i in range(1, steps + 1):
            if self._stop_flag:
                _pub_linear(0.0)
                return False
            _pub_linear(linear_speed * i / steps)
            self._spin(dt)

        # hold until distance reached
        while not self._stop_flag:
            _pub_linear(linear_speed)
            self._spin(dt)
            dx = self.odom_x - x0
            dy = self.odom_y - y0
            if math.sqrt(dx * dx + dy * dy) >= target:
                break

        interrupted = self._stop_flag

        # ramp down
        for i in range(steps, -1, -1):
            _pub_linear(linear_speed * i / max(steps, 1))
            self._spin(dt)
        _pub_linear(0.0)
        self._spin(dt)

        return not interrupted

    def slam_nudge(self, distance: float = 0.2, speed: float = 0.15, ramp_time: float = 1.0):
        """
        Move forward by `distance` m then back to origin — gives SLAM toolbox
        a translation segment to re-correct pose after a pure rotation.
        """
        print(f"[slam_nudge] forward {distance:.2f} m ...")
        self.translate(distance,  speed, ramp_time)
        time.sleep(0.3)
        print(f"[slam_nudge] backward {distance:.2f} m ...")
        self.translate(-distance, speed, ramp_time)
        print("[slam_nudge] Done.")


# ── TUI helpers ───────────────────────────────────────────────────────────────

def _bar(elapsed: float, duration: float, width: int = 30) -> str:
    if duration <= 0:
        return f"holding... {elapsed:6.1f}s elapsed"
    frac  = min(elapsed / duration, 1.0)
    done  = int(frac * width)
    left  = width - done
    pct   = frac * 100.0
    return f"|{'█' * done}{'░' * left}| {elapsed:5.1f}/{duration:.0f}s  {pct:5.1f}%"


def _read_float(prompt: str, default=None) -> float:
    while True:
        raw = input(prompt).strip()
        if raw == "" and default is not None:
            return default
        try:
            return float(raw)
        except ValueError:
            print("  Invalid number, try again.")


# ── Interactive mode ──────────────────────────────────────────────────────────

def run_interactive(node: SteadyRotator):
    print()
    print("=" * 54)
    print("  Steady Rotation Controller — Interactive Mode")
    print("=" * 54)
    print("  negative speed = CW (clockwise)")
    print("  positive speed = CCW (counter-clockwise)")
    print("  duration = 0 → hold until ENTER")
    print()

    while True:
        print("  ┌─────────────────────────────────┐")
        print("  │  [r] Rotate                     │")
        print("  │  [q] Quit                       │")
        print("  └─────────────────────────────────┘")
        cmd = input("  > ").strip().lower()

        if cmd == "q":
            print("  Exiting.")
            break

        if cmd != "r":
            continue

        speed     = _read_float("  Speed (rad/s, e.g. -0.5 for CW, 0.5 for CCW): ")
        duration  = _read_float("  Duration (s, 0 = hold until ENTER) [default 0]: ", default=0.0)
        ramp_time = _read_float("  Ramp time (s) [default 2.0]: ", default=2.0)

        direction = "CW" if speed < 0 else "CCW"
        print()
        print(f"  → {direction} @ {abs(speed):.3f} rad/s, "
              f"ramp {ramp_time:.1f}s, "
              f"{'hold until ENTER' if duration == 0 else f'hold {duration:.0f}s'}")
        print()

        # Spinner thread for "press ENTER to stop"
        enter_pressed = threading.Event()

        def _wait_enter():
            input("  Press ENTER to stop early...\n")
            enter_pressed.set()
            node.stop()

        t = threading.Thread(target=_wait_enter, daemon=True)

        def _on_steady():
            print(f"  ✓ Steady at {speed:+.3f} rad/s")
            print()
            if duration == 0:
                t.start()

        def _progress(elapsed, dur):
            sys.stdout.write(f"\r  {_bar(elapsed, dur)}  odom_w={node.odom_w:+.4f} rad/s  ")
            sys.stdout.flush()

        completed = node.rotate(
            speed=speed,
            duration=duration,
            ramp_time=ramp_time,
            on_steady=_on_steady,
            progress_cb=_progress,
        )

        enter_pressed.set()   # unblock thread if still waiting
        sys.stdout.write("\r" + " " * 70 + "\r")
        sys.stdout.flush()

        if completed:
            print(f"  ✓ Rotation complete.\n")
        else:
            print(f"  ⚠ Rotation stopped early.\n")

        nudge_dist = _read_float(
            "  SLAM nudge distance (m, 0 to skip) [default 0.2]: ", default=0.2
        )
        if nudge_dist > 0 and completed:
            time.sleep(0.5)
            node.slam_nudge(distance=nudge_dist)
            print()

        print(f"  !! Make sure robot is fully stopped before next move !!")
        input("  Press ENTER to confirm robot is stationary...\n")


# ── CLI mode (for shell integration) ─────────────────────────────────────────

def run_cli(node: SteadyRotator, speed: float, duration: float, ramp_time: float,
            post_nudge: float = 0.0):
    direction = "CW" if speed < 0 else "CCW"
    print(f"[steady_rotate] {direction} {abs(speed):.3f} rad/s  duration={duration:.0f}s  ramp={ramp_time:.1f}s")
    if post_nudge > 0:
        print(f"[steady_rotate] post-nudge: ±{post_nudge:.2f} m (SLAM correction)")
    print("[steady_rotate] Ramping up...")

    def _on_steady():
        print(f"[steady_rotate] Steady at {speed:+.3f} rad/s")

    def _progress(elapsed, dur):
        sys.stdout.write(f"\r[steady_rotate] {_bar(elapsed, dur)}  odom_w={node.odom_w:+.4f} rad/s  ")
        sys.stdout.flush()

    completed = node.rotate(
        speed=speed,
        duration=duration,
        ramp_time=ramp_time,
        on_steady=_on_steady,
        progress_cb=_progress,
    )

    sys.stdout.write("\r" + " " * 70 + "\r")
    sys.stdout.flush()

    if completed:
        print("[steady_rotate] Done. Robot stopped.")
    else:
        print("[steady_rotate] Interrupted. Robot stopped.")

    if post_nudge > 0 and completed:
        time.sleep(0.5)   # let robot fully settle
        node.slam_nudge(distance=post_nudge)


# ── Signal handling ───────────────────────────────────────────────────────────

_node_ref: SteadyRotator = None

def _sigint_handler(sig, frame):
    print("\n[steady_rotate] Caught SIGINT — stopping robot...")
    if _node_ref is not None:
        _node_ref.stop()
        _node_ref.hard_stop()
    sys.exit(0)


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    global _node_ref

    p = argparse.ArgumentParser(
        description="Rotate robot at steady angular velocity.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive menu
  python3 steady_rotate.py

  # CW 0.5 rad/s for 20 s (use from shell script alongside ros2 bag record)
  python3 steady_rotate.py --speed -0.5 --duration 20

  # CCW 1.0 rad/s, slow ramp
  python3 steady_rotate.py --speed 1.0 --duration 20 --ramp-time 3.0
        """,
    )
    p.add_argument("--speed",      type=float, default=None,
                   help="Angular velocity (rad/s). Negative = CW. Omit for interactive mode.")
    p.add_argument("--duration",   type=float, default=20.0,
                   help="Hold time at steady speed (s). 0 = until SIGINT. [default: 20]")
    p.add_argument("--ramp-time",  type=float, default=2.0,
                   help="Ramp-up / ramp-down duration (s). [default: 2.0]")
    p.add_argument("--post-nudge", type=float, default=0.0,
                   help="After rotation, move forward then back this many metres "
                        "so SLAM toolbox can re-correct. 0 = disabled. [default: 0]")
    args = p.parse_args()

    rclpy.init()
    node = SteadyRotator()
    _node_ref = node

    signal.signal(signal.SIGINT,  _sigint_handler)
    signal.signal(signal.SIGTERM, _sigint_handler)

    try:
        if args.speed is None:
            run_interactive(node)
        else:
            run_cli(node, args.speed, args.duration, args.ramp_time, args.post_nudge)
    finally:
        node.hard_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

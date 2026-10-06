#!/usr/bin/env python3

# Copyright 2026 fleet_adapter_r1 contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Optional single-container fake robot for the R1 fleet adapter.

Speaks the /r1_fleet protocol directly (no Nav2, no zenoh, no mock stack) so the
fleet adapter + RMF core can be smoke-tested quickly on a single ROS domain. It
publishes FleetRobotState and consumes FleetNavRequest / FleetActionRequest /
FleetModeRequest, simulating motion and pick/place by transitioning mode and
acknowledging the command's task_id.

For full end-to-end (and zenoh) verification, use r1_fleet_client + the mock R1
stack in the robot container instead.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from r1_interfaces.msg import (
    FleetActionRequest,
    FleetModeRequest,
    FleetNavRequest,
    FleetRobotState,
)


class FakeRobot(Node):

    def __init__(self):
        super().__init__('fake_robot')
        self.declare_parameter('robot_name', 'brunson')
        self.declare_parameter('map_name', 'L1')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)
        self.declare_parameter('linear_speed', 0.75)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('action_duration', 5.0)
        self.declare_parameter('update_rate', 5.0)

        self.robot_name = self.get_parameter('robot_name').value
        self.map_name = self.get_parameter('map_name').value
        self.x = float(self.get_parameter('start_x').value)
        self.y = float(self.get_parameter('start_y').value)
        self.yaw = float(self.get_parameter('start_yaw').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.action_duration = float(self.get_parameter('action_duration').value)
        self.rate = float(self.get_parameter('update_rate').value)

        self.mode = FleetRobotState.MODE_IDLE
        self.current_task_id = 0
        self.last_completed_task_id = 0
        self.task_completed = True
        self.battery_soc = 0.95

        self.nav_target = None     # (x, y, yaw, task_id)
        self.action_deadline = None  # (deadline_sec, task_id)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.state_pub = self.create_publisher(
            FleetRobotState, '/r1_fleet/robot_state', qos)
        self.create_subscription(
            FleetNavRequest, '/r1_fleet/nav_request', self._nav_cb, qos)
        self.create_subscription(
            FleetActionRequest, '/r1_fleet/action_request',
            self._action_cb, qos)
        self.create_subscription(
            FleetModeRequest, '/r1_fleet/mode_request', self._mode_cb, qos)

        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"Fake robot [{self.robot_name}] up on map [{self.map_name}] at "
            f"({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})")

    # ------------------------------------------------------------------ #
    def _nav_cb(self, msg: FleetNavRequest):
        if msg.robot_name and msg.robot_name != self.robot_name:
            return
        self.nav_target = (msg.x, msg.y, msg.yaw, msg.task_id)
        self.action_deadline = None
        self.current_task_id = msg.task_id
        self.mode = FleetRobotState.MODE_MOVING
        self.get_logger().info(
            f"[{self.robot_name}] nav -> ({msg.x:.2f}, {msg.y:.2f}, "
            f"{msg.yaw:.2f}) task {msg.task_id}")

    def _action_cb(self, msg: FleetActionRequest):
        if msg.robot_name and msg.robot_name != self.robot_name:
            return
        self.nav_target = None
        self.current_task_id = msg.task_id
        if msg.category == FleetActionRequest.CATEGORY_PLACE:
            self.mode = FleetRobotState.MODE_PLACING
        else:
            self.mode = FleetRobotState.MODE_PICKING
        now = self.get_clock().now().nanoseconds / 1e9
        self.action_deadline = (now + self.action_duration, msg.task_id)
        self.get_logger().info(
            f"[{self.robot_name}] action '{msg.category}' task {msg.task_id}")

    def _mode_cb(self, msg: FleetModeRequest):
        if msg.robot_name and msg.robot_name != self.robot_name:
            return
        if msg.mode == FleetModeRequest.MODE_STOP:
            self.nav_target = None
            self.action_deadline = None
            self.current_task_id = 0
            self.mode = FleetRobotState.MODE_IDLE
            self.get_logger().info(f"[{self.robot_name}] stop")

    # ------------------------------------------------------------------ #
    def _tick(self):
        if self.nav_target is not None:
            self._drive()
        elif self.action_deadline is not None:
            now = self.get_clock().now().nanoseconds / 1e9
            if now >= self.action_deadline[0]:
                self._complete(self.action_deadline[1])
                self.action_deadline = None

        # Drain battery slowly while busy
        if self.mode != FleetRobotState.MODE_IDLE:
            self.battery_soc = max(0.0, self.battery_soc - 0.00002)

        self._publish_state()

    def _drive(self):
        tx, ty, tyaw, task_id = self.nav_target
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        step = self.linear_speed * self.dt
        if dist <= max(step, 0.05):
            self.x, self.y = tx, ty
            self.yaw = tyaw
            self.nav_target = None
            self._complete(task_id)
        else:
            self.x += step * dx / dist
            self.y += step * dy / dist
            self.yaw = math.atan2(dy, dx)

    def _complete(self, task_id):
        self.last_completed_task_id = task_id
        self.task_completed = True
        self.current_task_id = 0
        self.mode = FleetRobotState.MODE_IDLE
        self.get_logger().info(f"[{self.robot_name}] completed task {task_id}")

    def _publish_state(self):
        msg = FleetRobotState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.robot_name = self.robot_name
        msg.map_name = self.map_name
        msg.x = float(self.x)
        msg.y = float(self.y)
        msg.yaw = float(self.yaw)
        msg.battery_soc = float(self.battery_soc)
        msg.mode = self.mode
        msg.task_id = int(self.current_task_id)
        msg.last_completed_task_id = int(self.last_completed_task_id)
        msg.task_completed = bool(self.task_completed)
        msg.status_message = ''
        self.state_pub.publish(msg)


def main(argv=sys.argv):
    rclpy.init(args=argv)
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)

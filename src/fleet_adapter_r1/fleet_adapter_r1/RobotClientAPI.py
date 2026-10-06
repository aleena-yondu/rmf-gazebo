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

"""ROS-based RobotAPI for the R1 fleet adapter.

Unlike the upstream RMF templates (which talk to a REST fleet manager), this
RobotAPI communicates with each robot over a small ROS topic protocol:

  robot  -> fleet : r1_interfaces/FleetRobotState  on /r1_fleet/robot_state
  fleet  -> robot : r1_interfaces/FleetNavRequest   on /r1_fleet/nav_request
  fleet  -> robot : r1_interfaces/FleetActionRequest on /r1_fleet/action_request
  fleet  -> robot : r1_interfaces/FleetModeRequest   on /r1_fleet/mode_request

These topics are bridged from the fleet ROS domain (42) to the robot ROS domain
by a pair of zenoh-bridge-ros2dds instances. Command completion is acknowledged
by the robot echoing the command's ``cmd_id`` back in
``FleetRobotState.last_completed_task_id`` together with ``task_completed``.
"""

import math
import threading

from r1_interfaces.msg import (
    FleetActionRequest,
    FleetModeRequest,
    FleetNavRequest,
    FleetRobotState,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Header


class RobotAPI:
    """Wrapper for ROS communication with the R1 robots."""

    def __init__(self, node: Node, config: dict):
        self.node = node
        comms = config.get("fleet_manager", {})
        self.state_topic = comms.get("state_topic", "/r1_fleet/robot_state")
        self.nav_topic = comms.get("nav_request_topic", "/r1_fleet/nav_request")
        self.action_topic = comms.get(
            "action_request_topic", "/r1_fleet/action_request"
        )
        self.mode_topic = comms.get("mode_request_topic", "/r1_fleet/mode_request")
        # Nominal linear speed (m/s) used to estimate remaining nav duration.
        self.nominal_speed = float(comms.get("nominal_linear_speed", 0.5))

        self._lock = threading.Lock()
        self._states = {}  # robot_name -> FleetRobotState
        self._nav_target = {}  # robot_name -> [x, y, yaw] (robot frame)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._state_sub = node.create_subscription(
            FleetRobotState, self.state_topic, self._state_cb, qos
        )
        self._nav_pub = node.create_publisher(FleetNavRequest, self.nav_topic, qos)
        self._action_pub = node.create_publisher(
            FleetActionRequest, self.action_topic, qos
        )
        self._mode_pub = node.create_publisher(FleetModeRequest, self.mode_topic, qos)

        self.connected = True

    # ------------------------------------------------------------------ #
    # Incoming state
    # ------------------------------------------------------------------ #
    def _state_cb(self, msg: FleetRobotState):
        with self._lock:
            self._states[msg.robot_name] = msg

    def _get_state(self, robot_name):
        with self._lock:
            return self._states.get(robot_name)

    def _header(self):
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        return header

    # ------------------------------------------------------------------ #
    # Queries used by the command handle
    # ------------------------------------------------------------------ #
    def check_connection(self):
        return self.connected

    def position(self, robot_name: str):
        """Return [x, y, yaw] in the robot's map frame, or None."""
        state = self._get_state(robot_name)
        if state is None:
            return None
        return [state.x, state.y, state.yaw]

    def map_name(self, robot_name: str):
        state = self._get_state(robot_name)
        return None if state is None else state.map_name

    def battery_soc(self, robot_name: str):
        """Return battery state of charge in [0.0, 1.0], or None."""
        state = self._get_state(robot_name)
        return None if state is None else float(state.battery_soc)

    def mode(self, robot_name: str):
        state = self._get_state(robot_name)
        return None if state is None else state.mode

    # ------------------------------------------------------------------ #
    # Commands
    # ------------------------------------------------------------------ #
    def navigate(self, robot_name, cmd_id, pose, map_name, speed_limit=0.0):
        """Request the robot to navigate to pose [x, y, yaw] (robot frame)."""
        msg = FleetNavRequest()
        msg.header = self._header()
        msg.robot_name = robot_name
        msg.task_id = int(cmd_id)
        msg.map_name = map_name
        msg.x = float(pose[0])
        msg.y = float(pose[1])
        msg.yaw = float(pose[2])
        msg.speed_limit = float(speed_limit) if speed_limit else 0.0
        with self._lock:
            self._nav_target[robot_name] = [msg.x, msg.y, msg.yaw]
        self._nav_pub.publish(msg)
        return True

    def start_activity(self, robot_name, cmd_id, category, description=""):
        """Request a custom action (perform_action), e.g. pick / place."""
        msg = FleetActionRequest()
        msg.header = self._header()
        msg.robot_name = robot_name
        msg.task_id = int(cmd_id)
        msg.category = category
        msg.description = description if isinstance(description, str) else ""
        self._action_pub.publish(msg)
        return True

    def start_process(self, robot_name, cmd_id, process, map_name):
        """Compatibility shim for the template dock() path."""
        return self.start_activity(robot_name, cmd_id, process, "")

    def stop(self, robot_name, cmd_id=0):
        msg = FleetModeRequest()
        msg.header = self._header()
        msg.robot_name = robot_name
        msg.task_id = int(cmd_id)
        msg.mode = FleetModeRequest.MODE_STOP
        self._mode_pub.publish(msg)
        return True

    def toggle_action(self, robot_name, toggle):
        """No-op: the robot reports picking/placing mode via FleetRobotState."""
        return True

    def requires_replan(self, robot_name):
        """The mock/real robot does not surface obstacle-replan signals here."""
        return False

    # ------------------------------------------------------------------ #
    # Completion polling
    # ------------------------------------------------------------------ #
    def _completed(self, robot_name, cmd_id):
        if cmd_id is None:
            return True
        state = self._get_state(robot_name)
        if state is None:
            return False
        return state.last_completed_task_id == int(cmd_id) and state.task_completed

    def navigation_completed(self, robot_name, cmd_id):
        return self._completed(robot_name, cmd_id)

    def process_completed(self, robot_name, cmd_id):
        return self._completed(robot_name, cmd_id)

    def docking_completed(self, robot_name, cmd_id):
        return self._completed(robot_name, cmd_id)

    def navigation_remaining_duration(self, robot_name, cmd_id):
        state = self._get_state(robot_name)
        with self._lock:
            target = self._nav_target.get(robot_name)
        if state is None or target is None:
            return 0.0
        dist = math.hypot(target[0] - state.x, target[1] - state.y)
        speed = self.nominal_speed if self.nominal_speed > 1e-3 else 0.5
        return dist / speed

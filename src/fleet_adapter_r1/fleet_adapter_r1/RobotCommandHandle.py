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

"""RobotCommandHandle for the R1 fleet adapter (classic full-control API).

Adapted from the Open-RMF Humble fleet adapter templates. Navigation goals from
RMF are translated into FleetNavRequest messages via the ROS ``RobotAPI``; the
robot's reported pose/battery/mode are fed back to RMF. Custom RMF
``perform_action`` requests (``pick`` / ``place``) are executed through
``perform_action`` -> ``RobotAPI.start_activity`` and completed when the robot
acknowledges via FleetRobotState.
"""

import enum
import json
import math
import threading
import time
from datetime import timedelta

from rclpy.duration import Duration
import rclpy

import rmf_adapter as adpt
import rmf_adapter.plan as plan

import numpy as np


class RobotState(enum.IntEnum):
    IDLE = 0
    MOVING = 1


class PlanWaypoint:
    """Lightweight wrapper around a plan.Waypoint with its path index."""

    def __init__(self, index, wp):
        self.index = index
        self.position = wp.position
        self.time = wp.time
        self.graph_index = wp.graph_index
        self.approach_lanes = wp.approach_lanes


class RobotCommandHandle(adpt.RobotCommandHandle):

    def __init__(self,
                 name,
                 fleet_name,
                 config,
                 node,
                 graph,
                 vehicle_traits,
                 transforms,
                 map_name,
                 start,
                 position,
                 charger_waypoint,
                 update_frequency,
                 lane_merge_distance,
                 adapter,
                 api):
        adpt.RobotCommandHandle.__init__(self)
        self.name = name
        self.fleet_name = fleet_name
        self.config = config
        self.node = node
        self.graph = graph
        self.vehicle_traits = vehicle_traits
        self.transforms = transforms
        self.map_name = map_name
        waypoint = self.graph.find_waypoint(charger_waypoint)
        assert waypoint, (f"Charger waypoint {charger_waypoint} does not exist "
                          f"in the navigation graph")
        self.charger_waypoint_index = waypoint.index
        self.update_frequency = update_frequency
        self.lane_merge_distance = lane_merge_distance
        self.update_handle = None
        self.battery_soc = 1.0
        self.api = api
        self.position = position  # (x, y, yaw) in RMF coordinates
        self.initialized = False
        self.state = RobotState.IDLE
        self.adapter = adapter

        self.requested_waypoints = []
        self.remaining_waypoints = []
        self.target_waypoint = None

        # RMF location trackers
        self.last_known_lane_index = None
        self.last_known_waypoint_index = None
        self.on_waypoint = None
        self.on_lane = None
        self.dock_waypoint_index = None

        # Action / perform_action tracking
        self.action_execution = None
        self.action_category = None
        self.action_waypoint_index = None

        self.current_cmd_id = 0

        self._lock = threading.Lock()
        self._follow_path_thread = None
        self._quit_path_event = threading.Event()
        self._dock_thread = None
        self._quit_dock_event = threading.Event()
        self._stopping_thread = None
        self._quit_stopping_event = threading.Event()
        self._action_thread = None
        self._quit_action_event = threading.Event()

        self.node.get_logger().info(
            f"[{self.name}] starting at: [{self.position[0]:.2f}, "
            f"{self.position[1]:.2f}, {self.position[2]:.2f}]")

        if start.lane is not None:
            self.last_known_lane_index = start.lane
            self.on_lane = start.lane
            self.last_known_waypoint_index = start.waypoint
        else:
            self.last_known_waypoint_index = start.waypoint
            self.on_waypoint = start.waypoint

        self.update_thread = threading.Thread(target=self.update)
        self.update_thread.start()

        self.initialized = True

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #
    def next_cmd_id(self):
        self.current_cmd_id += 1
        return self.current_cmd_id

    def sleep_for(self, seconds):
        goal_time = (
            self.node.get_clock().now() + Duration(nanoseconds=1e9 * seconds))
        while self.node.get_clock().now() <= goal_time:
            time.sleep(0.001)

    def clear(self):
        self.requested_waypoints = []
        self.remaining_waypoints = []
        self.state = RobotState.IDLE

    def interrupt(self):
        self._quit_dock_event.set()
        self._quit_path_event.set()
        self._quit_stopping_event.set()
        for thread in (self._follow_path_thread,
                       self._dock_thread,
                       self._stopping_thread):
            if thread is not None and thread.is_alive():
                thread.join()

    # ------------------------------------------------------------------ #
    # RobotCommandHandle interface
    # ------------------------------------------------------------------ #
    def stop(self):
        self.interrupt()
        with self._lock:
            self._quit_stopping_event.clear()

        def _stop():
            while not self._quit_stopping_event.is_set():
                self.node.get_logger().info(f"Requesting {self.name} to stop")
                if self.api.stop(self.name, self.next_cmd_id()):
                    break
                self._quit_stopping_event.wait(0.1)

        self._stopping_thread = threading.Thread(target=_stop)
        self._stopping_thread.start()

    def replan(self):
        if self.update_handle is not None:
            self.update_handle.replan()

    def follow_new_path(self,
                        waypoints,
                        next_arrival_estimator,
                        path_finished_callback):
        self.interrupt()
        with self._lock:
            self._follow_path_thread = None
            self._quit_path_event.clear()
            self.clear()

        self.node.get_logger().info(f"Received new path for {self.name}")
        self.remaining_waypoints = self.filter_waypoints(waypoints)
        assert next_arrival_estimator is not None
        assert path_finished_callback is not None

        def _follow_path():
            target_pose = None
            path_index = 0
            while self.remaining_waypoints or self.state == RobotState.MOVING:
                cmd_id = self.current_cmd_id
                if self._quit_path_event.is_set():
                    self.node.get_logger().info(
                        f"[{self.name}] aborting path request")
                    return

                if self.state == RobotState.IDLE or target_pose is None:
                    self.target_waypoint = self.remaining_waypoints[0]
                    path_index = self.remaining_waypoints[0].index
                    target_pose = self.target_waypoint.position
                    # RMF coordinates -> robot coordinates
                    [x, y] = self.transforms['rmf_to_robot'].transform(
                        target_pose[:2])
                    theta = (target_pose[2]
                             + self.transforms['rmf_to_robot'].get_rotation())
                    speed_limit = self.get_speed_limit(self.target_waypoint)
                    response = self.api.navigate(
                        self.name,
                        self.next_cmd_id(),
                        [x, y, theta],
                        self.map_name,
                        speed_limit)
                    if response:
                        self.remaining_waypoints = self.remaining_waypoints[1:]
                        self.state = RobotState.MOVING
                    else:
                        self.node.get_logger().info(
                            f"[{self.name}] failed to request navigation; "
                            f"retrying...")
                        self._quit_path_event.wait(0.1)

                elif self.state == RobotState.MOVING:
                    if self.api.requires_replan(self.name):
                        self.replan()
                    if self._quit_path_event.wait(0.1):
                        return
                    with self._lock:
                        if self.api.navigation_completed(self.name, cmd_id):
                            self.node.get_logger().info(
                                f"[{self.name}] reached destination for "
                                f"cmd_id {cmd_id}")
                            self.state = RobotState.IDLE
                            graph_index = self.target_waypoint.graph_index
                            if graph_index is not None:
                                self.on_waypoint = graph_index
                                self.last_known_waypoint_index = graph_index
                            else:
                                self.on_waypoint = None
                        else:
                            lane = self.get_current_lane()
                            if lane is not None:
                                self.on_waypoint = None
                                self.on_lane = lane
                            elif (self.target_waypoint.graph_index is not None
                                  and self.dist(self.position,
                                                target_pose) < 0.5):
                                self.on_waypoint = \
                                    self.target_waypoint.graph_index
                            elif (self.last_known_waypoint_index is not None
                                  and self.dist(
                                      self.position,
                                      self.graph.get_waypoint(
                                          self.last_known_waypoint_index
                                      ).location) < 0.5):
                                self.on_waypoint = \
                                    self.last_known_waypoint_index
                            else:
                                self.on_lane = None
                                self.on_waypoint = None
                    duration = self.api.navigation_remaining_duration(
                        self.name, cmd_id)
                    if path_index is not None and duration is not None:
                        next_arrival_estimator(
                            path_index, timedelta(seconds=duration))
            path_finished_callback()
            self.node.get_logger().info(
                f"[{self.name}] navigated along requested path")

        self._follow_path_thread = threading.Thread(target=_follow_path)
        self._follow_path_thread.start()

    def dock(self, dock_name, docking_finished_callback):
        """Minimal dock implementation via the action/process path."""
        self.interrupt()
        with self._lock:
            self._quit_dock_event.clear()
        assert docking_finished_callback is not None
        dock_waypoint = self.graph.find_waypoint(dock_name)
        self.dock_waypoint_index = (
            dock_waypoint.index if dock_waypoint is not None else None)

        def _dock():
            cmd_id = self.next_cmd_id()
            self.node.get_logger().info(
                f"[{self.name}] docking at {dock_name}")
            self.api.start_process(self.name, cmd_id, dock_name, self.map_name)
            while not self.api.process_completed(self.name, cmd_id):
                if self._quit_dock_event.wait(0.2):
                    self.node.get_logger().info(f"[{self.name}] aborting dock")
                    return
            with self._lock:
                if self.dock_waypoint_index is not None:
                    self.on_waypoint = self.dock_waypoint_index
                self.dock_waypoint_index = None
            docking_finished_callback()
            self.node.get_logger().info(f"[{self.name}] completed docking")

        self._dock_thread = threading.Thread(target=_dock)
        self._dock_thread.start()

    # ------------------------------------------------------------------ #
    # Custom actions (RMF perform_action): pick / place
    # ------------------------------------------------------------------ #
    def perform_action(self, category, description, execution):
        with self._lock:
            self.action_category = category
            self.action_execution = execution
            self.action_waypoint_index = self.last_known_waypoint_index
            self.on_waypoint = None
            self.on_lane = None
            self._quit_action_event.clear()
        if category == 'approach_bin':
            # The dispatch node supplies the bin pose in RMF coordinates;
            # rewrite it into the robot map frame before sending to the robot.
            desc = self._approach_bin_description(description)
        elif isinstance(description, str):
            desc = description
        else:
            try:
                desc = json.dumps(description)
            except (TypeError, ValueError):
                desc = ''
        cmd_id = self.next_cmd_id()
        self.node.get_logger().info(
            f"[{self.name}] starting action '{category}' (cmd_id {cmd_id})")
        self.api.start_activity(self.name, cmd_id, category, desc)

        def _await_action():
            while rclpy.ok():
                if self._quit_action_event.wait(0.2):
                    return
                if self.api.process_completed(self.name, cmd_id):
                    break
            self.complete_robot_action()

        self._action_thread = threading.Thread(target=_await_action)
        self._action_thread.start()

    def complete_robot_action(self):
        with self._lock:
            if self.action_execution is None:
                return
            self.action_execution.finished()
            self.action_execution = None
            self.action_category = None
        self.node.get_logger().info(
            f"[{self.name}] completed action")

    def _approach_bin_description(self, description):
        """Convert an ``approach_bin`` pose from RMF coords to robot coords.

        The dispatch node provides ``x, y, yaw`` in the RMF (nav-graph) frame.
        The robot navigates in its own map frame, so apply the inverse
        reference transform here (same convention as ``follow_new_path``), and
        return the JSON string sent to the robot over the action protocol.
        """
        if isinstance(description, str):
            try:
                description = json.loads(description)
            except (ValueError, TypeError):
                description = {}
        desc = dict(description) if isinstance(description, dict) else {}
        try:
            x = float(desc['x'])
            y = float(desc['y'])
            yaw = float(desc['yaw'])
        except (KeyError, TypeError, ValueError):
            self.node.get_logger().error(
                f"[{self.name}] approach_bin description missing x/y/yaw")
            return json.dumps(desc)
        rx, ry = self.transforms['rmf_to_robot'].transform([x, y])
        rtheta = yaw + self.transforms['rmf_to_robot'].get_rotation()
        desc['x'] = float(rx)
        desc['y'] = float(ry)
        desc['yaw'] = float(rtheta)
        desc['map_name'] = self.map_name
        return json.dumps(desc)

    # ------------------------------------------------------------------ #
    # State reporting
    # ------------------------------------------------------------------ #
    def get_position(self):
        position = self.api.position(self.name)
        if position is not None:
            x, y = self.transforms['robot_to_rmf'].transform(
                [position[0], position[1]])
            theta = (position[2]
                     + self.transforms['robot_to_rmf'].get_rotation())
            while theta > math.pi:
                theta -= 2.0 * math.pi
            while theta < -math.pi:
                theta += 2.0 * math.pi
            return [x, y, theta]
        self.node.get_logger().warn(
            f"Unable to retrieve position from {self.name}")
        return self.position

    def get_battery_soc(self):
        battery_soc = self.api.battery_soc(self.name)
        if battery_soc is not None:
            return battery_soc
        self.node.get_logger().warn(
            f"Unable to retrieve battery soc from {self.name}")
        return self.battery_soc

    def update(self):
        while rclpy.ok():
            self.position = self.get_position()
            self.battery_soc = self.get_battery_soc()
            if self.update_handle is not None:
                self.update_state()
            self.sleep_for(1.0 / self.update_frequency)

    def update_state(self):
        self.update_handle.update_battery_soc(self.battery_soc)
        with self._lock:
            if self.on_waypoint is not None:
                self.update_handle.update_current_waypoint(
                    self.on_waypoint, self.position[2])
            elif self.on_lane is not None:
                forward_lane = self.graph.get_lane(self.on_lane)
                entry_index = forward_lane.entry.waypoint_index
                exit_index = forward_lane.exit.waypoint_index
                reverse_lane = self.graph.lane_from(exit_index, entry_index)
                lane_indices = [self.on_lane]
                if reverse_lane is not None:
                    lane_indices.append(reverse_lane.index)
                self.update_handle.update_current_lanes(
                    self.position, lane_indices)
            elif self.dock_waypoint_index is not None:
                self.update_handle.update_off_grid_position(
                    self.position, self.dock_waypoint_index)
            elif self.action_execution is not None \
                    and self.action_waypoint_index is not None:
                self.update_handle.update_off_grid_position(
                    self.position, self.action_waypoint_index)
            elif (self.target_waypoint is not None
                  and self.target_waypoint.graph_index is not None):
                self.update_handle.update_off_grid_position(
                    self.position, self.target_waypoint.graph_index)
            else:
                self.update_handle.update_lost_position(
                    self.map_name, self.position,
                    max_merge_lane_distance=self.lane_merge_distance)

    # ------------------------------------------------------------------ #
    # Geometry helpers
    # ------------------------------------------------------------------ #
    def get_current_lane(self):
        def projection(current, target, entry, exit):
            p = np.array([current[0], current[1]])
            t = np.array(target)
            e0 = np.array(entry)
            e1 = np.array(exit)
            return np.dot(p - t, e1 - e0)

        if self.target_waypoint is None:
            return None
        approach_lanes = self.target_waypoint.approach_lanes
        if approach_lanes is None or len(approach_lanes) == 0:
            return None
        for lane_index in approach_lanes:
            lane = self.graph.get_lane(lane_index)
            p0 = self.graph.get_waypoint(lane.entry.waypoint_index).location
            p1 = self.graph.get_waypoint(lane.exit.waypoint_index).location
            p = self.position
            before_lane = projection(p, p0, p0, p1) < 0.0
            after_lane = projection(p, p1, p0, p1) >= 0.0
            if not before_lane and not after_lane:
                return lane_index
        return None

    def dist(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def get_speed_limit(self, target_waypoint):
        approach_lane_limit = np.inf
        for lane_index in target_waypoint.approach_lanes:
            lane = self.graph.get_lane(lane_index)
            lane_limit = lane.properties.speed_limit
            if lane_limit is not None and lane_limit < approach_lane_limit:
                approach_lane_limit = lane_limit
        return approach_lane_limit if approach_lane_limit != np.inf else 0.0

    def filter_waypoints(self, wps):
        assert len(wps) > 0
        p = np.array([self.position[0], self.position[1]])
        waypoints = [PlanWaypoint(i, wps[i]) for i in range(len(wps))]

        begin_at_index = 0
        for i in reversed(range(len(waypoints) - 1)):
            p0 = np.array(waypoints[i].position[:2])
            p1 = np.array(waypoints[i + 1].position[:2])
            dp_lane = p1 - p0
            lane_length = np.linalg.norm(dp_lane)
            if lane_length < 1e-3:
                continue
            n_lane = dp_lane / lane_length
            p_l = p - p0
            p_l_proj = np.dot(p_l, n_lane)
            if lane_length < p_l_proj:
                if np.linalg.norm(p - p1) <= self.lane_merge_distance:
                    begin_at_index = i + 1
                    break
                continue
            if p_l_proj < 0.0:
                if np.linalg.norm(p - p0) <= self.lane_merge_distance:
                    begin_at_index = i
                    break
                continue
            lane_dist = np.linalg.norm(p_l - p_l_proj * n_lane)
            if lane_dist <= self.lane_merge_distance:
                begin_at_index = i + 1
                break

        if begin_at_index > 0:
            del waypoints[:begin_at_index]
        return waypoints

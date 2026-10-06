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

"""Open-RMF fleet adapter for the R1 picking robots (classic full-control).

Reads the fleet config + RMF nav graph, registers the fleet with RMF, lazily
adds each robot once its state is received over the ROS protocol (bridged by
zenoh), and wires up navigation (patrol / go_to_place) plus custom
``perform_action`` tasks (``pick`` / ``place``).
"""

import argparse
import datetime
import sys
import threading
import time
from functools import partial

import rclpy
import rclpy.node
import rmf_adapter as adpt
import rmf_adapter.battery as battery
import rmf_adapter.geometry as geometry
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan
import rmf_adapter.vehicletraits as traits
import yaml
from rclpy.parameter import Parameter
from rmf_task_msgs.msg import TaskProfile, TaskType

from .RobotClientAPI import RobotAPI
from .RobotCommandHandle import RobotCommandHandle
from .transforms import Transform, estimate_transform


def _starts_from_nearest_waypoint(nav_graph, map_name, position, time_now):
    """Return a ``plan.Start`` seeded at the nearest graph waypoint on a map.

    Replacement for ``plan.compute_plan_starts()``, which segfaults in the
    installed RMF binaries (rmf_traffic 3.0.x paired with rmf_fleet_adapter
    2.1.x). RMF only needs an initial seed here; the robot's first state update
    corrects its actual on/off-grid position.
    """
    px, py = float(position[0]), float(position[1])
    best_index = None
    best_dist_sq = None
    for index in range(nav_graph.num_waypoints):
        waypoint = nav_graph.get_waypoint(index)
        if waypoint.map_name != map_name:
            continue
        loc = waypoint.location
        dist_sq = (float(loc[0]) - px) ** 2 + (float(loc[1]) - py) ** 2
        if best_dist_sq is None or dist_sq < best_dist_sq:
            best_dist_sq = dist_sq
            best_index = index
    if best_index is None:
        return []
    return [plan.Start(time_now, best_index, float(position[2]))]


def initialize_fleet(config_yaml, nav_graph_path, node, use_sim_time, server_uri):
    fleet_config = config_yaml["rmf_fleet"]

    # Profile and traits
    profile = traits.Profile(
        geometry.make_final_convex_circle(fleet_config["profile"]["footprint"]),
        geometry.make_final_convex_circle(fleet_config["profile"]["vicinity"]),
    )
    vehicle_traits = traits.VehicleTraits(
        linear=traits.Limits(*fleet_config["limits"]["linear"]),
        angular=traits.Limits(*fleet_config["limits"]["angular"]),
        profile=profile,
    )
    vehicle_traits.differential.reversible = fleet_config["reversible"]

    # Battery, mechanical and power systems
    battery_sys = battery.BatterySystem.make(
        fleet_config["battery_system"]["voltage"],
        fleet_config["battery_system"]["capacity"],
        fleet_config["battery_system"]["charging_current"],
    )
    mech_sys = battery.MechanicalSystem.make(
        fleet_config["mechanical_system"]["mass"],
        fleet_config["mechanical_system"]["moment_of_inertia"],
        fleet_config["mechanical_system"]["friction_coefficient"],
    )
    ambient_power_sys = battery.PowerSystem.make(
        fleet_config["ambient_system"]["power"]
    )
    tool_power_sys = battery.PowerSystem.make(fleet_config["tool_system"]["power"])
    motion_sink = battery.SimpleMotionPowerSink(battery_sys, mech_sys)
    ambient_sink = battery.SimpleDevicePowerSink(battery_sys, ambient_power_sys)
    tool_sink = battery.SimpleDevicePowerSink(battery_sys, tool_power_sys)

    nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

    fleet_name = fleet_config["name"]
    adapter = adpt.Adapter.make(f"{fleet_name}_fleet_adapter")
    assert adapter, (
        "Unable to initialize fleet adapter. Please ensure the "
        "RMF schedule node is running."
    )
    if use_sim_time:
        adapter.node.use_sim_time()
    adapter.start()
    time.sleep(1.0)

    fleet_handle = adapter.add_fleet(fleet_name, vehicle_traits, nav_graph, server_uri)

    state_freq = float(fleet_config.get("publish_fleet_state", 2.0))
    if state_freq > 0.0:
        fleet_handle.fleet_state_publish_period(
            datetime.timedelta(seconds=1.0 / state_freq)
        )

    ok = fleet_handle.set_task_planner_params(
        battery_sys,
        motion_sink,
        ambient_sink,
        tool_sink,
        fleet_config["recharge_threshold"],
        fleet_config["recharge_soc"],
        fleet_config["account_for_battery_drain"],
        fleet_config["task_capabilities"]["finishing_request"],
    )
    assert ok, "Unable to set task planner params"

    caps = fleet_config["task_capabilities"]

    # Legacy task-type acceptance (harmless; kept for backward compatibility)
    task_capabilities = []
    if caps.get("loop", False):
        task_capabilities.append(TaskType.TYPE_LOOP)
    if caps.get("delivery", False):
        task_capabilities.append(TaskType.TYPE_DELIVERY)
    if caps.get("clean", False):
        task_capabilities.append(TaskType.TYPE_CLEAN)

    def _task_request_check(capabilities, msg: TaskProfile):
        return msg.description.task_type in capabilities

    fleet_handle.accept_task_requests(partial(_task_request_check, task_capabilities))

    # Custom performable actions (pick / place)
    def _consider(description: dict):
        confirm = adpt.fleet_update_handle.Confirmation()
        confirm.accept()
        return confirm

    for action in caps.get("actions", []):
        fleet_handle.add_performable_action(action, _consider)
        node.get_logger().info(
            f"Fleet [{fleet_name}] is configured to perform action [{action}]"
        )

    # Coordinate transforms (identity by default)
    ref = config_yaml.get("reference_coordinates", None)
    if ref is not None and "rmf" in ref and "robot" in ref:
        transforms = {
            "rmf_to_robot": estimate_transform(ref["rmf"], ref["robot"]),
            "robot_to_rmf": estimate_transform(ref["robot"], ref["rmf"]),
        }
    else:
        transforms = {"rmf_to_robot": Transform(), "robot_to_rmf": Transform()}

    lane_merge_distance = fleet_config.get("lane_merge_distance", 0.1)

    api = RobotAPI(node, fleet_config)

    def _updater_inserter(cmd_handle, update_handle):
        cmd_handle.update_handle = update_handle

        def _action_executor(category, description, execution):
            cmd_handle.perform_action(category, description, execution)

        if hasattr(update_handle, "set_action_executor"):
            update_handle.set_action_executor(_action_executor)
        max_delay = cmd_handle.config.get("max_delay", None)
        if max_delay is not None:
            update_handle.set_maximum_delay(max_delay)
        if cmd_handle.charger_waypoint_index < cmd_handle.graph.num_waypoints:
            update_handle.set_charger_waypoint(cmd_handle.charger_waypoint_index)
        else:
            node.get_logger().warn(
                "Invalid charger waypoint; using nearest charger in the map"
            )

    missing_robots = dict(config_yaml["robots"])

    def _add_fleet_robots():
        while len(missing_robots) > 0:
            time.sleep(0.2)
            for robot_name in list(missing_robots.keys()):
                position = api.position(robot_name)
                if position is None:
                    node.get_logger().info(
                        f"Waiting for state from robot [{robot_name}]..."
                    )
                    continue

                robots_config = config_yaml["robots"][robot_name]
                rmf_config = robots_config["rmf_config"]
                robot_config = robots_config["robot_config"]
                map_name = rmf_config["start"]["map_name"]
                initial_waypoint = rmf_config["start"].get("waypoint", None)
                initial_orientation = rmf_config["start"].get("orientation", None)
                time_now = adapter.now()

                rmf_xy = transforms["robot_to_rmf"].transform(position[:2])
                rmf_position = [
                    rmf_xy[0],
                    rmf_xy[1],
                    position[2] + transforms["robot_to_rmf"].get_rotation(),
                ]

                if initial_waypoint is not None:
                    orientation = (
                        initial_orientation if initial_orientation is not None else 0.0
                    )
                    wp_index = nav_graph.find_waypoint(initial_waypoint).index
                    starts = [plan.Start(time_now, wp_index, orientation)]
                else:
                    starts = _starts_from_nearest_waypoint(
                        nav_graph, map_name, rmf_position, time_now
                    )

                if starts is None or len(starts) == 0:
                    node.get_logger().error(
                        f"Unable to determine StartSet for {robot_name}"
                    )
                    continue

                robot = RobotCommandHandle(
                    name=robot_name,
                    fleet_name=fleet_name,
                    config=robot_config,
                    node=node,
                    graph=nav_graph,
                    vehicle_traits=vehicle_traits,
                    transforms=transforms,
                    map_name=map_name,
                    start=starts[0],
                    position=rmf_position,
                    charger_waypoint=rmf_config["charger"]["waypoint"],
                    update_frequency=rmf_config.get("robot_state_update_frequency", 5),
                    lane_merge_distance=lane_merge_distance,
                    adapter=adapter,
                    api=api,
                )

                if robot.initialized:
                    fleet_handle.add_robot(
                        robot,
                        robot_name,
                        profile,
                        [starts[0]],
                        partial(_updater_inserter, robot),
                    )
                    node.get_logger().info(
                        f"Successfully added new robot: {robot_name}"
                    )
                    del missing_robots[robot_name]
                else:
                    node.get_logger().error(f"Failed to initialize robot: {robot_name}")

    threading.Thread(target=_add_fleet_robots, args=()).start()
    return adapter


def main(argv=sys.argv):
    rclpy.init(args=argv)
    adpt.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(
        prog="fleet_adapter", description="Configure and spin up the R1 fleet adapter"
    )
    parser.add_argument(
        "-c",
        "--config_file",
        type=str,
        required=True,
        help="Path to the config.yaml file",
    )
    parser.add_argument(
        "-n",
        "--nav_graph",
        type=str,
        required=True,
        help="Path to the RMF nav graph for this fleet",
    )
    parser.add_argument(
        "-s",
        "--server_uri",
        type=str,
        default="",
        help="URI of the RMF API server (websocket), optional",
    )
    parser.add_argument(
        "--use_sim_time", action="store_true", help="Use sim time, default: false"
    )
    args = parser.parse_args(args_without_ros[1:])
    print("Starting R1 fleet adapter...")

    with open(args.config_file, "r") as f:
        config_yaml = yaml.safe_load(f)

    fleet_name = config_yaml["rmf_fleet"]["name"]
    node = rclpy.node.Node(f"{fleet_name}_command_handle")

    if args.use_sim_time:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    server_uri = args.server_uri if args.server_uri else None

    adapter = initialize_fleet(
        config_yaml, args.nav_graph, node, args.use_sim_time, server_uri
    )

    rclpy_executor = rclpy.executors.SingleThreadedExecutor()
    rclpy_executor.add_node(node)
    try:
        rclpy_executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy_executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)

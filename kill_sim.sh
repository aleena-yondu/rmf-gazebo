#!/usr/bin/env bash
echo "Nuking all ROS 2, Gazebo, Nav2, and RViz processes..."
killall -9 gz sim parameter_bridge robot_state_publisher nav2_planner nav2_controller nav2_bt_navigator nav2_behavior_server nav2_amcl nav2_map_server rviz2 2>/dev/null
ros2 daemon stop
ros2 daemon start
echo "Cleanup complete."

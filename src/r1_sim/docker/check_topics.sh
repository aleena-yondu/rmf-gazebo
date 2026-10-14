#!/usr/bin/env bash
set -eo pipefail

# List the ROS 2 and Gazebo topics of the running r1_sim container from the host.
#
# Usage:
#   ./docker/check_topics.sh
#
# ROS_DOMAIN_ID and GZ_PARTITION must match the container (defaults 0 and r1).
# Exported values in this script do not carry over to your shell; see README.md.

source /opt/ros/jazzy/setup.bash

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export GZ_PARTITION="${GZ_PARTITION:-r1}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID} GZ_PARTITION=${GZ_PARTITION} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"

# A daemon started earlier with other settings lists only /parameter_events and /rosout.
ros2 daemon stop >/dev/null 2>&1 || true

echo
echo "== ROS 2 topics"
# The default 0.5 s discovery wait returns before the container's nodes are found.
ros2 topic list --no-daemon --spin-time 3

echo
echo "== Gazebo topics"
timeout 10 gz topic -l || echo "No Gazebo topics. Is the container running with GZ_PARTITION=${GZ_PARTITION}?"

echo
echo "== One ground-truth pose from Gazebo (/model/robot_1/odometry)"
timeout 10 gz topic -e -t /model/robot_1/odometry -n 1 || echo "No odometry message within 10 s."

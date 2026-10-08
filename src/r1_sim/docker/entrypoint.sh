#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

# Same ROS domain as the fleet manager. Host networking makes the topics visible.
# Leave discovery at the Jazzy default (subnet). ROS_LOCALHOST_ONLY is deprecated
# and prints a warning from every node even when set to 0.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset ROS_LOCALHOST_ONLY

mode="${1:-headless}"
if [[ "${mode}" == "headless" || "${mode}" == "gui" ]]; then
    shift
else
    mode="headless"
fi

if [[ "${mode}" == "gui" ]]; then
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
    launch_args=(headless:=false use_rviz:=true)
else
    # No window. --headless-rendering still runs the lidar camera.
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    launch_args=(headless:=true use_rviz:=false send_test_goal:=false)
fi

exec ros2 launch r1_sim goal_sim_ground_truth.launch.py \
    "${launch_args[@]}" \
    "$@"

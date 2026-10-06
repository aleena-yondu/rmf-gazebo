# Aggregate health status for the robot.
# Published by the health_monitor node in brs_description_cpp at a configurable
# period (default every 5 seconds) with reliable + transient_local QoS so a
# late subscriber (e.g. operator UI over webrtc_ros_bridge) sees the latest
# state immediately on connection.

std_msgs/Header header
string robot_name
HealthCode control
HealthCode battery
HealthCode navigation

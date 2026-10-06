# Navigation request sent by the fleet adapter (fleet_adapter_r1) to
# r1_fleet_client. Bridged across ROS domains via the zenoh bridge on topic
# /r1_fleet/nav_request.

std_msgs/Header header
string robot_name
# Monotonic id assigned by the fleet adapter; echoed back in FleetRobotState
uint32 task_id
string map_name

# Target pose in the robot map frame (meters, radians)
float64 x
float64 y
float64 yaw

# Optional maximum linear speed (m/s); 0.0 means no limit
float32 speed_limit

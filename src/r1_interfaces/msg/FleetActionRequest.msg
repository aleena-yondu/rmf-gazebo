# Custom action (RMF perform_action) request sent by the fleet adapter
# (fleet_adapter_r1) to r1_fleet_client. Bridged across ROS domains via the
# zenoh bridge on topic /r1_fleet/action_request.

# Action category string constants
string CATEGORY_PICK=pick
string CATEGORY_PLACE=place

std_msgs/Header header
string robot_name
# Monotonic id assigned by the fleet adapter; echoed back in FleetRobotState
uint32 task_id

# Action category (see CATEGORY_* constants above)
string category
# JSON-encoded action description (e.g. bin/tote details); may be empty
string description

# Mode/interrupt request sent by the fleet adapter (fleet_adapter_r1) to
# r1_fleet_client. Bridged across ROS domains via the zenoh bridge on topic
# /r1_fleet/mode_request.

# Requested mode string constants
string MODE_STOP=stop
string MODE_PAUSE=pause
string MODE_RESUME=resume

std_msgs/Header header
string robot_name
# Monotonic id assigned by the fleet adapter; echoed back in FleetRobotState
uint32 task_id

# Requested mode (see MODE_* constants above)
string mode

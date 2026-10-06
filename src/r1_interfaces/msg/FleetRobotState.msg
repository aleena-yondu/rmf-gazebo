# Consolidated robot state published by r1_fleet_client and consumed by the
# Open-RMF fleet adapter (fleet_adapter_r1). Bridged across ROS domains via the
# zenoh bridge on topic /r1_fleet/robot_state.

# Operating mode string constants
string MODE_IDLE=idle
string MODE_MOVING=moving
string MODE_PAUSED=paused
string MODE_WAITING=waiting
string MODE_CHARGING=charging
string MODE_PICKING=picking
string MODE_PLACING=placing
string MODE_ERROR=error

std_msgs/Header header
string robot_name
string map_name

# Pose in the robot map frame (meters, radians)
float64 x
float64 y
float64 yaw

# Battery state of charge in [0.0, 1.0]
float32 battery_soc

# Current operating mode (see MODE_* constants above)
string mode

# Task id currently being executed (0 when idle)
uint32 task_id
# Task id of the most recently finished nav/action request
uint32 last_completed_task_id
# True when last_completed_task_id finished successfully
bool task_completed

string status_message

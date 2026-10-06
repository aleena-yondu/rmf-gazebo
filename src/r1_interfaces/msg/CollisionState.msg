# Collision state for one robot arm, published by r1_collision_monitor's
# arm_effort_residual_node at its compute rate on
# /{namespace}/collision_state/arm_{left,right}.

std_msgs/Header header

# state values
uint8 FREE=0
uint8 MODERATE=1
uint8 SEVERE=2

string robot_name          # robot namespace (e.g. "bertha")
string arm                 # "left" or "right"
uint8 state                # FREE / MODERATE / SEVERE (debounced severity)
string[] flagged_joints    # joint names over threshold (worst first); empty when FREE
float64[] residuals        # residual magnitude (N*m-equiv), parallel to flagged_joints

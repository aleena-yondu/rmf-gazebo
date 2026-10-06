# Alert published by the navigation behaviour tree once the robot has finished
# navigating to a bin/tote AND has servoed its arms/torso to the access height,
# i.e. it is physically staged and ready for the operator to pick/place an item.
#
# Bridged robot->teleop over webrtc_ros_bridge on /{namespace}/operator_alert
# (reliable + transient_local + latched) so an operator monitoring several
# robots is told exactly which robot needs attention, even if they connect or
# switch focus to that robot after it became ready.

# target_type constants
string TARGET_BIN=BIN
string TARGET_TOTE=TOTE

# action constants (derived from target_type: BIN -> pick, TOTE -> place)
string ACTION_PICK=pick
string ACTION_PLACE=place

std_msgs/Header header

string robot_name        # Robot namespace / name (e.g. "bertha")
string target_type       # TARGET_BIN or TARGET_TOTE
string action            # ACTION_PICK or ACTION_PLACE

# Order context (copied from the active PickTicket on /teleop/order)
string item_name
string sku
int64 quantity
string order_id
string bin_barcode       # Expected bin barcode (PickTicket.bin_tag_txt)
string tote_barcode      # Physical tote barcode (PickTicket.tote_barcode)

string message           # Human-readable summary for the operator UI

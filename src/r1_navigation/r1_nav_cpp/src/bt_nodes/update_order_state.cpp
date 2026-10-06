#include "r1_nav_cpp/bt_nodes/update_order_state.hpp"
#include "behaviortree_cpp/blackboard.h"

namespace r1_bt_nodes
{

UpdateOrderState::UpdateOrderState(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  // Get the shared Nav2 node from the blackboard
  if (!config().blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "UpdateOrderState: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  // Now it's the Nav2 node that bt_navigator spins:
  RCLCPP_INFO(node_->get_logger(),
    "UpdateOrderState: using shared Nav2 node '%s'",
    node_->get_name());
  
  auto transient_local_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                                  .reliable()
                                  .transient_local();

  order_sub_ = node_->create_subscription<r1_interfaces::msg::PickTicket>(
    "/teleop/order",
    transient_local_qos,
    std::bind(&UpdateOrderState::orderCallback, this, std::placeholders::_1));
}

void UpdateOrderState::orderCallback(
  const r1_interfaces::msg::PickTicket::SharedPtr msg)
{
  curr_item_name_ = msg->item_name;
  curr_picks_remaining_ = msg->remaining_picks_in_batch;
  curr_order_id_ = msg->order_id;
  have_msg_ = true;
}

// ----------------------------------------------------------------------
// One-time blackboard initialization
// ----------------------------------------------------------------------
void UpdateOrderState::initBlackboard()
{
  auto bb = config().blackboard;

  // Initialize all blackboard variables used by this node
  bb->set("active_order", active_order_);
  bb->set("current_target", curr_target_);

  RCLCPP_INFO(
    node_->get_logger(),
    "UpdateOrderState: initializing blackboard: active_order=false, current_target=NONE");

  initialized_ = true;
}

BT::NodeStatus UpdateOrderState::tick()
{
  // Store last item name and spin to get any new messages
  last_item_name_ = curr_item_name_;
  last_picks_remaining_ = curr_picks_remaining_;
  last_order_id_ = curr_order_id_;
  rclcpp::spin_some(node_);
  auto bb = config().blackboard;

  if (curr_item_name_ == "NONE") {
    active_order_ = false;
    curr_target_ = "NONE";
    update_curr_target_flag_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "UpdateOrderState: RECEIVED order with item_name=NONE (no active order), last_item_name='%s', curr_item_name='%s'",
      last_item_name_.c_str(), curr_item_name_.c_str());
  }
  else if (last_picks_remaining_ != curr_picks_remaining_ || last_order_id_ != curr_order_id_ || last_item_name_ != curr_item_name_) {
    active_order_ = true;
    curr_target_ = "BIN";
    update_curr_target_flag_ = true;
    RCLCPP_INFO(
      node_->get_logger(),
      "UpdateOrderState: New order item_name: '%s' -> '%s'",
      last_item_name_.c_str(),
      curr_item_name_.c_str());
  }
  else {
    active_order_ = true;
    update_curr_target_flag_ = false;
    RCLCPP_INFO(
      node_->get_logger(),
      "UpdateOrderState: RECEIVED order with item_name='%s' (same as last)",
      curr_item_name_.c_str());
  }

  // --------------------------------------------------------------------
  // Run blackboard initialization exactly once
  // --------------------------------------------------------------------
  if (!initialized_) {
    initBlackboard();
  }

  // --------------------------------------------------------------------
  // Store updated active_order.
  // --------------------------------------------------------------------
  bb->set("active_order", active_order_);
  RCLCPP_INFO(
    node_->get_logger(),
    "UpdateOrderState: setting blackboard active_order=%s",
    active_order_ ? "true" : "false");

  // --------------------------------------------------------------------
  // If flag raised that there is a new active order, set current_target=BIN.
  // --------------------------------------------------------------------
  if (update_curr_target_flag_) {
    bb->set("current_target", curr_target_);
    RCLCPP_INFO(
      node_->get_logger(),
      "UpdateOrderState: active_order changed, setting current_target=%s",
      curr_target_.c_str());
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r1_bt_nodes

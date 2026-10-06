#include "r1_nav_cpp/bt_nodes/notify_operator_ready.hpp"

#include <cstdlib>

#include "behaviortree_cpp/bt_factory.h"

namespace r1_bt_nodes
{

NotifyOperatorReady::NotifyOperatorReady(
  const std::string & xml_tag_name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(xml_tag_name, config)
{
  if (!config.blackboard) {
    throw BT::RuntimeError("NotifyOperatorReady: blackboard is null");
  }

  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "NotifyOperatorReady: missing required blackboard entry [node] (rclcpp::Node::SharedPtr)");
  }

  // Latched (reliable + transient_local) so the webrtc_ros_bridge subscriber
  // caches the most recent alert and replays it on (re)connect. An operator
  // attaching to / switching focus to this robot is then notified immediately.
  rclcpp::QoS alert_qos(rclcpp::KeepLast(1));
  alert_qos.reliable().transient_local();
  pub_ = node_->create_publisher<r1_interfaces::msg::OperatorAlert>(topic_name_, alert_qos);

  rclcpp::QoS order_qos(rclcpp::KeepLast(1));
  order_qos.reliable().transient_local();
  order_sub_ = node_->create_subscription<r1_interfaces::msg::PickTicket>(
    "/teleop/order",
    order_qos,
    std::bind(&NotifyOperatorReady::orderCallback, this, std::placeholders::_1));
}

void NotifyOperatorReady::orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (msg->item_name == "NONE") {
    current_order_.reset();
  } else {
    current_order_ = msg;
  }
}

BT::PortsList NotifyOperatorReady::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "target_type", "BIN or TOTE - the target the robot is now staged at")
  };
}

BT::NodeStatus NotifyOperatorReady::tick()
{
  // Drain any pending /teleop/order messages so the alert carries fresh context.
  rclcpp::spin_some(node_);

  std::string target_type;
  if (!getInput("target_type", target_type) || target_type.empty()) {
    target_type = r1_interfaces::msg::OperatorAlert::TARGET_BIN;
    RCLCPP_WARN(
      node_->get_logger(),
      "NotifyOperatorReady: missing input [target_type], defaulting to BIN");
  }

  const bool is_bin = (target_type == r1_interfaces::msg::OperatorAlert::TARGET_BIN);

  r1_interfaces::msg::OperatorAlert alert;
  alert.header.stamp = node_->get_clock()->now();
  alert.target_type = target_type;
  alert.action = is_bin
    ? r1_interfaces::msg::OperatorAlert::ACTION_PICK
    : r1_interfaces::msg::OperatorAlert::ACTION_PLACE;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (current_order_) {
      alert.robot_name = current_order_->robot_name;
      alert.item_name = current_order_->item_name;
      alert.sku = current_order_->sku;
      alert.quantity = current_order_->quantity;
      alert.order_id = current_order_->order_id;
      alert.bin_barcode = current_order_->bin_tag_txt;
      alert.tote_barcode = current_order_->tote_barcode;
    }
  }

  // Fall back to ROS_NAMESPACE for the robot name if the order didn't carry one.
  if (alert.robot_name.empty()) {
    const char * ns_env = std::getenv("ROS_NAMESPACE");
    if (ns_env && ns_env[0] != '\0') {
      alert.robot_name = ns_env;
    }
  }

  const std::string who = alert.robot_name.empty() ? std::string("robot") : alert.robot_name;
  const std::string what = alert.item_name.empty() ? std::string("item") : alert.item_name;
  std::string where;
  if (is_bin) {
    where = alert.bin_barcode.empty() ? std::string() : " from bin " + alert.bin_barcode;
  } else {
    where = alert.tote_barcode.empty() ? std::string() : " into tote " + alert.tote_barcode;
  }
  alert.message = who + " is ready to " + alert.action + " " + what + where;

  pub_->publish(alert);
  RCLCPP_INFO(node_->get_logger(), "NotifyOperatorReady: %s", alert.message.c_str());

  // Always succeed so the enclosing Bin/Tote sequence proceeds to SetBlackboardString.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace r1_bt_nodes

#include "r1_nav_cpp/bt_nodes/debug_print.hpp"

#include <cmath>

namespace r1_bt_nodes
{

DebugPrint::DebugPrint(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  // Get shared rclcpp::Node from blackboard
  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("DebugPrint: missing required blackboard entry [node]");
  }
}

BT::NodeStatus DebugPrint::tick()
{
  std::string msg;
  if (!getInput("debug_message", msg)) {
    msg = "";
  }

  if (!msg.empty()) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[DebugPrint] %s",
      msg.c_str());
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "[DebugPrint] (no message)");
  }

  // Optional PoseStamped input
  geometry_msgs::msg::PoseStamped pose;
  if (getInput("print_pose", pose)) {
    const auto & p = pose.pose.position;
    const auto & q = pose.pose.orientation;

    // Compute yaw from quaternion manually (2D yaw)
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    const double yaw_deg = yaw_rad * 180.0 / M_PI;

    // Time difference between pose stamp and current node time
    rclcpp::Time now = node_->now();
    rclcpp::Time stamp = pose.header.stamp;
    double dt_sec = (now - stamp).seconds();

    RCLCPP_INFO(
      node_->get_logger(),
      "[DebugPrint] pose: x=%.3f, y=%.3f, yaw=%.1f deg, dt=%.3f s",
      p.x, p.y, yaw_deg, dt_sec);
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r1_bt_nodes

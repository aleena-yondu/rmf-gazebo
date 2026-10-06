#include "r1_nav_cpp/bt_nodes/publish_pose.hpp"

#include <utility>

#include "behaviortree_cpp/bt_factory.h"

namespace r1_bt_nodes
{

PublishPose::PublishPose(const std::string & xml_tag_name, const BT::NodeConfig & config)
: BT::SyncActionNode(xml_tag_name, config)
{
  // Expect a ROS2 node to be available on the blackboard under key "node"
  if (!config.blackboard) {
    throw BT::RuntimeError("PublishPose: blackboard is null");
  }

  if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError(
      "PublishPose: missing required blackboard entry [node] (rclcpp::Node::SharedPtr)");
  }

  pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name_, rclcpp::QoS(10));
}

BT::PortsList PublishPose::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "PoseStamped to publish")
  };
}

BT::NodeStatus PublishPose::tick()
{
  geometry_msgs::msg::PoseStamped pose_msg;

  if (!getInput("pose", pose_msg)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "PublishPose: missing input port [pose] or wrong type (expected PoseStamped)");
    return BT::NodeStatus::FAILURE;
  }

  pub_->publish(pose_msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace r1_bt_nodes
#include "r1_nav_cpp/bt_nodes/compute_spin_angle.hpp"

#include <cmath>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace r1_bt_nodes
{

ComputeSpinAngle::ComputeSpinAngle(
  const std::string & xml_tag_name,
  const BT::NodeConfig & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error(
      "ComputeSpinAngle: missing required blackboard entry [node]");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus ComputeSpinAngle::tick()
{
  geometry_msgs::msg::PoseStamped target_pose;
  if (!getInput("target_pose", target_pose)) {
    RCLCPP_ERROR(node_->get_logger(),
      "[ComputeSpinAngle] missing required input [target_pose]");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      "map", "base_link", tf2::TimePointZero, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(),
      "[ComputeSpinAngle] TF lookup failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  const double robot_x = tf_stamped.transform.translation.x;
  const double robot_y = tf_stamped.transform.translation.y;

  tf2::Quaternion q;
  tf2::fromMsg(tf_stamped.transform.rotation, q);
  double roll, pitch, robot_yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, robot_yaw);

  const double target_x = target_pose.pose.position.x;
  const double target_y = target_pose.pose.position.y;

  const double angle_to_target = std::atan2(
    target_y - robot_y, target_x - robot_x);

  double spin_angle = angle_to_target - robot_yaw;

  // Normalize to [-pi, pi]
  while (spin_angle > M_PI) { spin_angle -= 2.0 * M_PI; }
  while (spin_angle < -M_PI) { spin_angle += 2.0 * M_PI; }

  RCLCPP_INFO(node_->get_logger(),
    "[ComputeSpinAngle] robot(%.2f,%.2f yaw=%.2f) -> target(%.2f,%.2f) "
    "angle_to_target=%.2f spin=%.2f rad",
    robot_x, robot_y, robot_yaw, target_x, target_y,
    angle_to_target, spin_angle);

  setOutput("spin_angle", spin_angle);

  auto bb = config().blackboard;
  bb->set("spin_angle_bb", spin_angle);
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace r1_bt_nodes

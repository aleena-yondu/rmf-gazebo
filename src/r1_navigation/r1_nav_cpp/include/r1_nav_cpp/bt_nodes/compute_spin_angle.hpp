#ifndef R1_NAV_CPP__BT_NODES__COMPUTE_SPIN_ANGLE_HPP_
#define R1_NAV_CPP__BT_NODES__COMPUTE_SPIN_ANGLE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace r1_bt_nodes
{

/**
 * Computes the relative yaw rotation needed so the robot faces the
 * target position.  Output is bounded to [-pi, pi].
 */
class ComputeSpinAngle : public BT::SyncActionNode
{
public:
  ComputeSpinAngle(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "target_pose",
        "Target pose to face"),
      BT::OutputPort<double>(
        "spin_angle",
        "Computed spin angle in radians [-pi, pi]")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__COMPUTE_SPIN_ANGLE_HPP_

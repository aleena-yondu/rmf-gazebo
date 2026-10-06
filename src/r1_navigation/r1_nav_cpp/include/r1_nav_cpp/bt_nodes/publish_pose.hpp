#ifndef R1_NAV_CPP__BT_NODES__PUBLISH_POSE_HPP_
#define R1_NAV_CPP__BT_NODES__PUBLISH_POSE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r1_bt_nodes
{

class PublishPose : public BT::SyncActionNode
{
public:
  PublishPose(const std::string & xml_tag_name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
  std::string topic_name_{"/display_pose"};
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__PUBLISH_POSE_HPP_

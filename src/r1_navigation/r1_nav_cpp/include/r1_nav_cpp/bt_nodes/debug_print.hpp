#ifndef R1_NAV_CPP__BT_NODES__DEBUG_PRINT_HPP_
#define R1_NAV_CPP__BT_NODES__DEBUG_PRINT_HPP_

#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r1_bt_nodes
{

class DebugPrint : public BT::SyncActionNode
{
public:
  DebugPrint(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "debug_message",
        "",
        "Debug string to print"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "print_pose",
        "PoseStamped to print (optional)")
    };
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__DEBUG_PRINT_HPP_

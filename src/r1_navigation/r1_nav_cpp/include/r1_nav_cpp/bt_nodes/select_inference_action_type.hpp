#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "r1_interfaces/msg/bt_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace r1_bt_nodes
{

class SelectInferenceActionType : public BT::SyncActionNode
{
public:
  SelectInferenceActionType(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void stateCallback(const r1_interfaces::msg::BTState::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<r1_interfaces::msg::BTState>::SharedPtr state_sub_;

  std::mutex state_mutex_;
  r1_interfaces::msg::BTState latest_state_;
  bool has_received_state_{false};

  bool first_tick_{true};
  bool previous_pick_{true};
  bool previous_place_{false};
};

}  // namespace r1_bt_nodes

#pragma once

#include <behaviortree_cpp/decorator_node.h>

namespace r1_bt_nodes
{

class AlwaysRunning : public BT::DecoratorNode
{
public:
  AlwaysRunning(const std::string& name, const BT::NodeConfig& config)
  : BT::DecoratorNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    // Tick the child first
    const BT::NodeStatus child_status = child_node_->executeTick();

    // Ignore `child_status` completely
    (void)child_status;  // Prevent unused variable warning

    // Always return RUNNING
    return BT::NodeStatus::RUNNING;
  }
};

}  // namespace r1_bt_nodes
#ifndef R1_NAV_CPP__BT_NODES__SET_BLACKBOARD_STRING_HPP_
#define R1_NAV_CPP__BT_NODES__SET_BLACKBOARD_STRING_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace r1_bt_nodes
{

class SetBlackboardString : public BT::SyncActionNode
{
public:
  SetBlackboardString(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf)
  : BT::SyncActionNode(xml_tag_name, conf)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("output_key", "Blackboard key to set"),
      BT::InputPort<std::string>("value", "String value to store")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key;
    std::string value;

    if (!getInput("output_key", key)) {
      throw BT::RuntimeError("SetBlackboardString: missing input [output_key]");
    }

    if (!getInput("value", value)) {
      throw BT::RuntimeError("SetBlackboardString: missing input [value]");
    }

    config().blackboard->set(key, value);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__SET_BLACKBOARD_STRING_HPP_

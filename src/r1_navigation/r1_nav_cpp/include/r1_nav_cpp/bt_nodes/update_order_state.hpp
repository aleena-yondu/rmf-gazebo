#ifndef R1_NAV_CPP__BT_NODES__UPDATE_ORDER_STATE_HPP_
#define R1_NAV_CPP__BT_NODES__UPDATE_ORDER_STATE_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "r1_interfaces/msg/pick_ticket.hpp"

namespace r1_bt_nodes
{

/**
 * @brief BT node that:
 *  - Subscribes to /teleop/order (r1_interfaces::msg::PickTicket)
 *  - Sets blackboard["active_order"] = (last_item_name != "NONE")
 *  - When active_order transitions false->true, sets blackboard["is_bin_target"] = true
 *
 * It never starts nav by itself; it only updates these flags.
 */

class UpdateOrderState : public BT::SyncActionNode
{
public:
  explicit UpdateOrderState(
    const std::string & xml_tag_name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  void orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg);

  // NEW: one-time blackboard initialization
  void initBlackboard();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<r1_interfaces::msg::PickTicket>::SharedPtr order_sub_;

  std::string curr_item_name_{"NONE"};
  std::string last_item_name_{"NONE"};
  std::string last_order_id_{"NONE"};
  std::string curr_order_id_{"NONE"};
  std::int32_t curr_picks_remaining_{-1};
  std::int32_t last_picks_remaining_{-1};

  std::string curr_target_{"NONE"};
  bool update_curr_target_flag_{false};
  bool active_order_{false};
  bool have_msg_{false};

  // NEW: guard so initBlackboard() runs only once
  bool initialized_{false};
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__UPDATE_ORDER_STATE_HPP_

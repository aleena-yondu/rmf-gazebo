#ifndef R1_NAV_CPP__BT_NODES__SERVO_TO_TARGET_ACTION_HPP_
#define R1_NAV_CPP__BT_NODES__SERVO_TO_TARGET_ACTION_HPP_

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <cmath>
#include <atomic>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "r1_interfaces/action/servo_height.hpp"
#include "r1_interfaces/msg/pick_ticket.hpp"

namespace r1_bt_nodes
{

class ServoToTargetAction : public BT::StatefulActionNode
{
  public:
    using ServoHeight = r1_interfaces::action::ServoHeight;
    using GoalHandleServoHeight = rclcpp_action::ClientGoalHandle<ServoHeight>;

    ServoToTargetAction(
      const std::string & name,
      const BT::NodeConfig & conf);

    static BT::PortsList providedPorts()
    {
      return {
        BT::InputPort<std::string>("target_type", "BIN", "Target type: BIN, BIN_SEARCH, TOTE, or TOTE_SEARCH"),
        BT::InputPort<std::string>("bin", "Optional explicit bin shelf: A, B, or C"),
        BT::InputPort<int>("tote", "Optional explicit tote row: 0, 1, 2, or 3"),
        BT::InputPort<double>("server_timeout", 10.0, "Timeout for operation (s)")
      };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

  private:
    void orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg);
    void loadConfig();
    double getTargetHeight();

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<ServoHeight>::SharedPtr action_client_;

    rclcpp::Subscription<r1_interfaces::msg::PickTicket>::SharedPtr order_sub_;
    std::mutex data_mutex_;
    r1_interfaces::msg::PickTicket::SharedPtr current_order_;

    std::string target_type_;
    std::string explicit_bin_;
    int explicit_tote_{3};
    bool has_explicit_bin_{false};
    bool has_explicit_tote_{false};
    double timeout_;
    rclcpp::Time start_time_;

    // Per-shelf-letter (A/B/C) servo target height for BIN, taken from any
    // row's shelf_heights (all rows currently use the same set of shelf names).
    std::map<std::string, int> shelf_height_map_;
    // Per-row (e.g. "X1", "Y50") single search height for BIN_SEARCH.
    std::map<std::string, int> row_search_height_map_;
    std::map<int, int> tote_height_map_;
    std::map<int, int> tote_search_height_map_;

    GoalHandleServoHeight::SharedPtr goal_handle_;
    std::atomic<bool> goal_done_{false};
    bool goal_succeeded_{false};
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__SERVO_TO_TARGET_ACTION_HPP_

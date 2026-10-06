#ifndef R1_NAV_CPP__BT_NODES__SCAN_FOR_BARCODE_ACTION_HPP_
#define R1_NAV_CPP__BT_NODES__SCAN_FOR_BARCODE_ACTION_HPP_

#include <string>
#include <memory>
#include <mutex>
#include <atomic>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "r1_interfaces/action/scan_for_barcode.hpp"
#include "r1_interfaces/msg/pick_ticket.hpp"

namespace r1_bt_nodes
{

/**
 * @brief BT Action Node to call the scan_for_barcode action server
 * 
 * This node:
 * - Subscribes to /teleop/order to get the current order
 * - Generates the target barcode from the order's bin info
 * - Calls the scan_for_barcode action server
 * - Outputs whether the barcode was found
 * 
 * Inputs:
 *   - timeout: Scan timeout in seconds (default: 10.0)
 *   - target_barcode: Optional explicit barcode to scan for (if not provided, 
 *                     generates from current order)
 * 
 * Outputs:
 *   - barcode_found: true if target barcode was found during scan
 *   - scanned_barcode: the barcode string that was scanned for
 */
class ScanForBarcodeAction : public BT::StatefulActionNode
{
public:
  using ScanForBarcode = r1_interfaces::action::ScanForBarcode;
  using GoalHandleScan = rclcpp_action::ClientGoalHandle<ScanForBarcode>;

  ScanForBarcodeAction(
    const std::string & name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout", 10.0, "Scan timeout in seconds"),
      BT::InputPort<std::string>("target_barcode", "", 
        "Explicit barcode to scan for (empty = generate from order)"),
      BT::OutputPort<bool>("barcode_found", "True if barcode was found"),
      BT::OutputPort<std::string>("scanned_barcode", "The barcode that was scanned for")
    };
  }

  // BT Lifecycle methods
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Helper to generate barcode string from order
  std::string generateBarcodeFromOrder(const r1_interfaces::msg::PickTicket & order);

  // Order callback
  void orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg);

  // Action callbacks
  void goalResponseCallback(const GoalHandleScan::SharedPtr & goal_handle);
  void feedbackCallback(
    GoalHandleScan::SharedPtr goal_handle,
    const std::shared_ptr<const ScanForBarcode::Feedback> feedback);
  void resultCallback(const GoalHandleScan::WrappedResult & result);

  // ROS Node Handle (shared from Nav2)
  rclcpp::Node::SharedPtr node_;

  // Action client
  rclcpp_action::Client<ScanForBarcode>::SharedPtr action_client_;

  // Order subscriber
  rclcpp::Subscription<r1_interfaces::msg::PickTicket>::SharedPtr order_sub_;

  // State
  std::mutex data_mutex_;
  r1_interfaces::msg::PickTicket::SharedPtr current_order_;
  
  // Action state
  std::atomic<bool> goal_accepted_{false};
  std::atomic<bool> action_done_{false};
  std::atomic<bool> action_success_{false};
  GoalHandleScan::SharedPtr goal_handle_;
  
  // Parameters
  std::string target_barcode_;
  double timeout_;
  
  // Timing
  rclcpp::Time start_time_;
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__SCAN_FOR_BARCODE_ACTION_HPP_

#include <memory>
#include <string>
#include <chrono>

#include "r1_nav_cpp/bt_nodes/scan_for_barcode_action.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/qos.hpp"
#include "rclcpp/logging.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace r1_bt_nodes
{

ScanForBarcodeAction::ScanForBarcodeAction(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  goal_accepted_(false),
  action_done_(false),
  action_success_(false),
  timeout_(10.0)
{
  // Get shared Nav2 node from blackboard
  if (!config().blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "ScanForBarcodeAction: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "ScanForBarcodeAction: using shared Nav2 node '%s'",
    node_->get_name());

  // Create action client
  action_client_ = rclcpp_action::create_client<ScanForBarcode>(
    node_, "scan_for_barcode");

  // Order subscriber (KEEP_LAST=1, RELIABLE, TRANSIENT_LOCAL)
  rclcpp::QoS order_qos(1);
  order_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  order_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  order_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  order_sub_ = node_->create_subscription<r1_interfaces::msg::PickTicket>(
    "/teleop/order",
    order_qos,
    std::bind(&ScanForBarcodeAction::orderCallback, this, _1));

  RCLCPP_INFO(node_->get_logger(), "ScanForBarcodeAction: initialized successfully");
}

void ScanForBarcodeAction::orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (msg->item_name == "NONE") {
    current_order_.reset();
  } else {
    current_order_ = msg;
  }
}

std::string ScanForBarcodeAction::generateBarcodeFromOrder(
  const r1_interfaces::msg::PickTicket & order)
{
  if (!order.bin_tag_txt.empty()) {
    return order.bin_tag_txt;
  }
  const auto & bin = order.bin;
  char side_char = bin.side.empty() ? 'X' : bin.side[0];
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%c%ld-%02ld-%s-%02ld",
    side_char,
    static_cast<long>(bin.row),
    static_cast<long>(bin.bay),
    bin.shelf.c_str(),
    static_cast<long>(bin.bin_number));
  return std::string(buf);
}

BT::NodeStatus ScanForBarcodeAction::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "ScanForBarcodeAction: onStart()");

  // Reset state
  goal_accepted_ = false;
  action_done_ = false;
  action_success_ = false;
  goal_handle_.reset();

  // Read timeout from input port
  if (!getInput("timeout", timeout_)) {
    timeout_ = 10.0;
  }

  // Read target barcode - either explicit or generate from order
  std::string explicit_barcode;
  getInput("target_barcode", explicit_barcode);

  if (!explicit_barcode.empty()) {
    target_barcode_ = explicit_barcode;
  } else {
    // Generate from current order
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!current_order_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "ScanForBarcodeAction: no current order and no explicit barcode provided");
      return BT::NodeStatus::FAILURE;
    }
    target_barcode_ = generateBarcodeFromOrder(*current_order_);
  }

  // Output the barcode we're scanning for
  setOutput("scanned_barcode", target_barcode_);

  RCLCPP_INFO(
    node_->get_logger(),
    "ScanForBarcodeAction: scanning for barcode '%s' with timeout %.1fs",
    target_barcode_.c_str(), timeout_);

  // Check if action server is available
  if (!action_client_->wait_for_action_server(1s)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ScanForBarcodeAction: action server 'scan_for_barcode' not available");
    return BT::NodeStatus::FAILURE;
  }

  // Send goal
  auto goal_msg = ScanForBarcode::Goal();
  goal_msg.target_barcode = target_barcode_;
  goal_msg.timeout = static_cast<float>(timeout_);

  auto send_goal_options = rclcpp_action::Client<ScanForBarcode>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&ScanForBarcodeAction::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&ScanForBarcodeAction::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&ScanForBarcodeAction::resultCallback, this, _1);

  action_client_->async_send_goal(goal_msg, send_goal_options);

  start_time_ = node_->get_clock()->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ScanForBarcodeAction::onRunning()
{
  // Process callbacks
  rclcpp::spin_some(node_);

  // Check if action completed
  if (action_done_.load()) {
    bool found = action_success_.load();
    setOutput("barcode_found", found);
    
    RCLCPP_INFO(
      node_->get_logger(),
      "ScanForBarcodeAction: completed, barcode_found=%s",
      found ? "true" : "false");
    
    return BT::NodeStatus::SUCCESS;
    // Note: We return SUCCESS even if barcode not found - the BT can check barcode_found output
  }

  // Check for timeout (with buffer for action server timeout)
  const auto now = node_->get_clock()->now();
  const double elapsed = (now - start_time_).seconds();
  if (elapsed > timeout_ + 5.0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ScanForBarcodeAction: overall timeout (%.1fs > %.1fs + 5s buffer)",
      elapsed, timeout_);
    setOutput("barcode_found", false);
    return BT::NodeStatus::SUCCESS;  // Timeout is not a failure, just barcode not found
  }

  return BT::NodeStatus::RUNNING;
}

void ScanForBarcodeAction::onHalted()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "ScanForBarcodeAction: onHalted() - cancelling scan");

  // Cancel the goal if we have one
  if (goal_handle_ && goal_accepted_.load()) {
    action_client_->async_cancel_goal(goal_handle_);
  }

  setOutput("barcode_found", false);
}

void ScanForBarcodeAction::goalResponseCallback(
  const GoalHandleScan::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "ScanForBarcodeAction: goal was rejected by action server");
    action_done_ = true;
    action_success_ = false;
    return;
  }

  goal_handle_ = goal_handle;
  goal_accepted_ = true;
  
  RCLCPP_DEBUG(
    node_->get_logger(),
    "ScanForBarcodeAction: goal accepted by action server");
}

void ScanForBarcodeAction::feedbackCallback(
  GoalHandleScan::SharedPtr /*goal_handle*/,
  const std::shared_ptr<const ScanForBarcode::Feedback> feedback)
{
  RCLCPP_DEBUG(
    node_->get_logger(),
    "ScanForBarcodeAction: %.1fs remaining",
    feedback->time_left);
}

void ScanForBarcodeAction::resultCallback(
  const GoalHandleScan::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      action_success_ = result.result->success;
      RCLCPP_INFO(
        node_->get_logger(),
        "ScanForBarcodeAction: action succeeded, barcode %s",
        action_success_ ? "FOUND" : "NOT FOUND");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(
        node_->get_logger(),
        "ScanForBarcodeAction: action was aborted");
      action_success_ = false;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(
        node_->get_logger(),
        "ScanForBarcodeAction: action was canceled (halted by BT)");
      action_success_ = false;
      break;
    default:
      RCLCPP_ERROR(
        node_->get_logger(),
        "ScanForBarcodeAction: unknown result code");
      action_success_ = false;
      break;
  }

  action_done_ = true;
}

}  // namespace r1_bt_nodes

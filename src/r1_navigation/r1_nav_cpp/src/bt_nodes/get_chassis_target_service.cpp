#include <memory>
#include <string>

#include "r1_nav_cpp/bt_nodes/get_chassis_target_service.hpp"

namespace r1_bt_nodes
{

GetChassisTargetService::GetChassisTargetService(
  const std::string & xml_tag_name,
  const std::string & service_name,
  const BT::NodeConfig & conf)
: BtServiceNode<Service>(xml_tag_name, conf, service_name)
, last_target_type_("CENTER")
{
}

void GetChassisTargetService::on_tick()
{
  std::string target_type;
  if (!getInput("target_type", target_type)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GetChassisTargetService: missing required input [target_type]");
    should_send_request_ = false;
    return;
  }

  last_target_type_ = target_type;
  request_->target_type = target_type;

  RCLCPP_DEBUG(
    node_->get_logger(),
    "GetChassisTargetService: requesting chassis target for [%s]",
    target_type.c_str());
}

BT::NodeStatus GetChassisTargetService::on_completion(
  std::shared_ptr<ServiceResponse> response)
{
  if (!response) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "GetChassisTargetService: null response, returning FAILURE.");
    return BT::NodeStatus::FAILURE;
  }

  if (!response->success) {
    RCLCPP_WARN(
      node_->get_logger(),
      "GetChassisTargetService: service reported failure: %s",
      response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Stamp the best_estimate with "now" on the BT node
  geometry_msgs::msg::PoseStamped best_estimate_ = response->best_estimate;
  best_estimate_.header.stamp = node_->now();
  setOutput("best_estimate_pose", best_estimate_);

  // CENTER: no detection expected, just succeed
  if (last_target_type_ == "CENTER") {
    RCLCPP_INFO(
      node_->get_logger(),
      "GetChassisTargetService: service completed for target_type=CENTER.");
    return BT::NodeStatus::SUCCESS;
  }

  // BIN / TOTE: output detection info
  setOutput("target_detected", response->detected);
  setOutput("target_confidence", static_cast<double>(response->confidence));
  
  // Output target barcode (primarily for BIN)
  if (!response->target_barcode.empty()) {
    setOutput("target_barcode", response->target_barcode);
  }
  
  // Output scan_pose (for barcode scanning with extra standoff)
  setOutput("scan_pose", response->scan_pose);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "GetChassisTargetService: target_type=%s, detected=%s, confidence=%.3f, barcode='%s'",
    last_target_type_.c_str(),
    response->detected ? "true" : "false",
    response->confidence,
    response->target_barcode.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace r1_bt_nodes

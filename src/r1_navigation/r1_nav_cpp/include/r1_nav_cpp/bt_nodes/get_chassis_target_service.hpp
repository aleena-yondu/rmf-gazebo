#ifndef R1_NAV_CPP__BT_NODES__GET_CHASSIS_TARGET_SERVICE_HPP_
#define R1_NAV_CPP__BT_NODES__GET_CHASSIS_TARGET_SERVICE_HPP_

#include <string>
#include <memory>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/bt_service_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "r1_interfaces/srv/get_chassis_target.hpp"

namespace r1_bt_nodes
{

/**
 * @brief BT service node wrapper around r1_interfaces::srv::GetChassisTarget
 *
 * Inputs:
 *  - target_type          (IN, string): "BIN", "TOTE", or "CENTER"
 *
 * Outputs:
 *  - target_detected     (OUT, bool)         : true if barcode scanned enough times
 *  - target_confidence   (OUT, double)       : detection confidence (0.0-1.0)
 *  - best_estimate_pose  (OUT, PoseStamped)  : best pose (refined if detected, YAML otherwise)
 *  - target_barcode      (OUT, string)       : expected barcode for target (BIN only)
 *  - scan_pose           (OUT, PoseStamped)  : pose for barcode scanning (BIN only)
 */
class GetChassisTargetService
  : public nav2_behavior_tree::BtServiceNode<r1_interfaces::srv::GetChassisTarget>
{
public:
  using Service         = r1_interfaces::srv::GetChassisTarget;
  using ServiceRequest  = Service::Request;
  using ServiceResponse = Service::Response;

  GetChassisTargetService(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>(
          "target_type",
          "CENTER",
          "Target type: BIN, TOTE, or CENTER"),

        BT::OutputPort<bool>(
          "target_detected",
          "True if barcode was detected and cached"),
        BT::OutputPort<double>(
          "target_confidence",
          "Detection confidence (0.0-1.0)"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "best_estimate_pose",
          "Best pose (refined if detected, YAML otherwise)"),
        BT::OutputPort<std::string>(
          "target_barcode",
          "Expected barcode for target location (BIN only)"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>(
          "scan_pose",
          "Pose for barcode scanning (BIN only)")
      });
  }

protected:
  void on_tick() override;

  BT::NodeStatus on_completion(
    std::shared_ptr<ServiceResponse> response) override;

private:
  // Remember which target_type we requested, mainly for logging
  std::string last_target_type_;
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__GET_CHASSIS_TARGET_SERVICE_HPP_

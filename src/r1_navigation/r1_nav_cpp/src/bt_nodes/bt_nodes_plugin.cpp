#include "behaviortree_cpp/bt_factory.h"

#include "r1_nav_cpp/bt_nodes/always_running.hpp"
#include "r1_nav_cpp/bt_nodes/servo_to_target_action.hpp"
#include "r1_nav_cpp/bt_nodes/get_chassis_target_service.hpp"
#include "r1_nav_cpp/bt_nodes/update_order_state.hpp"
#include "r1_nav_cpp/bt_nodes/set_blackboard_string.hpp"
#include "r1_nav_cpp/bt_nodes/debug_print.hpp"
#include "r1_nav_cpp/bt_nodes/run_inference_bt_node.hpp"
#include "r1_nav_cpp/bt_nodes/publish_pose.hpp"
#include "r1_nav_cpp/bt_nodes/notify_operator_ready.hpp"
#include "r1_nav_cpp/bt_nodes/set_tolerance.hpp"
#include "r1_nav_cpp/bt_nodes/pan_scanner.hpp"
#include "r1_nav_cpp/bt_nodes/scan_for_barcode_action.hpp"
#include "r1_nav_cpp/bt_nodes/compute_spin_angle.hpp"
#include "r1_nav_cpp/bt_nodes/solve_ik_bt_node.hpp"
#include "r1_nav_cpp/bt_nodes/cycle_height_servo_action.hpp"
#include "r1_nav_cpp/bt_nodes/highlight_target_bt_node.hpp"
#include "r1_nav_cpp/bt_nodes/search_bins_bt_node.hpp"
#include "r1_nav_cpp/bt_nodes/follow_right_arm_trajectory.hpp"
#include "r1_nav_cpp/bt_nodes/select_inference_action_type.hpp"

BT_REGISTER_NODES(factory)
{

  // AlwaysRunning
  {
    factory.registerNodeType<r1_bt_nodes::AlwaysRunning>(
      "AlwaysRunning");
  }

  // PublishPose
  {
    factory.registerNodeType<r1_bt_nodes::PublishPose>(
      "PublishPose");
  }

  // NotifyOperatorReady
  {
    factory.registerNodeType<r1_bt_nodes::NotifyOperatorReady>(
      "NotifyOperatorReady");
  }

  // SetTolerance
  {
    factory.registerNodeType<r1_bt_nodes::SetTolerance>(
      "SetTolerance");
  }

  // ServoToTargetAction
  {
    factory.registerNodeType<r1_bt_nodes::ServoToTargetAction>(
      "ServoToTarget");
  }

  // RunInferenceBtNode
  {
    factory.registerNodeType<r1_bt_nodes::RunInferenceBtNode>(
      "RunInference");
  }

  // SelectInferenceActionType
  {
    factory.registerNodeType<r1_bt_nodes::SelectInferenceActionType>(
      "SelectInferenceActionType");
  }

  // GetChassisTargetService
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfig & config)
      {
        return std::make_unique<r1_bt_nodes::GetChassisTargetService>(
          name,
          "/get_chassis_target",
          config);
      };

    factory.registerBuilder<r1_bt_nodes::GetChassisTargetService>(
      "GetChassisTarget",
      builder);
  }

  // UpdateOrderState
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfig & config)
      {
        return std::make_unique<r1_bt_nodes::UpdateOrderState>(name, config);
      };

    factory.registerBuilder<r1_bt_nodes::UpdateOrderState>(
      "UpdateOrderState",
      builder);
  }

  // SetBlackboardString
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfig & config)
      {
        return std::make_unique<r1_bt_nodes::SetBlackboardString>(name, config);
      };

    factory.registerBuilder<r1_bt_nodes::SetBlackboardString>(
      "SetBlackboardString",
      builder);
  }

  // DebugPrint
  {
    BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfig & config)
      {
        return std::make_unique<r1_bt_nodes::DebugPrint>(name, config);
      };

    factory.registerBuilder<r1_bt_nodes::DebugPrint>(
      "DebugPrint",
      builder);
  }

  // PanScanner
  {
    factory.registerNodeType<r1_bt_nodes::PanScanner>(
      "PanScanner");
  }

  // ScanForBarcodeAction
  {
    factory.registerNodeType<r1_bt_nodes::ScanForBarcodeAction>(
      "ScanForBarcode");
  }

  // ComputeSpinAngle
  {
    factory.registerNodeType<r1_bt_nodes::ComputeSpinAngle>(
      "ComputeSpinAngle");
  }

  // SolveIkBtNode
  {
    factory.registerNodeType<r1_bt_nodes::SolveIkBtNode>(
      "SolveIk");
  }

  // CycleHeightServoAction
  {
    factory.registerNodeType<r1_bt_nodes::CycleHeightServoAction>(
      "CycleHeightServo");
  }

  // HighlightTargetBtNode
  {
    factory.registerNodeType<r1_bt_nodes::HighlightTargetBtNode>(
      "HighlightTarget");
  }

  // SearchBinsBtNode
  {
    factory.registerNodeType<r1_bt_nodes::SearchBinsBtNode>(
      "SearchBins");
  }

  // FollowRightArmTrajectory
  {
    factory.registerNodeType<r1_bt_nodes::FollowRightArmTrajectory>(
      "FollowRightArmTrajectory");
  }
}

#pragma once

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <r1_interfaces/action/highlight_target.hpp>
#include <r1_interfaces/srv/get_target_bounding_boxes.hpp>

#include <atomic>
#include <mutex>
#include <string>

namespace r1_bt_nodes
{

class HighlightTargetBtNode : public BT::StatefulActionNode
{
public:
  using HighlightTarget = r1_interfaces::action::HighlightTarget;
  using GoalHandle = rclcpp_action::ClientGoalHandle<HighlightTarget>;
  using GetTargetBoundingBoxes = r1_interfaces::srv::GetTargetBoundingBoxes;

  HighlightTargetBtNode(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    IDLE,
    WAIT_SERVICE,
    WAIT_GOAL_HANDLE,
    WAIT_RESULT
  };

  void resetInternal();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<HighlightTarget>::SharedPtr action_client_;
  rclcpp::Client<GetTargetBoundingBoxes>::SharedPtr service_client_;

  std::mutex mutex_;
  std::shared_ptr<GoalHandle> active_goal_handle_;

  std::shared_future<std::shared_ptr<GoalHandle>> goal_handle_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  std::shared_future<GetTargetBoundingBoxes::Response::SharedPtr> service_future_;

  std::string target_type_;
  std::atomic_bool halt_requested_{false};
  Phase phase_{Phase::IDLE};
};

}  // namespace r1_bt_nodes

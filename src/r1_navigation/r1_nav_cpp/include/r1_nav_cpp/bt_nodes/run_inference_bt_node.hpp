#pragma once

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <r1_interfaces/action/run_inference.hpp>

#include <atomic>
#include <mutex>
#include <string>


namespace r1_bt_nodes
{

class RunInferenceBtNode : public BT::StatefulActionNode
{
public:
  using RunInference = r1_interfaces::action::RunInference;
  using GoalHandle = rclcpp_action::ClientGoalHandle<RunInference>;

  RunInferenceBtNode(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    IDLE,
    WAIT_GOAL_HANDLE,
    WAIT_RESULT
  };

  bool getInputs(int64_t& steps, int64_t& sleep_ms, std::string& error_out);
  void resetInternal();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<RunInference>::SharedPtr client_;

  std::mutex mutex_;
  std::shared_ptr<GoalHandle> active_goal_handle_;

  // Track across ticks
  std::shared_future<std::shared_ptr<GoalHandle>> goal_handle_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;

  std::atomic_bool halt_requested_{false};
  Phase phase_{Phase::IDLE};
};

}  // namespace r1_bt_nodes
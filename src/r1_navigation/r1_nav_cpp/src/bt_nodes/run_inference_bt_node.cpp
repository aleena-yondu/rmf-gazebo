#include "r1_nav_cpp/bt_nodes/run_inference_bt_node.hpp"
#include "behaviortree_cpp/blackboard.h"

#include <chrono>

namespace r1_bt_nodes
{

RunInferenceBtNode::RunInferenceBtNode(const std::string& name,
                                       const BT::NodeConfig& config)
: BT::StatefulActionNode(name, config)
{
  // Get shared Nav2 node from blackboard
  if (!config.blackboard) {
    throw BT::RuntimeError(
      "RunInferenceBtNode: blackboard is null. This BT node must run inside Nav2 bt_navigator.");
  }

  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "RunInferenceBtNode: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  client_ = rclcpp_action::create_client<RunInference>(node_, "/run_inference");
}

BT::PortsList RunInferenceBtNode::providedPorts()
{
  return {
    BT::InputPort<int64_t>("steps", 50, "Number of timesteps to publish"),
    BT::InputPort<int64_t>("sleep", 20, "Milliseconds to sleep between timesteps"),
    BT::InputPort<std::string>("action_type", "pick", "Either pick or place"),
    BT::OutputPort<std::string>("message", "Status/feedback message from the node")
  };
}

bool RunInferenceBtNode::getInputs(int64_t& steps, int64_t& sleep_ms, std::string& error_out)
{
  auto steps_res = getInput<int64_t>("steps");
  if (!steps_res) {
    error_out = "Missing/invalid input port [steps]: " + steps_res.error();
    return false;
  }
  steps = steps_res.value();

  auto sleep_res = getInput<int64_t>("sleep");
  if (!sleep_res) {
    error_out = "Missing/invalid input port [sleep]: " + sleep_res.error();
    return false;
  }
  sleep_ms = sleep_res.value();

  if (steps <= 0) {
    error_out = "Input [steps] must be > 0";
    return false;
  }
  if (sleep_ms < 0) {
    error_out = "Input [sleep] must be >= 0";
    return false;
  }
  return true;
}

void RunInferenceBtNode::resetInternal()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_goal_handle_.reset();
  goal_handle_future_ = std::shared_future<std::shared_ptr<GoalHandle>>{};
  result_future_ = std::shared_future<GoalHandle::WrappedResult>{};
  phase_ = Phase::IDLE;
}

BT::NodeStatus RunInferenceBtNode::onStart()
{
  halt_requested_.store(false);
  resetInternal();

  int64_t steps = 0;
  int64_t sleep_ms = 0;
  std::string action_type;
  std::string err;
  if (!getInputs(steps, sleep_ms, err)) {
    setOutput("message", err);
    RCLCPP_ERROR(node_->get_logger(), "RunInferenceBtNode: %s", err.c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!getInput<std::string>("action_type", action_type)) {
    action_type = "pick";
    RCLCPP_INFO(
      node_->get_logger(),
      "RunInferenceBtNode: missing input [action_type], defaulting to 'pick'");
  }

  if (!client_) {
    const std::string msg = "Action client is null";
    setOutput("message", msg);
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Wait briefly for action server (non-fatal but practical)
  if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
    const std::string msg = "RunInference action server [/run_inference] not available";
    setOutput("message", msg);
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RunInference::Goal goal;
  goal.steps = steps;
  goal.sleep = sleep_ms;
  goal.action_type = action_type;

  rclcpp_action::Client<RunInference>::SendGoalOptions opts;

  opts.feedback_callback =
    [this](GoalHandle::SharedPtr /*gh*/,
           const std::shared_ptr<const RunInference::Feedback> feedback)
    {
      if (feedback) {
        setOutput("message", feedback->message);
      }
    };

  opts.goal_response_callback =
    [this](std::shared_ptr<GoalHandle> gh)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_handle_ = gh;
      setOutput("message", gh ? "Goal accepted by /run_inference server"
                              : "Goal rejected by /run_inference server");
    };

  // Store future across ticks
  goal_handle_future_ = client_->async_send_goal(goal, opts);
  phase_ = Phase::WAIT_GOAL_HANDLE;

  setOutput("message", "Goal sent; waiting for goal handle...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RunInferenceBtNode::onRunning()
{
  rclcpp::spin_some(node_);
  if (halt_requested_.load()) {
    setOutput("message", "Halt requested");
    return BT::NodeStatus::FAILURE;
  }

  if (phase_ == Phase::IDLE) {
    // Should not happen normally; treat as running
    return BT::NodeStatus::RUNNING;
  }

  // Step 1: wait for goal handle
  if (phase_ == Phase::WAIT_GOAL_HANDLE) {
    if (!goal_handle_future_.valid()) {
      setOutput("message", "Goal handle future invalid");
      RCLCPP_ERROR(node_->get_logger(), "RunInferenceBtNode: goal_handle_future_ invalid");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    if (goal_handle_future_.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    auto gh = goal_handle_future_.get();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_handle_ = gh;
    }

    if (!gh) {
      setOutput("message", "Goal rejected by server");
      RCLCPP_ERROR(node_->get_logger(), "RunInferenceBtNode: goal rejected by server");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    // Now request result and move to next phase
    result_future_ = client_->async_get_result(gh);
    phase_ = Phase::WAIT_RESULT;

    setOutput("message", "Goal accepted; waiting for result...");
    return BT::NodeStatus::RUNNING;
  }

  // Step 2: wait for result
  if (phase_ == Phase::WAIT_RESULT) {
    if (!result_future_.valid()) {
      setOutput("message", "Result future invalid");
      RCLCPP_ERROR(node_->get_logger(), "RunInferenceBtNode: result_future_ invalid");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    auto wrapped = result_future_.get();

    // Clear active goal state
    resetInternal();

    switch (wrapped.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: {
        const bool ok = wrapped.result && wrapped.result->success;
        setOutput("message", ok ? "RunInference succeeded"
                                : "RunInference returned success=false");
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      case rclcpp_action::ResultCode::ABORTED:
        setOutput("message", "RunInference aborted");
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        setOutput("message", "RunInference canceled");
        return BT::NodeStatus::FAILURE;

      default:
        setOutput("message", "RunInference unknown result code");
        return BT::NodeStatus::FAILURE;
    }
  }

  // Fallback
  return BT::NodeStatus::RUNNING;
}

void RunInferenceBtNode::onHalted()
{
  halt_requested_.store(true);

  std::shared_ptr<GoalHandle> gh;

  // First check if we already have the goal handle cached
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gh = active_goal_handle_;
  }

  // If no goal handle yet but we have a pending future, spin and wait for it
  // This handles the case where halt is called before goal_response_callback fires
  // We must spin the node to receive the goal response from the action server
  if (!gh && goal_handle_future_.valid()) {
    RCLCPP_INFO(node_->get_logger(), 
                "RunInferenceBtNode::onHalted: Waiting for goal handle future (with spinning)...");
    
    auto start_time = std::chrono::steady_clock::now();
    constexpr auto timeout = std::chrono::seconds(2);
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
      // Spin to process incoming messages (goal response callback)
      rclcpp::spin_some(node_);
      
      // Check if future is ready
      if (goal_handle_future_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        try {
          gh = goal_handle_future_.get();
          RCLCPP_INFO(node_->get_logger(), 
                      "RunInferenceBtNode::onHalted: Got goal handle from future");
        } catch (const std::exception& e) {
          RCLCPP_INFO(node_->get_logger(), 
                      "RunInferenceBtNode::onHalted: Exception getting goal handle: %s", e.what());
        }
        break;
      }
      
      // Also check if it was set via the callback
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_goal_handle_) {
          gh = active_goal_handle_;
          RCLCPP_INFO(node_->get_logger(), 
                      "RunInferenceBtNode::onHalted: Got goal handle from callback");
          break;
        }
      }
    }
    
    if (!gh) {
      RCLCPP_INFO(node_->get_logger(), 
                  "RunInferenceBtNode::onHalted: Timeout waiting for goal handle");
    }
  }

  // Cancel the goal if we have a valid handle
  if (gh && client_) {
    RCLCPP_INFO(node_->get_logger(), 
                "RunInferenceBtNode::onHalted: Sending cancel request to /run_inference");
    auto cancel_future = client_->async_cancel_goal(gh);
    
    // Wait for cancel acknowledgment with spinning
    auto start_time = std::chrono::steady_clock::now();
    constexpr auto timeout = std::chrono::seconds(2);
    bool got_response = false;
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
      rclcpp::spin_some(node_);
      
      if (cancel_future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        try {
          auto cancel_response = cancel_future.get();
          if (cancel_response) {
            RCLCPP_INFO(node_->get_logger(), 
                        "RunInferenceBtNode::onHalted: Cancel request acknowledged");
          } else {
            RCLCPP_INFO(node_->get_logger(), 
                        "RunInferenceBtNode::onHalted: Cancel response is null");
          }
          got_response = true;
        } catch (const std::exception& e) {
          RCLCPP_INFO(node_->get_logger(), 
                      "RunInferenceBtNode::onHalted: Exception during cancel: %s", e.what());
          got_response = true;
        }
        break;
      }
    }
    
    if (!got_response) {
      RCLCPP_INFO(node_->get_logger(), 
                  "RunInferenceBtNode::onHalted: Timeout waiting for cancel acknowledgment");
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), 
                "RunInferenceBtNode::onHalted: No active goal handle to cancel (gh=%s, client=%s)",
                gh ? "valid" : "null", client_ ? "valid" : "null");
  }

  // Reset internal state
  resetInternal();
}

}  // namespace r1_bt_nodes
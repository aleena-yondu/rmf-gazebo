#include "r1_nav_cpp/bt_nodes/highlight_target_bt_node.hpp"
#include "behaviortree_cpp/blackboard.h"

#include <chrono>

namespace r1_bt_nodes
{

HighlightTargetBtNode::HighlightTargetBtNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard) {
    throw BT::RuntimeError(
      "HighlightTargetBtNode: blackboard is null.");
  }

  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "HighlightTargetBtNode: 'node' not found on blackboard.");
  }

  action_client_ = rclcpp_action::create_client<HighlightTarget>(
    node_, "/highlight_target");

  service_client_ = node_->create_client<GetTargetBoundingBoxes>(
    "get_target_bounding_boxes");

  RCLCPP_INFO(node_->get_logger(),
    "HighlightTargetBtNode: initialized "
    "(action=/highlight_target, service=get_target_bounding_boxes)");
}

BT::PortsList HighlightTargetBtNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("target_type", "NONE", "BIN, TOTE, or NONE"),
    BT::OutputPort<std::string>("message", "Status/feedback message")
  };
}

void HighlightTargetBtNode::resetInternal()
{
  std::lock_guard<std::mutex> lock(mutex_);
  active_goal_handle_.reset();
  goal_handle_future_ = std::shared_future<std::shared_ptr<GoalHandle>>{};
  result_future_ = std::shared_future<GoalHandle::WrappedResult>{};
  service_future_ = std::shared_future<GetTargetBoundingBoxes::Response::SharedPtr>{};
  phase_ = Phase::IDLE;
}

BT::NodeStatus HighlightTargetBtNode::onStart()
{
  halt_requested_.store(false);
  resetInternal();

  std::string target_type;
  if (!getInput<std::string>("target_type", target_type)) {
    target_type = "NONE";
  }

  if (target_type == "NONE") {
    if (action_client_->wait_for_action_server(std::chrono::milliseconds(200))) {
      action_client_->async_cancel_all_goals();
      rclcpp::spin_some(node_);
      RCLCPP_INFO(node_->get_logger(),
        "HighlightTargetBtNode: target_type=NONE, cancelled all highlight goals");
    } else {
      RCLCPP_DEBUG(node_->get_logger(),
        "HighlightTargetBtNode: target_type=NONE, action server not available (nothing to cancel)");
    }
    setOutput("message", "Cancelled highlight");
    return BT::NodeStatus::SUCCESS;
  }

  target_type_ = target_type;
  RCLCPP_INFO(node_->get_logger(),
    "HighlightTargetBtNode::onStart: target_type=%s, requesting bounding boxes",
    target_type_.c_str());

  if (!service_client_->wait_for_service(std::chrono::seconds(1))) {
    setOutput("message", "get_target_bounding_boxes service not available");
    RCLCPP_ERROR(node_->get_logger(),
      "HighlightTargetBtNode: get_target_bounding_boxes service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<GetTargetBoundingBoxes::Request>();
  request->target_type = target_type_;

  service_future_ = service_client_->async_send_request(request);
  phase_ = Phase::WAIT_SERVICE;

  RCLCPP_DEBUG(node_->get_logger(),
    "HighlightTargetBtNode::onStart: service request sent, entering WAIT_SERVICE");
  setOutput("message", "Requesting bounding boxes for " + target_type_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HighlightTargetBtNode::onRunning()
{
  rclcpp::spin_some(node_);
  if (halt_requested_.load()) {
    RCLCPP_INFO(node_->get_logger(),
      "HighlightTargetBtNode::onRunning: halt requested during phase %d",
      static_cast<int>(phase_));
    setOutput("message", "Halt requested");
    return BT::NodeStatus::FAILURE;
  }

  if (phase_ == Phase::IDLE) {
    return BT::NodeStatus::RUNNING;
  }

  // Phase 1: wait for service response
  if (phase_ == Phase::WAIT_SERVICE) {
    if (!service_future_.valid()) {
      setOutput("message", "Service future invalid");
      RCLCPP_ERROR(node_->get_logger(),
        "HighlightTargetBtNode: service_future_ invalid");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    if (service_future_.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    auto srv_response = service_future_.get();
    if (!srv_response || !srv_response->found) {
      std::string msg = srv_response ? srv_response->message : "null response";
      setOutput("message", "GetTargetBoundingBoxes failed: " + msg);
      RCLCPP_ERROR(node_->get_logger(),
        "HighlightTargetBtNode: GetTargetBoundingBoxes failed: %s", msg.c_str());
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(),
      "HighlightTargetBtNode: GetTargetBoundingBoxes succeeded "
      "(target=BIN, neighbors: left=%s right=%s)",
      srv_response->has_left ? "yes" : "no",
      srv_response->has_right ? "yes" : "no");

    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      setOutput("message", "/highlight_target action server not available");
      RCLCPP_ERROR(node_->get_logger(),
        "HighlightTargetBtNode: /highlight_target action server not available");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    HighlightTarget::Goal goal;
    goal.target_type = target_type_;
    goal.target_bbox = srv_response->target;
    if (srv_response->has_left) {
      goal.negative_bboxes.push_back(srv_response->left);
    }
    if (srv_response->has_right) {
      goal.negative_bboxes.push_back(srv_response->right);
    }
    // Forward the 3D shelf plane straight from the service response into the
    // action goal. Same field-for-field definition as
    // r1_interfaces/srv/DetectBinsOnShelf.srv. The camera highlight server
    // can use these to render bboxes inside the correct 3D shelf context.
    goal.shelf_start = srv_response->shelf_start;
    goal.shelf_end = srv_response->shelf_end;
    goal.shelf_bottom_z = srv_response->shelf_bottom_z;
    goal.shelf_top_z = srv_response->shelf_top_z;
    goal.shelf_yaw_rad = srv_response->shelf_yaw_rad;

    rclcpp_action::Client<HighlightTarget>::SendGoalOptions opts;

    opts.feedback_callback =
      [this](GoalHandle::SharedPtr,
             const std::shared_ptr<const HighlightTarget::Feedback> feedback)
      {
        if (feedback) {
          RCLCPP_DEBUG(node_->get_logger(),
            "HighlightTargetBtNode: feedback: %s",
            feedback->current_detection_sub_state.c_str());
          setOutput("message", feedback->current_detection_sub_state);
        }
      };

    opts.goal_response_callback =
      [this](std::shared_ptr<GoalHandle> gh)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        active_goal_handle_ = gh;
      };

    goal_handle_future_ = action_client_->async_send_goal(goal, opts);
    phase_ = Phase::WAIT_GOAL_HANDLE;

    {
      const auto & v = goal.target_bbox.vertices;
      RCLCPP_INFO(node_->get_logger(),
        "HighlightTargetBtNode: sending goal type=%s "
        "target_bbox bl=(%.3f,%.3f,%.3f) tr=(%.3f,%.3f,%.3f) "
        "neg_count=%zu",
        target_type_.c_str(),
        v[0].x, v[0].y, v[0].z,
        v[2].x, v[2].y, v[2].z,
        goal.negative_bboxes.size());
    }

    setOutput("message", "Goal sent to /highlight_target");
    return BT::NodeStatus::RUNNING;
  }

  // Phase 2: wait for goal handle
  if (phase_ == Phase::WAIT_GOAL_HANDLE) {
    if (!goal_handle_future_.valid()) {
      setOutput("message", "Goal handle future invalid");
      RCLCPP_ERROR(node_->get_logger(),
        "HighlightTargetBtNode: goal_handle_future_ invalid");
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
      setOutput("message", "Goal rejected by /highlight_target server");
      RCLCPP_ERROR(node_->get_logger(),
        "HighlightTargetBtNode: goal rejected");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(),
      "HighlightTargetBtNode: goal accepted by /highlight_target (fire-and-forget, highlight active)");
    setOutput("message", "Highlight active");
    resetInternal();
    return BT::NodeStatus::SUCCESS;
  }

  // Phase 3: wait for result
  if (phase_ == Phase::WAIT_RESULT) {
    if (!result_future_.valid()) {
      setOutput("message", "Result future invalid");
      RCLCPP_ERROR(node_->get_logger(),
        "HighlightTargetBtNode: result_future_ invalid");
      resetInternal();
      return BT::NodeStatus::FAILURE;
    }

    if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }

    auto wrapped = result_future_.get();
    resetInternal();

    switch (wrapped.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: {
        const bool ok = wrapped.result && wrapped.result->success;
        RCLCPP_INFO(node_->get_logger(),
          "HighlightTargetBtNode: action SUCCEEDED, result.success=%s",
          ok ? "true" : "false");
        setOutput("message", ok ? "HighlightTarget succeeded"
                                : "HighlightTarget returned success=false");
        return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(node_->get_logger(),
          "HighlightTargetBtNode: action ABORTED by server");
        setOutput("message", "HighlightTarget aborted");
        return BT::NodeStatus::FAILURE;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(node_->get_logger(),
          "HighlightTargetBtNode: action CANCELED");
        setOutput("message", "HighlightTarget canceled");
        return BT::NodeStatus::FAILURE;
      default:
        RCLCPP_WARN(node_->get_logger(),
          "HighlightTargetBtNode: action returned unknown result code %d",
          static_cast<int>(wrapped.code));
        setOutput("message", "HighlightTarget unknown result code");
        return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void HighlightTargetBtNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(),
    "HighlightTargetBtNode::onHalted: halting during phase %d",
    static_cast<int>(phase_));
  halt_requested_.store(true);

  std::shared_ptr<GoalHandle> gh;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    gh = active_goal_handle_;
  }

  if (!gh && goal_handle_future_.valid()) {
    RCLCPP_DEBUG(node_->get_logger(),
      "HighlightTargetBtNode::onHalted: no goal handle yet, waiting for pending future");

    auto start_time = std::chrono::steady_clock::now();
    constexpr auto timeout = std::chrono::seconds(2);

    while (std::chrono::steady_clock::now() - start_time < timeout) {
      rclcpp::spin_some(node_);

      if (goal_handle_future_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        try {
          gh = goal_handle_future_.get();
        } catch (const std::exception & e) {
          RCLCPP_INFO(node_->get_logger(),
            "HighlightTargetBtNode::onHalted: exception getting goal handle: %s", e.what());
        }
        break;
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_goal_handle_) {
          gh = active_goal_handle_;
          break;
        }
      }
    }
  }

  if (gh && action_client_) {
    RCLCPP_INFO(node_->get_logger(),
      "HighlightTargetBtNode::onHalted: cancelling /highlight_target goal");
    auto cancel_future = action_client_->async_cancel_goal(gh);

    auto start_time = std::chrono::steady_clock::now();
    constexpr auto timeout = std::chrono::seconds(2);
    bool got_cancel = false;

    while (std::chrono::steady_clock::now() - start_time < timeout) {
      rclcpp::spin_some(node_);
      if (cancel_future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
        got_cancel = true;
        break;
      }
    }
    RCLCPP_INFO(node_->get_logger(),
      "HighlightTargetBtNode::onHalted: cancel %s",
      got_cancel ? "acknowledged" : "timed out");
  } else {
    RCLCPP_DEBUG(node_->get_logger(),
      "HighlightTargetBtNode::onHalted: no active goal to cancel (phase was %d)",
      static_cast<int>(phase_));
  }

  resetInternal();
  RCLCPP_INFO(node_->get_logger(), "HighlightTargetBtNode::onHalted: cleanup complete");
}

}  // namespace r1_bt_nodes

#include "r1_nav_cpp/bt_nodes/search_bins_bt_node.hpp"

#include "behaviortree_cpp/blackboard.h"

#include <chrono>
#include <cstdio>

namespace r1_bt_nodes
{

SearchBinsBtNode::SearchBinsBtNode(
  const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf)
{
  if (!config().blackboard) {
    throw BT::RuntimeError("SearchBinsBtNode: blackboard is null");
  }
  if (!config().blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "SearchBinsBtNode: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  detect_client_ = node_->create_client<DetectBinsOnShelf>("detect_bins_on_shelf");
  record_client_ = node_->create_client<RecordBinDetections>("record_bin_detections");
  get_plane_client_ = node_->create_client<GetShelfPlane>("get_shelf_plane");

  rclcpp::QoS order_qos(1);
  order_qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  order_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  order_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  order_sub_ = node_->create_subscription<r1_interfaces::msg::PickTicket>(
    "/teleop/order", order_qos,
    std::bind(&SearchBinsBtNode::orderCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(),
    "SearchBinsBtNode: initialized "
    "(camera=detect_bins_on_shelf, fuse=record_bin_detections, plane=get_shelf_plane)");
}

void SearchBinsBtNode::orderCallback(
  const r1_interfaces::msg::PickTicket::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (msg->item_name == "NONE") {
    current_order_.reset();
  } else {
    current_order_ = msg;
  }
}

std::string SearchBinsBtNode::deriveShelfKey() const
{
  // Prefer the input port if explicitly set.
  std::string explicit_key;
  // const_cast: getInput is non-const in BT v3 because it touches the blackboard.
  if (const_cast<SearchBinsBtNode *>(this)->getInput("shelf_key", explicit_key) &&
      !explicit_key.empty())
  {
    return explicit_key;
  }

  std::lock_guard<std::mutex> lock(order_mutex_);
  if (!current_order_) {
    return {};
  }
  const auto & order = *current_order_;
  char buf[8];
  std::snprintf(buf, sizeof(buf), "%02ld", static_cast<long>(order.bin.bay));
  return order.bin.side + std::to_string(order.bin.row) + "-" + buf + "-" + order.bin.shelf;
}

std::string SearchBinsBtNode::deriveBayKey() const
{
  // Strip the trailing "-<shelf>" off shelf_key_ ("X1-01-A" -> "X1-01"). The
  // shelf letter is always a single uppercase letter preceded by a dash, so
  // the last dash separates bay from shelf.
  if (shelf_key_.empty()) return {};
  auto pos = shelf_key_.rfind('-');
  if (pos == std::string::npos) return shelf_key_;
  return shelf_key_.substr(0, pos);
}

void SearchBinsBtNode::resetInternal()
{
  plane_future_ = std::shared_future<GetShelfPlane::Response::SharedPtr>{};
  detect_future_ = std::shared_future<DetectBinsOnShelf::Response::SharedPtr>{};
  record_future_ = std::shared_future<RecordBinDetections::Response::SharedPtr>{};
  phase_ = Phase::IDLE;
}

BT::NodeStatus SearchBinsBtNode::onStart()
{
  halt_requested_.store(false);
  resetInternal();

  if (!getInput<double>("timeout", timeout_s_)) timeout_s_ = 20.0;
  if (!getInput<double>("min_confidence", min_confidence_)) min_confidence_ = 0.99;

  shelf_key_ = deriveShelfKey();
  if (shelf_key_.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
      "SearchBinsBtNode: cannot resolve shelf_key (no /teleop/order, no input port)");
    return BT::NodeStatus::FAILURE;
  }

  start_time_ = node_->get_clock()->now();

  if (!get_plane_client_->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(node_->get_logger(),
      "SearchBinsBtNode: get_shelf_plane service not available");
    return BT::NodeStatus::FAILURE;
  }

  auto req = std::make_shared<GetShelfPlane::Request>();
  req->shelf_key = shelf_key_;
  plane_future_ = get_plane_client_->async_send_request(req);
  phase_ = Phase::WAIT_PLANE;

  RCLCPP_INFO(node_->get_logger(),
    "SearchBinsBtNode: shelf_key=%s timeout=%.1fs min_confidence=%.3f",
    shelf_key_.c_str(), timeout_s_, min_confidence_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SearchBinsBtNode::onRunning()
{
  rclcpp::spin_some(node_);

  if (halt_requested_.load()) {
    return BT::NodeStatus::FAILURE;
  }

  // Wall-time timeout: per the design, return SUCCESS regardless so the parent
  // BT proceeds with whatever the cache currently holds.
  const double elapsed = (node_->get_clock()->now() - start_time_).seconds();
  if (elapsed >= timeout_s_) {
    RCLCPP_INFO(node_->get_logger(),
      "SearchBinsBtNode: timeout reached (%.2fs >= %.2fs), returning SUCCESS",
      elapsed, timeout_s_);
    resetInternal();
    return BT::NodeStatus::SUCCESS;
  }

  if (phase_ == Phase::WAIT_PLANE) {
    if (!plane_future_.valid()) {
      return BT::NodeStatus::FAILURE;
    }
    if (plane_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    auto plane = plane_future_.get();
    if (!plane || !plane->found) {
      RCLCPP_ERROR(node_->get_logger(),
        "SearchBinsBtNode: get_shelf_plane returned not-found for '%s'",
        shelf_key_.c_str());
      return BT::NodeStatus::FAILURE;
    }
    shelf_start_ = plane->shelf_start;
    shelf_end_ = plane->shelf_end;
    shelf_bottom_z_ = plane->shelf_bottom_z;
    shelf_top_z_ = plane->shelf_top_z;
    shelf_yaw_rad_ = plane->shelf_yaw_rad;
    shelf_brackets_ = plane->shelf_brackets;
    left_apriltag_id_ = plane->left_apriltag_id;
    right_apriltag_id_ = plane->right_apriltag_id;
    bay_key_ = deriveBayKey();
    RCLCPP_INFO(node_->get_logger(),
      "SearchBinsBtNode: bay search plan ready (bay=%s) "
      "z=[%.3f, %.3f] m, %zu shelf brackets, "
      "left_apriltag=%d right_apriltag=%d",
      bay_key_.c_str(), shelf_bottom_z_, shelf_top_z_,
      shelf_brackets_.size(), left_apriltag_id_, right_apriltag_id_);
    phase_ = Phase::POLL_IDLE;
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::POLL_IDLE) {
    if (!detect_client_->service_is_ready()) {
      // Camera node not up yet; wait for a tick.
      return BT::NodeStatus::RUNNING;
    }
    auto req = std::make_shared<DetectBinsOnShelf::Request>();
    req->shelf_start = shelf_start_;
    req->shelf_end = shelf_end_;
    req->shelf_bottom_z = shelf_bottom_z_;       // bay-wide low (lowest shelf bottom)
    req->shelf_top_z = shelf_top_z_;             // bay-wide high (highest shelf top)
    req->shelf_yaw_rad = shelf_yaw_rad_;
    req->shelf_brackets = shelf_brackets_;       // per-shelf z brackets (optional metadata for camera)
    req->left_apriltag_id = left_apriltag_id_;   // observer-side LEFT bay-edge apriltag
    req->right_apriltag_id = right_apriltag_id_; // observer-side RIGHT bay-edge apriltag
    detect_future_ = detect_client_->async_send_request(req);
    phase_ = Phase::WAIT_DETECT;
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::WAIT_DETECT) {
    if (!detect_future_.valid()) {
      phase_ = Phase::POLL_IDLE;
      return BT::NodeStatus::RUNNING;
    }
    if (detect_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    auto resp = detect_future_.get();
    detect_future_ = std::shared_future<DetectBinsOnShelf::Response::SharedPtr>{};

    if (!resp || !resp->success) {
      RCLCPP_DEBUG(node_->get_logger(),
        "SearchBinsBtNode: DetectBinsOnShelf failed (%s); retrying",
        resp ? resp->message.c_str() : "null");
      phase_ = Phase::POLL_IDLE;
      return BT::NodeStatus::RUNNING;
    }

    if (resp->detections.empty()) {
      // Nothing to fuse this round; continue polling.
      phase_ = Phase::POLL_IDLE;
      return BT::NodeStatus::RUNNING;
    }

    if (!record_client_->service_is_ready()) {
      phase_ = Phase::POLL_IDLE;
      return BT::NodeStatus::RUNNING;
    }
    auto rreq = std::make_shared<RecordBinDetections::Request>();
    // Multi-shelf path: send bay_key so the warehouse partitions detections
    // by detection.center.z and runs PDA per shelf within the bay. shelf_key
    // is left empty; bay_key takes priority server-side.
    rreq->bay_key = bay_key_;
    rreq->shelf_key = "";
    rreq->detections = resp->detections;
    record_future_ = record_client_->async_send_request(rreq);
    phase_ = Phase::WAIT_RECORD;
    return BT::NodeStatus::RUNNING;
  }

  if (phase_ == Phase::WAIT_RECORD) {
    if (!record_future_.valid()) {
      phase_ = Phase::POLL_IDLE;
      return BT::NodeStatus::RUNNING;
    }
    if (record_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return BT::NodeStatus::RUNNING;
    }
    auto resp = record_future_.get();
    record_future_ = std::shared_future<RecordBinDetections::Response::SharedPtr>{};

    if (resp && resp->success) {
      const float conf = resp->best_bin_confidence;
      RCLCPP_INFO(node_->get_logger(),
        "SearchBinsBtNode: PDA report matched=%d missed=%d clutter=%d "
        "best_target_confidence=%.3f (gate %.3f)",
        resp->matched, resp->missed, resp->clutter, conf, min_confidence_);
      if (conf >= static_cast<float>(min_confidence_)) {
        RCLCPP_INFO(node_->get_logger(),
          "SearchBinsBtNode: target reached confidence gate, returning SUCCESS");
        resetInternal();
        return BT::NodeStatus::SUCCESS;
      }
    }
    phase_ = Phase::POLL_IDLE;
    return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::RUNNING;
}

void SearchBinsBtNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "SearchBinsBtNode::onHalted");
  halt_requested_.store(true);
  // Drop pending futures; they'll be discarded by their unique destructors.
  resetInternal();
}

}  // namespace r1_bt_nodes

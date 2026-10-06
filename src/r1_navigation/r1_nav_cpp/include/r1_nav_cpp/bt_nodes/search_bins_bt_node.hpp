#ifndef R1_NAV_CPP__BT_NODES__SEARCH_BINS_BT_NODE_HPP_
#define R1_NAV_CPP__BT_NODES__SEARCH_BINS_BT_NODE_HPP_

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "r1_interfaces/msg/pick_ticket.hpp"
#include "r1_interfaces/msg/shelf_bracket.hpp"
#include "r1_interfaces/srv/detect_bins_on_shelf.hpp"
#include "r1_interfaces/srv/get_shelf_plane.hpp"
#include "r1_interfaces/srv/record_bin_detections.hpp"

#include <vector>

namespace r1_bt_nodes
{

/**
 * @brief BT node that drives the camera-driven shelf bin search.
 *
 * Sibling responsibilities (this node owns ONLY the perception loop):
 *   - Navigation to the search-standoff pose is handled by the BT
 *     `NavigateWithToleranceAdjustment` subtree.
 *   - Servoing torso/arms to the search height is handled by `ServoToTarget`
 *     with `target_type=BIN_SEARCH`.
 *   - This node, in parallel, polls `DetectBinsOnShelf` on the camera node
 *     and forwards every batch of detections to `RecordBinDetections` on
 *     `tag_detections_node`, where the PDA-lite update fuses them into the
 *     joint (pos, width) belief.
 *
 * Returns SUCCESS once any of the following is true:
 *   1. The order's target bin's post-update confidence >= `min_confidence`.
 *   2. Wall-time elapsed >= `timeout` (default 20.0 s).
 *
 * Returns FAILURE only when an unrecoverable error occurs (no order,
 * no shelf plane could be resolved).
 *
 * On halt: cancels any pending service futures and returns immediately.
 */
class SearchBinsBtNode : public BT::StatefulActionNode
{
public:
  using DetectBinsOnShelf = r1_interfaces::srv::DetectBinsOnShelf;
  using RecordBinDetections = r1_interfaces::srv::RecordBinDetections;
  using GetShelfPlane = r1_interfaces::srv::GetShelfPlane;

  SearchBinsBtNode(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout", 20.0,
        "Wall-time budget for the camera search loop (s)"),
      BT::InputPort<double>("min_confidence", 0.99,
        "Stop early once the target bin's joint pos+width confidence reaches this"),
      BT::InputPort<std::string>("shelf_key", "",
        "Optional shelf_key. Empty = derive from current /teleop/order")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    IDLE,
    WAIT_PLANE,
    POLL_IDLE,           // ready to fire next DetectBinsOnShelf request
    WAIT_DETECT,         // DetectBinsOnShelf in flight
    WAIT_RECORD          // RecordBinDetections in flight
  };

  void resetInternal();
  void orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg);
  std::string deriveShelfKey() const;
  std::string deriveBayKey() const;     // strip the trailing "-<shelf>" off shelf_key

  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<DetectBinsOnShelf>::SharedPtr detect_client_;
  rclcpp::Client<RecordBinDetections>::SharedPtr record_client_;
  rclcpp::Client<GetShelfPlane>::SharedPtr get_plane_client_;

  rclcpp::Subscription<r1_interfaces::msg::PickTicket>::SharedPtr order_sub_;

  mutable std::mutex order_mutex_;
  r1_interfaces::msg::PickTicket::SharedPtr current_order_;

  std::shared_future<GetShelfPlane::Response::SharedPtr> plane_future_;
  std::shared_future<DetectBinsOnShelf::Response::SharedPtr> detect_future_;
  std::shared_future<RecordBinDetections::Response::SharedPtr> record_future_;

  std::atomic_bool halt_requested_{false};
  Phase phase_{Phase::IDLE};

  // Cached request fields after the GetShelfPlane response arrives.
  std::string shelf_key_;     // form "X1-01-A" (the order's target shelf)
  std::string bay_key_;       // form "X1-01" (used by RecordBinDetections multi-shelf path)
  geometry_msgs::msg::Point shelf_start_;
  geometry_msgs::msg::Point shelf_end_;
  // Bay-wide vertical extent (lowest shelf bottom_z .. highest shelf top_z).
  float shelf_bottom_z_{0.0f};
  float shelf_top_z_{0.0f};
  float shelf_yaw_rad_{0.0f};
  // Per-shelf vertical brackets ordered bottom -> top, copied verbatim from
  // the GetShelfPlane response and forwarded into DetectBinsOnShelf.
  std::vector<r1_interfaces::msg::ShelfBracket> shelf_brackets_;
  // Bay-edge apriltag IDs already converted to (left, right) by the warehouse.
  // Either may be -1 meaning "no apriltag installed at that edge".
  int32_t left_apriltag_id_{-1};
  int32_t right_apriltag_id_{-1};

  // BT input ports cached at onStart()
  double timeout_s_{20.0};
  double min_confidence_{0.99};

  rclcpp::Time start_time_;
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__SEARCH_BINS_BT_NODE_HPP_

#pragma once

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "r1_interfaces/action/solve_ik.hpp"
#include "r1_interfaces/msg/pick_ticket.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace r1_bt_nodes
{

class SolveIkBtNode : public BT::StatefulActionNode
{
public:
  using SolveIK = r1_interfaces::action::SolveIK;
  using GoalHandle = rclcpp_action::ClientGoalHandle<SolveIK>;

  SolveIkBtNode(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct ShelfIkConfig
  {
    std::string target_link;
    std::vector<geometry_msgs::msg::Pose> poses;
  };

  struct BinMapPosition
  {
    double x;
    double y;
  };

  struct BayData
  {
    double yaw_rad{0.0};  // facing yaw of the owning segment
    // shelf_key -> bin_number -> BinMapPosition (positions interpolated at startup)
    std::map<std::string, std::map<int, BinMapPosition>> bins;
  };

  struct RowData
  {
    // bay_key -> BayData
    std::map<std::string, BayData> bays;
  };

  enum class Phase
  {
    IDLE,
    WAIT_GOAL_HANDLE,
    WAIT_RESULT
  };

  void loadConfig();
  void loadWarehouseConfig();
  void resetInternal();
  void orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg);
  std::vector<geometry_msgs::msg::Pose> computeAdjustedPoses(
    const ShelfIkConfig & cfg,
    const r1_interfaces::msg::PickTicket & order);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<SolveIK>::SharedPtr client_;
  rclcpp::Subscription<r1_interfaces::msg::PickTicket>::SharedPtr order_sub_;

  std::mutex mutex_;
  std::shared_ptr<GoalHandle> active_goal_handle_;
  std::shared_future<std::shared_ptr<GoalHandle>> goal_handle_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;

  std::atomic_bool halt_requested_{false};
  Phase phase_{Phase::IDLE};

  std::map<std::string, ShelfIkConfig> shelf_configs_;
  bool config_loaded_{false};

  std::map<std::string, RowData> warehouse_rows_;
  bool warehouse_loaded_{false};
  double gripper_standoff_{0.30};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex order_mutex_;
  r1_interfaces::msg::PickTicket::SharedPtr current_order_;
};

}  // namespace r1_bt_nodes

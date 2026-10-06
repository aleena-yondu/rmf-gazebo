#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "r1_nav_cpp/bt_nodes/pan_scanner.hpp"

#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/qos.hpp"
#include "rclcpp/logging.hpp"

namespace r1_bt_nodes
{

PanScanner::PanScanner(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  default_arm_pose_({0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0}),
  pan_duration_(5.0),
  pan_min_(-0.25),
  pan_max_(0.25),
  pan_frequency_(0.5),
  pan2_min_(0.0),
  pan2_max_(0.0),
  pan2_frequency_(0.0),
  pan2_enabled_(false),
  state_(State::MOVING_TO_START)
{
  // Get shared Nav2 node from blackboard
  if (!config().blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "PanScanner: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "PanScanner: using shared Nav2 node '%s'",
    node_->get_name());

  // Initialize shelf-specific arm poses
  initializeShelfPoses();

  // Create publishers for arm joint commands
  arm_left_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_arm_left", 10);
  arm_right_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_arm_right", 10);

  // Subscribe to order topic to get current shelf height
  order_sub_ = node_->create_subscription<r1_interfaces::msg::PickTicket>(
    "/teleop/order", 10,
    std::bind(&PanScanner::orderCallback, this, std::placeholders::_1));

  // Initialize with default pose
  base_arm_pose_ = default_arm_pose_;
  target_left_joints_ = base_arm_pose_;
  target_right_joints_ = base_arm_pose_;

  RCLCPP_INFO(node_->get_logger(), "PanScanner: initialized successfully, subscribing to /teleop/order");
}

void PanScanner::orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  latest_order_ = msg;
}

void PanScanner::initializeShelfPoses()
{
  // Define base arm poses for each shelf height
  // Format: {joint1, joint2, joint3, joint4, joint5, joint6, joint7}
  // joint3 (index 2) is the pan joint that will be modified during scanning
  
  // Shelf A (bottom) - arm lowered
  shelf_arm_poses_["A"] = {0.0, 0.0, 0.0, -1.2, 0.0, 0.0, 0.0};
  
  // Shelf B (middle) - default/neutral position
  shelf_arm_poses_["B"] = {0.0, 0.0, 0.0, -1.5, 0.0, 0.0, 0.0};
  
  // Shelf C (top) - default/neutral position
  shelf_arm_poses_["C"] = {0.0, 0.0, 0.0, -1.7, 0.0, -0.05, 0.0};

  RCLCPP_INFO(
    node_->get_logger(),
    "PanScanner: initialized shelf poses for heights A, B, C");
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

double PanScanner::computePanAngle(double elapsed_time)
{
  // Sinusoidal panning between pan_min_ and pan_max_
  // angle = center + amplitude * sin(2*pi*frequency*t)
  double center = (pan_max_ + pan_min_) / 2.0;
  double amplitude = (pan_max_ - pan_min_) / 2.0;
  return center + amplitude * std::sin(2.0 * M_PI * pan_frequency_ * elapsed_time);
}

double PanScanner::computePan2Angle(double elapsed_time)
{
  // Sinusoidal panning for 4th joint, centered around the base pose value
  // pan2_min_ and pan2_max_ are offsets relative to base_arm_pose_[PAN2_JOINT_INDEX]
  // E.g., if base is -1.3 and pan2_min=-0.1, pan2_max=0.2, oscillates between -1.4 and -1.1
  double base_value = base_arm_pose_[PAN2_JOINT_INDEX];
  double actual_min = base_value + pan2_min_;
  double actual_max = base_value + pan2_max_;
  double center = (actual_max + actual_min) / 2.0;
  double amplitude = (actual_max - actual_min) / 2.0;
  return center + amplitude * std::sin(2.0 * M_PI * pan2_frequency_ * elapsed_time);
}

void PanScanner::publishArmCommands()
{
  const auto now = node_->get_clock()->now();

  // Left arm
  sensor_msgs::msg::JointState left_msg;
  left_msg.header.stamp = now;
  left_msg.name.resize(7);
  for (int i = 0; i < 7; ++i) {
    left_msg.name[i] = "left_arm_joint_" + std::to_string(i + 1);
  }
  left_msg.position = target_left_joints_;
  arm_left_pub_->publish(left_msg);

  // Right arm
  sensor_msgs::msg::JointState right_msg;
  right_msg.header.stamp = now;
  right_msg.name.resize(7);
  for (int i = 0; i < 7; ++i) {
    right_msg.name[i] = "right_arm_joint_" + std::to_string(i + 1);
  }
  right_msg.position = target_right_joints_;
  arm_right_pub_->publish(right_msg);
}

// ---------------------------------------------------------------------
// BT Lifecycle
// ---------------------------------------------------------------------

BT::NodeStatus PanScanner::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "PanScanner: onStart()");

  // Get shelf height from the latest order on /teleop/order topic
  std::string shelf_height;
  {
    std::lock_guard<std::mutex> lock(order_mutex_);
    if (latest_order_) {
      shelf_height = latest_order_->bin.shelf;
      RCLCPP_INFO(
        node_->get_logger(),
        "PanScanner: read shelf '%s' from current order (bin: %s%ld-%ld-%s-%ld)",
        shelf_height.c_str(),
        latest_order_->bin.side.c_str(),
        latest_order_->bin.row,
        latest_order_->bin.bay,
        latest_order_->bin.shelf.c_str(),
        latest_order_->bin.bin_number);
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "PanScanner: no order received on /teleop/order, using default pose");
      shelf_height = "";  // Will trigger default pose
    }
  }
  
  // Look up the arm pose for the shelf height from the order
  auto it = shelf_arm_poses_.find(shelf_height);
  if (it != shelf_arm_poses_.end()) {
    base_arm_pose_ = it->second;
    RCLCPP_INFO(
      node_->get_logger(),
      "PanScanner: using arm pose for shelf height '%s'",
      shelf_height.c_str());
  } else {
    base_arm_pose_ = default_arm_pose_;
    RCLCPP_WARN(
      node_->get_logger(),
      "PanScanner: shelf '%s' not in pose map, using default pose",
      shelf_height.c_str());
  }

  // Read pan parameters from BT ports for 3rd joint
  if (!getInput("pan_duration", pan_duration_)) {
    pan_duration_ = 5.0;
  }
  if (!getInput("pan_min", pan_min_)) {
    pan_min_ = -0.25;
  }
  if (!getInput("pan_max", pan_max_)) {
    pan_max_ = 0.25;
  }
  if (!getInput("pan_frequency", pan_frequency_)) {
    pan_frequency_ = 0.5;
  }

  // Read pan parameters from BT ports for 4th joint (index 3)
  // All three must be provided to enable secondary panning
  auto pan2_min_result = getInput("pan2_min", pan2_min_);
  auto pan2_max_result = getInput("pan2_max", pan2_max_);
  auto pan2_frequency_result = getInput("pan2_frequency", pan2_frequency_);
  pan2_enabled_ = pan2_min_result.has_value() && pan2_max_result.has_value() && pan2_frequency_result.has_value();

  RCLCPP_INFO(
    node_->get_logger(),
    "PanScanner: shelf=%s, duration=%.2fs, pan_min=%.3f, pan_max=%.3f, freq=%.2fHz",
    shelf_height.c_str(), pan_duration_, pan_min_, pan_max_, pan_frequency_);

  if (pan2_enabled_) {
    double base_value = base_arm_pose_[PAN2_JOINT_INDEX];
    RCLCPP_INFO(
      node_->get_logger(),
      "PanScanner: 4th joint panning enabled: offsets [%.3f, %.3f], actual range [%.3f, %.3f], freq=%.2fHz",
      pan2_min_, pan2_max_, base_value + pan2_min_, base_value + pan2_max_, pan2_frequency_);
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "PanScanner: 4th joint panning disabled (not all pan2 ports provided)");
  }

  // Initialize state
  state_ = State::MOVING_TO_START;
  start_time_ = node_->get_clock()->now();

  // Set both arms to the base pose
  target_left_joints_ = base_arm_pose_;
  target_right_joints_ = base_arm_pose_;

  // Publish initial commands to move arms to start position
  publishArmCommands();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PanScanner::onRunning()
{
  // Allow ROS to process callbacks
  rclcpp::spin_some(node_);

  const double elapsed = (node_->get_clock()->now() - start_time_).seconds();

  switch (state_) {
    case State::MOVING_TO_START:
      {
        // Keep publishing the base pose while settling
        publishArmCommands();

        if (elapsed >= SETTLE_TIME) {
          RCLCPP_INFO(
            node_->get_logger(),
            "PanScanner: arms settled, starting pan motion");
          state_ = State::PANNING;
          // Reset start time for panning phase
          start_time_ = node_->get_clock()->now();
        }
        return BT::NodeStatus::RUNNING;
      }

    case State::PANNING:
      {
        const double pan_elapsed = (node_->get_clock()->now() - start_time_).seconds();

        // Check if panning duration is complete
        if (pan_elapsed >= pan_duration_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "PanScanner: panning complete after %.2fs",
            pan_elapsed);
          state_ = State::DONE;
          return BT::NodeStatus::SUCCESS;
        }

        // Compute current pan angle for 3rd joint
        double pan_angle = computePanAngle(pan_elapsed);

        // Update left arm's 3rd joint (index 2) with pan angle
        target_left_joints_ = base_arm_pose_;
        target_left_joints_[PAN_JOINT_INDEX] = pan_angle;

        // Also oscillate 4th joint (index 3) if enabled
        if (pan2_enabled_) {
          double pan2_angle = computePan2Angle(pan_elapsed);
          target_left_joints_[PAN2_JOINT_INDEX] = pan2_angle;
        }

        // Right arm stays at base pose
        target_right_joints_ = base_arm_pose_;

        // Publish commands
        publishArmCommands();

        return BT::NodeStatus::RUNNING;
      }

    case State::DONE:
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void PanScanner::onHalted()
{
  RCLCPP_INFO(
    node_->get_logger(),
    "PanScanner: onHalted() - motion interrupted, returning arms to base pose");

  // Return arms to base pose on halt
  target_left_joints_ = base_arm_pose_;
  target_right_joints_ = base_arm_pose_;
  publishArmCommands();
}

}  // namespace r1_bt_nodes

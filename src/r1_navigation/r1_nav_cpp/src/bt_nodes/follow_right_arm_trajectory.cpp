#include "r1_nav_cpp/bt_nodes/follow_right_arm_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "behaviortree_cpp/blackboard.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/qos.hpp"

namespace r1_bt_nodes
{

namespace
{
// Left arm "usual servo position" and joint names, copied from
// r1_manipulation_cpp/src/ik_solver_node_servo.cpp so this node commands the
// left arm to the same pose the servo_height action does.
const std::vector<double> SERVO_LEFT_ARM = {
  0.075, -0.1102, -0.2511, -1.6806, -0.1023, -1.0468, 0.1};

const std::vector<std::string> LEFT_ARM_JOINT_NAMES = {
  "left_arm_joint1", "left_arm_joint2", "left_arm_joint3", "left_arm_joint4",
  "left_arm_joint5", "left_arm_joint6", "left_arm_joint7"};

constexpr size_t kNumArmJoints = 7;
}  // namespace

FollowRightArmTrajectory::FollowRightArmTrajectory(
  const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  if (!config.blackboard) {
    throw BT::RuntimeError("FollowRightArmTrajectory: blackboard is null.");
  }

  if (!config.blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "FollowRightArmTrajectory: 'node' not found on blackboard. "
      "This BT node must be used inside Nav2 bt_navigator.");
  }

  // Order subscription (transient-local so we pick up the latched current order)
  auto order_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  order_sub_ = node_->create_subscription<r1_interfaces::msg::PickTicket>(
    "/teleop/order", order_qos,
    std::bind(&FollowRightArmTrajectory::orderCallback, this, std::placeholders::_1));

  // Feedback subscriptions on the namespaced HDAS topics
  const char * ns_env = std::getenv("ROS_NAMESPACE");
  const std::string ns = (ns_env && ns_env[0] != '\0') ? ns_env : "bertha";

  auto sensor_qos = rclcpp::SensorDataQoS();
  right_arm_fb_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/" + ns + "/hdas/feedback_arm_right", sensor_qos,
    std::bind(&FollowRightArmTrajectory::rightArmCallback, this, std::placeholders::_1));
  right_gripper_fb_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/" + ns + "/hdas/feedback_gripper_right", sensor_qos,
    std::bind(&FollowRightArmTrajectory::rightGripperCallback, this, std::placeholders::_1));

  // Command publishers
  right_arm_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_arm_right", 10);
  right_gripper_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_position_gripper_right", 10);
  left_arm_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_arm_left", 10);

  RCLCPP_INFO(
    node_->get_logger(),
    "FollowRightArmTrajectory: initialized (feedback ns='%s')", ns.c_str());
}

BT::PortsList FollowRightArmTrajectory::providedPorts()
{
  return {
    BT::InputPort<double>("arm_tolerance", 0.05,
      "Per-joint convergence tolerance for the right arm (rad)"),
    BT::InputPort<double>("gripper_tolerance", 20.0,
      "Convergence tolerance for the right gripper (linear 0-100)"),
    BT::InputPort<double>("gripper_open", 100.0,
      "Commanded right gripper open value"),
    BT::InputPort<double>("velocity", 1.5,
      "JointState velocity for published arm targets"),
    BT::InputPort<double>("blend_tau_s", 0.5,
      "Exponential time constant (s) for decaying the initial offset"),
    BT::InputPort<double>("timeout", 30.0,
      "Safety timeout (s); FAILURE if not converged within this time")
  };
}

void FollowRightArmTrajectory::orderCallback(
  const r1_interfaces::msg::PickTicket::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(order_mutex_);
  if (msg->item_name == "NONE") {
    current_order_.reset();
  } else {
    current_order_ = msg;
  }
}

void FollowRightArmTrajectory::rightArmCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  const size_t n = std::min(msg->position.size(), kNumArmJoints);
  right_arm_fb_.assign(msg->position.begin(), msg->position.begin() + n);
  right_arm_fb_ready_ = (n == kNumArmJoints);
}

void FollowRightArmTrajectory::rightGripperCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  right_gripper_fb_ = msg->position[0];
  right_gripper_fb_ready_ = true;
}

bool FollowRightArmTrajectory::loadTrajectory(
  const std::string & shelf, std::string & error_out)
{
  traj_times_.clear();
  traj_positions_.clear();

  std::string pkg_share;
  try {
    pkg_share = ament_index_cpp::get_package_share_directory("r1_nav_cpp");
  } catch (const std::exception & e) {
    error_out = std::string("failed to get package share directory: ") + e.what();
    return false;
  }

  const std::string csv_path =
    pkg_share + "/config/right_arm_trajectories/" + shelf + ".csv";

  std::ifstream file(csv_path);
  if (!file.is_open()) {
    error_out = "could not open trajectory file '" + csv_path + "'";
    return false;
  }

  std::string line;
  bool header_skipped = false;
  size_t line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    if (line.empty()) {
      continue;
    }
    if (!header_skipped) {
      header_skipped = true;  // first non-empty line is the CSV header
      continue;
    }

    std::stringstream ss(line);
    std::string cell;
    std::vector<double> cols;
    while (std::getline(ss, cell, ',')) {
      try {
        cols.push_back(std::stod(cell));
      } catch (const std::exception &) {
        error_out = "malformed number at line " + std::to_string(line_no) +
                    " in '" + csv_path + "'";
        return false;
      }
    }

    // Expected: timestamp + 7 arm joints + gripper = 9 columns.
    if (cols.size() < 1 + kNumArmJoints) {
      error_out = "too few columns (" + std::to_string(cols.size()) +
                  ") at line " + std::to_string(line_no) + " in '" + csv_path + "'";
      return false;
    }

    traj_times_.push_back(cols[0]);
    traj_positions_.emplace_back(cols.begin() + 1, cols.begin() + 1 + kNumArmJoints);
  }

  if (traj_positions_.empty()) {
    error_out = "trajectory file '" + csv_path + "' contains no samples";
    return false;
  }

  return true;
}

void FollowRightArmTrajectory::publishRightArm(const std::vector<double> & positions)
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.position = positions;
  msg.velocity = std::vector<double>(kNumArmJoints, velocity_);
  msg.effort = std::vector<double>(kNumArmJoints, 0.0);
  right_arm_pub_->publish(msg);
}

void FollowRightArmTrajectory::publishRightGripperOpen()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.position = {gripper_open_};
  msg.velocity = {0.0};
  msg.effort = {0.0};
  right_gripper_pub_->publish(msg);
}

void FollowRightArmTrajectory::publishLeftArmServo()
{
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_->now();
  msg.name = LEFT_ARM_JOINT_NAMES;
  msg.position = SERVO_LEFT_ARM;
  msg.velocity = std::vector<double>(kNumArmJoints, velocity_);
  msg.effort = std::vector<double>(kNumArmJoints, 0.0);
  left_arm_pub_->publish(msg);
}

void FollowRightArmTrajectory::resetInternal()
{
  phase_ = Phase::IDLE;
  next_index_ = 0;
  traj_times_.clear();
  traj_positions_.clear();
  offset_.clear();
}

BT::NodeStatus FollowRightArmTrajectory::onStart()
{
  halt_requested_.store(false);
  resetInternal();

  if (!getInput<double>("arm_tolerance", arm_tolerance_)) { arm_tolerance_ = 0.05; }
  if (!getInput<double>("gripper_tolerance", gripper_tolerance_)) { gripper_tolerance_ = 20.0; }
  if (!getInput<double>("gripper_open", gripper_open_)) { gripper_open_ = 100.0; }
  if (!getInput<double>("velocity", velocity_)) { velocity_ = 1.5; }
  if (!getInput<double>("blend_tau_s", blend_tau_s_)) { blend_tau_s_ = 0.5; }
  if (!getInput<double>("timeout", timeout_)) { timeout_ = 30.0; }

  if (blend_tau_s_ <= 0.0) {
    RCLCPP_WARN(node_->get_logger(),
      "FollowRightArmTrajectory: blend_tau_s=%.3f <= 0, forcing 0.5", blend_tau_s_);
    blend_tau_s_ = 0.5;
  }

  // Service pending callbacks so we have the latest order + feedback.
  rclcpp::spin_some(node_);

  std::string shelf;
  {
    std::lock_guard<std::mutex> lock(order_mutex_);
    if (!current_order_) {
      RCLCPP_ERROR(node_->get_logger(), "FollowRightArmTrajectory: no active order");
      return BT::NodeStatus::FAILURE;
    }
    shelf = current_order_->bin.shelf;
  }

  if (shelf.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
      "FollowRightArmTrajectory: current order has empty bin.shelf");
    return BT::NodeStatus::FAILURE;
  }

  std::string err;
  if (!loadTrajectory(shelf, err)) {
    RCLCPP_ERROR(node_->get_logger(), "FollowRightArmTrajectory: %s", err.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Capture the arm's measured start pose to build the decaying offset. Poll a
  // few times in case feedback has not arrived yet.
  std::vector<double> q_measured;
  bool have_measured = false;
  for (int i = 0; i < 20 && rclcpp::ok(); ++i) {
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (right_arm_fb_ready_ && right_arm_fb_.size() == kNumArmJoints) {
        q_measured = right_arm_fb_;
        have_measured = true;
        break;
      }
    }
    rclcpp::spin_some(node_);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  if (!have_measured) {
    RCLCPP_ERROR(node_->get_logger(),
      "FollowRightArmTrajectory: no right arm feedback available to seed blend");
    return BT::NodeStatus::FAILURE;
  }

  offset_.assign(kNumArmJoints, 0.0);
  for (size_t j = 0; j < kNumArmJoints; ++j) {
    offset_[j] = traj_positions_.front()[j] - q_measured[j];
  }

  next_index_ = 0;
  phase_ = Phase::STREAM;
  start_time_ = node_->now();

  RCLCPP_INFO(node_->get_logger(),
    "FollowRightArmTrajectory: streaming shelf '%s' (%zu samples, tau=%.2fs)",
    shelf.c_str(), traj_positions_.size(), blend_tau_s_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowRightArmTrajectory::onRunning()
{
  rclcpp::spin_some(node_);

  if (halt_requested_.load()) {
    return BT::NodeStatus::FAILURE;
  }

  const double elapsed = (node_->now() - start_time_).seconds();

  if (elapsed > timeout_) {
    RCLCPP_ERROR(node_->get_logger(),
      "FollowRightArmTrajectory: timeout (%.2fs > %.2fs) before convergence",
      elapsed, timeout_);
    return BT::NodeStatus::FAILURE;
  }

  // Always keep the left arm commanded to its servo pose and the gripper open.
  publishLeftArmServo();
  publishRightGripperOpen();

  if (phase_ == Phase::STREAM) {
    // Publish every sample that is now due (catch-up), applying the decaying
    // offset so the commanded path starts at the measured pose and converges
    // onto the recorded trajectory.
    while (next_index_ < traj_positions_.size() &&
           traj_times_[next_index_] <= elapsed)
    {
      const double t_i = traj_times_[next_index_];
      const double w = std::exp(-t_i / blend_tau_s_);

      std::vector<double> q_cmd(kNumArmJoints, 0.0);
      for (size_t j = 0; j < kNumArmJoints; ++j) {
        q_cmd[j] = traj_positions_[next_index_][j] + offset_[j] * w;
      }
      publishRightArm(q_cmd);
      ++next_index_;
    }

    if (next_index_ >= traj_positions_.size()) {
      phase_ = Phase::SETTLE;
    }
    return BT::NodeStatus::RUNNING;
  }

  // SETTLE: hold the recorded final pose and wait for convergence.
  const std::vector<double> & final_pose = traj_positions_.back();
  publishRightArm(final_pose);

  std::vector<double> arm_fb;
  double grip_fb = 0.0;
  bool arm_ready = false;
  bool grip_ready = false;
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    arm_fb = right_arm_fb_;
    arm_ready = right_arm_fb_ready_;
    grip_fb = right_gripper_fb_;
    grip_ready = right_gripper_fb_ready_;
  }

  if (!arm_ready || !grip_ready || arm_fb.size() != kNumArmJoints) {
    return BT::NodeStatus::RUNNING;
  }

  bool arm_converged = true;
  for (size_t j = 0; j < kNumArmJoints; ++j) {
    if (std::abs(arm_fb[j] - final_pose[j]) > arm_tolerance_) {
      arm_converged = false;
      break;
    }
  }

  const bool gripper_converged =
    std::abs(grip_fb - gripper_open_) <= gripper_tolerance_;

  if (arm_converged && gripper_converged) {
    RCLCPP_INFO(node_->get_logger(),
      "FollowRightArmTrajectory: reached pre-inference pose (%.2fs)", elapsed);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void FollowRightArmTrajectory::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "FollowRightArmTrajectory: onHalted()");
  halt_requested_.store(true);
  resetInternal();
}

}  // namespace r1_bt_nodes

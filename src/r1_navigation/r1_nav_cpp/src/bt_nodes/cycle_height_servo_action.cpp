#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <cstdlib>

#include "r1_nav_cpp/bt_nodes/cycle_height_servo_action.hpp"
#include "behaviortree_cpp/blackboard.h"
#include "rclcpp/qos.hpp"
#include "rclcpp/logging.hpp"

namespace r1_bt_nodes
{

static bool allClose(
  const std::vector<double> & a,
  const std::vector<double> & b,
  double tol)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::fabs(a[i] - b[i]) > tol) {
      return false;
    }
  }
  return true;
}

CycleHeightServoAction::CycleHeightServoAction(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf),
  joint_tolerance_(0.1),
  timeout_(10.0),
  sub_state_(SubState::ARMS),
  current_index_(0)
{
  if (!config().blackboard->get("node", node_) || !node_) {
    throw BT::RuntimeError(
      "CycleHeightServoAction: 'node' not found on blackboard.");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "CycleHeightServoAction: using shared node '%s'",
    node_->get_name());

  arm_left_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_arm_left", 10);
  arm_right_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_arm_right", 10);
  torso_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/motion_target/target_joint_state_torso", 10);

  const char * ns_env = std::getenv("ROS_NAMESPACE");
  std::string ns_str = (ns_env != nullptr) ? std::string(ns_env) : std::string();

  std::string ns_prefix;
  if (ns_str.empty()) {
    ns_prefix = "";
  } else if (ns_str.front() == '/') {
    ns_prefix = ns_str;
  } else {
    ns_prefix = "/" + ns_str;
  }

  arm_left_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    ns_prefix + "/hdas/feedback_arm_left",
    rclcpp::QoS(10).best_effort().durability_volatile(),
    std::bind(&CycleHeightServoAction::leftArmFeedbackCallback, this, std::placeholders::_1));

  arm_right_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    ns_prefix + "/hdas/feedback_arm_right",
    rclcpp::QoS(10).best_effort().durability_volatile(),
    std::bind(&CycleHeightServoAction::rightArmFeedbackCallback, this, std::placeholders::_1));

  torso_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    ns_prefix + "/hdas/feedback_torso",
    rclcpp::QoS(10).best_effort().durability_volatile(),
    std::bind(&CycleHeightServoAction::torsoFeedbackCallback, this, std::placeholders::_1));

  current_left_joints_.assign(7, 0.0);
  current_right_joints_.assign(7, 0.0);
  current_torso_joints_.assign(4, 0.0);
}

// ---------------------------------------------------------------------------
// Feedback callbacks
// ---------------------------------------------------------------------------

void CycleHeightServoAction::leftArmFeedbackCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_left_joints_ = msg->position;
}

void CycleHeightServoAction::rightArmFeedbackCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_right_joints_ = msg->position;
}

void CycleHeightServoAction::torsoFeedbackCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  current_torso_joints_ = msg->position;
}

// ---------------------------------------------------------------------------
// Helpers — computeTorsoPose and arm positions reused from ServoToTargetAction
// ---------------------------------------------------------------------------

std::vector<double> CycleHeightServoAction::computeTorsoPose(double servo_level)
{
  std::vector<double> squat{1.74, -2.70, -0.96, 0.0};
  std::vector<double> stand{0.0, 0.0, 0.0, 0.0};

  double s = servo_level / 100.0;
  if (s < 0.0) s = 0.0;
  if (s > 1.0) s = 1.0;

  std::vector<double> result(squat.size(), 0.0);
  for (size_t i = 0; i < squat.size(); ++i) {
    result[i] = (1.0 - s) * squat[i] + s * stand[i];
  }
  return result;
}

void CycleHeightServoAction::publishCommands()
{
  const auto now = node_->get_clock()->now();

  sensor_msgs::msg::JointState left_msg;
  left_msg.header.stamp = now;
  left_msg.name.resize(7);
  for (int i = 0; i < 7; ++i) {
    left_msg.name[i] = "left_arm_joint_" + std::to_string(i + 1);
  }
  left_msg.position = target_left_joints_;
  arm_left_pub_->publish(left_msg);

  sensor_msgs::msg::JointState right_msg;
  right_msg.header.stamp = now;
  right_msg.name.resize(7);
  for (int i = 0; i < 7; ++i) {
    right_msg.name[i] = "right_arm_joint_" + std::to_string(i + 1);
  }
  right_msg.position = target_right_joints_;
  arm_right_pub_->publish(right_msg);

  sensor_msgs::msg::JointState torso_msg;
  torso_msg.header.stamp = now;
  torso_msg.name.resize(4);
  for (int i = 0; i < 4; ++i) {
    torso_msg.name[i] = "torso_joint_" + std::to_string(i + 1);
  }
  torso_msg.position = target_torso_joints_;
  torso_msg.velocity.resize(target_torso_joints_.size(), 1.0);
  torso_pub_->publish(torso_msg);
}

bool CycleHeightServoAction::checkTolerance()
{
  std::vector<double> left, right, torso;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    left = current_left_joints_;
    right = current_right_joints_;
    torso = current_torso_joints_;
  }

  if (sub_state_ == SubState::ARMS) {
    if (left.size() != target_left_joints_.size() ||
        right.size() != target_right_joints_.size())
    {
      return false;
    }
    return allClose(left, target_left_joints_, joint_tolerance_) &&
           allClose(right, target_right_joints_, joint_tolerance_);
  } else if (sub_state_ == SubState::TORSO) {
    if (torso.size() != target_torso_joints_.size()) {
      return false;
    }
    return allClose(torso, target_torso_joints_, joint_tolerance_);
  }

  return false;
}

// ---------------------------------------------------------------------------
// BT lifecycle
// ---------------------------------------------------------------------------

BT::NodeStatus CycleHeightServoAction::onStart()
{
  if (!getInput("server_timeout", timeout_)) {
    timeout_ = 10.0;
  }
  if (!getInput("joint_tolerance", joint_tolerance_)) {
    joint_tolerance_ = 0.1;
  }

  double height = CYCLE_HEIGHTS[current_index_];
  RCLCPP_INFO(node_->get_logger(),
    "CycleHeightServoAction: servoing to height %.1f (index %zu/%zu)",
    height, current_index_, CYCLE_HEIGHTS.size());

  current_index_ = (current_index_ + 1) % CYCLE_HEIGHTS.size();

  // Arm poses (POSES["2"] from ServoToTargetAction)
  target_left_joints_  = {0.075, -0.1102, -0.2511, -1.6806, -0.1023, -1.0468, 0.1};
  target_right_joints_ = {0.075,  0.1102,  0.2511, -1.6806, -0.1023, -1.0468, 0.1};

  target_torso_joints_ = computeTorsoPose(height);

  sub_state_ = SubState::ARMS;
  start_time_ = node_->get_clock()->now();

  publishCommands();
  return BT::NodeStatus::RUNNING;
}

void CycleHeightServoAction::logErrors()
{
  std::vector<double> left, right, torso;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    left = current_left_joints_;
    right = current_right_joints_;
    torso = current_torso_joints_;
  }

  auto maxErr = [](const std::vector<double> & a, const std::vector<double> & b) -> double {
    double m = 0.0;
    size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
      m = std::max(m, std::fabs(a[i] - b[i]));
    }
    return m;
  };

  if (sub_state_ == SubState::ARMS) {
    double le = (left.size() == target_left_joints_.size())
                  ? maxErr(left, target_left_joints_) : -1.0;
    double re = (right.size() == target_right_joints_.size())
                  ? maxErr(right, target_right_joints_) : -1.0;
    RCLCPP_INFO(node_->get_logger(),
      "CycleHeightServo [ARMS] left_max_err=%.4f (%zu joints)  right_max_err=%.4f (%zu joints)  tol=%.4f",
      le, left.size(), re, right.size(), joint_tolerance_);
  } else if (sub_state_ == SubState::TORSO) {
    double te = (torso.size() == target_torso_joints_.size())
                  ? maxErr(torso, target_torso_joints_) : -1.0;
    RCLCPP_INFO(node_->get_logger(),
      "CycleHeightServo [TORSO] max_err=%.4f (%zu joints)  tol=%.4f",
      te, torso.size(), joint_tolerance_);
  }
}

BT::NodeStatus CycleHeightServoAction::onRunning()
{
  // Spin to get feedback
  rclcpp::spin_some(node_);
  
  const double elapsed = (node_->get_clock()->now() - start_time_).seconds();

  bool should_log = (static_cast<int>(elapsed * 2) != static_cast<int>((elapsed - 0.05) * 2));

  if (elapsed > timeout_) {
    RCLCPP_ERROR(node_->get_logger(),
      "CycleHeightServoAction: timeout (%.2fs > %.2fs)", elapsed, timeout_);
    logErrors();
    return BT::NodeStatus::FAILURE;
  }


  switch (sub_state_) {
    case SubState::ARMS:
      publishCommands();
      if (should_log) { logErrors(); }
      if (checkTolerance()) {
        RCLCPP_INFO(node_->get_logger(),
          "CycleHeightServoAction: arms reached target, switching to TORSO");
        sub_state_ = SubState::TORSO;
      }
      return BT::NodeStatus::RUNNING;

    case SubState::TORSO:
      publishCommands();
      if (should_log) { logErrors(); }
      if (checkTolerance()) {
        RCLCPP_INFO(node_->get_logger(),
          "CycleHeightServoAction: torso reached target, DONE");
        sub_state_ = SubState::DONE;
        return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;

    case SubState::DONE:
      return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

void CycleHeightServoAction::onHalted()
{
  RCLCPP_INFO(node_->get_logger(),
    "CycleHeightServoAction: halted.");
}

}  // namespace r1_bt_nodes

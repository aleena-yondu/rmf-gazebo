#ifndef R1_NAV_CPP__BT_NODES__CYCLE_HEIGHT_SERVO_ACTION_HPP_
#define R1_NAV_CPP__BT_NODES__CYCLE_HEIGHT_SERVO_ACTION_HPP_

#include <string>
#include <vector>
#include <mutex>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace r1_bt_nodes
{

// Heights the node cycles through (servo_level 0–100).
// Edit this list to change the sequence.
inline const std::vector<double> CYCLE_HEIGHTS = {100.0, 80.0, 15.0};

class CycleHeightServoAction : public BT::StatefulActionNode
{
public:
  CycleHeightServoAction(
    const std::string & name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("server_timeout", 10.0, "Timeout per servo (s)"),
      BT::InputPort<double>("joint_tolerance", 0.1, "Joint convergence tolerance (rad)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void leftArmFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void rightArmFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void torsoFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  static std::vector<double> computeTorsoPose(double servo_level);
  void publishCommands();
  bool checkTolerance();
  void logErrors();

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torso_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_right_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr torso_sub_;

  std::mutex data_mutex_;
  std::vector<double> current_left_joints_;
  std::vector<double> current_right_joints_;
  std::vector<double> current_torso_joints_;

  std::vector<double> target_left_joints_;
  std::vector<double> target_right_joints_;
  std::vector<double> target_torso_joints_;

  double joint_tolerance_;
  double timeout_;

  enum class SubState { ARMS, TORSO, DONE };
  SubState sub_state_;
  rclcpp::Time start_time_;

  size_t current_index_;
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__CYCLE_HEIGHT_SERVO_ACTION_HPP_

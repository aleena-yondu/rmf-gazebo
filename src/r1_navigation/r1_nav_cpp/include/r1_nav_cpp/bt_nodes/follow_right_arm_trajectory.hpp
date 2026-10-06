#ifndef R1_NAV_CPP__BT_NODES__FOLLOW_RIGHT_ARM_TRAJECTORY_HPP_
#define R1_NAV_CPP__BT_NODES__FOLLOW_RIGHT_ARM_TRAJECTORY_HPP_

#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "r1_interfaces/msg/pick_ticket.hpp"

namespace r1_bt_nodes
{

// Streams a pre-recorded right-arm trajectory (selected by the current order's
// shelf) toward the optimal pre-inference pose, using a decaying-offset blend
// so the motion starts from the arm's actual measured pose (no initial jerk).
// The left arm is commanded to its usual servo pose and the right gripper is
// held open. The node only SUCCEEDS once the measured right arm + gripper
// converge to the recorded final pose within tolerance.
class FollowRightArmTrajectory : public BT::StatefulActionNode
{
public:
  FollowRightArmTrajectory(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    IDLE,
    STREAM,
    SETTLE
  };

  void orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg);
  void rightArmCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void rightGripperCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool loadTrajectory(const std::string & shelf, std::string & error_out);
  void publishRightArm(const std::vector<double> & positions);
  void publishRightGripperOpen();
  void publishLeftArmServo();
  void resetInternal();

  rclcpp::Node::SharedPtr node_;

  // Order tracking
  rclcpp::Subscription<r1_interfaces::msg::PickTicket>::SharedPtr order_sub_;
  std::mutex order_mutex_;
  r1_interfaces::msg::PickTicket::SharedPtr current_order_;

  // Feedback
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_arm_fb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr right_gripper_fb_sub_;
  std::mutex feedback_mutex_;
  std::vector<double> right_arm_fb_;
  bool right_arm_fb_ready_{false};
  double right_gripper_fb_{0.0};
  bool right_gripper_fb_ready_{false};

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_arm_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr right_gripper_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_arm_pub_;

  // Ports (resolved in onStart)
  double arm_tolerance_{0.05};
  double gripper_tolerance_{20.0};
  double gripper_open_{100.0};
  double velocity_{1.5};
  double blend_tau_s_{0.5};
  double timeout_{30.0};

  // Trajectory data
  std::vector<double> traj_times_;                 // per-sample timestamp (s)
  std::vector<std::vector<double>> traj_positions_;  // per-sample 7 joint positions
  std::vector<double> offset_;                     // recorded[0] - measured (7)

  // Runtime state
  Phase phase_{Phase::IDLE};
  size_t next_index_{0};
  rclcpp::Time start_time_;
  std::atomic_bool halt_requested_{false};
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__FOLLOW_RIGHT_ARM_TRAJECTORY_HPP_

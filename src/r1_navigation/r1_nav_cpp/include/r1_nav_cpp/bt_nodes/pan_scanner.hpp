#ifndef R1_NAV_CPP__BT_NODES__PAN_SCANNER_HPP_
#define R1_NAV_CPP__BT_NODES__PAN_SCANNER_HPP_

#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <unordered_map>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

// Messages
#include "sensor_msgs/msg/joint_state.hpp"
#include "r1_interfaces/msg/pick_ticket.hpp"

namespace r1_bt_nodes
{

/**
 * @brief PanScanner BT Node
 * 
 * This node performs a scanning motion with the left arm:
 * 1. Reads the current order from /teleop/order topic to get shelf height
 * 2. Moves both arms to a base position based on the shelf (A, B, C, etc.)
 * 3. Pans the 3rd joint (index 2) of the left arm between pan_min and pan_max
 *    for the specified duration.
 * 
 * Shelf heights correspond to different arm poses for scanning at different levels.
 */
class PanScanner : public BT::StatefulActionNode
{
public:
  PanScanner(
    const std::string & name,
    const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("pan_duration", 5.0, "Duration of panning motion (s)"),
      BT::InputPort<double>("pan_min", -0.25, "Minimum pan angle (rad) for 3rd joint"),
      BT::InputPort<double>("pan_max", 0.25, "Maximum pan angle (rad) for 3rd joint"),
      BT::InputPort<double>("pan_frequency", 0.5, "Pan frequency (Hz) for 3rd joint"),
      BT::InputPort<double>("pan2_min", "Min offset from base pose (rad) for 4th joint (optional)"),
      BT::InputPort<double>("pan2_max", "Max offset from base pose (rad) for 4th joint (optional)"),
      BT::InputPort<double>("pan2_frequency", "Pan frequency (Hz) for 4th joint (optional)")
    };
  }

  // BT Lifecycle methods
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  // Helper methods
  void publishArmCommands();
  double computePanAngle(double elapsed_time);
  double computePan2Angle(double elapsed_time);
  void initializeShelfPoses();
  void orderCallback(const r1_interfaces::msg::PickTicket::SharedPtr msg);

  // ROS Node Handle (shared from Nav2)
  rclcpp::Node::SharedPtr node_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_right_pub_;

  // Subscriber for current order (to get shelf height)
  rclcpp::Subscription<r1_interfaces::msg::PickTicket>::SharedPtr order_sub_;
  
  // Latest order data
  r1_interfaces::msg::PickTicket::SharedPtr latest_order_;
  std::mutex order_mutex_;

  // Base arm poses for different shelf heights
  // Key: shelf height ("A", "B", "C", etc.), Value: 7-element joint position vector
  std::unordered_map<std::string, std::vector<double>> shelf_arm_poses_;
  
  // Currently selected base arm pose (based on shelf from order)
  std::vector<double> base_arm_pose_;
  
  // Default pose used when shelf height is not recognized
  std::vector<double> default_arm_pose_;

  // Current target positions
  std::vector<double> target_left_joints_;
  std::vector<double> target_right_joints_;

  // Pan parameters for 3rd joint (index 2)
  double pan_duration_;
  double pan_min_;
  double pan_max_;
  double pan_frequency_;
  static constexpr int PAN_JOINT_INDEX = 2;  // 3rd joint (index 2)

  // Pan parameters for 4th joint (index 3)
  double pan2_min_;
  double pan2_max_;
  double pan2_frequency_;
  bool pan2_enabled_;  // True if all pan2 ports are provided
  static constexpr int PAN2_JOINT_INDEX = 3;  // 4th joint (index 3)

  // Timing
  rclcpp::Time start_time_;

  // State machine
  enum class State { MOVING_TO_START, PANNING, DONE };
  State state_;
  
  // Time spent in MOVING_TO_START before transitioning to PANNING
  static constexpr double SETTLE_TIME = 1.0;  // 1 second to reach start position
};

}  // namespace r1_bt_nodes

#endif  // R1_NAV_CPP__BT_NODES__PAN_SCANNER_HPP_

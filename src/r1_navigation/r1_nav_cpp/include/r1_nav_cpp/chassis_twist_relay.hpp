#ifndef R1_NAV_CPP__CHASSIS_TWIST_RELAY_HPP_
#define R1_NAV_CPP__CHASSIS_TWIST_RELAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <string>

namespace r1_nav_cpp
{

// Converts the already-computed chassis twist (geometry_msgs/TwistStamped on
// /motion_control/chassis_speed) into a geometry_msgs/TwistWithCovarianceStamped
// so it can be consumed by the robot_localization EKF (which cannot ingest a
// bare TwistStamped). A diagonal covariance is attached; unused dimensions get a
// large variance so the filter effectively ignores them.
class ChassisTwistRelay : public rclcpp::Node
{
public:
  ChassisTwistRelay();
  ~ChassisTwistRelay() override = default;

private:
  void onTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  double vx_variance_;
  double vy_variance_;
  double vyaw_variance_;
  double unused_variance_;
  double min_linear_velocity_;
  double min_angular_velocity_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
};

}  // namespace r1_nav_cpp

#endif  // R1_NAV_CPP__CHASSIS_TWIST_RELAY_HPP_

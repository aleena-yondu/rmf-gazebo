#include "r1_nav_cpp/chassis_twist_relay.hpp"

#include <array>
#include <cmath>

namespace r1_nav_cpp
{

ChassisTwistRelay::ChassisTwistRelay()
: Node("chassis_twist_relay")
{
  input_topic_ = this->declare_parameter<std::string>(
    "input_topic", "/motion_control/chassis_speed");
  output_topic_ = this->declare_parameter<std::string>(
    "output_topic", "/motion_control/chassis_twist_cov");

  // Per-axis variances for the fused dimensions (vx, vy, vyaw).
  vx_variance_ = this->declare_parameter<double>("vx_variance", 0.01);
  vy_variance_ = this->declare_parameter<double>("vy_variance", 0.01);
  vyaw_variance_ = this->declare_parameter<double>("vyaw_variance", 0.05);
  // Large variance for dimensions we do not measure (vz, vroll, vpitch).
  unused_variance_ = this->declare_parameter<double>("unused_variance", 1e6);

  // Optional deadband, mirroring synchro_drive_odometry's chassis_speed mode.
  min_linear_velocity_ = this->declare_parameter<double>("min_linear_velocity", 0.001);
  min_angular_velocity_ = this->declare_parameter<double>("min_angular_velocity", 0.001);

  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    input_topic_, rclcpp::QoS(10).best_effort().durability_volatile(),
    std::bind(&ChassisTwistRelay::onTwist, this, std::placeholders::_1));

  twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
    output_topic_, 10);

  RCLCPP_INFO(this->get_logger(),
              "ChassisTwistRelay started: %s (TwistStamped) -> %s (TwistWithCovarianceStamped)",
              input_topic_.c_str(), output_topic_.c_str());
}

void ChassisTwistRelay::onTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  geometry_msgs::msg::TwistWithCovarianceStamped out;
  out.header = msg->header;

  double vx = msg->twist.linear.x;
  double vy = msg->twist.linear.y;
  double wz = msg->twist.angular.z;

  if (std::abs(vx) < min_linear_velocity_) vx = 0.0;
  if (std::abs(vy) < min_linear_velocity_) vy = 0.0;
  if (std::abs(wz) < min_angular_velocity_) wz = 0.0;

  out.twist.twist.linear.x = vx;
  out.twist.twist.linear.y = vy;
  out.twist.twist.linear.z = 0.0;
  out.twist.twist.angular.x = 0.0;
  out.twist.twist.angular.y = 0.0;
  out.twist.twist.angular.z = wz;

  // Row-major 6x6 covariance: [vx, vy, vz, vroll, vpitch, vyaw].
  std::array<double, 36> cov{};
  cov[0] = vx_variance_;        // vx
  cov[7] = vy_variance_;        // vy
  cov[14] = unused_variance_;   // vz
  cov[21] = unused_variance_;   // vroll
  cov[28] = unused_variance_;   // vpitch
  cov[35] = vyaw_variance_;     // vyaw
  for (size_t i = 0; i < 36; ++i) {
    out.twist.covariance[i] = cov[i];
  }

  twist_pub_->publish(out);
}

}  // namespace r1_nav_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<r1_nav_cpp::ChassisTwistRelay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

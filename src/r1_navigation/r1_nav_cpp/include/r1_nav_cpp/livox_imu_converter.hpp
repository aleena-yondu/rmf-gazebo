#ifndef R1_NAV_CPP__LIVOX_IMU_CONVERTER_HPP_
#define R1_NAV_CPP__LIVOX_IMU_CONVERTER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

namespace r1_nav_cpp
{

// Prepares the raw Livox IMU stream (/livox/imu) for the robot_localization EKF.
// The Livox MID-360 reports linear acceleration in units of g, so it is scaled
// to m/s^2. The message carries no orientation, so orientation_covariance[0] is
// set to -1 to flag it as unavailable, and explicit diagonal covariances are
// attached to the angular velocity and linear acceleration.
class LivoxImuConverter : public rclcpp::Node
{
public:
  LivoxImuConverter();
  ~LivoxImuConverter() override = default;

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  double accel_scale_;
  double angular_velocity_variance_;
  double linear_acceleration_variance_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

}  // namespace r1_nav_cpp

#endif  // R1_NAV_CPP__LIVOX_IMU_CONVERTER_HPP_

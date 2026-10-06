#include "r1_nav_cpp/livox_imu_converter.hpp"

namespace r1_nav_cpp
{

LivoxImuConverter::LivoxImuConverter()
: Node("livox_imu_converter")
{
  input_topic_ = this->declare_parameter<std::string>("input_topic", "/livox/imu");
  output_topic_ = this->declare_parameter<std::string>("output_topic", "/livox/imu/converted");

  // Livox MID-360 reports linear acceleration in g. Set to 1.0 if the driver is
  // configured to publish m/s^2 already.
  accel_scale_ = this->declare_parameter<double>("accel_scale", 9.80665);

  angular_velocity_variance_ =
    this->declare_parameter<double>("angular_velocity_variance", 0.01);
  linear_acceleration_variance_ =
    this->declare_parameter<double>("linear_acceleration_variance", 0.05);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LivoxImuConverter::onImu, this, std::placeholders::_1));

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    output_topic_, rclcpp::SensorDataQoS());

  RCLCPP_INFO(this->get_logger(),
              "LivoxImuConverter started: %s -> %s (accel_scale=%.5f)",
              input_topic_.c_str(), output_topic_.c_str(), accel_scale_);
}

void LivoxImuConverter::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  sensor_msgs::msg::Imu out = *msg;

  // Convert acceleration from g to m/s^2.
  out.linear_acceleration.x = msg->linear_acceleration.x * accel_scale_;
  out.linear_acceleration.y = msg->linear_acceleration.y * accel_scale_;
  out.linear_acceleration.z = msg->linear_acceleration.z * accel_scale_;

  // No orientation estimate is provided by the Livox IMU.
  out.orientation = geometry_msgs::msg::Quaternion();
  out.orientation.w = 1.0;
  out.orientation_covariance.fill(0.0);
  out.orientation_covariance[0] = -1.0;  // flag orientation as unavailable

  out.angular_velocity_covariance.fill(0.0);
  out.angular_velocity_covariance[0] = angular_velocity_variance_;
  out.angular_velocity_covariance[4] = angular_velocity_variance_;
  out.angular_velocity_covariance[8] = angular_velocity_variance_;

  out.linear_acceleration_covariance.fill(0.0);
  out.linear_acceleration_covariance[0] = linear_acceleration_variance_;
  out.linear_acceleration_covariance[4] = linear_acceleration_variance_;
  out.linear_acceleration_covariance[8] = linear_acceleration_variance_;

  imu_pub_->publish(out);
}

}  // namespace r1_nav_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<r1_nav_cpp::LivoxImuConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

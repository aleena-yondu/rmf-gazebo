#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using std::placeholders::_1;

class CmdVelRelay : public rclcpp::Node
{
public:
  CmdVelRelay()
  : Node("cmd_vel_relay")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10,
      std::bind(&CmdVelRelay::cmdVelCallback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/motion_target/target_speed_chassis",
      10);

    RCLCPP_INFO(this->get_logger(), "CmdVelRelay started: bridging /cmd_vel -> /motion_target/target_speed_chassis");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    geometry_msgs::msg::TwistStamped stamped;
    // rclcpp::Time::to_msg() may not be available in all rclcpp versions,
    // so convert to builtin_interfaces::msg::Time manually.
    {
      const uint64_t ns = this->now().nanoseconds();
      stamped.header.stamp.sec = static_cast<int32_t>(ns / 1000000000ULL);
      stamped.header.stamp.nanosec = static_cast<uint32_t>(ns % 1000000000ULL);
    }
    stamped.header.frame_id = "base_link";
    stamped.twist = *msg;
    publisher_->publish(stamped);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelRelay>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

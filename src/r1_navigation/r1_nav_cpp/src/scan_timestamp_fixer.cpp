/**
 * @file scan_timestamp_fixer.cpp
 * @brief Fixes future timestamps in LaserScan messages by replacing with current time
 * 
 * This node addresses a critical issue where sensors (like Livox LiDAR) publish data
 * with timestamps that are 1-2 seconds in the future. This causes TF lookup failures
 * in SLAM Toolbox and other nodes because the TF buffer doesn't have data "from the future".
 * 
 * The node subscribes to a LaserScan topic, replaces the timestamp with current time,
 * and republishes the corrected scan.
 * 
 * @author GitHub Copilot
 * @date November 7, 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ScanTimestampFixer : public rclcpp::Node
{
public:
  ScanTimestampFixer() : Node("scan_timestamp_fixer")
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/scan_filtered_temp");
    this->declare_parameter<std::string>("output_topic", "/scan");
    
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    
    // Match pointcloud_to_laserscan's sensor-data QoS; otherwise the
    // best-effort /scan_raw publisher is incompatible with a reliable
    // default subscription.
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, 
      rclcpp::SensorDataQoS(),
      std::bind(&ScanTimestampFixer::scan_callback, this, std::placeholders::_1)
    );
    
    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      output_topic, 
      rclcpp::SensorDataQoS()
    );
    
    RCLCPP_INFO(this->get_logger(), 
                "Scan timestamp fixer started: %s -> %s", 
                input_topic.c_str(), 
                output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), 
                "Fixing future timestamps by replacing with current time");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Create a copy of the message
    auto corrected_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
    
    // Replace timestamp with current time
    corrected_msg->header.stamp = this->now();
    
    // Publish the corrected scan
    publisher_->publish(*corrected_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanTimestampFixer>());
  rclcpp::shutdown();
  return 0;
}

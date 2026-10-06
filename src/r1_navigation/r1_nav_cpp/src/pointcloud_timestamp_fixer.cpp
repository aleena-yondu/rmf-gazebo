/**
 * @file pointcloud_timestamp_fixer.cpp
 * @brief Fixes past/future timestamps in PointCloud2 messages using a calibrated offset
 * 
 * This node addresses timestamp synchronization issues where sensors (like Livox LiDAR)
 * publish pointclouds with timestamps that are seconds in the past or future.
 * 
 * CRITICAL: This node preserves relative timing between consecutive messages by calculating
 * a time offset on the first message and applying it consistently. This ensures that the
 * scan data correctly represents the robot's orientation at capture time, not processing time.
 * 
 * Using now() would cause rotation lag because the transform would use the current robot
 * orientation instead of the orientation when the LiDAR captured the data.
 * 
 * @author GitHub Copilot
 * @date November 26, 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudTimestampFixer : public rclcpp::Node
{
public:
  PointCloudTimestampFixer() 
    : Node("pointcloud_timestamp_fixer"),
      msg_count_(0),
      offset_calibrated_(false),
      offset_sum_(0.0),
      time_offset_(rclcpp::Duration::from_seconds(0.0))
  {
    // Declare parameters
    this->declare_parameter<std::string>("input_topic", "/livox/lidar_raw");
    this->declare_parameter<std::string>("output_topic", "/livox/lidar");
    this->declare_parameter<int>("calibration_samples", 10);
    
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    calibration_samples_ = this->get_parameter("calibration_samples").as_int();
    
    // Create subscriber with SensorDataQoS (best_effort, volatile) for LiDAR compatibility
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloudTimestampFixer::pointcloud_callback, this, std::placeholders::_1)
    );
    
    // Publish with SensorDataQoS to maintain compatibility with downstream nodes
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic, 
      rclcpp::SensorDataQoS()
    );
    
    RCLCPP_INFO(this->get_logger(), 
                "PointCloud timestamp fixer started: %s -> %s", 
                input_topic.c_str(), 
                output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), 
                "Calibrating timestamp offset using %d samples (preserves relative timing)", 
                calibration_samples_);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    auto original_time = rclcpp::Time(msg->header.stamp);
    auto current_time = this->now();
    
    // Calibration phase: Calculate average offset over first N messages
    if (!offset_calibrated_)
    {
      double diff = (current_time - original_time).seconds();
      offset_sum_ += diff;
      
      RCLCPP_INFO(this->get_logger(), 
                  "Calibrating [%lu/%d]: sensor_time=%f, ros_time=%f, offset=%.3fs",
                  msg_count_ + 1,
                  calibration_samples_,
                  original_time.seconds(),
                  current_time.seconds(),
                  diff);
      
      msg_count_++;
      
      if (msg_count_ >= static_cast<size_t>(calibration_samples_))
      {
        double avg_offset = offset_sum_ / calibration_samples_;
        time_offset_ = rclcpp::Duration::from_seconds(avg_offset);
        offset_calibrated_ = true;
        
        RCLCPP_INFO(this->get_logger(), 
                    "Calibration complete! Average offset: %.3fs (sensor is %s ROS time)",
                    avg_offset,
                    avg_offset > 0 ? "AHEAD of" : "BEHIND");
        RCLCPP_INFO(this->get_logger(), 
                    "Now applying consistent offset to preserve relative timing between scans");
      }
      return;  // Don't publish during calibration
    }
    
    // Create a copy and apply the calibrated offset
    auto corrected_msg = std::make_shared<sensor_msgs::msg::PointCloud2>(*msg);
    corrected_msg->header.stamp = rclcpp::Time(original_time + time_offset_);
    
    // Publish the corrected pointcloud
    publisher_->publish(*corrected_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  size_t msg_count_;
  int calibration_samples_;
  bool offset_calibrated_;
  double offset_sum_;
  rclcpp::Duration time_offset_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTimestampFixer>());
  rclcpp::shutdown();
  return 0;
}

#ifndef R1_NAV_CPP__FOOTPRINT_VISUALIZER_HPP_
#define R1_NAV_CPP__FOOTPRINT_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <memory>

namespace r1_nav_cpp
{

class FootprintVisualizer : public rclcpp::Node
{
public:
  FootprintVisualizer();
  ~FootprintVisualizer() override = default;

private:
  void timerCallback();
  void publishFootprint();
  
  // Parameters
  std::string base_frame_;
  std::string workspace_frame_;
  double publish_rate_;
  
  // Robot footprint vertices (rectangular approximation)
  std::vector<Eigen::Vector2d> footprint_vertices_;
  
  // TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace r1_nav_cpp

#endif  // R1_NAV_CPP__FOOTPRINT_VISUALIZER_HPP_

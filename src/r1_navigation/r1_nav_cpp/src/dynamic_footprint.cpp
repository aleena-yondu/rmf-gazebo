// dynamic_footprint.cpp
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

using namespace std::chrono_literals;

class DynamicFootprintPublisher : public rclcpp::Node
{
public:
    DynamicFootprintPublisher()
    : Node("dynamic_footprint_publisher"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Parameters
        publish_rate_ = 0.5;  // 2 Hz
        base_frame_ = "base_link";

        // Default footprint extents
        default_x_extent_ = 0.185;
        default_y_extent_ = 0.327;
        default_negative_y_extent_ = -0.327;

        // Links to monitor for extending the footprint
        relevant_links_ = {
            "torso_link1",
            "torso_link2",
            "torso_link3",
            "torso_link4",
            "right_gripper_finger_link1",
            "right_gripper_finger_link2",
            "left_gripper_finger_link1",
            "left_gripper_finger_link2"
        };

        // Publisher for PolygonStamped (for visualization)
        footprint_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/dynamic_footprint",
            rclcpp::QoS(1).transient_local()
        );

        // Service clients to update costmap footprints
        local_costmap_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
            "/local_costmap/local_costmap/set_parameters"
        );
        
        global_costmap_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
            "/global_costmap/global_costmap/set_parameters"
        );

        // Timer
        auto period = std::chrono::duration<double>(publish_rate_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&DynamicFootprintPublisher::publish_footprint, this)
        );

        RCLCPP_INFO(this->get_logger(), "Dynamic Footprint Publisher has started.");
        RCLCPP_INFO(this->get_logger(), "Will update footprints via services to local_costmap and global_costmap");
    }

private:
    void publish_footprint()
    {
        // 1. Get current dynamic extents
        double x_ext, y_ext, neg_y_ext;
        if (!calculate_dynamic_extents(x_ext, y_ext, neg_y_ext)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Could not calculate dynamic extents, using defaults.");
        }

        // 2. Create the polygon points
        std::vector<geometry_msgs::msg::Point32> points(6);
        points[0].x = x_ext;  points[0].y = y_ext;  points[0].z = 0.0;
        points[1].x = -0.144; points[1].y = y_ext;  points[1].z = 0.0;
        points[2].x = -0.382; points[2].y = 0.119;  points[2].z = 0.0;
        points[3].x = -0.382; points[3].y = -0.119; points[3].z = 0.0;
        points[4].x = -0.144; points[4].y = neg_y_ext; points[4].z = 0.0;
        points[5].x = x_ext;  points[5].y = neg_y_ext; points[5].z = 0.0;

        // 3. Publish for visualization
        auto footprint_msg = geometry_msgs::msg::PolygonStamped();
        footprint_msg.header.stamp = this->now();
        footprint_msg.header.frame_id = base_frame_;
        footprint_msg.polygon.points = points;
        footprint_pub_->publish(footprint_msg);

        // 4. Update costmaps via service calls
        update_costmap_footprint(points);
    }

    void update_costmap_footprint(const std::vector<geometry_msgs::msg::Point32>& points)
    {
        // Convert points to string format: [[x1,y1],[x2,y2],...]
        std::string footprint_str = "[";
        for (size_t i = 0; i < points.size(); ++i) {
            footprint_str += "[" + std::to_string(points[i].x) + "," + 
                            std::to_string(points[i].y) + "]";
            if (i < points.size() - 1) footprint_str += ",";
        }
        footprint_str += "]";

        // Create parameter update request
        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        rcl_interfaces::msg::Parameter param;
        param.name = "footprint";
        param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        param.value.string_value = footprint_str;
        request->parameters.push_back(param);

        // Send to local costmap
        if (local_costmap_client_->service_is_ready()) {
            local_costmap_client_->async_send_request(request,
                [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
                    auto result = future.get();
                    if (!result->results.empty() && result->results[0].successful) {
                        RCLCPP_DEBUG(this->get_logger(), "Local costmap footprint updated");
                    }
                });
        }

        // Send to global costmap
        if (global_costmap_client_->service_is_ready()) {
            global_costmap_client_->async_send_request(request,
                [this](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
                    auto result = future.get();
                    if (!result->results.empty() && result->results[0].successful) {
                        RCLCPP_DEBUG(this->get_logger(), "Global costmap footprint updated");
                    }
                });
        }
    }

    bool calculate_dynamic_extents(double &x_ext, double &y_ext, double &neg_y_ext)
    {
        // Start with default values
        x_ext = default_x_extent_;
        y_ext = default_y_extent_;
        neg_y_ext = default_negative_y_extent_;
        
        bool success = true;
        // Check each relevant link
        for (const auto &link : relevant_links_) {
            try {
                // Get the transform from base_link to the link
                // Use a small timeout to get recent transform
                geometry_msgs::msg::TransformStamped t = tf_buffer_.lookupTransform(
                    base_frame_,
                    link,
                    tf2::TimePointZero,  // Get the latest available transform
                    tf2::durationFromSec(0.1)  // 100ms timeout
                );
                
                // Update extents based on the link's translation
                x_ext = std::max(x_ext, t.transform.translation.x);
                y_ext = std::max(y_ext, t.transform.translation.y);
                neg_y_ext = std::min(neg_y_ext, t.transform.translation.y);

            } catch (const tf2::TransformException &ex) {
                // If a transform fails, just skip this link silently
                // (torso and gripper links may not always exist in TF)
                success = false;
            }
        }

        return success;
    }

    // Member variables
    double publish_rate_;
    std::string base_frame_;
    
    double default_x_extent_;
    double default_y_extent_;
    double default_negative_y_extent_;
    
    std::vector<std::string> relevant_links_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_pub_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr local_costmap_client_;
    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr global_costmap_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicFootprintPublisher>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Unhandled exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

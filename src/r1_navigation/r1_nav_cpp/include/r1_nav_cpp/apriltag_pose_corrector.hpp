#ifndef R1_NAV_CPP__APRILTAG_POSE_CORRECTOR_HPP_
#define R1_NAV_CPP__APRILTAG_POSE_CORRECTOR_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "r1_interfaces/msg/tag_detections.hpp"

namespace r1_nav_cpp
{

/**
 * @brief Computes the robot's map-frame pose from AprilTag detections and
 *        publishes it as a generic pose correction on /pose_correction.
 *
 * This node does NOT publish any TF. It is a source of absolute pose
 * observations for a downstream consumer (e.g. slam_toolbox localization's
 * /pose_correction subscriber), which fuses them as unary priors in its pose
 * graph and owns the map->odom TF itself.
 *
 * Position-only correction: only the tag's measured POSITION is used (its
 * measured orientation is intentionally ignored, as a single planar fiducial
 * gives an unreliable rotation estimate). Using the robot's current map-frame
 * orientation to place the observed tag, it solves the robot's xy so the tag
 * lands on its known map position:
 *   p_map_base = p_map_tag_known - R_map_base_current * (T_base_cam * p_cam_tag)
 * The published orientation is the robot's currently reported yaw, emitted with
 * a large yaw covariance so downstream fusion effectively ignores it. The xy
 * covariance grows with tag range and viewing obliqueness, so a close, head-on
 * tag yields a confident xy fix while a far/oblique tag yields a loose one.
 */
class AprilTagPoseCorrector : public rclcpp::Node
{
public:
  AprilTagPoseCorrector();
  ~AprilTagPoseCorrector() override = default;

private:
  // === Setup ===
  void declareParameters();
  // Orchestrates tag-pose loading: prefer the pre-computed pure-pose file
  // (apriltag_poses.yaml) when present, otherwise derive poses from the
  // segment-based warehouse.yaml. Both produce identical map-frame poses.
  bool loadTagPoses();
  // Resolve the LOCATION/LIDAR warehouse config directory under the package
  // share (config/warehouse_configs/<location>/<lidar>). Returns false if the
  // package share dir cannot be located.
  bool resolveWarehouseConfigDir(std::string & dir_out) const;
  // Load pre-computed pure poses (x, y, z, yaw_deg per tag id) from
  // apriltag_poses.yaml, using the same tag-axis convention as the warehouse
  // loader (+Z = aisle normal, +Y = world up).
  bool loadTagPosesFromPosesYaml(const std::string & yaml_path);
  bool loadTagPosesFromWarehouseYaml();

  // === Callbacks ===
  void detectionsCallback(const r1_interfaces::msg::TagDetections::SharedPtr msg);

  // === Frame names + topics (parameters) ===
  std::string map_frame_;
  std::string base_frame_;
  std::string camera_frame_;
  std::string detections_topic_;
  std::string pose_correction_topic_;

  // === Covariance model + gates (parameters) ===
  double max_tag_range_m_{4.0};
  double xy_stddev_base_m_{0.05};
  double xy_stddev_per_m_{0.03};
  double yaw_stddev_base_rad_{0.05};
  double yaw_stddev_per_m_{0.05};
  double cos_incidence_floor_{0.2};
  double large_variance_{1.0e6};
  double max_incidence_deg_{75.0};
  // Timeout (s) for the map->base_link lookup at the detection stamp before
  // falling back to the latest available transform.
  double transform_timeout_s_{0.1};

  // === Tag map-frame poses, keyed by apriltag_id ===
  std::unordered_map<int, tf2::Transform> tag_map_pose_;

  // === ROS interfaces ===
  rclcpp::Subscription<r1_interfaces::msg::TagDetections>::SharedPtr detections_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_correction_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace r1_nav_cpp

#endif  // R1_NAV_CPP__APRILTAG_POSE_CORRECTOR_HPP_

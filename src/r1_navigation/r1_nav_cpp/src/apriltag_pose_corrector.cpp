#include "r1_nav_cpp/apriltag_pose_corrector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <limits>
#include <string>
#include <unordered_set>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "yaml-cpp/yaml.h"

namespace r1_nav_cpp
{

AprilTagPoseCorrector::AprilTagPoseCorrector()
: rclcpp::Node("apriltag_pose_corrector")
{
  declareParameters();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!loadTagPoses()) {
    RCLCPP_WARN(get_logger(),
      "AprilTagPoseCorrector: no tag map poses loaded; no pose corrections "
      "will be published until a valid apriltag_poses.yaml or warehouse.yaml "
      "is available.");
  } else {
    RCLCPP_INFO(get_logger(),
      "AprilTagPoseCorrector: loaded %zu AprilTag map-frame poses",
      tag_map_pose_.size());
  }

  rclcpp::QoS correction_qos(rclcpp::KeepLast(10));
  correction_qos.reliable();
  pose_correction_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    pose_correction_topic_, correction_qos);

  rclcpp::QoS det_qos(rclcpp::KeepLast(10));
  det_qos.best_effort();
  detections_sub_ = this->create_subscription<r1_interfaces::msg::TagDetections>(
    detections_topic_, det_qos,
    std::bind(&AprilTagPoseCorrector::detectionsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "AprilTagPoseCorrector ready: subscribed to %s, publishing robot pose "
    "corrections on %s (frame=%s), gates=(range=%.2f m, incidence=%.1f deg).",
    detections_topic_.c_str(), pose_correction_topic_.c_str(),
    map_frame_.c_str(), max_tag_range_m_, max_incidence_deg_);
}

// ============================================================================
// Setup
// ============================================================================

void AprilTagPoseCorrector::declareParameters()
{
  map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "zed_tracking_link");
  detections_topic_ = this->declare_parameter<std::string>("detections_topic", "/detections");
  pose_correction_topic_ =
    this->declare_parameter<std::string>("pose_correction_topic", "/pose_correction");

  max_tag_range_m_ = this->declare_parameter<double>("max_tag_range_m", 4.0);
  xy_stddev_base_m_ = this->declare_parameter<double>("xy_stddev_base_m", 0.05);
  xy_stddev_per_m_ = this->declare_parameter<double>("xy_stddev_per_m", 0.03);
  yaw_stddev_base_rad_ = this->declare_parameter<double>("yaw_stddev_base_rad", 0.05);
  yaw_stddev_per_m_ = this->declare_parameter<double>("yaw_stddev_per_m", 0.05);
  cos_incidence_floor_ = this->declare_parameter<double>("cos_incidence_floor", 0.2);
  large_variance_ = this->declare_parameter<double>("large_variance", 1.0e6);
  max_incidence_deg_ = this->declare_parameter<double>("max_incidence_deg", 75.0);
  transform_timeout_s_ = this->declare_parameter<double>("transform_timeout_s", 0.1);
}

bool AprilTagPoseCorrector::resolveWarehouseConfigDir(std::string & dir_out) const
{
  std::string nav_pkg;
  try {
    nav_pkg = ament_index_cpp::get_package_share_directory("r1_nav_cpp");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to locate r1_nav_cpp share dir: %s", e.what());
    return false;
  }

  const char * loc_env = std::getenv("LOCATION");
  std::string location = (loc_env && loc_env[0] != '\0') ? loc_env : "yondu";
  std::transform(location.begin(), location.end(), location.begin(), ::tolower);

  const char * lidar_env = std::getenv("LIDAR");
  std::string lidar = (lidar_env && lidar_env[0] != '\0') ? lidar_env : "livox";
  std::transform(lidar.begin(), lidar.end(), lidar.begin(), ::tolower);

  dir_out = nav_pkg + "/config/warehouse_configs/" + location + "/" + lidar;
  return true;
}

bool AprilTagPoseCorrector::loadTagPoses()
{
  std::string dir;
  if (resolveWarehouseConfigDir(dir)) {
    const std::string poses_path = dir + "/apriltag_poses.yaml";
    std::ifstream poses_file(poses_path);
    if (poses_file.good()) {
      RCLCPP_INFO(get_logger(),
        "AprilTagPoseCorrector: found pre-computed pose file, loading tag poses "
        "from %s (preferred over warehouse.yaml).", poses_path.c_str());
      if (loadTagPosesFromPosesYaml(poses_path)) {
        return true;
      }
      RCLCPP_WARN(get_logger(),
        "AprilTagPoseCorrector: apriltag_poses.yaml at %s failed to load or was "
        "empty; falling back to segment-based warehouse.yaml.",
        poses_path.c_str());
    } else {
      RCLCPP_INFO(get_logger(),
        "AprilTagPoseCorrector: no apriltag_poses.yaml at %s; deriving tag poses "
        "from warehouse.yaml.", poses_path.c_str());
    }
  }
  return loadTagPosesFromWarehouseYaml();
}

bool AprilTagPoseCorrector::loadTagPosesFromPosesYaml(const std::string & yaml_path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(),
      "Failed to load apriltag_poses.yaml at '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  YAML::Node tags = root["apriltags"];
  if (!tags || !tags.IsMap()) {
    RCLCPP_ERROR(get_logger(),
      "apriltag_poses.yaml at '%s' has no 'apriltags' map", yaml_path.c_str());
    return false;
  }

  tag_map_pose_.clear();
  size_t skipped = 0;
  for (auto it = tags.begin(); it != tags.end(); ++it) {
    const std::string key = it->first.as<std::string>();
    int tag_id = -1;
    try {
      tag_id = std::stoi(key);
    } catch (...) {
      ++skipped;
      RCLCPP_WARN(get_logger(),
        "apriltag_poses.yaml: non-integer tag key '%s' skipped", key.c_str());
      continue;
    }

    YAML::Node node = it->second;
    if (!node || !node.IsMap() || !node["x"] || !node["y"]) {
      ++skipped;
      RCLCPP_WARN(get_logger(),
        "apriltag_poses.yaml: tag '%s' missing x/y; skipped", key.c_str());
      continue;
    }

    const double x = node["x"].as<double>(0.0);
    const double y = node["y"].as<double>(0.0);
    const double z = node["z"].as<double>(0.0);
    // Rebuild the SAME tag basis as the warehouse loader so both formats yield
    // identical poses: +Z = aisle normal (yaw_deg direction), +Y = world up,
    // +X = Y x Z.
    const double yaw_rad = node["yaw_deg"].as<double>(0.0) * M_PI / 180.0;
    const double sy_ = std::sin(yaw_rad);
    const double cy_ = std::cos(yaw_rad);
    const tf2::Matrix3x3 R(
      -sy_, 0.0,  cy_,
       cy_, 0.0,  sy_,
       0.0, 1.0,  0.0);

    tf2::Transform t;
    t.setBasis(R);
    t.setOrigin(tf2::Vector3(x, y, z));
    tag_map_pose_[tag_id] = t;

    RCLCPP_INFO(get_logger(),
      "AprilTag id=%d (pure pose): map_pose=(x=%.3f, y=%.3f, z=%.3f), "
      "facing yaw=%.1f deg", tag_id, x, y, z, yaw_rad * 180.0 / M_PI);
  }

  if (skipped > 0) {
    RCLCPP_WARN(get_logger(),
      "apriltag_poses.yaml: %zu entr(ies) skipped", skipped);
  }

  return !tag_map_pose_.empty();
}

bool AprilTagPoseCorrector::loadTagPosesFromWarehouseYaml()
{
  std::string dir;
  if (!resolveWarehouseConfigDir(dir)) {
    return false;
  }
  const std::string yaml_path = dir + "/warehouse.yaml";

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(),
      "Failed to load warehouse.yaml at '%s': %s", yaml_path.c_str(), e.what());
    return false;
  }

  YAML::Node rows = root["warehouse"]["rows"];
  if (!rows || !rows.IsMap()) {
    RCLCPP_ERROR(get_logger(),
      "warehouse.yaml at '%s' has no 'warehouse.rows' map", yaml_path.c_str());
    return false;
  }

  tag_map_pose_.clear();
  // Tag IDs that have been registered via a `start_apriltag_id`. Start tags
  // take precedence over end tags: any end-tag entry for an id present (now
  // or later) in this set is dropped, regardless of YAML iteration order.
  std::unordered_set<int> start_tag_ids;
  size_t skipped_no_tag = 0;
  size_t skipped_no_shelf_b = 0;
  size_t end_skipped_start_priority = 0;
  size_t end_skipped_duplicate = 0;
  size_t end_skipped_no_distance = 0;

  for (auto row_it = rows.begin(); row_it != rows.end(); ++row_it) {
    YAML::Node row_node = row_it->second;
    const std::string row_name = row_it->first.as<std::string>();

    // Shelf B height in meters is the z of the AprilTag mounting plane.
    double shelf_b_z = 0.0;
    bool have_shelf_b = false;
    YAML::Node heights_m = row_node["shelf_heights_meters"];
    if (heights_m && heights_m.IsMap() && heights_m["B"]) {
      shelf_b_z = heights_m["B"].as<double>();
      have_shelf_b = true;
    }

    YAML::Node segments = row_node["segments"];
    if (!segments || !segments.IsSequence()) {
      continue;
    }

    for (size_t s = 0; s < segments.size(); ++s) {
      YAML::Node seg_node = segments[s];
      const double yaw_rad = seg_node["yaw_deg"].as<double>(0.0) * M_PI / 180.0;
      const double sx = seg_node["start_point"]["x"].as<double>(0.0);
      const double sy = seg_node["start_point"]["y"].as<double>(0.0);
      const double ex = seg_node["end_point"]["x"].as<double>(0.0);
      const double ey = seg_node["end_point"]["y"].as<double>(0.0);
      const double dx = ex - sx;
      const double dy = ey - sy;
      const double seg_len = std::sqrt(dx * dx + dy * dy);
      if (seg_len < 1e-6) {
        continue;
      }
      const double ux = dx / seg_len;
      const double uy = dy / seg_len;

      // Tag rotation matrix (columns are tag axes in map frame):
      //   +Z = aisle normal (yaw_deg direction), +Y = world up, +X = Y x Z
      // R = [ -sin(yaw)  0  cos(yaw) ]
      //     [  cos(yaw)  0  sin(yaw) ]
      //     [   0        1  0        ]
      const double sy_ = std::sin(yaw_rad);
      const double cy_ = std::cos(yaw_rad);
      const tf2::Matrix3x3 R(
        -sy_, 0.0,  cy_,
         cy_, 0.0,  sy_,
         0.0, 1.0,  0.0);

      // Every tag on this segment shares R and shelf_b_z; only the distance
      // along the segment changes between start and end fiducials.
      auto make_tag_pose = [&](double dist_along_segment) {
        tf2::Transform t;
        t.setBasis(R);
        t.setOrigin(tf2::Vector3(
          sx + dist_along_segment * ux,
          sy + dist_along_segment * uy,
          shelf_b_z));
        return t;
      };

      YAML::Node bays = seg_node["bays"];
      if (!bays || !bays.IsMap()) {
        continue;
      }

      for (auto bay_it = bays.begin(); bay_it != bays.end(); ++bay_it) {
        YAML::Node bay_node = bay_it->second;
        const std::string bay_name = bay_it->first.as<std::string>();

        // Schema rename: prefer `start_apriltag_id`, fall back to the legacy
        // `apriltag_id` key for un-migrated configs.
        int start_id = -1;
        if (bay_node["start_apriltag_id"]) {
          start_id = bay_node["start_apriltag_id"].as<int>(-1);
        } else if (bay_node["apriltag_id"]) {
          start_id = bay_node["apriltag_id"].as<int>(-1);
        }
        int end_id = -1;
        if (bay_node["end_apriltag_id"]) {
          end_id = bay_node["end_apriltag_id"].as<int>(-1);
        }

        if (start_id < 0 && end_id < 0) {
          ++skipped_no_tag;
          continue;
        }
        if (!have_shelf_b) {
          ++skipped_no_shelf_b;
          continue;
        }

        // --- Start tag: always wins. Overwrites any prior end-tag entry
        // that shared this id, and blocks future end-tag entries with the
        // same id via `start_tag_ids`.
        if (start_id >= 0) {
          const double start_d = bay_node["start_distance"].as<double>(0.0);
          const tf2::Transform t = make_tag_pose(start_d);
          tag_map_pose_[start_id] = t;
          start_tag_ids.insert(start_id);
          RCLCPP_INFO(get_logger(),
            "AprilTag id=%d (start) in row='%s' bay='%s': "
            "map_pose=(x=%.3f, y=%.3f, z=%.3f), facing yaw=%.1f deg",
            start_id, row_name.c_str(), bay_name.c_str(),
            t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(),
            yaw_rad * 180.0 / M_PI);
        }

        // --- End tag: register so detections of end fiducials are usable
        // as localization observations, but only when no start tag with the
        // same id has been (or will be) registered. start_id == end_id in
        // the same bay is also treated as start-priority.
        if (end_id >= 0) {
          if (end_id == start_id ||
              start_tag_ids.find(end_id) != start_tag_ids.end())
          {
            ++end_skipped_start_priority;
          } else if (!bay_node["end_distance"]) {
            ++end_skipped_no_distance;
            RCLCPP_WARN(get_logger(),
              "AprilTag id=%d declared as end tag in row='%s' bay='%s' but "
              "no `end_distance` key is present; skipping",
              end_id, row_name.c_str(), bay_name.c_str());
          } else if (tag_map_pose_.find(end_id) != tag_map_pose_.end()) {
            // Same id was registered as an end tag in a previously
            // processed bay; keep the first instance for deterministic
            // behavior. (If the configured positions agree, the choice
            // doesn't matter; if they disagree, the warning surfaces it.)
            ++end_skipped_duplicate;
            RCLCPP_WARN(get_logger(),
              "AprilTag id=%d declared as end tag in multiple bays "
              "(row='%s' bay='%s'); keeping first registered pose",
              end_id, row_name.c_str(), bay_name.c_str());
          } else {
            const double end_d = bay_node["end_distance"].as<double>(0.0);
            const tf2::Transform t = make_tag_pose(end_d);
            tag_map_pose_[end_id] = t;
            RCLCPP_INFO(get_logger(),
              "AprilTag id=%d (end) in row='%s' bay='%s': "
              "map_pose=(x=%.3f, y=%.3f, z=%.3f), facing yaw=%.1f deg",
              end_id, row_name.c_str(), bay_name.c_str(),
              t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(),
              yaw_rad * 180.0 / M_PI);
          }
        }
      }
    }
  }

  if (skipped_no_tag > 0) {
    RCLCPP_INFO(get_logger(),
      "AprilTagPoseCorrector: %zu bay(s) had no apriltag id "
      "(start or end) and were skipped",
      skipped_no_tag);
  }
  if (skipped_no_shelf_b > 0) {
    RCLCPP_WARN(get_logger(),
      "AprilTagPoseCorrector: %zu bay(s) skipped because their row had no "
      "shelf_heights_meters['B'] entry",
      skipped_no_shelf_b);
  }
  if (end_skipped_start_priority > 0) {
    RCLCPP_INFO(get_logger(),
      "AprilTagPoseCorrector: %zu end-tag entr(ies) ignored because the "
      "same id is registered as a start_apriltag_id (start takes priority)",
      end_skipped_start_priority);
  }
  if (end_skipped_duplicate > 0) {
    RCLCPP_WARN(get_logger(),
      "AprilTagPoseCorrector: %zu duplicate end-tag entr(ies) ignored",
      end_skipped_duplicate);
  }
  if (end_skipped_no_distance > 0) {
    RCLCPP_WARN(get_logger(),
      "AprilTagPoseCorrector: %zu end-tag entr(ies) skipped due to missing "
      "`end_distance`",
      end_skipped_no_distance);
  }

  return !tag_map_pose_.empty();
}

// ============================================================================
// /detections callback: compute robot pose(s) and publish a pose correction
// ============================================================================

void AprilTagPoseCorrector::detectionsCallback(
  const r1_interfaces::msg::TagDetections::SharedPtr msg)
{
  if (msg->poses.empty() || tag_map_pose_.empty()) {
    return;
  }

  // base_link -> camera_frame is a static URDF transform; safe to look up
  // without time stamping (TimePointZero gets the latest).
  geometry_msgs::msg::TransformStamped baselink_to_cam_msg;
  try {
    baselink_to_cam_msg = tf_buffer_->lookupTransform(
      base_frame_, camera_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "detectionsCallback: %s -> %s lookup failed: %s",
      base_frame_.c_str(), camera_frame_.c_str(), ex.what());
    return;
  }
  tf2::Transform T_base_cam;
  tf2::fromMsg(baselink_to_cam_msg.transform, T_base_cam);

  // Current robot orientation in the map frame. We deliberately ignore the
  // tag's measured orientation and instead rotate the observed tag position
  // into the map frame using the robot's currently reported orientation. The
  // same yaw is passed straight through to the correction (with a large yaw
  // covariance) so downstream fusion does not act on it.
  geometry_msgs::msg::TransformStamped map_to_base_msg;
  try {
    map_to_base_msg = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, msg->header.stamp,
      tf2::durationFromSec(transform_timeout_s_));
  } catch (const tf2::TransformException & ex) {
    try {
      map_to_base_msg = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "detectionsCallback: %s -> %s lookup failed (%s); latest also failed "
        "(%s). Cannot compute correction without current orientation.",
        map_frame_.c_str(), base_frame_.c_str(), ex.what(), ex2.what());
      return;
    }
  }
  tf2::Transform T_map_base_current;
  tf2::fromMsg(map_to_base_msg.transform, T_map_base_current);
  const tf2::Matrix3x3 R_map_base = T_map_base_current.getBasis();
  const double current_yaw = tf2::getYaw(T_map_base_current.getRotation());

  const double cos_incidence_gate =
    (max_incidence_deg_ > 0.0 && max_incidence_deg_ < 90.0)
    ? std::cos(max_incidence_deg_ * M_PI / 180.0)
    : -1.0;

  // With the single-tag assumption we still robustly handle >1 pose per
  // message by selecting the best-geometry observation (closest + most
  // head-on) and publishing exactly one correction.
  bool have_best = false;
  double best_score = -std::numeric_limits<double>::infinity();
  tf2::Vector3 best_p_map_base(0.0, 0.0, 0.0);
  double best_sigma_xy = 0.0;

  size_t n_skipped_unknown = 0;
  size_t n_skipped_range = 0;
  size_t n_skipped_incidence = 0;

  for (const auto & ps : msg->poses) {
    int tag_id = -1;
    try {
      tag_id = std::stoi(ps.header.frame_id);
    } catch (...) {
      ++n_skipped_unknown;
      continue;
    }
    auto it = tag_map_pose_.find(tag_id);
    if (it == tag_map_pose_.end()) {
      ++n_skipped_unknown;
      continue;
    }
    const tf2::Transform & T_map_tag = it->second;

    // Position-only observation: use just the tag's translation in the camera
    // frame; its measured orientation is intentionally discarded.
    const tf2::Vector3 p_cam_tag(
      ps.pose.position.x, ps.pose.position.y, ps.pose.position.z);
    const double tag_range = p_cam_tag.length();
    if (tag_range > max_tag_range_m_) {
      ++n_skipped_range;
      continue;
    }

    // Robot (base_link) xy in map from this single tag, using the CURRENT
    // robot orientation to place the observed tag:
    //   p_map_tag_known = p_map_base + R_map_base * (T_base_cam * p_cam_tag)
    //   => p_map_base   = p_map_tag_known - R_map_base * (T_base_cam * p_cam_tag)
    const tf2::Vector3 p_base_tag = T_base_cam * p_cam_tag;
    const tf2::Vector3 & p_map_tag_known = T_map_tag.getOrigin();
    const tf2::Vector3 p_map_base = p_map_tag_known - R_map_base * p_base_tag;

    // Viewing obliqueness: angle between the tag's outward normal (+Z of the
    // known tag pose, i.e. the aisle normal) and the tag->camera direction.
    // cos_inc = 1 => head-on; -> 0 => edge-on. Uses the known (config) tag
    // normal and the camera position implied by the solved robot xy, so it
    // does not depend on the tag's measured orientation.
    const tf2::Vector3 n_map = T_map_tag.getBasis().getColumn(2);
    const tf2::Vector3 p_map_cam = p_map_base + R_map_base * T_base_cam.getOrigin();
    tf2::Vector3 tag_to_cam = p_map_cam - p_map_tag_known;
    double cos_inc = 1.0;
    if (tag_to_cam.length() > 1e-6) {
      tag_to_cam.normalize();
      cos_inc = std::fabs(n_map.dot(tag_to_cam));
    }

    if (cos_inc < cos_incidence_gate) {
      ++n_skipped_incidence;
      continue;
    }

    const double cos_eff = std::max(cos_inc, cos_incidence_floor_);
    const double sigma_xy =
      (xy_stddev_base_m_ + xy_stddev_per_m_ * tag_range) / cos_eff;

    // Prefer close, head-on observations (higher score is better).
    const double score = cos_inc / std::max(tag_range, 1e-3);
    if (!have_best || score > best_score) {
      have_best = true;
      best_score = score;
      best_p_map_base = p_map_base;
      best_sigma_xy = sigma_xy;
    }
  }

  if (!have_best) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "detectionsCallback: no usable tags this message "
      "(unknown=%zu, out_of_range=%zu, too_oblique=%zu)",
      n_skipped_unknown, n_skipped_range, n_skipped_incidence);
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped out;
  // Preserve the original detection message's timestamp so downstream fusion
  // (e.g. slam_toolbox) can age-gate and time-align the correction correctly.
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = map_frame_;

  // xy comes from the tag (position-only); yaw is passed straight through from
  // the robot's currently reported orientation and is not corrected.
  out.pose.pose.position.x = best_p_map_base.x();
  out.pose.pose.position.y = best_p_map_base.y();
  out.pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, current_yaw);
  out.pose.pose.orientation = tf2::toMsg(q);

  // 6x6 row-major covariance. Trust x, y from the geometry-driven model; leave
  // yaw (pass-through) and z/roll/pitch effectively unconstrained (large
  // variance) so downstream fusion ignores everything but the xy fix.
  std::array<double, 36> & cov = out.pose.covariance;
  cov.fill(0.0);
  const double var_xy = best_sigma_xy * best_sigma_xy;
  cov[0] = var_xy;            // x-x
  cov[7] = var_xy;            // y-y
  cov[14] = large_variance_;  // z-z
  cov[21] = large_variance_;  // roll-roll
  cov[28] = large_variance_;  // pitch-pitch
  cov[35] = large_variance_;  // yaw-yaw (pass-through, effectively ignored)

  pose_correction_pub_->publish(out);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
    "PoseCorrection: published robot xy (x=%.3f, y=%.3f) with pass-through "
    "yaw=%.3f rad, sigma_xy=%.3f m (yaw uncorrected).",
    out.pose.pose.position.x, out.pose.pose.position.y, current_yaw,
    best_sigma_xy);
}

}  // namespace r1_nav_cpp

// ============================================================================
// main
// ============================================================================

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<r1_nav_cpp::AprilTagPoseCorrector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

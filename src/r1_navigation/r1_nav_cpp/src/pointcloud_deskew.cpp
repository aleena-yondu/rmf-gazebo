#include "r1_nav_cpp/pointcloud_deskew.hpp"

#include <climits>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace r1_nav_cpp
{

PointCloudDeskew::PointCloudDeskew(const rclcpp::NodeOptions & options)
: Node("pointcloud_deskew", options)
{
  this->declare_parameter("enabled",         true);
  this->declare_parameter("input_topic",     std::string("/ouster/points"));
  this->declare_parameter("output_topic",    std::string("/ouster/points_deskewed"));
  this->declare_parameter("imu_topic",       std::string("/imu/data_bias_corrected"));
  this->declare_parameter("timestamp_field", std::string("t"));  // Ouster default
  // 0.02 rad/s ≈ 1.15 deg/s. At 100 ms scan that is ≤0.115° of yaw
  // distortion across the cloud, well below SLAM matching tolerance.
  this->declare_parameter("wz_gate_thresh_rad_s", 0.02);
  this->declare_parameter("linear_deskew_enabled", false);
  this->declare_parameter("odom_topic", std::string(""));
  this->declare_parameter("linear_gate_thresh_m_s", 0.02);
  // When > 0, log a periodic summary of the per-scan motion the deskew
  // saw (max |dx|/|dy|, scan_dt, vx/vy at scan-end). Lets a field operator
  // sanity-check /synchro_odom magnitudes vs commanded motion without
  // needing a separate diagnostic node.
  this->declare_parameter("debug_motion_log_period_s", 0.0);
  // /synchro_odom publishes twist in this frame (its child_frame_id). The
  // lever-arm correction re-expresses it at the lidar origin so it is
  // numerically usable inside the lidar's local cloud math.
  this->declare_parameter("base_frame", std::string("base_link"));

  // Project IMU's angular velocity vector into the LIDAR frame (using
  // /tf) before extracting wz. Required when the IMU is mounted at a
  // non-trivial tilt relative to base_link/LIDAR (e.g. ZED head IMU
  // through the torso chain on R1). Without this, rotations whose
  // wx/wy in IMU coords are non-zero contribute zero to the deskew
  // correction, producing azimuth-locked distortion in the deskewed
  // cloud at the bearings where the LIDAR's XY-plane projection of the
  // missed rotation is largest. Default OFF for backward compatibility
  // with the OUSTER pipeline; LIVOX yaml flips it on.
  this->declare_parameter("project_imu_to_lidar", false);
  // Optional: explicit LIDAR target frame for the projection. Empty
  // string (default) means "use the first cloud's header.frame_id",
  // which is the right thing on every pipeline we have.
  this->declare_parameter("lidar_frame_id", std::string(""));
  // Optional: hardcoded R_lidar_imu row 2 = [pv0, pv1, pv2] override
  // for project_imu_to_lidar. If non-zero (i.e. any element != 0), the
  // node SKIPS the TF lookup and uses this triple directly. Use this
  // when the URDF is unreliable (joints not where URDF expects, mount
  // off from spec, etc.) — measure the row from a steady pure-yaw
  // rotation: row = (wx, wy, wz) / |omega| averaged over the window.
  // See scripts/calibrate_imu_to_lidar_row.py.
  this->declare_parameter("imu_lidar_row2", std::vector<double>{0.0, 0.0, 0.0});

  enabled_         = this->get_parameter("enabled").as_bool();
  timestamp_field_ = this->get_parameter("timestamp_field").as_string();
  wz_gate_thresh_  = this->get_parameter("wz_gate_thresh_rad_s").as_double();
  linear_deskew_enabled_ = this->get_parameter("linear_deskew_enabled").as_bool();
  linear_gate_thresh_m_s_ = this->get_parameter("linear_gate_thresh_m_s").as_double();
  debug_motion_log_period_s_ =
      this->get_parameter("debug_motion_log_period_s").as_double();
  last_debug_motion_log_ = this->get_clock()->now();
  base_frame_           = this->get_parameter("base_frame").as_string();
  project_imu_to_lidar_ = this->get_parameter("project_imu_to_lidar").as_bool();
  lidar_frame_param_    = this->get_parameter("lidar_frame_id").as_string();

  // Linear deskew needs a TF buffer to resolve the static base_link -> lidar
  // offset (lever arm). Reuse the IMU-projection buffer if it was already
  // created by the project_imu_to_lidar branch; otherwise create one here.
  if (linear_deskew_enabled_ && !tf_buffer_) {
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  if (project_imu_to_lidar_) {
    auto override_row = this->get_parameter("imu_lidar_row2").as_double_array();
    bool override_active = override_row.size() == 3 &&
        (override_row[0] != 0.0 || override_row[1] != 0.0 || override_row[2] != 0.0);
    if (override_active) {
      // Hardcoded override (calibrated from data) -- skip TF lookup.
      pv0_ = override_row[0];
      pv1_ = override_row[1];
      pv2_ = override_row[2];
      projection_resolved_.store(true, std::memory_order_release);
      const double mag = std::sqrt(pv0_ * pv0_ + pv1_ * pv1_ + pv2_ * pv2_);
      RCLCPP_INFO(this->get_logger(),
        "project_imu_to_lidar=ENABLED with HARDCODED row2=[%.4f, %.4f, %.4f] "
        "(|row|=%.4f, expected ~1.0). TF lookup SKIPPED. To re-enable URDF "
        "lookup, clear the imu_lidar_row2 parameter (set all 3 to 0).",
        pv0_, pv1_, pv2_, mag);
      if (std::abs(mag - 1.0) > 0.05) {
        RCLCPP_WARN(this->get_logger(),
          "imu_lidar_row2 magnitude %.4f differs from 1.0 by >5%%. "
          "A row of a rotation matrix should be a unit vector. "
          "Re-derive from a steady pure-yaw rotation.", mag);
      }
    } else {
      // Default: discover via TF from URDF.
      tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      if (!lidar_frame_param_.empty()) {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        lidar_frame_id_ = lidar_frame_param_;
      }
      RCLCPP_INFO(this->get_logger(),
        "project_imu_to_lidar=ENABLED via TF (lidar_frame_id='%s' -- %s). "
        "Set imu_lidar_row2:[pv0,pv1,pv2] to override URDF if joint state "
        "doesn't match physical mount.",
        lidar_frame_param_.c_str(),
        lidar_frame_param_.empty()
            ? "will resolve from first cloud's frame_id"
            : "explicit override");
    }
  }

  auto sensor_qos = rclcpp::SensorDataQoS();

  // Lambda wrappers: callbacks take UniquePtr by value (move-only), so we
  // can't use std::bind. The lambda explicitly moves into the member fn.
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    this->get_parameter("imu_topic").as_string(), sensor_qos,
    [this](sensor_msgs::msg::Imu::UniquePtr msg) { this->onImu(std::move(msg)); });

  const auto odom_topic = this->get_parameter("odom_topic").as_string();
  if (linear_deskew_enabled_ && !odom_topic.empty()) {
    // Match /synchro_odom's RELIABLE QoS (synchro_drive_odometry uses the
    // default rclcpp publisher QoS = KEEP_LAST 10 RELIABLE) and run a deeper
    // queue than SensorDataQoS's depth=5 so a 100 ms cloud callback that
    // delays this executor never silently drops a 50 Hz odom sample.
    rclcpp::QoS odom_qos(50);
    odom_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    odom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, odom_qos,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) { this->onOdom(std::move(msg)); });
  } else if (linear_deskew_enabled_) {
    RCLCPP_WARN(this->get_logger(),
      "linear_deskew_enabled=true but odom_topic is empty; linear deskew disabled until "
      "an odometry topic is configured.");
  }

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("input_topic").as_string(), sensor_qos,
    [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) { this->onCloud(std::move(msg)); });

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    this->get_parameter("output_topic").as_string(), sensor_qos);

  RCLCPP_INFO(this->get_logger(),
    "PointCloudDeskew: enabled=%s  in=%s  out=%s  timestamp_field='%s'  "
    "wz_gate=%.4f rad/s  linear=%s odom='%s' lin_gate=%.4f m/s",
    enabled_ ? "true" : "false",
    this->get_parameter("input_topic").as_string().c_str(),
    this->get_parameter("output_topic").as_string().c_str(),
    timestamp_field_.c_str(),
    wz_gate_thresh_,
    (linear_deskew_enabled_ && odom_sub_) ? "true" : "false",
    odom_topic.c_str(),
    linear_gate_thresh_m_s_);
}

void PointCloudDeskew::onImu(sensor_msgs::msg::Imu::UniquePtr msg)
{
  int64_t t_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL
               + static_cast<int64_t>(msg->header.stamp.nanosec);

  // Default = legacy behavior: use raw IMU wz. This is correct when the
  // IMU is mounted with axes aligned to the LIDAR (Ouster pipeline today,
  // until field-validated otherwise).
  double wz_to_store = msg->angular_velocity.z;

  // When the IMU/LIDAR mount has a non-trivial rotation, raw wz_imu is
  // not the rotation rate around LIDAR-Z. Project once the TF resolves.
  if (project_imu_to_lidar_) {
    if (!projection_resolved_.load(std::memory_order_acquire)) {
      // Snapshot the lidar frame name (set by onCloud or by the param).
      std::string lidar_frame;
      {
        std::lock_guard<std::mutex> lk(frame_mutex_);
        lidar_frame = lidar_frame_id_;
      }
      if (!lidar_frame.empty() && tf_buffer_) {
        try {
          // R_lidar_imu rotates a vector expressed in the IMU frame into
          // the LIDAR frame's coordinates. tf2's lookupTransform(target,
          // source) returns the pose of `source` in `target`, whose
          // rotation IS R_target_source = R_lidar_imu.
          auto tf = tf_buffer_->lookupTransform(
              lidar_frame, msg->header.frame_id, tf2::TimePointZero);
          tf2::Quaternion q;
          tf2::fromMsg(tf.transform.rotation, q);
          tf2::Matrix3x3 m(q);
          // Row 2 of R_lidar_imu — what we dot into omega_imu to get
          // omega_lidar.z. Stored as plain doubles (write happens once
          // before the release-store of projection_resolved_, after which
          // this function never modifies them).
          pv0_ = m[2][0];
          pv1_ = m[2][1];
          pv2_ = m[2][2];
          projection_resolved_.store(true, std::memory_order_release);
          RCLCPP_INFO(this->get_logger(),
            "IMU->LIDAR projection resolved: lidar='%s' imu='%s' "
            "row2=[%.4f, %.4f, %.4f] (raw wz_imu would have under-"
            "corrected by ~%.2f%% on pure base-Z rotation, plus an "
            "azimuth-locked term proportional to sqrt(pv0^2+pv1^2)=%.4f).",
            lidar_frame.c_str(), msg->header.frame_id.c_str(),
            pv0_, pv1_, pv2_,
            (1.0 - std::abs(pv2_)) * 100.0,
            std::sqrt(pv0_ * pv0_ + pv1_ * pv1_));
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "project_imu_to_lidar=true but TF lookup '%s' <- '%s' failed: "
            "%s. Falling back to raw wz_imu until TF becomes available.",
            lidar_frame.c_str(), msg->header.frame_id.c_str(), ex.what());
        }
      }
    }
    if (projection_resolved_.load(std::memory_order_acquire)) {
      wz_to_store = pv0_ * msg->angular_velocity.x
                  + pv1_ * msg->angular_velocity.y
                  + pv2_ * msg->angular_velocity.z;
    }
  }

  std::lock_guard<std::mutex> lk(buf_mutex_);
  imu_buf_.push_back({t_ns, wz_to_store});
  if (imu_buf_.size() > IMU_BUF_MAX) {
    imu_buf_.pop_front();
  }
}

void PointCloudDeskew::onOdom(nav_msgs::msg::Odometry::UniquePtr msg)
{
  int64_t t_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL
               + static_cast<int64_t>(msg->header.stamp.nanosec);

  std::lock_guard<std::mutex> lk(odom_buf_mutex_);
  odom_buf_.push_back({t_ns, msg->twist.twist.linear.x, msg->twist.twist.linear.y});
  if (odom_buf_.size() > ODOM_BUF_MAX) {
    odom_buf_.pop_front();
  }
}

bool PointCloudDeskew::resolveLeverArm(const std::string & lidar_frame)
{
  if (lever_arm_resolved_.load(std::memory_order_acquire)) {
    return true;
  }
  if (!tf_buffer_ || lidar_frame.empty() || base_frame_.empty()) {
    return false;
  }
  try {
    // tf2 lookupTransform(target, source) -> pose of `source` in `target`
    // frame. The translation field is the position of `source` origin
    // expressed in `target` coords. We want r_lever = position of lidar
    // origin in lidar frame coords = lookup(target=lidar, source=base).
    // That returns the pose of base in lidar coords, whose translation is
    // -r_lever, so we negate.
    auto tf = tf_buffer_->lookupTransform(
        lidar_frame, base_frame_, tf2::TimePointZero);
    r_lever_x_ = -tf.transform.translation.x;
    r_lever_y_ = -tf.transform.translation.y;
    lever_arm_resolved_.store(true, std::memory_order_release);
    RCLCPP_INFO(this->get_logger(),
      "Lever-arm resolved: lidar='%s' base='%s' r_lever=[%.4f, %.4f, ?] m. "
      "At wz=1 rad/s this contributes %.3f m/s extra |v_lidar|.",
      lidar_frame.c_str(), base_frame_.c_str(),
      r_lever_x_, r_lever_y_,
      std::sqrt(r_lever_x_ * r_lever_x_ + r_lever_y_ * r_lever_y_));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Lever-arm TF lookup '%s' <- '%s' failed: %s. Linear deskew will "
      "ignore the offset between base and lidar until TF becomes available.",
      lidar_frame.c_str(), base_frame_.c_str(), ex.what());
    return false;
  }
}

// Lock-free interpolator over a caller-owned IMU snapshot.
// Tracks front and back clamps separately because they mean very
// different things (see header comment): front = real IMU stall, back =
// expected rear-edge effect during rotation. Empty buffer counts as
// front (real problem -- nothing to interpolate from).
double PointCloudDeskew::interpolateWzSnapshot(
    const std::deque<ImuSample> & snap,
    int64_t t_ns,
    int & out_front_clamps,
    int & out_back_clamps)
{
  if (snap.empty()) {
    out_front_clamps++;
    return 0.0;
  }
  if (t_ns <= snap.front().t_ns) {
    out_front_clamps++;
    return snap.front().wz;
  }
  if (t_ns >= snap.back().t_ns) {
    out_back_clamps++;
    return snap.back().wz;
  }

  // Binary search for surrounding pair
  size_t lo = 0, hi = snap.size() - 1;
  while (hi - lo > 1) {
    size_t mid = (lo + hi) / 2;
    if (snap[mid].t_ns <= t_ns) lo = mid;
    else                        hi = mid;
  }

  const double dt = static_cast<double>(snap[hi].t_ns - snap[lo].t_ns);
  if (dt < 1.0) return snap[lo].wz;
  const double alpha = static_cast<double>(t_ns - snap[lo].t_ns) / dt;
  return snap[lo].wz + alpha * (snap[hi].wz - snap[lo].wz);
}

bool PointCloudDeskew::interpolateOdomSnapshot(
    const std::deque<OdomSample> & snap,
    int64_t t_ns,
    double & out_vx,
    double & out_vy,
    int & out_front_clamps,
    int & out_back_clamps)
{
  if (snap.empty()) {
    out_front_clamps++;
    out_vx = 0.0;
    out_vy = 0.0;
    return false;
  }
  if (t_ns <= snap.front().t_ns) {
    out_front_clamps++;
    out_vx = snap.front().vx;
    out_vy = snap.front().vy;
    return true;
  }
  if (t_ns >= snap.back().t_ns) {
    out_back_clamps++;
    out_vx = snap.back().vx;
    out_vy = snap.back().vy;
    return true;
  }

  size_t lo = 0, hi = snap.size() - 1;
  while (hi - lo > 1) {
    size_t mid = (lo + hi) / 2;
    if (snap[mid].t_ns <= t_ns) lo = mid;
    else                        hi = mid;
  }

  const double dt = static_cast<double>(snap[hi].t_ns - snap[lo].t_ns);
  if (dt < 1.0) {
    out_vx = snap[lo].vx;
    out_vy = snap[lo].vy;
    return true;
  }
  const double alpha = static_cast<double>(t_ns - snap[lo].t_ns) / dt;
  out_vx = snap[lo].vx + alpha * (snap[hi].vx - snap[lo].vx);
  out_vy = snap[lo].vy + alpha * (snap[hi].vy - snap[lo].vy);
  return true;
}

void PointCloudDeskew::onCloud(sensor_msgs::msg::PointCloud2::UniquePtr msg)
{
  // Capture the LIDAR frame name from the first cloud so onImu can
  // resolve the IMU->LIDAR rotation. Cheap to do unconditionally.
  if (project_imu_to_lidar_ && !projection_resolved_.load(std::memory_order_acquire)) {
    std::lock_guard<std::mutex> lk(frame_mutex_);
    if (lidar_frame_id_.empty()) {
      lidar_frame_id_ = msg->header.frame_id;
    }
  }

  // Resolve the static lidar lever-arm against base_link the first time we
  // see a cloud (TF tree is rarely populated at constructor time). Skipped
  // if linear deskew is disabled.
  if (linear_deskew_enabled_ && !lever_arm_resolved_.load(std::memory_order_acquire)) {
    resolveLeverArm(msg->header.frame_id);
  }

  // Passthrough fast paths: hand the same allocation downstream by moving
  // the UniquePtr. With intra-process comms this is a pointer transfer,
  // not a serialize round-trip.
  if (!enabled_) {
    cloud_pub_->publish(std::move(msg));
    return;
  }

  // Discover the timestamp field and its datatype.
  //   UINT32  : Ouster 't'        — ns since scan start (RELATIVE; small,
  //                                  0 .. ~1e8 ns over a 100 ms scan).
  //   FLOAT64 : LIVOX 'timestamp' — ABSOLUTE epoch ns (verified against
  //             this workspace's livox_ros_driver2/src/comm/pub_handler.cpp
  //             ::GetEthPacketTimestamp, which is custom-modified to
  //             return std::chrono::system_clock::now() ns; lddc.cpp:328
  //             then casts the per-point offset_time into a FLOAT64 of
  //             this absolute value). The two conventions differ in
  //             time_base_ns below — do NOT add header_t_ns to a LIVOX
  //             t_field (would land ~3.6e18 ns past the IMU buffer's
  //             newest sample, silently using only the latest wz on every
  //             point, AND would int32-overflow when the resulting
  //             ~3.6e9 s value is written back into header.stamp.sec).
  uint8_t ts_datatype = 0;
  for (const auto & f : msg->fields) {
    if (f.name == timestamp_field_) { ts_datatype = f.datatype; break; }
  }
  if (ts_datatype != sensor_msgs::msg::PointField::UINT32 &&
      ts_datatype != sensor_msgs::msg::PointField::FLOAT64) {
    RCLCPP_WARN_ONCE(this->get_logger(),
      "timestamp_field '%s' missing or unsupported datatype (%u) \u2014 "
      "passthrough. Supported: UINT32 (Ouster 't'), FLOAT64 (LIVOX 'timestamp').",
      timestamp_field_.c_str(), static_cast<unsigned>(ts_datatype));
    cloud_pub_->publish(std::move(msg));
    return;
  }
  // FLOAT64 t_field is absolute epoch ns (LIVOX), UINT32 is scan-relative
  // ns (Ouster). Add `time_base_ns` to a t_field to get an absolute time;
  // for LIVOX this collapses to the t_field itself.
  const bool ts_is_absolute =
      (ts_datatype == sensor_msgs::msg::PointField::FLOAT64);

  // Motion gate: peek at the most recent ~100 ms of IMU/odom samples (covers
  // the scan acquisition window). Only passthrough when BOTH angular and
  // linear motion are below their thresholds; a straight-driving robot still
  // needs linear deskew even if |wz| is near zero.
  if (wz_gate_thresh_ > 0.0 || linear_gate_thresh_m_s_ > 0.0) {
    double recent_max_abs_wz = 0.0;
    double recent_max_lin_speed = 0.0;
    {
      std::lock_guard<std::mutex> lk(buf_mutex_);
      // Walk backwards from the newest sample for ~100 ms of history.
      // (~10 samples at 100 Hz IMU, ~20 at 200 Hz). Bounded so worst case
      // is a tiny constant-time loop, not the whole 5s buffer.
      if (!imu_buf_.empty()) {
        const int64_t newest_t = imu_buf_.back().t_ns;
        const int64_t window_ns = 100000000LL;  // 100 ms
        for (auto it = imu_buf_.rbegin(); it != imu_buf_.rend(); ++it) {
          if (newest_t - it->t_ns > window_ns) break;
          double a = std::abs(it->wz);
          if (a > recent_max_abs_wz) recent_max_abs_wz = a;
        }
      }
    }
    if (linear_deskew_enabled_ && odom_sub_) {
      std::lock_guard<std::mutex> lk(odom_buf_mutex_);
      if (!odom_buf_.empty()) {
        // Always include the latest sample so a stop->go transition cannot
        // bypass deskew while the historical window still contains zeros.
        recent_max_lin_speed = std::hypot(
            odom_buf_.back().vx, odom_buf_.back().vy);
        const int64_t newest_t = odom_buf_.back().t_ns;
        const int64_t window_ns = 100000000LL;  // 100 ms
        for (auto it = odom_buf_.rbegin(); it != odom_buf_.rend(); ++it) {
          if (newest_t - it->t_ns > window_ns) break;
          double speed = std::hypot(it->vx, it->vy);
          if (speed > recent_max_lin_speed) recent_max_lin_speed = speed;
        }
      } else {
        // No odom yet: avoid incorrectly bypassing a translating scan.
        recent_max_lin_speed = linear_gate_thresh_m_s_;
      }
    }
    const bool angular_idle =
        wz_gate_thresh_ <= 0.0 || recent_max_abs_wz < wz_gate_thresh_;
    const bool linear_idle =
        !linear_deskew_enabled_ || !odom_sub_ ||
        linear_gate_thresh_m_s_ <= 0.0 || recent_max_lin_speed < linear_gate_thresh_m_s_;
    if (angular_idle && linear_idle) {
      cloud_pub_->publish(std::move(msg));
      return;
    }
  }

  // header.stamp:
  //   Ouster: scan-start absolute time, t_field is offset-from-scan-start.
  //   LIVOX : pkg.base_time, which on this platform's driver equals the
  //           absolute system_clock ns of the first point in the packet
  //           (lddc.cpp:310-316 + pub_handler.cpp:281-283 custom mod).
  // For Ouster, abs_time(point) = header_t_ns + t_field.
  // For LIVOX,  abs_time(point) = t_field (header_t_ns is just the first
  // point's absolute time and is informational only).
  const int64_t header_t_ns =
      static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL
    + static_cast<int64_t>(msg->header.stamp.nanosec);
  const int64_t time_base_ns = ts_is_absolute ? 0 : header_t_ns;

  // Pass 1: find max(t) and min(t) in the timestamp field's own units.
  // For Ouster (relative) max_t_ns ~= scan duration in ns (~1e8). For
  // LIVOX (absolute) max_t_ns is the latest point's absolute epoch ns
  // (~1.78e18); min_t_ns ~= header_t_ns. The point-to-point arithmetic
  // below uses (max_t_ns - t_ns_off), so the absolute offset cancels and
  // works for both conventions.
  int64_t max_t_ns = std::numeric_limits<int64_t>::min();
  int64_t min_t_ns = std::numeric_limits<int64_t>::max();
  if (ts_datatype == sensor_msgs::msg::PointField::UINT32) {
    sensor_msgs::PointCloud2ConstIterator<uint32_t> it_t(*msg, timestamp_field_);
    for (; it_t != it_t.end(); ++it_t) {
      int64_t v = static_cast<int64_t>(*it_t);
      if (v > max_t_ns) max_t_ns = v;
      if (v < min_t_ns) min_t_ns = v;
    }
  } else {  // FLOAT64
    sensor_msgs::PointCloud2ConstIterator<double> it_t(*msg, timestamp_field_);
    for (; it_t != it_t.end(); ++it_t) {
      int64_t v = static_cast<int64_t>(*it_t);
      if (v > max_t_ns) max_t_ns = v;
      if (v < min_t_ns) min_t_ns = v;
    }
  }

  if (max_t_ns == std::numeric_limits<int64_t>::min() ||
      max_t_ns <= min_t_ns) {
    // Empty cloud, all-equal timestamps, or otherwise unusable — passthrough
    cloud_pub_->publish(std::move(msg));
    return;
  }

  // One-shot startup sanity log: prints the convention this branch
  // is operating under, so a future driver change with different
  // semantics fails loudly instead of silently. Expected:
  //   OUSTER : header≈min, max-min ~= 100 ms (1e8 ns), max-header ~= 1e8
  //   LIVOX  : header≈min (both absolute), max-header ~= 100 ms (1e8 ns).
  // If you ever see max-header on the order of 1e18 here, the t_field
  // semantics have changed and the time_base_ns branch is wrong.
  if (!first_scan_logged_) {
    first_scan_logged_ = true;
    const int64_t scan_span_ns = max_t_ns - min_t_ns;
    const int64_t end_abs_ns   = time_base_ns + max_t_ns;
    RCLCPP_INFO(this->get_logger(),
      "first_scan datatype=%s ts_is_absolute=%s | header.stamp_ns=%ld "
      "min_t=%ld max_t=%ld scan_span=%.3f ms | end_abs_ns=%ld "
      "(end-header)=%.3f ms",
      (ts_datatype == sensor_msgs::msg::PointField::UINT32) ? "UINT32" : "FLOAT64",
      ts_is_absolute ? "true" : "false",
      static_cast<long>(header_t_ns),
      static_cast<long>(min_t_ns),
      static_cast<long>(max_t_ns),
      static_cast<double>(scan_span_ns) * 1e-6,
      static_cast<long>(end_abs_ns),
      static_cast<double>(end_abs_ns - header_t_ns) * 1e-6);
    if (scan_span_ns > 1000000000LL) {
      RCLCPP_ERROR(this->get_logger(),
        "first_scan span = %.3f s exceeds 1 s; t_field semantics likely "
        "differ from the assumed convention. Deskew math may be invalid.",
        static_cast<double>(scan_span_ns) * 1e-9);
    }
  }

  // Deskew in place: rotate (x, y) of every point inside the inbound
  // UniquePtr's data buffer, advance header.stamp to scan-end, and then
  // hand ownership of the same allocation downstream via std::move.
  // Eliminates the persistent out_buf_ memcpy (was ~1.5 MB at 10 Hz =
  // 15 MB/s wasted on Ouster) and lets intra-process subscribers skip
  // the serialization round-trip.
  //
  // For UINT32 (Ouster, relative), end_t_ns = header_t_ns + max_t_ns.
  // For FLOAT64 (LIVOX, absolute), end_t_ns = max_t_ns directly --
  // adding header_t_ns again would double the epoch and overflow int32.
  const int64_t end_t_ns = time_base_ns + max_t_ns;
  msg->header.stamp.sec     = static_cast<int32_t>(end_t_ns / 1000000000LL);
  msg->header.stamp.nanosec = static_cast<uint32_t>(end_t_ns % 1000000000LL);

  sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");

  // Snapshot the IMU/odom buffers once under their locks, then iterate the
  // snapshots unlocked. The previous code held buf_mutex_ across the entire
  // ~24k point pass on LIVOX (and the ~1024 unique-t cache misses on Ouster),
  // blocking the IMU producer thread (onImu) for the full deskew duration.
  // std::deque copy is O(n) over small bounded buffers, so it's cheap
  // relative to a per-scan point pass.
  std::deque<ImuSample> snap;
  {
    std::lock_guard<std::mutex> lk(buf_mutex_);
    snap = imu_buf_;
  }
  std::deque<OdomSample> odom_snap;
  if (linear_deskew_enabled_ && odom_sub_) {
    std::lock_guard<std::mutex> lk(odom_buf_mutex_);
    odom_snap = odom_buf_;
  }

  int odom_front_clamps = 0;
  int odom_back_clamps = 0;
  int odom_end_front_clamps = 0;
  int odom_end_back_clamps = 0;
  double end_vx_base = 0.0;
  double end_vy_base = 0.0;
  const bool have_end_odom =
      linear_deskew_enabled_ && odom_sub_ &&
      interpolateOdomSnapshot(
          odom_snap, end_t_ns, end_vx_base, end_vy_base,
          odom_end_front_clamps, odom_end_back_clamps);

  // wz at scan-end: needed both for the lever-arm correction at the T_e
  // side of the trapezoidal velocity integration and for symmetry with
  // wz_at_t. Looking it up here is a single dummy lookup, not per-point.
  int end_wz_front = 0;
  int end_wz_back = 0;
  const double end_wz = interpolateWzSnapshot(
      snap, end_t_ns, end_wz_front, end_wz_back);

  // Re-express scan-end base-frame twist at the lidar origin, modulo the
  // lever-arm (skipped if not resolved). r_lever is constant in the lidar
  // frame because livox_frame and base_link share orientation per URDF.
  const bool use_lever =
      linear_deskew_enabled_ && lever_arm_resolved_.load(std::memory_order_acquire);
  const double end_vx_lidar =
      use_lever ? end_vx_base - end_wz * r_lever_y_ : end_vx_base;
  const double end_vy_lidar =
      use_lever ? end_vy_base + end_wz * r_lever_x_ : end_vy_base;

  // Batch trig + IMU interpolation: cache (wz, c, s) per unique per-point
  // timestamp. Ouster columns share the same 't' across all 32/64/128 rows
  // so on OS1-128 this reduces ~131k IMU lookups + ~131k cos/sin calls to
  // ~1024 each. LIVOX MID-360 has unique offset_time per point so the cache
  // always misses; ~24k trig calls per scan is still <1 ms on AGX Orin.
  int64_t prev_t_ns = INT64_MIN;
  double  cached_c  = 1.0;
  double  cached_s  = 0.0;
  double  cached_dx_rot = 0.0;
  double  cached_dy_rot = 0.0;

  // Debug accumulators: track the largest per-scan correction so a
  // field operator can sanity-check the magnitude is reasonable.
  double max_abs_dx = 0.0;
  double max_abs_dy = 0.0;
  double max_abs_theta = 0.0;

  // Per-scan IMU-lookup clamp counters. Front and back clamps are tracked
  // separately because they have very different meanings:
  //   FRONT clamps (point t_ns < oldest IMU sample): a real problem --
  //     the IMU producer thread has stalled OR the buffer is too small
  //     for the cloud's history depth. WARN at low threshold (>0.5%).
  //   BACK clamps (point t_ns > newest IMU sample): expected during
  //     rotation. Livox publishes the cloud at scan-end; with the ZED
  //     head IMU at 100 Hz the most recent IMU sample is up to 10 ms
  //     older than the last point, so 5-10% of points-per-scan land past
  //     the buffer's back. We only WARN when this gets unreasonable
  //     (>50%, suggesting IMU thread starvation, not just rear-edge
  //     normality), and silently use snap.back().wz as the best estimate
  //     of wz at scan-end otherwise.
  int front_clamps  = 0;
  int back_clamps   = 0;
  int unique_lookups = 0;

  // For both branches, the IMU lookup uses time_base_ns + t_ns_off:
  //   UINT32 / Ouster   : time_base_ns = header_t_ns (scan start)
  //   FLOAT64 / LIVOX   : time_base_ns = 0 (t_ns_off is already absolute)
  // Helper to update per-scan accumulators on each unique-timestamp cache
  // miss. Captures the rotated displacement actually written into the
  // points, plus the rotation magnitude, so the periodic debug log
  // reflects the real correction (not the raw vx/vy).
  auto track_max = [&](double theta) {
    if (std::abs(theta) > max_abs_theta) max_abs_theta = std::abs(theta);
    if (std::abs(cached_dx_rot) > max_abs_dx) max_abs_dx = std::abs(cached_dx_rot);
    if (std::abs(cached_dy_rot) > max_abs_dy) max_abs_dy = std::abs(cached_dy_rot);
  };

  if (ts_datatype == sensor_msgs::msg::PointField::UINT32) {
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_t(*msg, timestamp_field_);
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_t) {
      int64_t t_ns_off = static_cast<int64_t>(*iter_t);
      if (t_ns_off != prev_t_ns) {
        double  wz       = interpolateWzSnapshot(
            snap, time_base_ns + t_ns_off, front_clamps, back_clamps);
        int64_t delta_ns = max_t_ns - t_ns_off;
        const double dt_s = static_cast<double>(delta_ns) * 1e-9;
        double  theta    = wz * dt_s;
        cached_c   = std::cos(theta);
        cached_s   = std::sin(theta);
        cached_dx_rot = 0.0;
        cached_dy_rot = 0.0;
        if (have_end_odom) {
          double vx_base = 0.0;
          double vy_base = 0.0;
          if (interpolateOdomSnapshot(
                odom_snap, time_base_ns + t_ns_off, vx_base, vy_base,
                odom_front_clamps, odom_back_clamps)) {
            const double vx_lidar =
                use_lever ? vx_base - wz * r_lever_y_ : vx_base;
            const double vy_lidar =
                use_lever ? vy_base + wz * r_lever_x_ : vy_base;
            const double dx = 0.5 * (vx_lidar + end_vx_lidar) * dt_s;
            const double dy = 0.5 * (vy_lidar + end_vy_lidar) * dt_s;
            cached_dx_rot = cached_c * dx + cached_s * dy;
            cached_dy_rot = -cached_s * dx + cached_c * dy;
          }
        }
        track_max(theta);
        prev_t_ns  = t_ns_off;
        unique_lookups++;
      }
      float x = *iter_x;
      float y = *iter_y;
      *iter_x = static_cast<float>(x * cached_c + y * cached_s - cached_dx_rot);
      *iter_y = static_cast<float>(-x * cached_s + y * cached_c - cached_dy_rot);
    }
  } else {  // FLOAT64 (LIVOX) — t_ns_off is absolute epoch ns
    sensor_msgs::PointCloud2Iterator<double> iter_t(*msg, timestamp_field_);
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_t) {
      int64_t t_ns_off = static_cast<int64_t>(*iter_t);
      if (t_ns_off != prev_t_ns) {
        double  wz       = interpolateWzSnapshot(
            snap, time_base_ns + t_ns_off, front_clamps, back_clamps);
        int64_t delta_ns = max_t_ns - t_ns_off;
        const double dt_s = static_cast<double>(delta_ns) * 1e-9;
        double  theta    = wz * dt_s;
        cached_c   = std::cos(theta);
        cached_s   = std::sin(theta);
        cached_dx_rot = 0.0;
        cached_dy_rot = 0.0;
        if (have_end_odom) {
          double vx_base = 0.0;
          double vy_base = 0.0;
          if (interpolateOdomSnapshot(
                odom_snap, time_base_ns + t_ns_off, vx_base, vy_base,
                odom_front_clamps, odom_back_clamps)) {
            const double vx_lidar =
                use_lever ? vx_base - wz * r_lever_y_ : vx_base;
            const double vy_lidar =
                use_lever ? vy_base + wz * r_lever_x_ : vy_base;
            const double dx = 0.5 * (vx_lidar + end_vx_lidar) * dt_s;
            const double dy = 0.5 * (vy_lidar + end_vy_lidar) * dt_s;
            cached_dx_rot = cached_c * dx + cached_s * dy;
            cached_dy_rot = -cached_s * dx + cached_c * dy;
          }
        }
        track_max(theta);
        prev_t_ns  = t_ns_off;
        unique_lookups++;
      }
      float x = *iter_x;
      float y = *iter_y;
      *iter_x = static_cast<float>(x * cached_c + y * cached_s - cached_dx_rot);
      *iter_y = static_cast<float>(-x * cached_s + y * cached_c - cached_dy_rot);
    }
  }

  // Front-clamp detector: anything above 0.5% of unique lookups falling
  // BEFORE the oldest IMU sample is a real problem (buffer too short,
  // IMU producer stalled, or scan timestamps gone backwards). Surfacing
  // this at the deskew boundary is earlier than the EKF noticing missing
  // /imu/data_bias_corrected. Throttled to once per second.
  //
  // Suppress when snap.size() is tiny -- on the first cloud after node
  // startup the IMU subscription has rarely received more than a couple
  // samples, and warning loudly there is just noise. ~25 samples covers
  // 100 ms at 200 Hz IMU plus headroom.
  constexpr size_t kImuStartupSnapMin = 25;
  if (unique_lookups > 0 && snap.size() >= kImuStartupSnapMin &&
      front_clamps * 200 > unique_lookups) {  // > 0.5%
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "IMU interpolation FRONT-clamped on %d / %d unique lookups (%.1f%%) "
      "-- scan timestamps fall before oldest IMU sample. IMU producer "
      "stalled, buffer too short, or scan time went backwards. "
      "(snap_size=%zu, buf_max=%zu)",
      front_clamps, unique_lookups,
      100.0 * static_cast<double>(front_clamps)
            / static_cast<double>(unique_lookups),
      snap.size(), IMU_BUF_MAX);
  }
  // Back-clamp detector: rear-edge clamping during rotation is normal
  // (cloud publishes at scan-end; ZED IMU at 100 Hz means the last
  // ~10 ms of points are newer than snap.back()). 5-15% is expected.
  // Only WARN when this becomes unreasonable, which would indicate
  // the IMU producer thread has fallen well behind real time -- the
  // cloud arrival jitter masks it from the EKF.
  if (unique_lookups > 0 &&
      back_clamps * 2 > unique_lookups) {  // > 50%
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "IMU interpolation BACK-clamped on %d / %d unique lookups (%.1f%%) "
      "-- IMU producer is far behind cloud time. Expected ~10%% during "
      "rotation, but %.1f%% indicates IMU thread starvation. "
      "(snap_size=%zu, buf_max=%zu)",
      back_clamps, unique_lookups,
      100.0 * static_cast<double>(back_clamps)
            / static_cast<double>(unique_lookups),
      100.0 * static_cast<double>(back_clamps)
            / static_cast<double>(unique_lookups),
      snap.size(), IMU_BUF_MAX);
  }
  // Suppress the "no usable odom sample" warning on the first few clouds
  // after node startup, when the odom subscription has not yet received
  // anything. Only WARN once odom has had time to populate the buffer.
  constexpr size_t kOdomStartupSnapMin = 5;
  if (linear_deskew_enabled_ && odom_sub_ && !have_end_odom &&
      odom_snap.size() >= kOdomStartupSnapMin) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Linear deskew enabled but no usable odom sample was available at scan end; "
      "falling back to yaw-only deskew for this cloud. (odom_snap_size=%zu)",
      odom_snap.size());
  }
  if (unique_lookups > 0 &&
      odom_front_clamps * 200 > unique_lookups) {  // > 0.5%
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Odom interpolation FRONT-clamped on %d / %d unique lookups (%.1f%%) "
      "-- scan timestamps fall before oldest odom sample. Odom producer "
      "stalled, buffer too short, or scan time went backwards. "
      "(snap_size=%zu, buf_max=%zu)",
      odom_front_clamps, unique_lookups,
      100.0 * static_cast<double>(odom_front_clamps)
            / static_cast<double>(unique_lookups),
      odom_snap.size(), ODOM_BUF_MAX);
  }
  if (unique_lookups > 0 &&
      odom_back_clamps * 2 > unique_lookups) {  // > 50%
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Odom interpolation BACK-clamped on %d / %d unique lookups (%.1f%%) "
      "-- odom producer is far behind cloud time. Expected some rear-edge "
      "clamping during motion, but %.1f%% indicates odom thread starvation. "
      "(snap_size=%zu, buf_max=%zu, end_front=%d, end_back=%d)",
      odom_back_clamps, unique_lookups,
      100.0 * static_cast<double>(odom_back_clamps)
            / static_cast<double>(unique_lookups),
      100.0 * static_cast<double>(odom_back_clamps)
            / static_cast<double>(unique_lookups),
      odom_snap.size(), ODOM_BUF_MAX,
      odom_end_front_clamps, odom_end_back_clamps);
  }

  // Periodic motion summary for field operators. Off by default
  // (debug_motion_log_period_s = 0). At a typical 1.0 s cadence this is
  // chatty but actionable for a single test session, then can be turned
  // back off without rebuilding via the param interface.
  if (debug_motion_log_period_s_ > 0.0) {
    const auto now = this->get_clock()->now();
    if ((now - last_debug_motion_log_).seconds() >= debug_motion_log_period_s_) {
      const double scan_dt_ms = static_cast<double>(max_t_ns - min_t_ns) * 1e-6;
      RCLCPP_INFO(this->get_logger(),
        "deskew_motion: scan_dt=%.2f ms | max|theta|=%.4f rad | "
        "max|dx|=%.4f m max|dy|=%.4f m | end_v_base=(%.3f, %.3f) m/s end_wz=%.4f rad/s | "
        "linear=%s lever=%s have_end_odom=%s odom_snap=%zu imu_snap=%zu",
        scan_dt_ms, max_abs_theta, max_abs_dx, max_abs_dy,
        end_vx_base, end_vy_base, end_wz,
        (linear_deskew_enabled_ && odom_sub_) ? "true" : "false",
        use_lever ? "true" : "false",
        have_end_odom ? "true" : "false",
        odom_snap.size(), snap.size());
      last_debug_motion_log_ = now;
    }
  }

  cloud_pub_->publish(std::move(msg));
}

}  // namespace r1_nav_cpp

// Register with class_loader so this node can be loaded into a
// ComposableNodeContainer (with use_intra_process_comms: true).
RCLCPP_COMPONENTS_REGISTER_NODE(r1_nav_cpp::PointCloudDeskew)

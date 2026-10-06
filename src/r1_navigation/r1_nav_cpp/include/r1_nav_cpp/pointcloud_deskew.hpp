#ifndef R1_NAV_CPP__POINTCLOUD_DESKEW_HPP_
#define R1_NAV_CPP__POINTCLOUD_DESKEW_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace r1_nav_cpp
{

/**
 * 2D LiDAR Deskewing Node (z-axis rotation only, ground robot)
 *
 * Corrects within-scan motion distortion caused by robot rotation during the
 * LiDAR frame acquisition window (~100 ms for Ouster 1024 @ 10 Hz).
 *
 * --- Ouster timestamp convention (verified from live topic) ---
 * header.stamp = absolute time of the FIRST acquired point (scan start, t = 0).
 * The 't' field of each point = nanoseconds elapsed since scan start.
 * Columns are ordered by azimuth angle, NOT acquisition time; the wrap point
 * (t drops from ~100 ms back to 0) can fall at any column index each frame.
 *
 * --- Two bugs found and fixed ---
 *
 * Bug 1 — Wrong IMU lookup time:
 *   Original code assumed header.stamp = scan END (last point), computing:
 *     t_point_ns = header_t_ns - (max_t - t_field)
 *   This shifted every IMU lookup ~100 ms into the past (previous frame).
 *   During rotation this caused deskew to apply the wrong angular velocity,
 *   distorting the scan shape.
 *   Fix: t_point_ns = header_t_ns + t_field  (scan-start convention)
 *
 * Bug 2 — Output header.stamp not updated after deskew:
 *   After deskew all points are referenced to the scan-END frame (max_t).
 *   But the output cloud kept header.stamp = scan start.
 *   SLAM Toolbox looked up the robot pose at scan start, while the cloud
 *   represented the scene at scan end — a systematic yaw error of
 *     Δyaw = wz × scan_duration  (≈ wz × 100 ms per frame).
 *   Symptom: fixed-magnitude yaw snap during rotation (10–20° at ~2 rad/s),
 *   vanishes when stopped (wz = 0 → error = 0), never accumulates.
 *   Fix: output header.stamp = header.stamp + max_t  (scan end time)
 *
 * --- Processing summary ---
 * Reference frame = scan-end time (max_t).
 * Each point rotated back by wz * (max_t - t_field) to align to reference.
 * Output header.stamp advanced to scan-end so downstream pose lookups are correct.
 *
 * Assumptions
 * -----------
 * The per-point timestamp field must be uint32 and represent nanoseconds since
 * scan start. This matches the Ouster 'original' / 'native' point type ('t' field).
 * Other LiDARs use different field names or types; set timestamp_field accordingly.
 * If the field is absent the node falls back to passthrough and logs a warning.
 *
 * Parameters
 * ----------
 * enabled          : bool   – false = passthrough mode (no modification)
 * input_topic      : string – raw point cloud    (default: /ouster/points)
 * output_topic     : string – deskewed cloud     (default: /ouster/points_deskewed)
 * imu_topic        : string – bias-corrected IMU (default: /imu/data_bias_corrected)
 * timestamp_field  : string – per-point ns-from-scan-start field (default: "t", Ouster)
 */
class PointCloudDeskew : public rclcpp::Node
{
public:
  explicit PointCloudDeskew(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // UniquePtr callbacks: enable intra-process zero-copy when this node is
  // loaded into the same ComposableNodeContainer as the IMU producer
  // (online_imu_bias_estimator) and/or the LiDAR driver. The cloud is the
  // largest message in the system (~1.5 MB for Ouster 65k pts), so the
  // serialize→deserialize round-trip we avoid is the headline win.
  void onImu(sensor_msgs::msg::Imu::UniquePtr msg);
  void onOdom(nav_msgs::msg::Odometry::UniquePtr msg);
  void onCloud(sensor_msgs::msg::PointCloud2::UniquePtr msg);

  struct ImuSample {
    int64_t t_ns;
    double  wz;
  };

  struct OdomSample {
    int64_t t_ns;
    double  vx;
    double  vy;
  };

  // Interpolate wz from a (caller-owned, lock-free) snapshot of the IMU
  // buffer at absolute time t_ns. Returns the boundary value if t_ns is
  // outside the buffer span. The caller passes two counters so the two
  // clamp regimes can be distinguished:
  //   out_front_clamps : t_ns < snap.front().t_ns
  //                      (real problem -- buffer too short, IMU stalled,
  //                      or scan timestamp gone backwards). Always WARN.
  //   out_back_clamps  : t_ns > snap.back().t_ns
  //                      (normal during rotation: the cloud publishes at
  //                      scan-end, and at 100 Hz IMU the last 5-10 ms of
  //                      points are newer than the most recent IMU sample.
  //                      ~5-10% rear-edge clamping per scan is expected.)
  // Snapshotting in the caller eliminates the per-point lock contention
  // with the IMU producer thread that the previous in-place version had.
  static double interpolateWzSnapshot(
      const std::deque<ImuSample> & snap,
      int64_t t_ns,
      int & out_front_clamps,
      int & out_back_clamps);

  static bool interpolateOdomSnapshot(
      const std::deque<OdomSample> & snap,
      int64_t t_ns,
      double & out_vx,
      double & out_vy,
      int & out_front_clamps,
      int & out_back_clamps);

  // Resolve the static lidar-origin offset (r_lever) in the lidar frame so
  // /synchro_odom's base_link twist can be re-expressed at the lidar origin
  // for accurate within-scan translation. Lazy because TF tree is not yet
  // populated at constructor time. Single-writer (cloud thread) /
  // single-reader pattern guarded by lever_arm_resolved_.
  bool resolveLeverArm(const std::string & lidar_frame);

  bool        enabled_;
  std::string timestamp_field_;  // per-point ns-from-scan-start field name

  // Below this |wz| (rad/s) deskew is a no-op-grade correction (≤0.1° of
  // yaw distortion across a 100 ms Ouster scan, well under SLAM matching
  // tolerance). When the recent IMU window stays below the threshold we
  // passthrough the inbound cloud and skip BOTH the max_t scan and the
  // rotation loop entirely. Set to 0.0 to disable gating.
  double      wz_gate_thresh_;
  bool        linear_deskew_enabled_;
  double      linear_gate_thresh_m_s_;
  // Optional debug instrumentation: prints a one-line summary of the
  // per-scan motion at a controlled cadence so a field operator can
  // verify /synchro_odom magnitudes match commanded motion.
  double      debug_motion_log_period_s_;
  rclcpp::Time last_debug_motion_log_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr        imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    cloud_pub_;

  // Defensive headroom only: these caps protect against FRONT-clamping
  // (oldest samples being dropped before the deskew callback can interpolate
  // against them) and against startup transients where the IMU/odom streams
  // come up before the first cloud. They do NOT fix BACK-clamping
  // (deskew falling behind the cloud's leading edge), which is an
  // executor / callback-scheduling issue tracked as a separate follow-up.
  mutable std::mutex       buf_mutex_;
  std::deque<ImuSample>    imu_buf_;
  static constexpr size_t  IMU_BUF_MAX = 1000;  // ~5 s at 200 Hz / 10 s at 100 Hz

  mutable std::mutex       odom_buf_mutex_;
  std::deque<OdomSample>   odom_buf_;
  static constexpr size_t  ODOM_BUF_MAX = 400;  // ~8 s at 50 Hz

  // Lever-arm: r_lever_in_lidar = lidar_origin - base_origin, expressed in
  // lidar coords. With livox_frame and base_link sharing axes (URDF rpy
  // 0,0,0) this collapses to the URDF translation. Used to re-express
  // base_link twist at the lidar origin: v_lidar = v_base + omega x r_lever.
  std::string              base_frame_;  // /synchro_odom child_frame_id
  double                   r_lever_x_ = 0.0;
  double                   r_lever_y_ = 0.0;
  std::atomic<bool>        lever_arm_resolved_{false};

  // One-shot startup sanity log gate: prints the t_field datatype, the
  // header.stamp / min_t / max_t / scan_span on the first deskewed cloud
  // so future driver changes (e.g. LIVOX switching the timestamp field
  // back to scan-relative) fail loudly instead of silently miscomputing
  // every per-point IMU lookup.
  bool                     first_scan_logged_ = false;

  // ── IMU-axis projection (project_imu_to_lidar) ──────────────────────
  // The deskew rotates points around the LIDAR's Z-axis using the IMU's
  // angular velocity. If the IMU is mounted with non-trivial roll/pitch
  // relative to the LIDAR (or to base_link if the LIDAR is level), then
  // raw wz_imu is NOT the rotation rate around LIDAR-Z. We need:
  //   wz_lidar = (R_lidar_imu * omega_imu).z = R_lidar_imu[2,:] · omega_imu
  // This was the "A5" plan item, deferred until field-confirmed at the
  // 5π/4 azimuth on the LIVOX pipeline (zed_tracking_link_imu mounted
  // through a torso/zed-head chain that contributes a substantial tilt).
  //
  // When project_imu_to_lidar_ is false, the buffer stores raw
  // msg->angular_velocity.z (legacy behavior, preserved for OUSTER until
  // field-validated there too).
  bool                     project_imu_to_lidar_ = false;
  std::string              lidar_frame_param_;   // optional override; if
                                                 // empty, resolved from
                                                 // first cloud's frame_id.

  // Lazy resolution of the IMU→LIDAR rotation. Both subscribers can race
  // (IMU at 200 Hz, cloud at 10 Hz), so the cloud's frame_id is written
  // under frame_mutex_ exactly once and the projection components are
  // published via the projection_resolved_ atomic with release ordering.
  std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex                                  frame_mutex_;
  std::string                                 lidar_frame_id_;  // resolved
  // Row 2 of R_lidar_imu — apply as
  //   wz_lidar = pv0_*wx_imu + pv1_*wy_imu + pv2_*wz_imu
  double                                      pv0_ = 0.0;
  double                                      pv1_ = 0.0;
  double                                      pv2_ = 1.0;
  std::atomic<bool>                           projection_resolved_{false};

  // NOTE: the prior `out_buf_` persistent staging buffer was removed when
  // onCloud() was switched to deskew in place inside the inbound UniquePtr.
  // The persistent buffer existed to avoid per-frame heap allocation of the
  // output cloud, but with UniquePtr ownership transfer the inbound msg IS
  // the output buffer, so no allocation happens here at all.
};

}  // namespace r1_nav_cpp

#endif  // R1_NAV_CPP__POINTCLOUD_DESKEW_HPP_

#ifndef R1_NAV_CPP__ONLINE_IMU_BIAS_ESTIMATOR_HPP_
#define R1_NAV_CPP__ONLINE_IMU_BIAS_ESTIMATOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <array>

namespace r1_nav_cpp
{

class OnlineImuBiasEstimator : public rclcpp::Node
{
public:
  // NodeOptions ctor enables composability and intra-process comms via the
  // rclcpp_components container. The default-arg form preserves the existing
  // standalone-executable code path.
  explicit OnlineImuBiasEstimator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OnlineImuBiasEstimator() override = default;

private:
  // UniquePtr callbacks: required for intra-process zero-copy. With a shared
  // container running this node + producers/consumers on the same topics,
  // rclcpp routes the same heap allocation through without serialization.
  void onImu(sensor_msgs::msg::Imu::UniquePtr msg);
  void onOdom(nav_msgs::msg::Odometry::UniquePtr msg);
  void statusTick();

  // Parameters
  std::string input_topic_;
  std::string output_topic_;
  std::string odom_topic_;
  std::string output_frame_id_;
  double v_thresh_;
  double w_thresh_;       // legacy single-threshold; used as fallback for enter/exit
  double w_thresh_enter_; // become-stationary threshold (tighter)
  double w_thresh_exit_;  // leave-stationary threshold (looser); enables hysteresis
  bool use_z_only_;
  int warmup_samples_;
  // Deadline for warmup accumulation in wall-clock seconds.
  double warmup_timeout_sec_;
  int min_stationary_count_;
  // Odom freshness deadline (seconds) for stationarity gating.
  double odom_timeout_sec_;
  int bias_update_samples_;   // accumulate this many stationary samples before updating bias
  double bias_update_alpha_;  // blend factor when applying batch mean (0=keep old, 1=replace)
  // Whether to also batch-update gyro_bias_x and gyro_bias_y (in addition
  // to z) once we exit warmup. Default false to preserve legacy behavior;
  // LIVOX opts in via YAML. Useful even in 2-D mode because the corrected
  // wx/wy is published, and other consumers may depend on it. Independent
  // of dynamic_bias_model_, which is z-only.
  bool batch_update_xy_;
  // Whether to overwrite the published angular_velocity_covariance[8]
  // (corrected wz variance) with the regime-aware value below. Default
  // false so the OUSTER pipeline keeps the source IMU's covariance
  // untouched; LIVOX opts in via YAML.
  bool set_yaw_rate_covariance_;
  // Whether to overwrite the published orientation_covariance[8] (yaw
  // attitude variance) with the regime-aware value below. Default false
  // so OUSTER keeps the source IMU's orientation covariance untouched;
  // LIVOX opts in via YAML. See class doc at the top of the .cpp for
  // why this matters: ZED publishes an extremely tight orientation_
  // covariance[8] (~1e-6 rad^2) reflecting instantaneous attitude-filter
  // uncertainty, NOT long-term yaw drift. Without this floor the EKF's
  // absolute-yaw fusion at index 5 dominates the corrected vyaw
  // observation at index 11 by ~30x in steady state.
  bool set_yaw_orientation_covariance_;
  // Dynamic bias model: "none" | "linear" | "quadratic"
  // Each model has its own fitted parameter set — values are NOT shared.
  std::string dynamic_bias_model_;
  double linear_b0_;   // linear:    b(ω) = linear_b0 + linear_k·ω
  double linear_k_;
  double quad_b0_;     // quadratic: b(ω) = quad_b0 + quad_k·ω + quad_k2·ω²
  double quad_k_;
  double quad_k2_;

  // Optional calibrated projection for nav yaw. When enabled, the outgoing
  // angular_velocity.z is row2 dot the bias-corrected IMU angular velocity,
  // and the message frame can be overridden (LIVOX uses base_link) so the
  // EKF does not depend on a potentially wrong torso/head URDF transform.
  bool project_angular_velocity_z_;
  double av_row0_;
  double av_row1_;
  double av_row2_;

  // Output covariance (rad^2/s^2) on the corrected gyro_z, set on every
  // outgoing IMU message so the EKF can adapt its trust to the current
  // regime (warmup vs converged stationary/translation vs rotation).
  double yaw_rate_cov_warmup_;
  double yaw_rate_cov_converged_;
  double yaw_rate_cov_rotating_;

  // Output covariance (rad^2) on the published yaw attitude
  // (orientation_covariance[8]), per regime. Only used when
  // set_yaw_orientation_covariance_ is true.
  double yaw_orientation_cov_warmup_;
  double yaw_orientation_cov_converged_;
  double yaw_orientation_cov_rotating_;

  // State
  std::array<double, 3> gyro_bias_;
  double last_odom_lin_;
  rclcpp::Time last_time_;
  bool last_stationary_;
  int consec_stationary_;

  // Warmup accumulator (startup only)
  struct WarmupAccum {
    int n;
    double wx, wy, wz;
  };
  WarmupAccum warmup_accum_;
  bool warmup_start_initialized_;
  rclcpp::Time warmup_start_time_;

  // Batch bias update accumulator (ongoing, resets on motion).
  // x and y mirror the existing z accumulator; only used when
  // batch_update_xy_ is true and we're past warmup.
  int    bias_accum_n_;
  double bias_accum_wx_;
  double bias_accum_wy_;
  double bias_accum_wz_;

  // Number of completed batch updates so far (any axis). Used as a
  // convergence proxy for the published yaw-rate covariance.
  int    bias_updates_applied_;

  // Odom liveness bookkeeping for stationarity gating.
  bool odom_received_;
  rclcpp::Time last_odom_rx_time_;

  // Output low-pass filter (2nd-order Butterworth IIR) applied to corrected wz before publishing
  bool   lpf_enabled_;
  // Design-time sample rate used to generate LPF coefficients.
  double lpf_sample_rate_hz_;
  double b0_, b1_, b2_, a1_, a2_;  // biquad coefficients
  double lpf_z1_, lpf_z2_;          // direct form II transposed state

  // Output rate gating. Bias estimation, stationary detection, warmup
  // accumulation, batch updates and the LPF all run at the full input
  // rate (~400 Hz on this platform). Only the publish to output_topic_ is
  // throttled to publish_rate_ Hz so downstream consumers (notably the
  // robot_localization EKF) don't get flooded. publish_rate_ <= 0
  // disables gating.
  double publish_rate_;
  rclcpp::Time last_publish_time_;
  bool publish_time_initialized_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace r1_nav_cpp

#endif  // R1_NAV_CPP__ONLINE_IMU_BIAS_ESTIMATOR_HPP_

// OnlineImuBiasEstimator
// ----------------------
// Subscribes to a raw IMU stream, estimates and subtracts the gyroscope
// zero-rate bias online, and republishes a sensor_msgs/Imu on
// `output_topic` (default `/imu/data_bias_corrected`).
//
// FIELD-BY-FIELD SEMANTICS OF THE PUBLISHED MESSAGE
// (read this before changing anything that consumes /imu/data_bias_corrected):
//
//   MUTATED on every published frame:
//     angular_velocity.{x,y,z}            <- raw - gyro_bias_[i]
//     angular_velocity_covariance[8]      <- regime-aware value when
//                                            set_yaw_rate_covariance_ is true
//                                            (see yaw_rate_cov_*)
//     angular_velocity_covariance[0,4]    <- pass-through; we don't fuse
//                                            vroll/vpitch on this stack
//     orientation_covariance[8]           <- regime-aware value when
//                                            set_yaw_orientation_covariance_
//                                            is true (see yaw_orientation_cov_*).
//                                            Default OFF; LIVOX opts in.
//
//   PASS-THROUGH (NOT modified — comes through exactly as the source IMU
//   produced it):
//     orientation                         <- ZED's 6-DoF (gyro+accel)
//                                            attitude. Roll/pitch are
//                                            gravity-corrected; yaw is
//                                            gyro-only with ZED's INTERNAL
//                                            bias estimate, NOT ours.
//     orientation_covariance[0..7]        <- ZED's per-axis attitude-filter
//                                            uncertainty. We only override
//                                            [8] (yaw) when opted in.
//     linear_acceleration                 <- pass-through
//     linear_acceleration_covariance      <- pass-through
//     header                              <- pass-through (we publish on
//                                            the source stamp)
//
// CONSEQUENCE: integrating the published `angular_velocity.z` does NOT in
// general reproduce the published `orientation.yaw`, because ZED integrates
// using its own internal bias estimate while we publish (raw - our bias).
// Rotating `orientation` to match would require knowing ZED's internal
// bias, which the SDK does not expose. Consumers that care about long-loop
// yaw drift bound must NOT rely on `orientation.yaw` here -- it is only
// safe as a relative attitude observation (which is how the EKF uses it
// via imu0_relative=true). Long-loop yaw drift bound on this stack is
// provided by slam_toolbox's map->odom scan match, not by this topic.
//
//   pointcloud_deskew correctly uses `angular_velocity` only.
//   The EKF fuses `orientation` (index 5, with imu0_relative) AND
//     `angular_velocity` (index 11). The set_yaw_orientation_covariance
//     opt-in exists so the EKF can be told the absolute-yaw observation
//     is much weaker than ZED's instantaneous attitude-filter sigma
//     suggests.
#include "r1_nav_cpp/online_imu_bias_estimator.hpp"
#include <cmath>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>

namespace r1_nav_cpp
{

OnlineImuBiasEstimator::OnlineImuBiasEstimator(const rclcpp::NodeOptions & options)
: Node("online_imu_bias_estimator", options),
  project_angular_velocity_z_(false),
  av_row0_(0.0), av_row1_(0.0), av_row2_(1.0),
  last_odom_lin_(0.0),
  last_stationary_(false),
  consec_stationary_(0),
  warmup_accum_{0, 0.0, 0.0, 0.0},
  warmup_start_initialized_(false),
  bias_accum_n_(0),
  bias_accum_wx_(0.0),
  bias_accum_wy_(0.0),
  bias_accum_wz_(0.0),
  bias_updates_applied_(0),
  odom_received_(false),
  lpf_enabled_(false),
  lpf_sample_rate_hz_(400.0),
  b0_(1.0), b1_(0.0), b2_(0.0), a1_(0.0), a2_(0.0),
  lpf_z1_(0.0), lpf_z2_(0.0),
  publish_rate_(100.0),
  publish_time_initialized_(false)
{
  // Declare parameters
  this->declare_parameter("input_topic", "/ouster/imu");
  this->declare_parameter("output_topic", "/imu/data_bias_corrected");
  this->declare_parameter("odom_topic", "/synchro_odom");
  this->declare_parameter("lin_stationary_thresh", 0.02);
  this->declare_parameter("ang_stationary_thresh", 0.02);
  // Hysteresis on the angular stationary threshold. If left at 0.0 they
  // fall back to ang_stationary_thresh, preserving the legacy single-
  // threshold behavior. Otherwise: become-stationary requires
  // |wz| < ang_stationary_thresh_enter; leave-stationary requires
  // |wz| >= ang_stationary_thresh_exit. Set enter <= exit.
  this->declare_parameter("ang_stationary_thresh_enter", 0.0);
  this->declare_parameter("ang_stationary_thresh_exit",  0.0);
  this->declare_parameter("initial_gyro_bias", std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter("bias_update_samples", 200);  // accumulate this many stationary frames per update
  this->declare_parameter("bias_update_alpha",   0.5);  // blend factor: 1.0=replace, 0.5=average with old
  // Default false to preserve legacy OUSTER behavior; LIVOX yaml opts in.
  this->declare_parameter("batch_update_xy", false);
  this->declare_parameter("use_z_only_stationary", true);
  this->declare_parameter("warmup_samples", 300);
  this->declare_parameter("warmup_timeout_sec", 15.0);
  this->declare_parameter("min_stationary_count", 50);
  this->declare_parameter("odom_timeout_sec", 1.0);
  this->declare_parameter("dynamic_bias_model", std::string("linear"));
  this->declare_parameter("linear_b0", 0.0);
  this->declare_parameter("linear_k",  0.0);
  this->declare_parameter("quad_b0",   0.0);
  this->declare_parameter("quad_k",    0.0);
  this->declare_parameter("quad_k2",   0.0);
  this->declare_parameter("lpf_cutoff_hz", 0.0);  // 0 = disabled
  this->declare_parameter("lpf_sample_rate_hz", 400.0);

  // Optional nav yaw projection. Set a calibrated row and output_frame_id
  // when the physical ZED/head/torso pose does not match URDF well enough
  // for robot_localization to transform IMU angular velocity correctly.
  this->declare_parameter("project_angular_velocity_z", false);
  this->declare_parameter("angular_velocity_row2", std::vector<double>{0.0, 0.0, 1.0});
  this->declare_parameter("output_frame_id", std::string(""));

  // High-level override: when true, force the bias estimator to publish
  // raw bias-corrected angular velocity in the source IMU frame so that
  // robot_localization performs the IMU->base_link transform via the
  // URDF TF chain. Disables project_angular_velocity_z and clears
  // output_frame_id regardless of their explicit values, so the URDF
  // chain can be A/B tested with a single knob without leaving the two
  // legacy parameters in an inconsistent state.
  this->declare_parameter("trust_urdf_tf_chain", false);

  // Output rate cap for the published corrected IMU (Hz). The ZED head IMU
  // publishes at ~400 Hz on this platform; bias estimation, stationary
  // detection, warmup, batch updates and the LPF all run at that full rate
  // (more samples => better averaging). However the EKF consumes this
  // topic at a 30 Hz periodicUpdate cycle and was integrating ~13 IMU
  // measurements per cycle, pushing work past its 33 ms budget. 100 Hz
  // gives the EKF ~3.3 samples per cycle while preserving the original
  // assumed sample rate everywhere downstream. Set to 0 (or negative) to
  // disable gating and publish every input frame -- only useful when
  // debugging IMU latency directly.
  this->declare_parameter("publish_rate", 100.0);

  // Output gyro_z covariance per regime. Defaults below are reasonable
  // for a 100 Hz consumer-grade MEMS gyro (ZED head IMU class):
  //   warmup    : 1e-2  (we don't trust corrected wz yet)
  //   converged : 4e-4  (~ (0.02 rad/s)^2; bias-corrected stationary noise)
  //   rotating  : 1e-3  (broader to absorb b(ω) model fit error)
  // Default false to preserve OUSTER behavior (it leaves the source
  // IMU's covariance untouched); LIVOX yaml opts in.
  this->declare_parameter("set_yaw_rate_covariance", false);
  this->declare_parameter("yaw_rate_cov_warmup",    1e-2);
  this->declare_parameter("yaw_rate_cov_converged", 4e-4);
  this->declare_parameter("yaw_rate_cov_rotating",  1e-3);

  // Output yaw attitude covariance (rad^2) per regime, written into
  // orientation_covariance[8] when set_yaw_orientation_covariance is
  // true. Defaults below match the same warmup/converged/rotating
  // ordering as yaw_rate_cov_* and are sane starting points for a ZED
  // 6-DoF (gyro+accel) attitude source where yaw is gyro-only:
  //   warmup    : 1e-2  (~5.7 deg sigma; we don't trust attitude yet)
  //   converged : 1e-4  (~0.57 deg sigma; matches steady-state yaw drift
  //                      across a few seconds of integration)
  //   rotating  : 1e-3  (~1.8 deg sigma; broader to absorb the larger
  //                      gyro-noise contribution during fast rotation)
  // Default false so OUSTER's source-IMU covariance flows through
  // unchanged; LIVOX opts in via YAML.
  this->declare_parameter("set_yaw_orientation_covariance", false);
  this->declare_parameter("yaw_orientation_cov_warmup",    1e-2);
  this->declare_parameter("yaw_orientation_cov_converged", 1e-4);
  this->declare_parameter("yaw_orientation_cov_rotating",  1e-3);

  // Get parameters
  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  output_frame_id_ = this->get_parameter("output_frame_id").as_string();
  v_thresh_ = this->get_parameter("lin_stationary_thresh").as_double();
  w_thresh_ = this->get_parameter("ang_stationary_thresh").as_double();
  double w_enter = this->get_parameter("ang_stationary_thresh_enter").as_double();
  double w_exit  = this->get_parameter("ang_stationary_thresh_exit").as_double();
  // Fall back to the single legacy threshold if hysteresis params unset.
  w_thresh_enter_ = (w_enter > 0.0) ? w_enter : w_thresh_;
  w_thresh_exit_  = (w_exit  > 0.0) ? w_exit  : w_thresh_;
  if (w_thresh_exit_ < w_thresh_enter_) {
    RCLCPP_WARN(this->get_logger(),
      "ang_stationary_thresh_exit (%.4f) < enter (%.4f); swapping for valid hysteresis.",
      w_thresh_exit_, w_thresh_enter_);
    std::swap(w_thresh_enter_, w_thresh_exit_);
  }
  use_z_only_ = this->get_parameter("use_z_only_stationary").as_bool();
  bias_update_samples_ = this->get_parameter("bias_update_samples").as_int();
  bias_update_alpha_   = this->get_parameter("bias_update_alpha").as_double();
  batch_update_xy_     = this->get_parameter("batch_update_xy").as_bool();
  warmup_samples_ = this->get_parameter("warmup_samples").as_int();
  warmup_timeout_sec_ = this->get_parameter("warmup_timeout_sec").as_double();
  min_stationary_count_ = this->get_parameter("min_stationary_count").as_int();
  odom_timeout_sec_ = this->get_parameter("odom_timeout_sec").as_double();
  dynamic_bias_model_ = this->get_parameter("dynamic_bias_model").as_string();
  linear_b0_ = this->get_parameter("linear_b0").as_double();
  linear_k_  = this->get_parameter("linear_k").as_double();
  quad_b0_   = this->get_parameter("quad_b0").as_double();
  quad_k_    = this->get_parameter("quad_k").as_double();
  quad_k2_   = this->get_parameter("quad_k2").as_double();

  project_angular_velocity_z_ = this->get_parameter("project_angular_velocity_z").as_bool();
  auto av_row = this->get_parameter("angular_velocity_row2").as_double_array();
  if (av_row.size() == 3) {
    av_row0_ = av_row[0];
    av_row1_ = av_row[1];
    av_row2_ = av_row[2];
  } else {
    RCLCPP_WARN(this->get_logger(),
      "angular_velocity_row2 must have exactly 3 elements; using [0, 0, 1].");
  }
  const bool trust_urdf_tf_chain = this->get_parameter("trust_urdf_tf_chain").as_bool();
  if (trust_urdf_tf_chain) {
    if (project_angular_velocity_z_ || !output_frame_id_.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "trust_urdf_tf_chain=true overriding project_angular_velocity_z (%s) and "
        "output_frame_id ('%s'). Publishing raw bias-corrected angular_velocity in the "
        "source IMU frame so robot_localization uses the URDF TF chain.",
        project_angular_velocity_z_ ? "true" : "false", output_frame_id_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
        "trust_urdf_tf_chain=true: publishing in source IMU frame; URDF TF chain governs IMU->base_link.");
    }
    project_angular_velocity_z_ = false;
    output_frame_id_.clear();
  }
  if (project_angular_velocity_z_) {
    const double row_norm = std::sqrt(av_row0_ * av_row0_ + av_row1_ * av_row1_ + av_row2_ * av_row2_);
    RCLCPP_INFO(this->get_logger(),
      "Projecting outgoing angular_velocity.z with calibrated row=[%.4f %.4f %.4f] "
      "(|row|=%.4f), output_frame_id='%s'.",
      av_row0_, av_row1_, av_row2_, row_norm, output_frame_id_.c_str());
  }

  set_yaw_rate_covariance_ = this->get_parameter("set_yaw_rate_covariance").as_bool();
  yaw_rate_cov_warmup_    = this->get_parameter("yaw_rate_cov_warmup").as_double();
  yaw_rate_cov_converged_ = this->get_parameter("yaw_rate_cov_converged").as_double();
  yaw_rate_cov_rotating_  = this->get_parameter("yaw_rate_cov_rotating").as_double();

  set_yaw_orientation_covariance_ = this->get_parameter("set_yaw_orientation_covariance").as_bool();
  yaw_orientation_cov_warmup_    = this->get_parameter("yaw_orientation_cov_warmup").as_double();
  yaw_orientation_cov_converged_ = this->get_parameter("yaw_orientation_cov_converged").as_double();
  yaw_orientation_cov_rotating_  = this->get_parameter("yaw_orientation_cov_rotating").as_double();

  publish_rate_ = this->get_parameter("publish_rate").as_double();
  if (publish_rate_ > 0.0) {
    RCLCPP_INFO(this->get_logger(),
                "Output publish rate cap: %.1f Hz (bias estimation runs at full input rate; only "
                "the corrected IMU publish is throttled)", publish_rate_);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Output publish rate cap: DISABLED (publishing on every input frame)");
  }

  // 2nd-order Butterworth LPF (bilinear transform with frequency pre-warping)
  double lpf_cutoff_hz = this->get_parameter("lpf_cutoff_hz").as_double();
  lpf_sample_rate_hz_ = this->get_parameter("lpf_sample_rate_hz").as_double();
  if (lpf_cutoff_hz > 0.0) {
    if (lpf_sample_rate_hz_ <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
        "Invalid lpf_sample_rate_hz=%.3f; using 400.0 Hz.", lpf_sample_rate_hz_);
      lpf_sample_rate_hz_ = 400.0;
    }
    lpf_enabled_ = true;
    const double fs = lpf_sample_rate_hz_;
    double wc  = std::tan(M_PI * lpf_cutoff_hz / fs);  // pre-warped analog freq
    double wc2 = wc * wc;
    double k   = 1.0 + M_SQRT2 * wc + wc2;  // Butterworth denominator (Q=1/√2)
    b0_ = wc2 / k;
    b1_ = 2.0 * b0_;
    b2_ = b0_;
    a1_ = 2.0 * (wc2 - 1.0) / k;
    a2_ = (1.0 - M_SQRT2 * wc + wc2) / k;
    RCLCPP_INFO(this->get_logger(),
      "Output LPF: 2nd-order Butterworth fc=%.1f Hz fs=%.1f Hz  b=[%.5f %.5f %.5f]  a=[%.5f %.5f]",
      lpf_cutoff_hz, fs, b0_, b1_, b2_, a1_, a2_);
  }

  // Initialize biases
  auto gyro_bias_vec = this->get_parameter("initial_gyro_bias").as_double_array();
  for (size_t i = 0; i < 3 && i < gyro_bias_vec.size(); ++i) {
    gyro_bias_[i] = gyro_bias_vec[i];
  }
  auto sensor_qos = rclcpp::SensorDataQoS();

  // Lambda wrappers (instead of std::bind) — needed because the callbacks
  // take UniquePtr by value, which std::bind cannot forward (UniquePtr is
  // move-only). The lambda explicitly moves into the member function.
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    input_topic_, sensor_qos,
    [this](sensor_msgs::msg::Imu::UniquePtr msg) { this->onImu(std::move(msg)); });
  if (!odom_topic_.empty()) {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      [this](nav_msgs::msg::Odometry::UniquePtr msg) { this->onOdom(std::move(msg)); });
  }

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, sensor_qos);

  // Status timer (every 3 seconds)
  status_timer_ = this->create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&OnlineImuBiasEstimator::statusTick, this));

  RCLCPP_INFO(this->get_logger(),
              "IMU in: %s | IMU out: %s | Odom: %s | "
              "min_stationary=%d bias_update=%d/%.1f xy=%s | "
              "ang_thresh enter/exit=%.4f/%.4f | model=%s | "
              "set_yaw_rate_cov=%s warmup/conv/rot=%.5f/%.5f/%.5f | "
              "set_yaw_orient_cov=%s warmup/conv/rot=%.5f/%.5f/%.5f",
              input_topic_.c_str(), output_topic_.c_str(),
              odom_topic_.empty() ? "(disabled)" : odom_topic_.c_str(),
              min_stationary_count_, bias_update_samples_, bias_update_alpha_,
              batch_update_xy_ ? "on" : "off",
              w_thresh_enter_, w_thresh_exit_,
              dynamic_bias_model_.c_str(),
              set_yaw_rate_covariance_ ? "on" : "off",
              yaw_rate_cov_warmup_, yaw_rate_cov_converged_, yaw_rate_cov_rotating_,
              set_yaw_orientation_covariance_ ? "on" : "off",
              yaw_orientation_cov_warmup_,
              yaw_orientation_cov_converged_,
              yaw_orientation_cov_rotating_);
}

void OnlineImuBiasEstimator::onOdom(nav_msgs::msg::Odometry::UniquePtr msg)
{
  double vx = msg->twist.twist.linear.x;
  double vy = msg->twist.twist.linear.y;
  last_odom_lin_ = std::hypot(vx, vy);
  last_odom_rx_time_ = this->now();
  odom_received_ = true;
}

void OnlineImuBiasEstimator::onImu(sensor_msgs::msg::Imu::UniquePtr msg)
{
  const rclcpp::Time now = this->now();
  if (!warmup_start_initialized_) {
    warmup_start_time_ = now;
    warmup_start_initialized_ = true;
  }

  double wx = msg->angular_velocity.x;
  double wy = msg->angular_velocity.y;
  double wz = msg->angular_velocity.z;

  // Stationarity test with hysteresis -- run BEFORE warmup so warmup
  // can refuse motion samples (plan item C2). The metric uses the
  // CURRENTLY-CORRECTED wz so the recovery loop self-heals after a
  // USB-HID re-enumeration injects a step into the raw signal (plan
  // item C1). Cold-start case: gyro_bias_[2] = 0 by default, so the
  // first frames see |wz - 0| = |wz| and behave identically to the
  // legacy raw-wz check until warmup writes a non-zero seed -- no
  // circular dependency on the seed.
  //
  // Why corrected, not raw:
  //   raw     wz_meas = wz_true + bias  (so bias bigger than the
  //                                      stationary threshold makes
  //                                      a parked robot look "rotating"
  //                                      forever; consec_stationary_
  //                                      never latches; the bias never
  //                                      gets re-estimated).
  //   corrected wz_meas - bias_est = wz_true (when est is right) so
  //                                   the threshold compares against
  //                                   noise around true zero.
  //
  // The wx/wy axes are also corrected when use_z_only_ is false; same
  // self-healing rationale.
  const double wz_corr_for_gate = wz - gyro_bias_[2];
  const double wx_corr_for_gate = wx - gyro_bias_[0];
  const double wy_corr_for_gate = wy - gyro_bias_[1];
  double w_metric = use_z_only_
      ? std::abs(wz_corr_for_gate)
      : std::sqrt(wx_corr_for_gate * wx_corr_for_gate +
                  wy_corr_for_gate * wy_corr_for_gate +
                  wz_corr_for_gate * wz_corr_for_gate);
  bool odom_fresh = true;
  double odom_age_sec = 0.0;
  if (!odom_topic_.empty()) {
    odom_fresh = false;
    if (odom_received_) {
      odom_age_sec = (now - last_odom_rx_time_).seconds();
      odom_fresh = (odom_age_sec >= 0.0) && (odom_age_sec <= odom_timeout_sec_);
    }
    if (!odom_fresh) {
      const char * reason = odom_received_ ? "stale" : "missing";
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "odom gating fallback: /synchro_odom %s (age=%.3f s, timeout=%.3f s). "
        "Using angular-only stationarity gating until odom recovers.",
        reason, odom_age_sec, odom_timeout_sec_);
    }
  }
  bool v_ok = odom_topic_.empty() || !odom_fresh || (last_odom_lin_ < v_thresh_);
  double w_thresh_eff = last_stationary_ ? w_thresh_exit_ : w_thresh_enter_;
  bool stationary = v_ok && (w_metric < w_thresh_eff);
  last_stationary_ = stationary;

  // Warmup seeding (averages all three axes during the cold start window).
  // Plan item C2: gate warmup on stationary so motion at boot does not
  // poison the seed. If motion is detected, hold the counter and emit
  // one throttled WARN per second.
  bool in_warmup = (warmup_accum_.n < warmup_samples_);
  if (in_warmup && warmup_timeout_sec_ > 0.0 && warmup_start_initialized_) {
    const double warmup_elapsed = (now - warmup_start_time_).seconds();
    if (warmup_elapsed >= warmup_timeout_sec_) {
      if (warmup_accum_.n > 0) {
        const double n = static_cast<double>(warmup_accum_.n);
        gyro_bias_[0] = warmup_accum_.wx / n;
        gyro_bias_[1] = warmup_accum_.wy / n;
        gyro_bias_[2] = warmup_accum_.wz / n;
      }
      RCLCPP_WARN(this->get_logger(),
        "warmup timeout hit after %.2f s (%d/%d samples). Forcing warmup complete with %s bias seed.",
        warmup_elapsed, warmup_accum_.n, warmup_samples_,
        warmup_accum_.n > 0 ? "partial-mean" : "initial");
      warmup_accum_.n = warmup_samples_;
      in_warmup = false;
    }
  }
  if (in_warmup) {
    if (stationary) {
      warmup_accum_.n++;
      warmup_accum_.wx += wx;
      warmup_accum_.wy += wy;
      warmup_accum_.wz += wz;

      double n = static_cast<double>(warmup_accum_.n);
      gyro_bias_[0] = warmup_accum_.wx / n;
      gyro_bias_[1] = warmup_accum_.wy / n;
      gyro_bias_[2] = warmup_accum_.wz / n;

      // Emit a one-shot INFO at the exact moment warmup completes via
      // the normal "enough stationary samples" path. The timeout path
      // (above, ~line ~415) already emits a WARN. Silent success was
      // making it hard to tell from outside the node whether warmup
      // ever finished or whether the gbias values you see at runtime
      // came from a partial / zero seed.
      if (warmup_accum_.n >= warmup_samples_) {
        in_warmup = false;
        const double warmup_elapsed = warmup_start_initialized_
            ? (now - warmup_start_time_).seconds() : 0.0;
        RCLCPP_INFO(this->get_logger(),
          "warmup complete after %.2f s (%d/%d stationary samples). "
          "Initial gbias=[%.5f, %.5f, %.5f] rad/s. Batch updates take "
          "over from here.",
          warmup_elapsed, warmup_accum_.n, warmup_samples_,
          gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "warmup paused (motion detected): %d/%d samples accumulated, "
        "v=%.3f w_metric_corr=%.4f thresh=%.4f. Hold robot stationary "
        "for the full %d-sample warmup.",
        warmup_accum_.n, warmup_samples_,
        last_odom_lin_, w_metric, w_thresh_eff, warmup_samples_);
    }
  }

  if (stationary) {
    if (consec_stationary_ < min_stationary_count_) {
      consec_stationary_++;
    } else {
      // Confirmed stationary: accumulate all axes into the batch.
      bias_accum_n_++;
      bias_accum_wx_ += wx;
      bias_accum_wy_ += wy;
      bias_accum_wz_ += wz;

      if (bias_accum_n_ >= bias_update_samples_) {
        const double n_inv = 1.0 / static_cast<double>(bias_accum_n_);
        const double mean_x = bias_accum_wx_ * n_inv;
        const double mean_y = bias_accum_wy_ * n_inv;
        const double mean_z = bias_accum_wz_ * n_inv;

        // Always blend z (matches the prior behavior).
        gyro_bias_[2] = (1.0 - bias_update_alpha_) * gyro_bias_[2]
                        + bias_update_alpha_ * mean_z;
        // Only update x/y once we're past warmup AND the option is on.
        // Inside warmup, the running arithmetic mean above already does
        // a better job of seeding x/y from a clean cold-start window.
        if (batch_update_xy_ && !in_warmup) {
          gyro_bias_[0] = (1.0 - bias_update_alpha_) * gyro_bias_[0]
                          + bias_update_alpha_ * mean_x;
          gyro_bias_[1] = (1.0 - bias_update_alpha_) * gyro_bias_[1]
                          + bias_update_alpha_ * mean_y;
        }
        bias_updates_applied_++;
        // Throttled to one line/30 s (2026-07-02): the yaw-drift diagnosis
        // is done and on a long tour the per-batch cadence (~1-2 s while
        // stationary) flooded the odom tmux window. The update counter in
        // the message still shows how many batches were applied between
        // prints; full per-batch detail via --log-level ...:=debug.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 30000,
          "[bias_update #%d] batch_mean=[%.5f %.5f %.5f]  new_bias=[%.5f %.5f %.5f]  (n=%d)",
          bias_updates_applied_, mean_x, mean_y, mean_z,
          gyro_bias_[0], gyro_bias_[1], gyro_bias_[2], bias_accum_n_);
        bias_accum_n_   = 0;
        bias_accum_wx_  = 0.0;
        bias_accum_wy_  = 0.0;
        bias_accum_wz_  = 0.0;
      }
    }
  } else {
    // Motion detected: reset both hysteresis counter and batch accumulator.
    consec_stationary_ = 0;
    bias_accum_n_      = 0;
    bias_accum_wx_     = 0.0;
    bias_accum_wy_     = 0.0;
    bias_accum_wz_     = 0.0;
  }

  // --- Three-regime z-axis correction ---
  //
  //   STATIONARY  (v < v_thresh, |wz| < w_thresh) : EMA updates + correct with EMA only
  //   TRANSLATION (v >= v_thresh, |wz| < w_thresh): EMA frozen  + correct with EMA only
  //   ROTATION    (|wz| >= w_thresh)              : EMA frozen  + correct with EMA + dynamic b(ω)
  //
  // The dynamic model b(ω) was calibrated against (gyro_z - stationary_bias) so it captures
  // only the velocity-dependent residual. During translation (wz≈0) it contributes nothing
  // useful (b0≈0, linear/quad terms ≈ 0); applying it would add noise, so EMA alone suffices.

  // Use the leave-stationary threshold for the rotation regime cutoff so
  // the regime label is consistent with the hysteretic stationary state.
  // Use the BIAS-CORRECTED wz (same metric the stationarity gate uses)
  // so a temporarily large bias estimate cannot mark a parked robot as
  // ROT and (a) trigger the dynamic_bias_model on a zero signal or
  // (b) write the rotating-regime covariances into the published msg.
  bool rotating = (std::abs(wz_corr_for_gate) >= w_thresh_exit_);

  double ema_corr_z = gyro_bias_[2];  // always applied (frozen estimation during any motion)
  double dyn_corr_z = 0.0;            // only during rotation

  if (rotating && dynamic_bias_model_ != "none") {
    if (dynamic_bias_model_ == "linear") {
      dyn_corr_z = linear_b0_ + linear_k_ * wz;
    } else if (dynamic_bias_model_ == "quadratic") {
      dyn_corr_z = quad_b0_ + quad_k_ * wz + quad_k2_ * wz * wz;
    }
  }

  double total_corr_z = ema_corr_z + dyn_corr_z;
  double wz_corrected = wz - total_corr_z;

  // 2nd-order Butterworth LPF — only applied when |wz| < w_thresh_exit_ (stationary/slow).
  // During fast rotation the signal >> noise (SNR ~100:1), filtering only adds phase lag.
  if (lpf_enabled_ && !rotating) {
    double w     = wz_corrected - a1_ * lpf_z1_ - a2_ * lpf_z2_;
    wz_corrected = b0_ * w + b1_ * lpf_z1_ + b2_ * lpf_z2_;
    lpf_z2_      = lpf_z1_;
    lpf_z1_      = w;
  } else {
    // Keep filter state tracking the unfiltered signal to avoid transient on re-entry
    lpf_z2_ = lpf_z1_;
    lpf_z1_ = wz_corrected;
  }

  const char * regime = stationary ? "STA" : (rotating ? "ROT" : "TRN");

  // Mutate the inbound UniquePtr in place — no per-sample deep copy.
  // This is what makes the bias estimator cheap on the IMU thread (was
  // ~720 byte memcpy at 200 Hz = 144 KB/s on the hot path) AND what lets
  // the downstream EKF / pointcloud_deskew receive the corrected msg
  // intra-process zero-copy when this node is loaded into the same
  // ComposableNodeContainer as them.
  const double wx_corrected = wx - gyro_bias_[0];
  const double wy_corrected = wy - gyro_bias_[1];
  if (project_angular_velocity_z_) {
    msg->angular_velocity.x = 0.0;
    msg->angular_velocity.y = 0.0;
    msg->angular_velocity.z = av_row0_ * wx_corrected
                            + av_row1_ * wy_corrected
                            + av_row2_ * wz_corrected;
    if (!output_frame_id_.empty()) {
      msg->header.frame_id = output_frame_id_;
    }
  } else {
    msg->angular_velocity.x = wx_corrected;
    msg->angular_velocity.y = wy_corrected;
    msg->angular_velocity.z = wz_corrected;
  }

  // Set the corrected gyro_z covariance (index 8 = zz) per regime so the
  // EKF can adapt its trust. Other diagonal elements (xx index 0, yy
  // index 4) are left at the source IMU's published covariance — we
  // don't fuse those with the EKF on this platform. Opt-in via
  // set_yaw_rate_covariance to preserve legacy behavior on pipelines
  // that rely on the source covariance flowing through unchanged.
  double yaw_rate_cov = msg->angular_velocity_covariance[8];
  if (set_yaw_rate_covariance_) {
    if (in_warmup) {
      yaw_rate_cov = yaw_rate_cov_warmup_;
    } else if (rotating) {
      yaw_rate_cov = yaw_rate_cov_rotating_;
    } else {
      yaw_rate_cov = yaw_rate_cov_converged_;
    }
    msg->angular_velocity_covariance[8] = yaw_rate_cov;
  }

  // Set the yaw attitude covariance (orientation_covariance[8]) per
  // regime. Opt-in via set_yaw_orientation_covariance to preserve
  // OUSTER pass-through behavior. This exists because ZED publishes
  // an extremely tight orientation_covariance[8] (~1e-6 rad^2 in field
  // tests) reflecting its instantaneous 6-DoF (gyro+accel) attitude-
  // filter uncertainty, NOT long-term yaw drift -- yaw is gyro-only
  // on this stack (no magnetometer fusion, no VIO) so the published
  // attitude has no actual drift bound. With ZED's R the EKF would
  // give absolute-yaw fusion (index 5) ~30x more steady-state weight
  // than the corrected vyaw observation (index 11). Replacing R with
  // a regime-aware floor lets the two contribute on the same order of
  // magnitude and prevents yaw observations from dominating vyaw.
  // Other elements of orientation_covariance (roll, pitch, off-diag)
  // are left untouched -- ZED's gravity-tilt corrected roll/pitch
  // sigmas are accurate.
  double yaw_orientation_cov = msg->orientation_covariance[8];
  if (set_yaw_orientation_covariance_) {
    if (in_warmup) {
      yaw_orientation_cov = yaw_orientation_cov_warmup_;
    } else if (rotating) {
      yaw_orientation_cov = yaw_orientation_cov_rotating_;
    } else {
      yaw_orientation_cov = yaw_orientation_cov_converged_;
    }
    msg->orientation_covariance[8] = yaw_orientation_cov;
  }

  // Per-sample regime / correction trace: promoted to INFO 2026-05-13
  // (1 s throttle = 1 line/s, ~720 lines on a 12 min tour, tolerable
  // while we're chasing yaw drift). Now includes raw wx/wy so we can
  // see whether the projection has real signal to project or is
  // being driven by chassis-compliance transients during accel; and
  // the final published angular_velocity.z (after projection + LPF)
  // so we can compare what we feed the EKF vs the raw / corrected
  // intermediate values. Demote back to DEBUG once drift is fixed.
  const double published_wz =
      project_angular_velocity_z_ ? msg->angular_velocity.z : wz_corrected;
  // DEBUG (was INFO @1 Hz): per-second raw/corrected gyro dump drowned the
  // odom tmux window. Re-enable with --log-level online_imu_bias_estimator:=debug.
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[%s] raw=[wx=%.5f wy=%.5f wz=%.5f] | ema_z=%.5f dyn_z=%.5f total_z=%.5f | "
    "corrected_z=%.5f published_z=%.5f | vyaw_cov=%.5f orient_yaw_cov=%.5f | "
    "consec=%d v=%.3f",
    regime, wx, wy, wz, ema_corr_z, dyn_corr_z, total_corr_z,
    wz_corrected, published_wz, yaw_rate_cov, yaw_orientation_cov,
    consec_stationary_, last_odom_lin_);

  // Output rate gating. All bias estimation, stationary detection, warmup
  // accumulation, batch updates, and the LPF math above run on every input
  // frame so they get the benefit of the full ~400 Hz sample stream. We
  // only skip the actual publish when we're inside the publish_rate_
  // period -- that bounds downstream EKF load (the robot_localization
  // periodicUpdate at 30 Hz was integrating ~13 IMU samples per cycle and
  // tripping its "Failed to meet update rate" warning). 0.5 ms slack on
  // the period check absorbs per-call timing noise so the effective rate
  // doesn't drift below publish_rate_.
  if (publish_rate_ > 0.0 && publish_time_initialized_) {
    const double min_period = 1.0 / publish_rate_;
    const rclcpp::Time msg_stamp(msg->header.stamp);
    const double since_last = (msg_stamp - last_publish_time_).seconds();
    if (since_last + 5e-4 < min_period) {
      return;
    }
  }
  last_publish_time_ = rclcpp::Time(msg->header.stamp);
  publish_time_initialized_ = true;

  // Move ownership of the (now-corrected) msg into publish() so rclcpp
  // can hand the same allocation to intra-process subscribers without
  // a serialize/deserialize round-trip.
  imu_pub_->publish(std::move(msg));
}

void OnlineImuBiasEstimator::statusTick()
{
  // 3 s status heartbeat, throttled to one line/min (2026-07-02): the yaw
  // drift diagnosis that justified INFO-every-tick is done, and the odom
  // tmux window was unreadable. Full-rate detail is still available with
  // --log-level online_imu_bias_estimator:=debug.
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000,
               "stationary=%s consec=%d/%d accum=%d/%d updates=%d | gbias=[%.5f, %.5f, %.5f]",
               last_stationary_ ? "true" : "false",
               consec_stationary_, min_stationary_count_,
               bias_accum_n_, bias_update_samples_,
               bias_updates_applied_,
               gyro_bias_[0], gyro_bias_[1], gyro_bias_[2]);
}

}  // namespace r1_nav_cpp

// Register with class_loader so this node can be loaded into a
// ComposableNodeContainer (with use_intra_process_comms: true).
RCLCPP_COMPONENTS_REGISTER_NODE(r1_nav_cpp::OnlineImuBiasEstimator)

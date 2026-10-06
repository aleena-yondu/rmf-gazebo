#include "r1_nav_cpp/synchro_drive_odometry.hpp"
#include "r1_nav_cpp/synchro_drive_kinematics.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>

namespace r1_nav_cpp
{

SynchroDriveOdometry::SynchroDriveOdometry(const rclcpp::NodeOptions & options)
: Node("synchro_drive_odometry", options),
  wheel_velocity_units_("mps"),
  steer_offsets_(Eigen::Vector3d::Zero()),
  steer_joint_names_({"", "", ""}),
  drive_joint_names_({"", "", ""}),
  steer_joint_indices_({0, 1, 2}),
  drive_joint_indices_({0, 1, 2}),
  joint_name_mapping_requested_(false),
  joint_name_mapping_initialized_(false),
  joint_name_validation_disabled_logged_(false),
  slip_constraint_weight_(0.2),
  x_(0.0), y_(0.0), theta_(0.0),
  time_initialized_(false),
  publish_time_initialized_(false),
  diagnostics_log_period_ms_(0),
  chassis_feedback_qos_depth_(50),
  per_wheel_speed_threshold_(0.005),
  max_dt_(0.1),
  base_linear_variance_(0.005),
  base_yaw_variance_(0.05),
  slip_variance_gain_(2.0),
  omega_variance_gain_(0.05),
  yaw_slip_variance_gain_(2.0),
  last_drive_residual_norm_(0.0),
  last_solver_used_fallback_(false)
{
  // Declare parameters
  this->declare_parameter("wheel_radius", 0.076);
  this->declare_parameter("wheel_base_radius", 0.327);

  // Per-wheel positions in base_link frame, ordered as
  // [x0, y0, x1, y1, x2, y2] for wheels 0=front-left, 1=front-right,
  // 2=rear. Defaults match the historical hard-coded R1 URDF values.
  // Promoting these to parameters lets a URDF retune stay in sync with
  // the kinematics without code changes; the startup assert below
  // catches a code/URDF drift loudly.
  this->declare_parameter(
    "wheel_positions_xy", std::vector<double>{
      0.16897, 0.28,
      0.16897, -0.28,
      -0.32703, 0.0});
  // Tolerance (m) on the assert that all 3 wheels lie at radius
  // wheel_base_radius from base_link origin. 1 cm covers honest
  // measurement noise without hiding a real config error.
  this->declare_parameter("wheel_base_radius_tolerance", 0.01);
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "base_link");
  this->declare_parameter("publish_tf", false);
  // Output rate cap (Hz). Pose is integrated at the full input rate (HDAS
  // 200 Hz on this platform); only the published Odometry msg + TF are
  // throttled to this rate. Choose >= EKF rate (30 Hz) plus headroom; 50 Hz
  // gives the EKF ~1.6 samples per cycle without flooding its measurement
  // queue and triggering "Failed to meet update rate" warnings on the
  // robot_localization side. Set to 0 (or negative) to disable gating
  // entirely and publish on every input frame -- not recommended on
  // 200 Hz inputs unless you're debugging odometry timing directly.
  this->declare_parameter("publish_rate", 50.0);
  // Keep enough subscriber history to absorb short executor scheduling stalls
  // without dropping HDAS's 5 ms-spaced wheel frames. Depth 50 is ~250 ms at
  // 200 Hz; sustained outages are still handled by max_dt on message stamps.
  this->declare_parameter("chassis_feedback_qos_depth", 50);

  // Subscribe directly to the raw HDAS chassis feedback topic. The previous
  // `timestamp_corrector` indirection (republished as
  // `/<ROS_NAMESPACE>/hdas/feedback_chassis`) was added when HDAS firmware
  // shipped header.stamp values 2-3 s ahead of system time. As of the
  // 2026-05-04 hdas_clock_probe.py validation the HDAS clock now agrees
  // with system time to within ~1 ms, so the corrector is unnecessary --
  // and actively harmful, because it overwrites `header.stamp = now()` per
  // message and thereby destroys HDAS's intra-burst 5 ms spacing during
  // wire stalls. Subscribing to the raw topic preserves that spacing and
  // is still TF-safe given the small residual offset.
  this->declare_parameter("chassis_feedback_topic", std::string("/hdas/feedback_chassis"));
  this->declare_parameter("publish_topic", "/synchro_odom");

  // Calibration and Deadband
  this->declare_parameter("yaw_scale_factor", 1.0);
  this->declare_parameter("vx_scale_factor", 1.0);
  this->declare_parameter("vy_scale_factor", 1.0);
  this->declare_parameter("min_linear_velocity", 0.001);
  this->declare_parameter("min_angular_velocity", 0.001);
  this->declare_parameter("use_chassis_speed", false);
  this->declare_parameter("chassis_speed_topic", std::string("/motion_control/chassis_speed"));
  this->declare_parameter("wheel_velocity_units", std::string("mps"));
  this->declare_parameter("steer_joint_names", std::vector<std::string>{});
  this->declare_parameter("drive_joint_names", std::vector<std::string>{});
  // Throttled summary INFO log period for diagnostics. 0 (default) disables
  // the log entirely; the /synchro_odom/diagnostics topic still publishes per
  // frame regardless. Useful to set to e.g. 5000 ms during field validation,
  // 0 in production.
  this->declare_parameter("diagnostics_log_period_ms", 0);

  // Per-wheel steering zero offsets [front-left, front-right, rear] (rad).
  // Subtracted from the reported steer angle before forming the wheel
  // direction so a non-zero mechanical zero doesn't fold into vy bias.
  this->declare_parameter("steer_offsets_rad", std::vector<double>{0.0, 0.0, 0.0});

  // Soft weight on the perpendicular ("zero-slip") constraint in the
  // weighted-LSQ wheel-twist solve. Drive constraints are weight 1.0.
  this->declare_parameter("slip_constraint_weight", 0.2);

  // Per-wheel stationary detection: max(|v_i|) below this is treated as
  // "robot at rest". Replaces the old summed-speed heuristic which masked
  // slow creep and slip on rotation entry.
  this->declare_parameter("per_wheel_speed_threshold", 0.005);

  // Upper bound on integration step (s). Frames spaced longer than this
  // are clamped (and a throttled WARN is emitted) so a single USB hiccup
  // doesn't integrate stale velocity straight into pose.
  this->declare_parameter("max_dt", 0.1);

  // Adaptive published-covariance gains. See header for the formula.
  this->declare_parameter("base_linear_variance", 0.005);
  this->declare_parameter("base_yaw_variance", 0.05);
  this->declare_parameter("slip_variance_gain", 2.0);
  this->declare_parameter("omega_variance_gain", 0.05);
  this->declare_parameter("yaw_slip_variance_gain", 2.0);

  // Get parameters
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  wheel_base_radius_ = this->get_parameter("wheel_base_radius").as_double();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  publish_tf_ = this->get_parameter("publish_tf").as_bool();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  chassis_feedback_qos_depth_ =
    std::max(1, static_cast<int>(this->get_parameter("chassis_feedback_qos_depth").as_int()));

  yaw_scale_factor_ = this->get_parameter("yaw_scale_factor").as_double();
  vx_scale_factor_ = this->get_parameter("vx_scale_factor").as_double();
  vy_scale_factor_ = this->get_parameter("vy_scale_factor").as_double();
  min_linear_velocity_ = this->get_parameter("min_linear_velocity").as_double();
  min_angular_velocity_ = this->get_parameter("min_angular_velocity").as_double();
  use_chassis_speed_ = this->get_parameter("use_chassis_speed").as_bool();
  wheel_velocity_units_ = this->get_parameter("wheel_velocity_units").as_string();
  if (wheel_velocity_units_ != "mps" && wheel_velocity_units_ != "radps") {
    RCLCPP_WARN(
      this->get_logger(),
      "wheel_velocity_units='%s' is invalid; expected 'mps' or 'radps'. "
      "Falling back to current behavior: mps.",
      wheel_velocity_units_.c_str());
    wheel_velocity_units_ = "mps";
  }

  auto offsets = this->get_parameter("steer_offsets_rad").as_double_array();
  for (size_t i = 0; i < 3 && i < offsets.size(); ++i) {
    steer_offsets_(i) = offsets[i];
  }

  auto steer_names = this->get_parameter("steer_joint_names").as_string_array();
  auto drive_names = this->get_parameter("drive_joint_names").as_string_array();
  if (steer_names.empty() && drive_names.empty()) {
    joint_name_mapping_requested_ = false;
  } else if (steer_names.size() == 3 && drive_names.size() == 3) {
    joint_name_mapping_requested_ = true;
    for (size_t i = 0; i < 3; ++i) {
      steer_joint_names_[i] = steer_names[i];
      drive_joint_names_[i] = drive_names[i];
    }
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "steer_joint_names and drive_joint_names must both be empty or both have "
      "exactly 3 entries. Disabling JointState name validation and using legacy "
      "position[0..2]/velocity[0..2] indexing.");
    joint_name_mapping_requested_ = false;
  }

  slip_constraint_weight_ = this->get_parameter("slip_constraint_weight").as_double();
  per_wheel_speed_threshold_ = this->get_parameter("per_wheel_speed_threshold").as_double();
  max_dt_ = this->get_parameter("max_dt").as_double();

  base_linear_variance_ = this->get_parameter("base_linear_variance").as_double();
  base_yaw_variance_ = this->get_parameter("base_yaw_variance").as_double();
  slip_variance_gain_ = this->get_parameter("slip_variance_gain").as_double();
  omega_variance_gain_ = this->get_parameter("omega_variance_gain").as_double();
  yaw_slip_variance_gain_ = this->get_parameter("yaw_slip_variance_gain").as_double();
  diagnostics_log_period_ms_ = this->get_parameter("diagnostics_log_period_ms").as_int();

  // Wheel positions in base_link frame. Pulled from the
  // `wheel_positions_xy` parameter (see declare_parameter above) so a
  // URDF retune doesn't require a code change. Defaults match the
  // historical hard-coded R1 layout:
  //   Wheel 0 (front-left)  : ( 0.16897,  0.280)
  //   Wheel 1 (front-right) : ( 0.16897, -0.280)
  //   Wheel 2 (rear)        : (-0.32703,  0.000)
  // The kinematics in computeRobotVelocity() build the per-wheel rolling
  // direction directly from `steering_angles(i) - steer_offsets_(i)` --
  // the steering encoder reports angle relative to base_link, NOT to a
  // wheel-local mounting frame.
  {
    auto wxy = this->get_parameter("wheel_positions_xy").as_double_array();
    if (wxy.size() != 6) {
      RCLCPP_ERROR(
        this->get_logger(),
        "wheel_positions_xy must have exactly 6 entries (x0,y0,x1,y1,x2,y2); "
        "got %zu. Falling back to hard-coded R1 defaults.", wxy.size());
      wxy = {0.16897, 0.28, 0.16897, -0.28, -0.32703, 0.0};
    }
    for (int i = 0; i < 3; ++i) {
      wheel_positions_(i, 0) = wxy[2 * i + 0];
      wheel_positions_(i, 1) = wxy[2 * i + 1];
    }
  }

  // Assert that ||wheel_positions_(i)|| ~= wheel_base_radius. The drive
  // kinematics implicitly assume all three wheels sit on a circle of
  // radius wheel_base_radius around base_link; if a URDF edit moves a
  // wheel without updating wheel_base_radius (or vice versa), the
  // forward-kinematics matrix and the rotation regime detection silently
  // disagree. Loud failure beats silent drift.
  const double r_tol = this->get_parameter("wheel_base_radius_tolerance").as_double();
  for (int i = 0; i < 3; ++i) {
    const double r_i = std::hypot(wheel_positions_(i, 0), wheel_positions_(i, 1));
    if (std::abs(r_i - wheel_base_radius_) > r_tol) {
      RCLCPP_WARN(
        this->get_logger(),
        "wheel_base_radius=%.4f does not match ||wheel_positions(%d)||=%.4f "
        "(tolerance=%.3f). Update wheel_base_radius or wheel_positions_xy "
        "to match the URDF; the kinematics implicitly assume all wheels "
        "lie on a single circle of radius wheel_base_radius.",
        wheel_base_radius_, i, r_i, r_tol);
    }
  }

  if (use_chassis_speed_) {
    std::string speed_topic = this->get_parameter("chassis_speed_topic").as_string();
    chassis_speed_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      speed_topic, rclcpp::QoS(10).best_effort().durability_volatile(),
      std::bind(&SynchroDriveOdometry::chassisSpeedCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Chassis speed mode: subscribing to %s", speed_topic.c_str());
  } else {
    std::string topic = this->get_parameter("chassis_feedback_topic").as_string();
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      topic, rclcpp::QoS(chassis_feedback_qos_depth_).best_effort().durability_volatile(),
      std::bind(&SynchroDriveOdometry::chassisFeedbackCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
      this->get_logger(),
      "Wheel kinematics mode: subscribing to %s (best_effort/volatile, depth=%d)",
      topic.c_str(), chassis_feedback_qos_depth_);
  }

  // Create publisher
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
    this->get_parameter(
      "publish_topic").as_string(), 10);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    this->get_parameter("publish_topic").as_string() + "/diagnostics", 10);

  // Create TF broadcaster if needed
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  RCLCPP_INFO(this->get_logger(), "Synchro Drive Odometry started.");
  RCLCPP_INFO(
    this->get_logger(),
    "Using URDF wheel positions. Wheel radius: %.3fm | wheel_velocity_units=%s",
    wheel_radius_, wheel_velocity_units_.c_str());
  if (joint_name_mapping_requested_) {
    RCLCPP_INFO(
      this->get_logger(),
      "JointState name validation enabled: steer=[%s, %s, %s] drive=[%s, %s, %s]",
      steer_joint_names_[0].c_str(), steer_joint_names_[1].c_str(),
      steer_joint_names_[2].c_str(), drive_joint_names_[0].c_str(),
      drive_joint_names_[1].c_str(), drive_joint_names_[2].c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "JointState name validation disabled: using legacy position[0..2]/velocity[0..2] mapping");
  }
  for (int i = 0; i < 3; ++i) {
    const double xi = wheel_positions_(i, 0);
    const double yi = wheel_positions_(i, 1);
    const double r = std::hypot(xi, yi);
    const double bearing_deg = std::atan2(yi, xi) * 180.0 / M_PI;
    RCLCPP_INFO(
      this->get_logger(),
      "  Wheel %d: pos=(%.3f, %.3f) r=%.3f bearing=%.1f° steer_offset=%.4f rad",
      i, xi, yi, r, bearing_deg, steer_offsets_(i));
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Scales: vx=%.4f vy=%.4f yaw=%.4f | slip_w=%.3f | base_var=[lin=%.4f yaw=%.4f] | "
    "slip_gain=%.3f omega_gain=%.3f yaw_slip_gain=%.3f | max_dt=%.3fs | per_wheel_thresh=%.4f",
    vx_scale_factor_, vy_scale_factor_, yaw_scale_factor_,
    slip_constraint_weight_,
    base_linear_variance_, base_yaw_variance_,
    slip_variance_gain_, omega_variance_gain_, yaw_slip_variance_gain_,
    max_dt_, per_wheel_speed_threshold_);
  if (publish_rate_ > 0.0) {
    RCLCPP_INFO(
      this->get_logger(),
      "Publish rate cap: %.1f Hz (pose integrated at full input rate; only the published "
      "Odometry + TF are throttled)", publish_rate_);
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Publish rate cap: DISABLED (publishing on every input frame)");
  }
}

bool SynchroDriveOdometry::initializeJointMapping(const sensor_msgs::msg::JointState & msg)
{
  if (!joint_name_mapping_requested_) {
    return true;
  }

  if (msg.name.empty()) {
    if (!joint_name_validation_disabled_logged_) {
      RCLCPP_WARN(
        this->get_logger(),
        "JointState.name is empty; disabling name validation for this publisher "
        "and using legacy position[0..2]/velocity[0..2] indexing.");
      joint_name_validation_disabled_logged_ = true;
    }
    joint_name_mapping_requested_ = false;
    return true;
  }

  auto find_name = [&msg](const std::string & target) -> int {
      auto it = std::find(msg.name.begin(), msg.name.end(), target);
      if (it == msg.name.end()) {
        return -1;
      }
      return static_cast<int>(std::distance(msg.name.begin(), it));
    };

  for (int i = 0; i < 3; ++i) {
    steer_joint_indices_[i] = find_name(steer_joint_names_[i]);
    drive_joint_indices_[i] = find_name(drive_joint_names_[i]);
    if (steer_joint_indices_[i] < 0 || drive_joint_indices_[i] < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "JointState.name does not contain configured steer/drive names. "
        "Missing mapping at wheel %d (steer='%s' idx=%d, drive='%s' idx=%d).",
        i, steer_joint_names_[i].c_str(), steer_joint_indices_[i],
        drive_joint_names_[i].c_str(), drive_joint_indices_[i]);
      return false;
    }
  }

  joint_name_mapping_initialized_ = true;
  RCLCPP_INFO(
    this->get_logger(),
    "JointState mapping resolved: steer_idx=[%d,%d,%d] drive_idx=[%d,%d,%d]",
    steer_joint_indices_[0], steer_joint_indices_[1], steer_joint_indices_[2],
    drive_joint_indices_[0], drive_joint_indices_[1], drive_joint_indices_[2]);
  return true;
}

bool SynchroDriveOdometry::extractWheelState(
  const sensor_msgs::msg::JointState & msg,
  Eigen::Vector3d & steering_angles,
  Eigen::Vector3d & wheel_velocities)
{
  if (msg.position.size() < 3 || msg.velocity.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Joint state incomplete: pos=%zu, vel=%zu",
      msg.position.size(), msg.velocity.size());
    return false;
  }

  if (joint_name_mapping_requested_ && !joint_name_mapping_initialized_ &&
    !initializeJointMapping(msg))
  {
    return false;
  }

  for (int i = 0; i < 3; ++i) {
    const int steer_idx = joint_name_mapping_requested_ ? steer_joint_indices_[i] : i;
    const int drive_idx = joint_name_mapping_requested_ ? drive_joint_indices_[i] : i;
    if (steer_idx < 0 || drive_idx < 0 ||
      static_cast<size_t>(steer_idx) >= msg.position.size() ||
      static_cast<size_t>(drive_idx) >= msg.velocity.size())
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "JointState index out of range for wheel %d: steer_idx=%d pos_size=%zu, "
        "drive_idx=%d vel_size=%zu",
        i, steer_idx, msg.position.size(), drive_idx, msg.velocity.size());
      return false;
    }

    steering_angles(i) = msg.position[steer_idx];
    wheel_velocities(i) = msg.velocity[drive_idx];
    if (wheel_velocity_units_ == "radps") {
      wheel_velocities(i) *= wheel_radius_;
    }
  }

  if (!steering_angles.allFinite() || !wheel_velocities.allFinite()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "JointState contains non-finite steering or wheel velocity values; skipping frame.");
    return false;
  }

  return true;
}

void SynchroDriveOdometry::chassisFeedbackCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Use the HDAS sensor stamp directly. As of 2026-05-04 HDAS publishes
  // header.stamp aligned to system time within ~1 ms (validated by
  // hdas_clock_probe.py), so we no longer route through timestamp_corrector
  // -- preserving HDAS's true 5 ms intra-burst spacing during wire stalls.
  auto current_time = rclcpp::Time(msg->header.stamp);

  // Immediately compute the latest twist from the new data
  try {
    Eigen::Vector3d steering_angles;
    Eigen::Vector3d wheel_velocities;
    if (!extractWheelState(*msg, steering_angles, wheel_velocities)) {
      return;
    }

    // Initialize time only after the first valid wheel frame. This prevents
    // malformed JointState messages from consuming a real motion interval.
    if (!time_initialized_) {
      last_time_ = current_time;
      time_initialized_ = true;
      return;
    }

    double dt = (current_time - last_time_).seconds();

    if (dt <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Received message with non-increasing timestamp, skipping. dt=%.4f", dt);
      return;
    }

    if (dt > max_dt_) {
      // Long gap: the previous twist is no longer trustworthy as a model
      // for what happened between last_time_ and current_time.
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Wheel feedback dt=%.4fs exceeds max_dt=%.3fs (gap); skipping "
        "integration this frame and publishing zero twist with bumped cov.",
        dt, max_dt_);
      last_time_ = current_time;
      last_drive_residual_norm_ = 0.0;
      last_solver_used_fallback_ = false;
      publishOdometry(current_time, Eigen::Vector3d::Zero(), /*gap=*/ true);
      return;
    }

    Eigen::Vector3d robot_vel;
    if (!computeRobotVelocity(steering_angles, wheel_velocities, robot_vel)) {
      last_time_ = current_time;
      publishOdometry(current_time, Eigen::Vector3d::Zero(), /*gap=*/ true);
      return;
    }

    last_time_ = current_time;

    // Apply calibration scales exactly once, here. Both the integrated
    // pose and the published twist must be derived from this same scaled
    // vector so they remain consistent (the previous code scaled inside
    // integrateOdometry only, so /synchro_odom twist.linear/angular were
    // unscaled while pose.x/y/yaw were scaled -- they disagreed).
    robot_vel(0) *= vx_scale_factor_;
    robot_vel(1) *= vy_scale_factor_;
    robot_vel(2) *= yaw_scale_factor_;

    // Deadband on the FINAL published twist, after calibration scales. This
    // matches the chassis-speed callback's ordering exactly.
    applyTwistDeadband(robot_vel);

    // Sanity check against expected control ranges (post-scale, post-deadband).
    constexpr double max_linear = 2.0;     // m/s
    constexpr double max_angular = 4.0;    // rad/s
    if (std::abs(robot_vel(0)) > max_linear || std::abs(robot_vel(1)) > max_linear) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Linear velocity exceeds expected: vx=%.3f, vy=%.3f", robot_vel(0), robot_vel(1));
    }
    if (std::abs(robot_vel(2)) > max_angular) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Angular velocity exceeds expected: omega=%.3f", robot_vel(2));
    }

    if (!robot_vel.allFinite() || !integrateOdometry(robot_vel, dt)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Wheel odometry produced non-finite state; holding pose and publishing bumped covariance.");
      publishOdometry(current_time, Eigen::Vector3d::Zero(), /*gap=*/ true);
      return;
    }

    // Publish with the message timestamp
    publishOdometry(current_time, robot_vel);

  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Error in odometry calculation: %s", e.what());
  }
}

void SynchroDriveOdometry::chassisSpeedCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  auto current_time = rclcpp::Time(msg->header.stamp);

  if (!time_initialized_) {
    last_time_ = current_time;
    time_initialized_ = true;
    return;
  }

  double dt = (current_time - last_time_).seconds();

  if (dt <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Chassis speed: non-increasing timestamp, skipping. dt=%.4f", dt);
    return;
  }

  if (dt > max_dt_) {
    // See chassisFeedbackCallback for rationale: skip integration on
    // long gaps; publish unchanged pose + zero twist + bumped cov so
    // the EKF leans on its predict step instead of integrating stale
    // velocity for max_dt.
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Chassis speed dt=%.4fs exceeds max_dt=%.3fs (gap); skipping "
      "integration and publishing zero twist with bumped cov.",
      dt, max_dt_);
    last_time_ = current_time;
    last_drive_residual_norm_ = 0.0;
    publishOdometry(current_time, Eigen::Vector3d::Zero(), /*gap=*/ true);
    return;
  }

  last_time_ = current_time;

  // Apply calibration scales exactly once, here.
  Eigen::Vector3d robot_vel(
    msg->twist.linear.x * vx_scale_factor_,
    msg->twist.linear.y * vy_scale_factor_,
    msg->twist.angular.z * yaw_scale_factor_);

  // Deadband on the FINAL published twist, identically to the wheel path.
  applyTwistDeadband(robot_vel);

  // Chassis-speed source has no per-wheel slip residual -- keep the
  // adaptive-covariance slip term at zero in this mode.
  last_drive_residual_norm_ = 0.0;
  last_solver_used_fallback_ = false;
  if (!integrateOdometry(robot_vel, dt)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(), 5000,
      "Chassis speed odometry produced non-finite state; holding pose and publishing bumped covariance.");
    publishOdometry(current_time, Eigen::Vector3d::Zero(), /*gap=*/ true);
    return;
  }
  publishOdometry(current_time, robot_vel);
}

bool SynchroDriveOdometry::computeRobotVelocity(
  const Eigen::Vector3d & steering_angles,
  const Eigen::Vector3d & wheel_velocities,
  Eigen::Vector3d & robot_velocity)
{
  WheelGeometry geometry;
  geometry.positions = wheel_positions_;
  geometry.steer_offsets = steer_offsets_;
  const auto result = solveBodyTwist(
    geometry, steering_angles, wheel_velocities,
    slip_constraint_weight_, per_wheel_speed_threshold_);
  last_drive_residual_norm_ = result.drive_residual_norm;
  last_solver_used_fallback_ = result.used_fallback_solver;
  if (!result.success) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Wheel kinematics solve failed (%s); publishing bumped covariance and holding pose.",
      result.failure_reason.c_str());
    robot_velocity = Eigen::Vector3d::Zero();
    return false;
  }

  robot_velocity = toEigen(result.twist);

  // NOTE: deadband + scale ordering is intentionally NOT done here. Callers
  // apply calibration scales first, then call applyTwistDeadband() so the
  // deadband always operates on the final published twist, identically across
  // the wheel-kinematics and chassis-speed code paths.

  if (last_solver_used_fallback_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Wheel kinematics used fallback solver; twist is finite but should be reviewed.");
  }

  return true;
}

void SynchroDriveOdometry::applyTwistDeadband(Eigen::Vector3d & robot_velocity) const
{
  if (std::abs(robot_velocity(0)) < min_linear_velocity_) {robot_velocity(0) = 0.0;}
  if (std::abs(robot_velocity(1)) < min_linear_velocity_) {robot_velocity(1) = 0.0;}
  if (std::abs(robot_velocity(2)) < min_angular_velocity_) {robot_velocity(2) = 0.0;}
}

bool SynchroDriveOdometry::integrateOdometry(const Eigen::Vector3d & robot_vel, double dt)
{
  // NOTE: scale factors (vx/vy/yaw_scale_factor_) are applied upstream,
  // in the callbacks. Do NOT reapply them here — the integrated pose and
  // the published twist must be derived from the same scaled vector.
  if (!robot_vel.allFinite() || !std::isfinite(dt) || dt <= 0.0) {
    return false;
  }

  double next_x = x_;
  double next_y = y_;
  double next_theta = theta_;
  integrateMidpoint(next_x, next_y, next_theta, toTwist2D(robot_vel), dt);
  if (!std::isfinite(next_x) || !std::isfinite(next_y) || !std::isfinite(next_theta)) {
    return false;
  }

  x_ = next_x;
  y_ = next_y;
  theta_ = next_theta;
  return true;
}

void SynchroDriveOdometry::publishOdometry(
  const rclcpp::Time & current_time,
  const Eigen::Vector3d & robot_vel,
  bool gap)
{
  // Output rate gating. Pose has already been integrated at the full input
  // rate by the caller, so the integrated pose state is up to date; we just
  // skip the message + TF publish if it's too soon since the last one. This
  // bounds downstream load (in particular the EKF's measurement queue
  // backlog at robot_localization::RosFilter::periodicUpdate) without losing
  // any wheel-encoder information.
  if (publish_rate_ > 0.0 && publish_time_initialized_) {
    const double min_period = 1.0 / publish_rate_;
    const double since_last = (current_time - last_publish_time_).seconds();
    // Tolerate small clock jitter -- skip only when we're clearly inside
    // the period. The 0.5 ms slack avoids drifting the effective rate
    // below `publish_rate_` due to per-call timing noise.
    if (since_last + 5e-4 < min_period) {
      return;
    }
  }
  last_publish_time_ = current_time;
  publish_time_initialized_ = true;

  // Create odometry message
  auto odom_msg = nav_msgs::msg::Odometry();
  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  // Position
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  // Orientation (quaternion from yaw)
  auto quat = eulerToQuaternion(0.0, 0.0, theta_);
  odom_msg.pose.pose.orientation.x = quat.x();
  odom_msg.pose.pose.orientation.y = quat.y();
  odom_msg.pose.pose.orientation.z = quat.z();
  odom_msg.pose.pose.orientation.w = quat.w();

  // Adaptive covariance for the TWIST (velocity) channel:
  //   var_lin = base_lin + slip_gain * residual^2 + omega_gain * |wz|
  //   var_yaw = base_yaw + yaw_slip_gain * residual^2
  // Higher residual ⇒ wheels disagree on a single body twist (slip);
  // higher |wz| ⇒ in-place rotation regime where vx/vy are most exposed
  // to slip even when the LSQ residual is small (uniform slip).
  const double r2 = last_drive_residual_norm_ * last_drive_residual_norm_;
  const double abs_wz = std::abs(robot_vel(2));
  double lin_var = base_linear_variance_ +
    slip_variance_gain_ * r2 +
    omega_variance_gain_ * abs_wz;
  double yaw_var = base_yaw_variance_ +
    yaw_slip_variance_gain_ * r2;
  if (gap) {
    // Frame gap: this odom carries no fresh wheel information for the
    // interval [last_time_, current_time]. Bump twist variance so the
    // EKF leans on its predict step rather than treating this zero
    // twist as a hard "robot is stationary" measurement. 1 m^2/s^2 and
    // 1 rad^2/s^2 are large enough that the EKF essentially ignores
    // this update relative to the IMU's vyaw observation, while still
    // staying within numerically sane limits.
    lin_var = std::max(lin_var, 1.0);
    yaw_var = std::max(yaw_var, 1.0);
  }

  // Pose covariance.
  // The EKF (livox/ekf.yaml & ouster/ekf.yaml) only fuses TWIST channels
  // (vx, vy) from this odom -- never pose. A previous version of this
  // code wrote `lin_var` (m²/s², a velocity-noise variance) into the
  // pose.covariance[0/7] (m², an absolute-pose variance) and `yaw_var`
  // (rad²/s²) into pose.covariance[35] (rad²). Dimensions disagreed,
  // and if odom0_config is ever flipped to fuse pose, the EKF would
  // silently over-trust a wheel-only pose with a velocity-noise number.
  //
  // Honest fix: mark every pose channel as "I have no fit-for-fusion
  // pose estimate" by setting a large variance on the diagonal. Proper
  // integrated covariance (R·twist_cov·R^T·dt² accumulators) is the
  // long-term answer if this odom is ever fused as pose; until then
  // a 1e6 wall prevents an accidental config flip from corrupting EKF.
  odom_msg.pose.covariance[0] = 1e6;    // x  -- not fit for fusion (no integrated cov)
  odom_msg.pose.covariance[7] = 1e6;    // y  -- not fit for fusion
  odom_msg.pose.covariance[14] = 1e6;   // z  (disabled)
  odom_msg.pose.covariance[21] = 1e6;   // roll  (disabled)
  odom_msg.pose.covariance[28] = 1e6;   // pitch (disabled)
  odom_msg.pose.covariance[35] = 1e6;   // yaw -- not fit for fusion

  // Velocity (in robot frame)
  odom_msg.twist.twist.linear.x = robot_vel(0);
  odom_msg.twist.twist.linear.y = robot_vel(1);
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = robot_vel(2);

  // Twist covariance
  odom_msg.twist.covariance[0] = lin_var;   // vx variance
  odom_msg.twist.covariance[7] = lin_var;   // vy variance
  odom_msg.twist.covariance[14] = 1e6;      // vz (disabled)
  odom_msg.twist.covariance[21] = 1e6;      // vroll (disabled)
  odom_msg.twist.covariance[28] = 1e6;      // vpitch (disabled)
  odom_msg.twist.covariance[35] = yaw_var;  // vyaw variance

  // Publish odometry
  odom_pub_->publish(odom_msg);
  publishDiagnostics(robot_vel, lin_var, yaw_var, gap);

  // Publish transform if enabled
  if (publish_tf_) {
    publishTransform(current_time, quat);
  }
}

void SynchroDriveOdometry::publishDiagnostics(
  const Eigen::Vector3d & robot_vel,
  double lin_var,
  double yaw_var,
  bool gap)
{
  if (!diagnostics_pub_) {
    return;
  }

  const double abs_wz = std::abs(robot_vel(2));

  diagnostic_msgs::msg::DiagnosticArray msg;
  msg.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = std::string(this->get_name()) + ": wheel_odometry";
  status.hardware_id = "synchro_drive";
  if (gap) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "frame gap (no fresh wheel data)";
  } else if (last_solver_used_fallback_) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "kinematics solve used fallback decomposition";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ok";
  }

  auto add_kv = [&status](const std::string & key, double value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.6g", value);
      kv.value = buf;
      status.values.push_back(kv);
    };
  auto add_kv_bool = [&status](const std::string & key, bool value) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = key;
      kv.value = value ? "true" : "false";
      status.values.push_back(kv);
    };

  add_kv("drive_residual_norm", last_drive_residual_norm_);
  add_kv("abs_wz", abs_wz);
  add_kv("lin_var", lin_var);
  add_kv("yaw_var", yaw_var);
  add_kv_bool("gap", gap);
  add_kv_bool("fallback_solver", last_solver_used_fallback_);

  msg.status.push_back(std::move(status));
  diagnostics_pub_->publish(msg);

  if (diagnostics_log_period_ms_ > 0) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(), diagnostics_log_period_ms_,
      "Wheel odom diag: residual=%.4f abs_wz=%.4f lin_var=%.4f yaw_var=%.4f "
      "gap=%s fallback_solver=%s",
      last_drive_residual_norm_, abs_wz, lin_var, yaw_var,
      gap ? "true" : "false",
      last_solver_used_fallback_ ? "true" : "false");
  }
}

void SynchroDriveOdometry::publishTransform(
  const rclcpp::Time & current_time,
  const Eigen::Quaterniond & quat)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = current_time;
  t.header.frame_id = odom_frame_;
  t.child_frame_id = base_frame_;

  t.transform.translation.x = x_;
  t.transform.translation.y = y_;
  t.transform.translation.z = 0.0;

  t.transform.rotation.x = quat.x();
  t.transform.rotation.y = quat.y();
  t.transform.rotation.z = quat.z();
  t.transform.rotation.w = quat.w();

  tf_broadcaster_->sendTransform(t);
}

Eigen::Quaterniond SynchroDriveOdometry::eulerToQuaternion(
  double roll, double pitch, double yaw)
{
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Quaterniond q;
  q.w() = cy * cp * cr + sy * sp * sr;
  q.x() = cy * cp * sr - sy * sp * cr;
  q.y() = sy * cp * sr + cy * sp * cr;
  q.z() = sy * cp * cr - cy * sp * sr;

  return q;
}

}  // namespace r1_nav_cpp

// Register with class_loader so this node can be loaded into a
// ComposableNodeContainer (with use_intra_process_comms: true).
RCLCPP_COMPONENTS_REGISTER_NODE(r1_nav_cpp::SynchroDriveOdometry)

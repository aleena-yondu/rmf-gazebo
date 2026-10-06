#ifndef R1_NAV_CPP__SYNCHRO_DRIVE_ODOMETRY_HPP_
#define R1_NAV_CPP__SYNCHRO_DRIVE_ODOMETRY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

namespace r1_nav_cpp
{

class SynchroDriveOdometry : public rclcpp::Node
{
public:
  // NodeOptions ctor enables composability via rclcpp_components. Default-arg
  // form preserves the standalone-executable code path.
  explicit SynchroDriveOdometry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SynchroDriveOdometry() override = default;

private:
  // Callbacks
  void chassisFeedbackCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void chassisSpeedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // Helper methods
  bool extractWheelState(
    const sensor_msgs::msg::JointState & msg,
    Eigen::Vector3d & steering_angles,
    Eigen::Vector3d & wheel_velocities);
  bool initializeJointMapping(const sensor_msgs::msg::JointState & msg);
  bool computeRobotVelocity(
    const Eigen::Vector3d & steering_angles,
    const Eigen::Vector3d & wheel_velocities,
    Eigen::Vector3d & robot_velocity);
  void applyTwistDeadband(Eigen::Vector3d & robot_velocity) const;
  bool integrateOdometry(const Eigen::Vector3d & robot_vel, double dt);
  // gap=true publishes the unchanged pose with zero twist and an
  // inflated twist covariance, used when the inter-message dt has
  // exceeded max_dt (frame gap from a USB stall etc.). gap=false is
  // the normal-frame path.
  void publishOdometry(
    const rclcpp::Time & current_time,
    const Eigen::Vector3d & robot_vel,
    bool gap = false);
  void publishDiagnostics(
    const Eigen::Vector3d & robot_vel,
    double lin_var,
    double yaw_var,
    bool gap);
  void publishTransform(const rclcpp::Time & current_time, const Eigen::Quaterniond & quat);
  Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);

  // Parameters
  double wheel_radius_;
  double wheel_base_radius_;
  std::string wheel_velocity_units_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_;
  double publish_rate_;

  // Wheel geometry — ACTUAL R1 robot configuration.
  // wheel_positions_(i) = (x, y) of wheel i in base_link frame. Used by
  // the LSQ build in computeRobotVelocity. The previous wheel_angles_
  // member ("base mounting direction at steer=0") was dead code: the
  // kinematics consume steering_angles(i) - steer_offsets_(i) directly
  // because the steering encoder is already reported in base_link.
  Eigen::Matrix<double, 3, 2> wheel_positions_;

  // Per-wheel mechanical zero offset of the steering joint (rad).
  // Subtracted from msg->position[i] before forming the wheel direction.
  Eigen::Vector3d steer_offsets_;
  std::array<std::string, 3> steer_joint_names_;
  std::array<std::string, 3> drive_joint_names_;
  std::array<int, 3> steer_joint_indices_;
  std::array<int, 3> drive_joint_indices_;
  bool joint_name_mapping_requested_;
  bool joint_name_mapping_initialized_;
  bool joint_name_validation_disabled_logged_;

  // Soft weight on the perpendicular ("zero-slip") constraint in the
  // weighted-LSQ wheel-twist solve. 1.0 = same trust as the drive
  // constraint (pure ICR model); <1.0 lets vy/yaw float more freely.
  double slip_constraint_weight_;

  // Robot state
  double x_, y_, theta_;
  rclcpp::Time last_time_;
  bool time_initialized_;

  // Output rate gating. Pose is integrated at the full input rate (200 Hz
  // on this platform), but the published Odometry message + TF are limited
  // to publish_rate_ Hz. publish_rate_ <= 0 disables gating (publish on
  // every input frame). See publishOdometry().
  rclcpp::Time last_publish_time_;
  bool publish_time_initialized_;

  bool use_chassis_speed_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_speed_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Periodic INFO log gating (ms). 0 disables the throttled summary log; the
  // /synchro_odom/diagnostics topic continues to publish per-frame regardless.
  int diagnostics_log_period_ms_;
  int chassis_feedback_qos_depth_;

  // Calibration and Deadband
  double yaw_scale_factor_;
  double vx_scale_factor_;
  double vy_scale_factor_;
  double min_linear_velocity_;
  double min_angular_velocity_;

  // Per-wheel speed (m/s) below which we declare the body twist exactly
  // zero. Replaces the old "sum of |v_i| < threshold" heuristic, which
  // masked slow creep and slip on rotation entry.
  double per_wheel_speed_threshold_;

  // Upper bound on integration step (s). Protects against single-frame
  // gaps (USB hiccup, namespace re-bind) integrating stale velocity
  // straight into pose.
  double max_dt_;

  // Adaptive published-covariance gains. The wheel odom variance is
  //   var = base + slip_gain * residual^2 + omega_gain * |wz|
  // so the EKF down-weights wheel vx/vy when the LSQ residual flags slip
  // or when the body is in a rotation regime that exposes vx/vy to slip.
  double base_linear_variance_;
  double base_yaw_variance_;
  double slip_variance_gain_;
  double omega_variance_gain_;
  double yaw_slip_variance_gain_;

  // Drive-constraint residual norm from the latest LSQ solve (m/s).
  // Populated by computeRobotVelocity, consumed by publishOdometry.
  double last_drive_residual_norm_;
  bool last_solver_used_fallback_;
};

}  // namespace r1_nav_cpp

#endif  // R1_NAV_CPP__SYNCHRO_DRIVE_ODOMETRY_HPP_

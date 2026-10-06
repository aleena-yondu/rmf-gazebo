// Copyright 2026 R1 Navigation Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef R1_NAV_CPP__SYNCHRO_DRIVE_KINEMATICS_HPP_
#define R1_NAV_CPP__SYNCHRO_DRIVE_KINEMATICS_HPP_

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <string>

namespace r1_nav_cpp
{

struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct WheelGeometry
{
  Eigen::Matrix<double, 3, 2> positions{Eigen::Matrix<double, 3, 2>::Zero()};
  Eigen::Vector3d steer_offsets{Eigen::Vector3d::Zero()};
};

struct KinematicsResult
{
  Twist2D twist{};
  double drive_residual_norm{0.0};
  bool success{true};
  bool used_fallback_solver{false};
  std::string failure_reason{};
};

inline bool isFinite(double value)
{
  return std::isfinite(value);
}

inline bool isFinite(const Twist2D & twist)
{
  return isFinite(twist.vx) && isFinite(twist.vy) && isFinite(twist.wz);
}

inline bool isFinite(const Eigen::Vector3d & value)
{
  return value.allFinite();
}

inline Eigen::Vector3d toEigen(const Twist2D & twist)
{
  return Eigen::Vector3d(twist.vx, twist.vy, twist.wz);
}

inline Twist2D toTwist2D(const Eigen::Vector3d & twist)
{
  return Twist2D{twist(0), twist(1), twist(2)};
}

inline KinematicsResult solveBodyTwist(
  const WheelGeometry & geometry,
  const Eigen::Vector3d & steering_angles,
  const Eigen::Vector3d & wheel_velocities,
  double slip_weight,
  double per_wheel_zero_thresh)
{
  KinematicsResult result;

  if (!geometry.positions.allFinite() || !geometry.steer_offsets.allFinite() ||
    !steering_angles.allFinite() || !wheel_velocities.allFinite())
  {
    result.success = false;
    result.failure_reason = "non-finite kinematics input";
    return result;
  }

  const double max_wheel_speed = std::max(
    {std::abs(wheel_velocities(0)),
      std::abs(wheel_velocities(1)),
      std::abs(wheel_velocities(2))});
  if (max_wheel_speed < per_wheel_zero_thresh) {
    return result;
  }

  Eigen::Matrix<double, 6, 3> A;
  Eigen::Matrix<double, 6, 1> b;

  for (int i = 0; i < 3; ++i) {
    const double alpha = steering_angles(i) - geometry.steer_offsets(i);
    const double v_wheel = wheel_velocities(i);
    const double cos_a = std::cos(alpha);
    const double sin_a = std::sin(alpha);
    const double xi = geometry.positions(i, 0);
    const double yi = geometry.positions(i, 1);

    A(2 * i, 0) = cos_a;
    A(2 * i, 1) = sin_a;
    A(2 * i, 2) = -yi * cos_a + xi * sin_a;
    b(2 * i) = v_wheel;

    A(2 * i + 1, 0) = -sin_a;
    A(2 * i + 1, 1) = cos_a;
    A(2 * i + 1, 2) = yi * sin_a + xi * cos_a;
    b(2 * i + 1) = 0.0;
  }

  const double w = std::max(1e-6, slip_weight);
  Eigen::Matrix<double, 6, 1> w_diag;
  w_diag << 1.0, w, 1.0, w, 1.0, w;

  const Eigen::Matrix3d AtWA = A.transpose() * w_diag.asDiagonal() * A;
  const Eigen::Vector3d AtWb = A.transpose() * (w_diag.cwiseProduct(b));

  // Cheap rank screen. For our 6x3 overdetermined system, weighted_A has full
  // column rank iff det(A^T W A) != 0. Computing the 3x3 determinant avoids
  // building a full COD factorization on every wheel frame; the COD path
  // below only runs as a fallback when LDLT itself fails or returns
  // non-finite values (numerically ill-conditioned but not exactly singular
  // matrices).
  const double atwa_det = AtWA.determinant();
  if (!std::isfinite(atwa_det) || std::abs(atwa_det) < 1e-12) {
    result.success = false;
    result.failure_reason = "rank-deficient weighted LSQ system (det~0)";
    return result;
  }

  Eigen::Vector3d robot_velocity = Eigen::Vector3d::Zero();
  Eigen::LDLT<Eigen::Matrix3d> ldlt(AtWA);
  if (ldlt.info() == Eigen::Success) {
    robot_velocity = ldlt.solve(AtWb);
  }

  if (ldlt.info() != Eigen::Success || !robot_velocity.allFinite()) {
    // Fallback: build the weighted system and use a rank-revealing
    // decomposition. Only constructed when LDLT failed, so the 200 Hz
    // happy path stays at one LDLT solve per frame.
    Eigen::Matrix<double, 6, 3> weighted_A;
    Eigen::Matrix<double, 6, 1> weighted_b;
    for (int i = 0; i < 6; ++i) {
      const double sqrt_w = std::sqrt(w_diag(i));
      weighted_A.row(i) = sqrt_w * A.row(i);
      weighted_b(i) = sqrt_w * b(i);
    }
    Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix<double, 6, 3>> cod(weighted_A);
    if (cod.rank() < 3) {
      result.success = false;
      result.failure_reason = "rank-deficient weighted LSQ system (COD)";
      return result;
    }
    robot_velocity = cod.solve(weighted_b);
    result.used_fallback_solver = true;
  }

  if (!robot_velocity.allFinite()) {
    result.success = false;
    result.failure_reason = "non-finite weighted LSQ solution";
    return result;
  }

  const Eigen::Matrix<double, 6, 1> residual = b - A * robot_velocity;
  result.drive_residual_norm = std::sqrt(
    residual(0) * residual(0) +
    residual(2) * residual(2) +
    residual(4) * residual(4));
  result.twist = toTwist2D(robot_velocity);
  return result;
}

inline void integrateMidpoint(
  double & x, double & y, double & theta, const Twist2D & twist,
  double dt)
{
  const double half_theta = theta + twist.wz * dt * 0.5;
  const double cos_theta = std::cos(half_theta);
  const double sin_theta = std::sin(half_theta);

  const double vx_world = twist.vx * cos_theta - twist.vy * sin_theta;
  const double vy_world = twist.vx * sin_theta + twist.vy * cos_theta;

  x += vx_world * dt;
  y += vy_world * dt;
  theta += twist.wz * dt;
  theta = std::atan2(std::sin(theta), std::cos(theta));
}

}  // namespace r1_nav_cpp

#endif  // R1_NAV_CPP__SYNCHRO_DRIVE_KINEMATICS_HPP_

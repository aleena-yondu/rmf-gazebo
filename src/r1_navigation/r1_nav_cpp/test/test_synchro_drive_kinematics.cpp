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

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "r1_nav_cpp/synchro_drive_kinematics.hpp"

namespace
{

r1_nav_cpp::WheelGeometry defaultGeometry()
{
  r1_nav_cpp::WheelGeometry geometry;
  geometry.positions << 0.16897, 0.28,
    0.16897, -0.28,
    -0.32703, 0.0;
  geometry.steer_offsets = Eigen::Vector3d::Zero();
  return geometry;
}

void synthesizeWheelStates(
  const r1_nav_cpp::WheelGeometry & geometry,
  const r1_nav_cpp::Twist2D & twist,
  Eigen::Vector3d & steering_angles,
  Eigen::Vector3d & wheel_velocities)
{
  for (int i = 0; i < 3; ++i) {
    const double x = geometry.positions(i, 0);
    const double y = geometry.positions(i, 1);
    const double vx_i = twist.vx - twist.wz * y;
    const double vy_i = twist.vy + twist.wz * x;
    steering_angles(i) = std::atan2(vy_i, vx_i);
    wheel_velocities(i) = std::hypot(vx_i, vy_i);
  }
}

void expectTwistNear(const r1_nav_cpp::Twist2D & actual, const r1_nav_cpp::Twist2D & expected)
{
  constexpr double kTol = 1e-9;
  EXPECT_NEAR(actual.vx, expected.vx, kTol);
  EXPECT_NEAR(actual.vy, expected.vy, kTol);
  EXPECT_NEAR(actual.wz, expected.wz, kTol);
}

}  // namespace

TEST(SynchroDriveKinematics, StraightForward)
{
  const auto geometry = defaultGeometry();
  const Eigen::Vector3d steering = Eigen::Vector3d::Zero();
  const Eigen::Vector3d speeds = Eigen::Vector3d::Constant(0.5);

  const auto result = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);

  ASSERT_TRUE(result.success);
  expectTwistNear(result.twist, r1_nav_cpp::Twist2D{0.5, 0.0, 0.0});
  EXPECT_NEAR(result.drive_residual_norm, 0.0, 1e-12);
}

TEST(SynchroDriveKinematics, CrabLeft)
{
  const auto geometry = defaultGeometry();
  const Eigen::Vector3d steering = Eigen::Vector3d::Constant(M_PI_2);
  const Eigen::Vector3d speeds = Eigen::Vector3d::Constant(0.4);

  const auto result = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);

  ASSERT_TRUE(result.success);
  expectTwistNear(result.twist, r1_nav_cpp::Twist2D{0.0, 0.4, 0.0});
  EXPECT_NEAR(result.drive_residual_norm, 0.0, 1e-12);
}

TEST(SynchroDriveKinematics, PureRotation)
{
  const auto geometry = defaultGeometry();
  const r1_nav_cpp::Twist2D expected{0.0, 0.0, 0.8};
  Eigen::Vector3d steering;
  Eigen::Vector3d speeds;
  synthesizeWheelStates(geometry, expected, steering, speeds);

  const auto result = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);

  ASSERT_TRUE(result.success);
  expectTwistNear(result.twist, expected);
  EXPECT_NEAR(result.drive_residual_norm, 0.0, 1e-12);
}

TEST(SynchroDriveKinematics, ArbitraryTwistRecovery)
{
  const auto geometry = defaultGeometry();
  const r1_nav_cpp::Twist2D expected{0.35, -0.12, 0.45};
  Eigen::Vector3d steering;
  Eigen::Vector3d speeds;
  synthesizeWheelStates(geometry, expected, steering, speeds);

  const auto result = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);

  ASSERT_TRUE(result.success);
  expectTwistNear(result.twist, expected);
  EXPECT_NEAR(result.drive_residual_norm, 0.0, 1e-12);
}

TEST(SynchroDriveKinematics, SlipResidualIncreases)
{
  const auto geometry = defaultGeometry();
  const r1_nav_cpp::Twist2D expected{0.3, 0.05, -0.2};
  Eigen::Vector3d steering;
  Eigen::Vector3d speeds;
  synthesizeWheelStates(geometry, expected, steering, speeds);

  const auto clean = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);
  speeds(1) += 0.15;
  const auto slipped = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);

  ASSERT_TRUE(clean.success);
  ASSERT_TRUE(slipped.success);

  // Clean case should be essentially numerically zero.
  EXPECT_NEAR(clean.drive_residual_norm, 0.0, 1e-9);

  // Injecting 0.15 m/s of slip on one wheel produces an LSQ drive-row
  // residual on the order of |delta| * sqrt((N-3)/N) projected through the
  // weighted normal equations. For our 3-wheel geometry that empirically
  // lands near 0.034 m/s; we assert > 0.02 m/s to keep the test loose
  // enough to tolerate small kinematics tweaks but tight enough to catch a
  // regression that silently drops the residual to zero.
  EXPECT_GT(slipped.drive_residual_norm, 0.02);

  // And the difference must dominate the numerical noise of the clean
  // case (which is essentially zero).
  EXPECT_GT(slipped.drive_residual_norm - clean.drive_residual_norm, 0.02);
}

TEST(SynchroDriveKinematics, NonFiniteInputRejected)
{
  const auto geometry = defaultGeometry();
  Eigen::Vector3d steering = Eigen::Vector3d::Zero();
  Eigen::Vector3d speeds = Eigen::Vector3d::Constant(0.4);

  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();

  // NaN in steering angle.
  steering(1) = kNaN;
  auto res = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);
  EXPECT_FALSE(res.success);
  EXPECT_EQ(res.twist.vx, 0.0);
  EXPECT_EQ(res.twist.vy, 0.0);
  EXPECT_EQ(res.twist.wz, 0.0);

  // Inf in wheel velocity.
  steering = Eigen::Vector3d::Zero();
  speeds(2) = kInf;
  res = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);
  EXPECT_FALSE(res.success);
  EXPECT_EQ(res.twist.vx, 0.0);
  EXPECT_EQ(res.twist.vy, 0.0);
  EXPECT_EQ(res.twist.wz, 0.0);

  // NaN in geometry must also fail safely.
  r1_nav_cpp::WheelGeometry bad_geometry = geometry;
  bad_geometry.positions(0, 0) = kNaN;
  speeds = Eigen::Vector3d::Constant(0.4);
  res = r1_nav_cpp::solveBodyTwist(bad_geometry, Eigen::Vector3d::Zero(), speeds, 0.2, 0.005);
  EXPECT_FALSE(res.success);
}

TEST(SynchroDriveKinematics, StationaryGate)
{
  const auto geometry = defaultGeometry();
  const Eigen::Vector3d steering = Eigen::Vector3d::Constant(0.7);
  const Eigen::Vector3d speeds = Eigen::Vector3d::Constant(0.001);

  const auto result = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);

  ASSERT_TRUE(result.success);
  expectTwistNear(result.twist, r1_nav_cpp::Twist2D{0.0, 0.0, 0.0});
  EXPECT_NEAR(result.drive_residual_norm, 0.0, 1e-12);
}

TEST(SynchroDriveKinematics, DegenerateGeometryFailsSafely)
{
  r1_nav_cpp::WheelGeometry geometry;
  geometry.positions.setZero();
  geometry.steer_offsets.setZero();
  const Eigen::Vector3d steering = Eigen::Vector3d::Zero();
  const Eigen::Vector3d speeds = Eigen::Vector3d::Constant(0.2);

  const auto result = r1_nav_cpp::solveBodyTwist(geometry, steering, speeds, 0.2, 0.005);

  EXPECT_FALSE(result.success);
  expectTwistNear(result.twist, r1_nav_cpp::Twist2D{0.0, 0.0, 0.0});
  EXPECT_NEAR(result.drive_residual_norm, 0.0, 1e-12);
}

TEST(SynchroDriveKinematics, IntegrateMidpoint)
{
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  const r1_nav_cpp::Twist2D twist{1.0, 0.0, M_PI};
  const double dt = 0.1;

  r1_nav_cpp::integrateMidpoint(x, y, theta, twist, dt);

  EXPECT_NEAR(x, std::cos(M_PI * dt * 0.5) * dt, 1e-12);
  EXPECT_NEAR(y, std::sin(M_PI * dt * 0.5) * dt, 1e-12);
  EXPECT_NEAR(theta, M_PI * dt, 1e-12);
}

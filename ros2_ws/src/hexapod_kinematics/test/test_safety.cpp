// The publish gate. No servo is commanded from this file.
// A joint past ±120° is joint_limit. A 1 rad jump is rate_limit.
// A step of a few hundredths inside the window is allow.
// A NaN is non_finite. A solver near_singular is kept.

#include "hexapod_kinematics/geometry.hpp"
#include "hexapod_kinematics/safety.hpp"

#include <array>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace hexapod_kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfWindowRad = 240.0 * kPi / 180.0 / 2.0;

constexpr std::array<LegId, kLegCount> kOrder = {
    LegId::Lf, LegId::Lm, LegId::Lr, LegId::Rr, LegId::Rm, LegId::Rf};

// Directions copied from config.py SERVOS ids 1..18. The test locks
// the table the gate reads, so a quieter window cannot sneak in.
constexpr int kDirection[kJointCount] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, -1, 1, 1, -1, 1, 1,
};

Vec3 default_foot(LegId id) {
  constexpr double kX1 = 93.60;
  constexpr double kY1 = 50.805;
  constexpr double kY2 = 73.535;
  constexpr double kIx = 70.0;
  constexpr double kIy = 110.0;
  constexpr double kHeight = 70.0;
  switch (id) {
    case LegId::Lf:
      return {kIx + kX1, kIy + kY1 - 20.0, -kHeight};
    case LegId::Lm:
      return {0.0, kIy + kY2, -kHeight};
    case LegId::Lr:
      return {-kIx - kX1, kIy + kY1 - 20.0, -kHeight};
    case LegId::Rr:
      return {-kIx - kX1, -(kIy + kY1 - 20.0), -kHeight};
    case LegId::Rm:
      return {0.0, -(kIy + kY2), -kHeight};
    case LegId::Rf:
      return {kIx + kX1, -(kIy + kY1 - 20.0), -kHeight};
  }
  return {};
}

std::array<Vec3, kLegCount> default_feet() {
  std::array<Vec3, kLegCount> feet{};
  for (LegId id : kOrder) {
    feet[static_cast<std::size_t>(leg_index(id))] = default_foot(id);
  }
  return feet;
}

std::array<JointAngles, kLegCount> stand_angles() {
  const PoseResult solved = solve_pose(default_feet(), max_step_rad());
  EXPECT_TRUE(solved.ok);
  return solved.angles;
}

void set_joint(std::array<JointAngles, kLegCount>& pose, int joint_id, double radians) {
  JointAngles& leg = pose[static_cast<std::size_t>((joint_id - 1) / 3)];
  switch ((joint_id - 1) % 3) {
    case 0:
      leg.coxa_rad = radians;
      break;
    case 1:
      leg.femur_rad = radians;
      break;
    default:
      leg.tibia_rad = radians;
      break;
  }
}

PoseResult command_of(const std::array<JointAngles, kLegCount>& angles) {
  PoseResult command;
  command.ok = true;
  command.reason = IkReason::None;
  command.angles = angles;
  return command;
}

TEST(Safety, ServoTableMatchesTheDriverLegRows) {
  // 240° travel, offset 0, and the per-joint direction. Head joints
  // are not rows of this table.
  EXPECT_EQ(leg_servo_or_null(0), nullptr);
  EXPECT_EQ(leg_servo_or_null(19), nullptr);
  for (int joint_id = 1; joint_id <= kJointCount; ++joint_id) {
    const ServoLimit* servo = leg_servo_or_null(joint_id);
    ASSERT_NE(servo, nullptr) << joint_id;
    EXPECT_EQ(servo->direction, kDirection[joint_id - 1]) << joint_id;
    EXPECT_DOUBLE_EQ(servo->offset_rad, 0.0) << joint_id;
    EXPECT_NEAR(servo->max_radians, 240.0 * kPi / 180.0, 1e-12) << joint_id;
  }
  EXPECT_STREQ(leg_servo_or_null(1)->name, "coxa_joint_LF");
  EXPECT_STREQ(leg_servo_or_null(11)->name, "femur_joint_RR");
  EXPECT_STREQ(leg_servo_or_null(18)->name, "tibia_joint_RF");
  // The measured gait-5 peak times 1.5. safety.cpp names the command.
  EXPECT_NEAR(max_step_rad(), 0.17622116866774196 * 1.5, 0.0);
}

TEST(Safety, AJointPastTheHalfWindowIsJointLimit) {
  // 2.2 rad is past ±120° (about ±2.094 rad). The previous pose is the
  // stand, so this is also a large step. The window is checked first,
  // and the decision names joint 8, the left-rear femur.
  const std::array<JointAngles, kLegCount> previous = stand_angles();
  std::array<JointAngles, kLegCount> now = previous;
  set_joint(now, 8, 2.2);

  const SafetyDecision decision = gate(command_of(now), &previous);

  EXPECT_FALSE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::JointLimit);
  EXPECT_EQ(decision.joint_id, 8);
  EXPECT_DOUBLE_EQ(decision.q_now_rad, 2.2);
}

TEST(Safety, ExactlyTheHalfWindowIsInside) {
  // The driver refuses when the absolute value is greater than half
  // the travel. A joint sitting on ±120°, with no step from the last
  // sent pose, is still allowed. The angle is not trimmed.
  std::array<JointAngles, kLegCount> previous = stand_angles();
  set_joint(previous, 8, kHalfWindowRad);
  const std::array<JointAngles, kLegCount> now = previous;

  const SafetyDecision positive = gate(command_of(now), &previous);
  EXPECT_TRUE(positive.allow);
  EXPECT_EQ(positive.reason, IkReason::None);

  std::array<JointAngles, kLegCount> negative_previous = stand_angles();
  set_joint(negative_previous, 8, -kHalfWindowRad);
  const SafetyDecision negative = gate(command_of(negative_previous), &negative_previous);
  EXPECT_TRUE(negative.allow);
  EXPECT_EQ(negative.reason, IkReason::None);
}

TEST(Safety, AOneRadianJumpIsRateLimit) {
  // Inside the window, and a radian away from the last sent pose.
  // That is the knee flip this check exists to refuse. Joint 8 again,
  // so a pass does not always mean "the first joint".
  const std::array<JointAngles, kLegCount> previous = stand_angles();
  std::array<JointAngles, kLegCount> now = previous;
  const double jumped = previous[static_cast<std::size_t>(leg_index(LegId::Lr))].femur_rad + 1.0;
  set_joint(now, 8, jumped);

  const SafetyDecision decision = gate(command_of(now), &previous);

  EXPECT_FALSE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::RateLimit);
  EXPECT_EQ(decision.joint_id, 8);
  EXPECT_DOUBLE_EQ(decision.q_now_rad, jumped);
  EXPECT_DOUBLE_EQ(decision.q_previous_rad, jumped - 1.0);
  EXPECT_LT(std::abs(jumped), kHalfWindowRad);
}

TEST(Safety, AStepOfAFewHundredthsIsAllowed) {
  const std::array<JointAngles, kLegCount> previous = stand_angles();
  std::array<JointAngles, kLegCount> now = previous;
  for (int joint_id = 1; joint_id <= kJointCount; ++joint_id) {
    const int leg = (joint_id - 1) / 3;
    const int hinge = (joint_id - 1) % 3;
    JointAngles& angles = now[static_cast<std::size_t>(leg)];
    double* slot = hinge == 0 ? &angles.coxa_rad : hinge == 1 ? &angles.femur_rad : &angles.tibia_rad;
    *slot += 0.02;
  }

  const SafetyDecision decision = gate(command_of(now), &previous);

  EXPECT_TRUE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::None);
  EXPECT_EQ(decision.joint_id, 0);
}

TEST(Safety, AStepEqualToTheLimitIsAllowed) {
  // The contract is |Δq| ≤ max_step_rad. The equal case is allowed.
  // Both ends are written down. Adding the limit to the stand femur
  // and subtracting that femur back is a slightly larger double, and
  // that rounding is not what this check is about.
  std::array<JointAngles, kLegCount> previous = stand_angles();
  std::array<JointAngles, kLegCount> now = previous;
  set_joint(previous, 2, 0.0);
  set_joint(now, 2, max_step_rad());

  const SafetyDecision decision = gate(command_of(now), &previous);

  EXPECT_TRUE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::None);
}

TEST(Safety, ANanIsNonFiniteBeforeTheWindow) {
  const std::array<JointAngles, kLegCount> previous = stand_angles();
  std::array<JointAngles, kLegCount> now = previous;
  set_joint(now, 3, std::numeric_limits<double>::quiet_NaN());

  const SafetyDecision decision = gate(command_of(now), &previous);

  EXPECT_FALSE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::NonFinite);
  EXPECT_EQ(decision.joint_id, 3);
  EXPECT_TRUE(std::isnan(decision.q_now_rad));
}

TEST(Safety, SolverNearSingularIsKept) {
  // 0.02 mm short of straight, on leg 6. A 0.5 mm slip there moves
  // the knee by about 0.37 rad, past the measured limit, so solve_pose
  // refuses. A foot 0.2 mm short asks for about 0.116 rad and is
  // allowed at this limit. The gate repeats the solver's reason.
  const LinkLengths& links = link_lengths();
  const Hip& mount = hip(LegId::Rf);
  const double reach = links.coxa_mm + links.femur_mm + links.tibia_mm - 0.02;
  std::array<Vec3, kLegCount> feet = default_feet();
  feet[static_cast<std::size_t>(leg_index(LegId::Rf))] = {
      mount.x_mm + reach * std::cos(mount.mount_yaw_rad),
      mount.y_mm + reach * std::sin(mount.mount_yaw_rad),
      links.hip_z_mm + links.coxa_femur_z_mm};
  const PoseResult command = solve_pose(feet, max_step_rad());
  ASSERT_FALSE(command.ok);
  ASSERT_EQ(command.reason, IkReason::NearSingular);

  const std::array<JointAngles, kLegCount> previous = stand_angles();
  const SafetyDecision decision = gate(command, &previous);

  EXPECT_FALSE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::NearSingular);
  EXPECT_EQ(decision.joint_id, 0);
}

TEST(Safety, NoPreviousSentPoseIsNoReference) {
  // A solved stand, and no pose this gate has sent. The window is fine.
  // The missing reference is the refusal.
  const PoseResult command = solve_pose(default_feet(), max_step_rad());
  ASSERT_TRUE(command.ok);

  const SafetyDecision decision = gate(command, nullptr);

  EXPECT_FALSE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::NoReference);
  EXPECT_EQ(decision.joint_id, 0);
}

TEST(Safety, AWindowFailureWinsOverAMissingReference) {
  // Checked before the reference. A joint at 2.2 rad with no previous
  // pose is joint_limit.
  std::array<JointAngles, kLegCount> now{};
  set_joint(now, 8, 2.2);

  const SafetyDecision decision = gate(command_of(now), nullptr);

  EXPECT_FALSE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::JointLimit);
  EXPECT_EQ(decision.joint_id, 8);
}

TEST(Safety, ANonFiniteAngleWinsOverASolverRefusal) {
  // Finite is the first check. An unreachable pose that also carries
  // a NaN is non_finite, and the joint is named.
  PoseResult command;
  command.ok = false;
  command.reason = IkReason::Unreachable;
  command.leg = LegId::Lf;
  set_joint(command.angles, 1, std::numeric_limits<double>::infinity());

  const SafetyDecision decision = gate(command, nullptr);

  EXPECT_FALSE(decision.allow);
  EXPECT_EQ(decision.reason, IkReason::NonFinite);
  EXPECT_EQ(decision.joint_id, 1);
}

}  // namespace
}  // namespace hexapod_kinematics

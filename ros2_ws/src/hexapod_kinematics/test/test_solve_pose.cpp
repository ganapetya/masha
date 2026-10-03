// solve_pose is six calls to solve_leg, in leg order, and one decision.
// A default stance must match the one-leg answers. The first bad foot
// fails the whole pose and names that leg. Nothing here raises.

#include "hexapod_kinematics/geometry.hpp"
#include "hexapod_kinematics/leg_ik.hpp"
#include "hexapod_kinematics/safety.hpp"

#include <array>
#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace hexapod_kinematics {
namespace {

constexpr std::array<LegId, kLegCount> kOrder = {
    LegId::Lf, LegId::Lm, LegId::Lr, LegId::Rr, LegId::Rm, LegId::Rf};

// build_in_pose.py DEFAULT_POSE. Same expressions as test_leg_ik.
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

std::array<Vec3, kLegCount> default_pose() {
  std::array<Vec3, kLegCount> feet{};
  for (LegId id : kOrder) {
    feet[static_cast<std::size_t>(leg_index(id))] = default_foot(id);
  }
  return feet;
}

void expect_angles_match(const JointAngles& got, const JointAngles& want, LegId id) {
  EXPECT_DOUBLE_EQ(got.coxa_rad, want.coxa_rad) << static_cast<int>(id);
  EXPECT_DOUBLE_EQ(got.femur_rad, want.femur_rad) << static_cast<int>(id);
  EXPECT_DOUBLE_EQ(got.tibia_rad, want.tibia_rad) << static_cast<int>(id);
}

void expect_angles_zero(const JointAngles& got, LegId id) {
  EXPECT_DOUBLE_EQ(got.coxa_rad, 0.0) << static_cast<int>(id);
  EXPECT_DOUBLE_EQ(got.femur_rad, 0.0) << static_cast<int>(id);
  EXPECT_DOUBLE_EQ(got.tibia_rad, 0.0) << static_cast<int>(id);
}

TEST(SolvePose, DefaultPoseMatchesEachSolveLeg) {
  // No new triangle. Each slot is the one-leg answer for that foot.
  const std::array<Vec3, kLegCount> feet = default_pose();
  const PoseResult pose = solve_pose(feet, max_step_rad());

  ASSERT_TRUE(pose.ok);
  EXPECT_EQ(pose.reason, IkReason::None);
  for (LegId id : kOrder) {
    const IkResult one = solve_leg(id, feet[static_cast<std::size_t>(leg_index(id))],
                                   max_step_rad());
    ASSERT_TRUE(one.ok) << static_cast<int>(id);
    expect_angles_match(pose.angles[static_cast<std::size_t>(leg_index(id))], one.angles, id);
  }
}

TEST(SolvePose, FirstBadLegFailsTheWholePose) {
  // Leg 4 is unreachable, and leg 6 is unreachable too. The result
  // names leg 4, because the loop counts from leg 1. Legs 5 and 6
  // are not solved. Legs 1..3 keep the angles solve_leg returned,
  // and the pose is still a refusal.
  std::array<Vec3, kLegCount> feet = default_pose();
  feet[static_cast<std::size_t>(leg_index(LegId::Rr))] = {0.0, 0.0, 500.0};
  feet[static_cast<std::size_t>(leg_index(LegId::Rf))] = {0.0, 0.0, 500.0};

  const PoseResult pose = solve_pose(feet, max_step_rad());

  EXPECT_FALSE(pose.ok);
  EXPECT_EQ(pose.reason, IkReason::Unreachable);
  EXPECT_EQ(pose.leg, LegId::Rr);

  for (LegId id : {LegId::Lf, LegId::Lm, LegId::Lr}) {
    const IkResult one = solve_leg(id, feet[static_cast<std::size_t>(leg_index(id))],
                                   max_step_rad());
    ASSERT_TRUE(one.ok) << static_cast<int>(id);
    expect_angles_match(pose.angles[static_cast<std::size_t>(leg_index(id))], one.angles, id);
  }
  expect_angles_zero(pose.angles[static_cast<std::size_t>(leg_index(LegId::Rr))], LegId::Rr);
  expect_angles_zero(pose.angles[static_cast<std::size_t>(leg_index(LegId::Rm))], LegId::Rm);
  expect_angles_zero(pose.angles[static_cast<std::size_t>(leg_index(LegId::Rf))], LegId::Rf);
}

TEST(SolvePose, AStraightFootIsNearSingularAndNamesThatLeg) {
  // 0.02 mm short of straight, level with the femur hinge. The same
  // point the planar test refuses. A foot 0.2 mm short is under the
  // measured limit. Put it on leg 6 so five good legs are stored and
  // the pose still comes back near_singular.
  const LinkLengths& links = link_lengths();
  const Hip& mount = hip(LegId::Rf);
  const double reach = links.coxa_mm + links.femur_mm + links.tibia_mm - 0.02;
  std::array<Vec3, kLegCount> feet = default_pose();
  feet[static_cast<std::size_t>(leg_index(LegId::Rf))] = {
      mount.x_mm + reach * std::cos(mount.mount_yaw_rad),
      mount.y_mm + reach * std::sin(mount.mount_yaw_rad),
      links.hip_z_mm + links.coxa_femur_z_mm};

  const PoseResult pose = solve_pose(feet, max_step_rad());

  EXPECT_FALSE(pose.ok);
  EXPECT_EQ(pose.reason, IkReason::NearSingular);
  EXPECT_EQ(pose.leg, LegId::Rf);
  expect_angles_zero(pose.angles[static_cast<std::size_t>(leg_index(LegId::Rf))], LegId::Rf);
  for (LegId id : {LegId::Lf, LegId::Lm, LegId::Lr, LegId::Rr, LegId::Rm}) {
    const IkResult one = solve_leg(id, default_foot(id), max_step_rad());
    ASSERT_TRUE(one.ok) << static_cast<int>(id);
    expect_angles_match(pose.angles[static_cast<std::size_t>(leg_index(id))], one.angles, id);
  }
}

TEST(SolvePose, ANonFiniteFootNamesThatLegAndDoesNotThrow) {
  // Leg 2's y is not a number. Legs after it stay 0. Leg 1 was a
  // normal stand foot, so its angles are present and still not a
  // command, because the pose is refused.
  std::array<Vec3, kLegCount> feet = default_pose();
  feet[static_cast<std::size_t>(leg_index(LegId::Lm))].y_mm =
      std::numeric_limits<double>::quiet_NaN();

  const PoseResult pose = solve_pose(feet, max_step_rad());

  EXPECT_FALSE(pose.ok);
  EXPECT_EQ(pose.reason, IkReason::NonFinite);
  EXPECT_EQ(pose.leg, LegId::Lm);

  const IkResult first = solve_leg(LegId::Lf, default_foot(LegId::Lf), max_step_rad());
  ASSERT_TRUE(first.ok);
  expect_angles_match(pose.angles[static_cast<std::size_t>(leg_index(LegId::Lf))], first.angles,
                      LegId::Lf);
  for (LegId id : {LegId::Lm, LegId::Lr, LegId::Rr, LegId::Rm, LegId::Rf}) {
    expect_angles_zero(pose.angles[static_cast<std::size_t>(leg_index(id))], id);
  }
}

}  // namespace
}  // namespace hexapod_kinematics

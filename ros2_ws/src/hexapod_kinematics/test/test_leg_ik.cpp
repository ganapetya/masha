// The stand contract: solve_leg on all six DEFAULT_POSE feet matches
// the vendor radians within 0.005. The targets are what
// set_leg_position returns for that pose. Degrees are written beside
// them. A shared femur/tibia zero is what geometry.cpp stores. This
// test is what would force a per-leg table if that zero could not pass.

#include "hexapod_kinematics/leg_ik.hpp"
#include "hexapod_kinematics/safety.hpp"

#include "gtest/gtest.h"

namespace hexapod_kinematics {
namespace {

constexpr double kContractRad = 0.005;

// build_in_pose.py DEFAULT_POSE. Same expressions as test_forward.
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

struct StandTarget {
  const char* name;
  LegId leg;
  double coxa_rad;
  double femur_rad;
  double tibia_rad;
};

// Readings from set_leg_position on those six feet.
// Corner coxa is the plan's 0.1207 rad (6.91 deg), written here with
// the extra digits the call returns. PLAN.md's 0.7555 and -0.650 are
// the corner femur and tibia (43.29 deg and -37.24 deg).
//
// Middle legs are a different triangle, about 4 mm closer to the hip
// in the plane. Their vendor femur is 0.761599 rad (43.64 deg) and
// their vendor tibia is -0.687032 rad (-39.36 deg). The right side
// negates the coxa and keeps the femur and tibia of its left mirror.
constexpr StandTarget kStand[] = {
    {"LF", LegId::Lf, 0.1206814507, 0.7554684412, -0.6500234354},   // 6.915, 43.285, -37.244 deg
    {"LM", LegId::Lm, 0.0, 0.7615987475, -0.6870318910},             // 0, 43.636, -39.364 deg
    {"LR", LegId::Lr, -0.1206814507, 0.7554684412, -0.6500234354},  // -6.915, 43.285, -37.244 deg
    {"RR", LegId::Rr, 0.1206814507, 0.7554684412, -0.6500234354},   // 6.915, 43.285, -37.244 deg
    {"RM", LegId::Rm, 0.0, 0.7615987475, -0.6870318910},             // 0, 43.636, -39.364 deg
    {"RF", LegId::Rf, -0.1206814507, 0.7554684412, -0.6500234354},  // -6.915, 43.285, -37.244 deg
};

TEST(LegIk, StandContractWithinHalfAHundredthOfARadian) {
  for (const StandTarget& target : kStand) {
    const IkResult solved = solve_leg(target.leg, default_foot(target.leg), max_step_rad());
    ASSERT_TRUE(solved.ok) << target.name;
    EXPECT_EQ(solved.reason, IkReason::None) << target.name;
    EXPECT_NEAR(solved.angles.coxa_rad, target.coxa_rad, kContractRad) << target.name;
    EXPECT_NEAR(solved.angles.femur_rad, target.femur_rad, kContractRad) << target.name;
    EXPECT_NEAR(solved.angles.tibia_rad, target.tibia_rad, kContractRad) << target.name;
  }
}

TEST(LegIk, AFarFootIsUnreachableAndDoesNotThrow) {
  const Vec3 far{163.6, 140.8, 500.0};
  const IkResult solved = solve_leg(LegId::Lf, far, max_step_rad());
  EXPECT_FALSE(solved.ok);
  EXPECT_EQ(solved.reason, IkReason::Unreachable);
  EXPECT_DOUBLE_EQ(solved.angles.femur_rad, 0.0);
  EXPECT_DOUBLE_EQ(solved.angles.tibia_rad, 0.0);
}

}  // namespace
}  // namespace hexapod_kinematics

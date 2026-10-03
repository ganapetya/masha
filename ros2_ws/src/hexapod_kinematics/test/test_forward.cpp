// The forward walk, and the round trip forward(solve_leg(p)).
// A grid around DEFAULT_POSE must come back within 0.5 mm.
// Points the solver marks unreachable or near_singular are skipped.
// The stand contract lives in test_leg_ik.

#include "hexapod_kinematics/forward.hpp"
#include "hexapod_kinematics/geometry.hpp"
#include "hexapod_kinematics/leg_ik.hpp"
#include "hexapod_kinematics/safety.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace hexapod_kinematics {
namespace {

// Same step limit the safety gate holds. Around the stand it does not
// refuse a foot.

// build_in_pose.py DEFAULT_POSE. INITIAL_X 70, INITIAL_Y 110, height 70.
// X1 93.60, Y1 50.805, Y2 73.535.
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

double distance_mm(const Vec3& a, const Vec3& b) {
  return std::hypot(a.x_mm - b.x_mm, a.y_mm - b.y_mm, a.z_mm - b.z_mm);
}

TEST(Forward, StraightLegLandsOnTheMountRay) {
  // Servo angles equal to the joint zero, so the drawing angles are
  // 0, 0, 0. The coxa points along the left-front mount (45 degrees).
  // Femur and tibia lie straight out on that same heading.
  //   reach = 45 + 77.1 + 115.6 = 237.7 mm
  //   x = 93.60 + 237.7 * cos(45°)
  //   y = 50.805 + 237.7 * sin(45°)
  //   z = hip_z + coxa_femur_z = 1.13
  // The vertical 1.13 is coxa_femur_z. A straight leg does not cancel it.
  const JointZero& zero = joint_zero();
  const JointAngles straight{zero.coxa_rad, zero.femur_rad, zero.tibia_rad};
  const Vec3 foot = forward_foot(LegId::Lf, straight);

  EXPECT_NEAR(foot.x_mm, 261.67928188804234, 1e-9);
  EXPECT_NEAR(foot.y_mm, 218.88428188804232, 1e-9);
  EXPECT_NEAR(foot.z_mm, 1.13, 1e-12);
}

TEST(Forward, RoundTripAroundDefaultPose) {
  // ±40 mm in x and y, z from -100 to -40, six legs. The exact stand
  // feet (z = -70) are in this set. A skipped point is one the solver
  // refused. Every point it accepts must come back within 0.5 mm.
  const double kOffsets[] = {-40.0, -20.0, 0.0, 20.0, 40.0};
  const double kHeights[] = {-100.0, -80.0, -70.0, -60.0, -40.0};
  const LegId kLegs[] = {LegId::Lf, LegId::Lm, LegId::Lr,
                         LegId::Rr, LegId::Rm, LegId::Rf};
  int solved = 0;
  int skipped = 0;

  for (LegId leg : kLegs) {
    const Vec3 home = default_foot(leg);
    for (double dx : kOffsets) {
      for (double dy : kOffsets) {
        for (double z : kHeights) {
          const Vec3 foot{home.x_mm + dx, home.y_mm + dy, z};
          const IkResult answer = solve_leg(leg, foot, max_step_rad());
          if (!answer.ok) {
            EXPECT_TRUE(answer.reason == IkReason::Unreachable ||
                        answer.reason == IkReason::NearSingular);
            ++skipped;
            continue;
          }
          const Vec3 back = forward_foot(leg, answer.angles);
          const double err = distance_mm(back, foot);
          EXPECT_LE(err, 0.5) << "leg " << static_cast<int>(leg) << " foot ("
                              << foot.x_mm << ", " << foot.y_mm << ", " << foot.z_mm
                              << ") err " << err;
          ++solved;
        }
      }
    }
  }

  // 6 * 5 * 5 * 5 = 750. This neighborhood is bent for the current
  // lengths, so the solver accepts every sample.
  EXPECT_EQ(skipped, 0);
  EXPECT_EQ(solved, 750);
}

}  // namespace
}  // namespace hexapod_kinematics

// Locks the knee triangle to Figure 6.1 in PLAN.md.
// One hand-built triangle, the ring, the straight-leg edge, the
// coxa_femur_z shift, and which fold a standing Masha keeps.
// Forward kinematics and the 0.005 rad stand contract are later files.

#include "hexapod_kinematics/planar_leg.hpp"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace hexapod_kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Stand-in for the measured max_step_rad. safety.cpp does not hold that
// constant yet. With the current lengths, a 0.5 mm error at the
// left-front stand asks the knee for about 0.0066 rad. The same error
// 0.2 mm inside full stretch asks for about 0.116 rad. 0.05 rad sits
// between them, so the stand passes and that near-straight foot fails.
constexpr double kStandInMaxStepRad = 0.05;

LinkLengths equal_links() {
  LinkLengths links;
  links.femur_mm = 100.0;
  links.tibia_mm = 100.0;
  return links;
}

TEST(Planar, HandBuiltEquilateralMatchesTheBook) {
  // L1 = L2 = r = 100 mm, foot on the book's +x axis. All three sides
  // are equal, so alpha = beta = 60 degrees and gamma = 0.
  //   righty: femur = -60 deg, tibia = +120 deg
  //   lefty:  femur = +60 deg, tibia = -120 deg
  // A zero offset, so the commanded fold is the book fold. The robot's
  // fitted offset is the stand contract, not this triangle.
  const PlaneTarget foot{100.0, 0.0};
  const JointZero zero{};
  const PlanarResult solved =
      solve_planar(foot, equal_links(), kStandInMaxStepRad, zero);

  ASSERT_TRUE(solved.ok);
  EXPECT_EQ(solved.reason, IkReason::None);
  EXPECT_NEAR(solved.alpha_rad, kPi / 3.0, 1e-12);
  EXPECT_NEAR(solved.beta_rad, kPi / 3.0, 1e-12);
  EXPECT_NEAR(solved.gamma_rad, 0.0, 1e-12);

  EXPECT_NEAR(solved.righty.femur_rad, -kPi / 3.0, 1e-12);
  EXPECT_NEAR(solved.righty.tibia_rad, 2.0 * kPi / 3.0, 1e-12);
  EXPECT_NEAR(solved.lefty.femur_rad, kPi / 3.0, 1e-12);
  EXPECT_NEAR(solved.lefty.tibia_rad, -2.0 * kPi / 3.0, 1e-12);

  EXPECT_EQ(solved.standing, KneeBranch::Lefty);
  EXPECT_DOUBLE_EQ(solved.standing_angles.femur_rad, solved.lefty.femur_rad);
  EXPECT_DOUBLE_EQ(solved.standing_angles.tibia_rad, solved.lefty.tibia_rad);
}

TEST(Planar, JointZeroIsAddedAfterTheBookAngles) {
  // A non-zero offset must move only the commanded fold. The book
  // angles stay on the law of cosines. The coxa component is not a
  // femur or tibia angle.
  JointZero zero;
  zero.coxa_rad = 1.0;
  zero.femur_rad = 0.2;
  zero.tibia_rad = -0.3;
  const PlaneTarget foot{100.0, 0.0};
  const PlanarResult solved =
      solve_planar(foot, equal_links(), kStandInMaxStepRad, zero);

  ASSERT_TRUE(solved.ok);
  EXPECT_NEAR(solved.lefty.femur_rad, kPi / 3.0, 1e-12);
  EXPECT_NEAR(solved.lefty.tibia_rad, -2.0 * kPi / 3.0, 1e-12);
  EXPECT_NEAR(solved.righty.femur_rad, -kPi / 3.0, 1e-12);
  EXPECT_NEAR(solved.standing_angles.femur_rad, solved.lefty.femur_rad + 0.2, 1e-12);
  EXPECT_NEAR(solved.standing_angles.tibia_rad, solved.lefty.tibia_rad - 0.3, 1e-12);
}

TEST(Planar, OutsideTheOuterRingIsUnreachable) {
  // 10 mm past L1 + L2, straight out. No triangle, and no clamped angle
  // written along this direction. A clamp would put the femur near 0.
  const LinkLengths& links = link_lengths();
  const PlaneTarget far{links.femur_mm + links.tibia_mm + 10.0, 0.0};
  const PlanarResult solved =
      solve_planar(far, links, kStandInMaxStepRad, joint_zero());

  EXPECT_FALSE(solved.ok);
  EXPECT_EQ(solved.reason, IkReason::Unreachable);
  EXPECT_DOUBLE_EQ(solved.lefty.femur_rad, 0.0);
  EXPECT_DOUBLE_EQ(solved.righty.femur_rad, 0.0);
  EXPECT_DOUBLE_EQ(solved.standing_angles.femur_rad, 0.0);
}

TEST(Planar, InsideTheInnerHoleIsUnreachable) {
  // Closer than |L1 - L2|. Same refusal as the outer radius: the
  // cosine is outside [-1, 1].
  const LinkLengths& links = link_lengths();
  const double inner = std::fabs(links.femur_mm - links.tibia_mm);
  const PlaneTarget hole{inner * 0.25, 0.0};
  const PlanarResult solved =
      solve_planar(hole, links, kStandInMaxStepRad, joint_zero());

  EXPECT_FALSE(solved.ok);
  EXPECT_EQ(solved.reason, IkReason::Unreachable);
  EXPECT_DOUBLE_EQ(solved.standing_angles.tibia_rad, 0.0);
}

TEST(Planar, JustInsideTheOuterRingIsNearSingular) {
  // 0.2 mm short of L1 + L2, level with the femur hinge. The cosine is
  // still inside [-1, 1], so an angle exists. A 0.5 mm slip would move
  // the knee by more than the stand-in limit, and the two folds are
  // already close to the same straight line.
  const LinkLengths& links = link_lengths();
  const PlaneTarget almost{links.femur_mm + links.tibia_mm - 0.2, 0.0};
  const PlanarResult solved =
      solve_planar(almost, links, kStandInMaxStepRad, joint_zero());

  EXPECT_FALSE(solved.ok);
  EXPECT_EQ(solved.reason, IkReason::NearSingular);
  EXPECT_LT(std::fabs(solved.lefty.tibia_rad), 0.15);
  EXPECT_LT(std::fabs(solved.righty.tibia_rad), 0.15);
  EXPECT_NEAR(solved.lefty.tibia_rad, -solved.righty.tibia_rad, 1e-12);
}

TEST(Planar, StandFootKeepsLeftyAndClearsTheSoftEdge) {
  // DEFAULT_POSE left-front foot. The knee is bent, so the same stand-in
  // limit that refused the near-straight point allows this one.
  // Lefty is femur-up and tibia-down. Righty is the other fold.
  const Vec3 foot{163.6, 140.8, -70.0};
  const PlaneTarget target = plane_target(hip(LegId::Lf), foot, link_lengths());
  const PlanarResult solved =
      solve_planar(target, link_lengths(), kStandInMaxStepRad, joint_zero());

  ASSERT_TRUE(solved.ok);
  EXPECT_EQ(solved.reason, IkReason::None);
  EXPECT_EQ(solved.standing, KneeBranch::Lefty);
  EXPECT_GT(solved.lefty.femur_rad, 0.0);
  EXPECT_LT(solved.lefty.tibia_rad, 0.0);
  EXPECT_LT(solved.righty.femur_rad, 0.0);
  EXPECT_GT(solved.righty.tibia_rad, 0.0);
  const JointZero& zero = joint_zero();
  EXPECT_DOUBLE_EQ(solved.standing_angles.femur_rad,
                   solved.lefty.femur_rad + zero.femur_rad);
  EXPECT_DOUBLE_EQ(solved.standing_angles.tibia_rad,
                   solved.lefty.tibia_rad + zero.tibia_rad);
}

TEST(Planar, RaisingTheFootLiftsTheFemur) {
  // Same horizontal stand foot, z from -70 toward -50. The kept fold
  // must increase the femur angle and make the tibia more negative.
  const LinkLengths& links = link_lengths();
  const Hip& lf = hip(LegId::Lf);
  const Vec3 low{163.6, 140.8, -70.0};
  const Vec3 raised{163.6, 140.8, -50.0};
  const PlanarResult at_stand = solve_planar(
      plane_target(lf, low, links), links, kStandInMaxStepRad, joint_zero());
  const PlanarResult higher = solve_planar(
      plane_target(lf, raised, links), links, kStandInMaxStepRad, joint_zero());

  ASSERT_TRUE(at_stand.ok);
  ASSERT_TRUE(higher.ok);
  EXPECT_GT(higher.standing_angles.femur_rad, at_stand.standing_angles.femur_rad);
  EXPECT_LT(higher.standing_angles.tibia_rad, at_stand.standing_angles.tibia_rad);
}

TEST(Planar, CoxaFemurZMovesOnlyTheBookVertical) {
  // One extra millimetre of gap between the yaw axis and the femur
  // hinge lowers the book's y by that millimetre. The horizontal
  // distance from the femur hinge is a different number.
  const Hip& lf = hip(LegId::Lf);
  const Vec3 foot{163.6, 140.8, -70.0};
  const LinkLengths& links = link_lengths();
  const PlaneTarget original = plane_target(lf, foot, links);

  LinkLengths shifted = links;
  shifted.coxa_femur_z_mm += 1.0;
  const PlaneTarget moved = plane_target(lf, foot, shifted);

  EXPECT_NEAR(moved.z_plane_mm, original.z_plane_mm - 1.0, 1e-12);
  EXPECT_DOUBLE_EQ(moved.x_plane_mm, original.x_plane_mm);
  EXPECT_NEAR(original.x_plane_mm, 69.01359579015127, 1e-9);
  EXPECT_NEAR(original.z_plane_mm, -71.13, 1e-9);
}

TEST(Planar, ANonFiniteTargetIsRefused) {
  const PlaneTarget bad{std::numeric_limits<double>::quiet_NaN(), -70.0};
  const PlanarResult solved =
      solve_planar(bad, link_lengths(), kStandInMaxStepRad, joint_zero());
  EXPECT_FALSE(solved.ok);
  EXPECT_EQ(solved.reason, IkReason::NonFinite);
}

}  // namespace
}  // namespace hexapod_kinematics

// Locks the coxa yaw to the horizontal formula in PLAN.md.
// A foot on the mount ray is 0. The left-front stand foot stays within
// 0.01 rad of the vendor tape. Height is not part of the angle.
// The knee triangle and the stand contract are later files.

#include "hexapod_kinematics/coxa.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace hexapod_kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(Coxa, FootOnTheLfMountRayIsZero) {
  // LF mount yaw is +45°. A foot placed along that ray has no yaw left.
  const double along_mm = 80.0;
  const double c = std::cos(kPi / 4.0);
  const double s = std::sin(kPi / 4.0);
  const Vec3 foot{93.60 + along_mm * c, 50.805 + along_mm * s, -70.0};
  EXPECT_NEAR(solve_coxa(LegId::Lf, foot), 0.0, 1e-12);
}

TEST(Coxa, LfStandFootIsNearTheVendorTape) {
  // DEFAULT_POSE left-front foot from PLAN.md, millimetres.
  // The vendor tape is 0.1207 rad (6.91°). The geometric leftover is a
  // few thousandths of a radian higher. 0.01 rad is this step's window.
  // Joint zero is not applied in solve_coxa.
  const Vec3 foot{163.6, 140.8, -70.0};
  EXPECT_NEAR(solve_coxa(LegId::Lf, foot), 0.1207, 0.01);
}

TEST(Coxa, HeightDoesNotChangeTheYaw) {
  const Vec3 low{163.6, 140.8, -70.0};
  const Vec3 raised{163.6, 140.8, -40.0};
  EXPECT_DOUBLE_EQ(solve_coxa(LegId::Lf, low), solve_coxa(LegId::Lf, raised));
}

TEST(Coxa, LmHomeFootIsStraightOut) {
  // LM hip is (0, 73.535) and coxa = 0 points at +90°.
  // build_in_pose DEFAULT_POSE puts that foot at
  // (0, INITIAL_Y + Y2, -70) = (0, 183.535, -70), straight out along +Y.
  const Vec3 foot{0.0, 73.535 + 110.0, -70.0};
  EXPECT_NEAR(solve_coxa(LegId::Lm, foot), 0.0, 1e-12);
}

TEST(Coxa, LegSixMirrorsLegOne) {
  // RF mirrors LF across the body x axis: same x, negated y, negated
  // mount yaw. The coxa angle negates with the foot.
  const Vec3 lf{163.6, 140.8, -70.0};
  const Vec3 rf{163.6, -140.8, -70.0};
  EXPECT_NEAR(solve_coxa(LegId::Rf, rf), -solve_coxa(LegId::Lf, lf), 1e-12);
}

TEST(Coxa, WrapsAHeadingPastPi) {
  // RF mount yaw is -45°. A foot due behind that hip, on the body's -X
  // side, has heading +pi. pi - (-pi/4) = 5pi/4, which is past +pi.
  // The representative in (-pi, pi] is -3pi/4.
  const Vec3 behind{93.60 - 10.0, -50.805, -70.0};
  EXPECT_NEAR(solve_coxa(LegId::Rf, behind), -3.0 * kPi / 4.0, 1e-12);
}

}  // namespace
}  // namespace hexapod_kinematics

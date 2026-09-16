#include <cmath>

#include <gtest/gtest.h>

#include "proud_up/follow_gait.hpp"

using proud_up::follow_gait::aep_pep;
using proud_up::follow_gait::body_of_world_fixed;
using proud_up::follow_gait::com_in_support;
using proud_up::follow_gait::GaitCommand;
using proud_up::follow_gait::kDefaultPose;
using proud_up::follow_gait::kGroupA;
using proud_up::follow_gait::kGroupB;
using proud_up::follow_gait::kPi;
using proud_up::follow_gait::kTwoPi;
using proud_up::follow_gait::sample;
using proud_up::follow_gait::se2_exp;
using proud_up::follow_gait::sigma;
using proud_up::follow_gait::sigma_dot;
using proud_up::follow_gait::TwistMm;
using proud_up::follow_gait::Vec3;

namespace {

GaitCommand cmd_vx() {
  GaitCommand c;
  c.v.vx = 50.0;  // mm/s
  c.v.vy = 0.0;
  c.v.wz = 0.0;
  c.period_s = 0.70;
  c.lift_mm = 25.0;
  return c;
}

}  // namespace

TEST(FollowGait, SigmaEndpointsAreRest) {
  EXPECT_NEAR(sigma(0.0), 0.0, 1e-12);
  EXPECT_NEAR(sigma(1.0), 1.0, 1e-12);
  EXPECT_NEAR(sigma_dot(0.0), 0.0, 1e-12);
  EXPECT_NEAR(sigma_dot(1.0), 0.0, 1e-12);
  EXPECT_GT(sigma(0.5), 0.49);
  EXPECT_LT(sigma(0.5), 0.51);
}

TEST(FollowGait, VxPositiveMovesAepForward) {
  const Vec3 lf = kDefaultPose[0];
  const auto ends = aep_pep(lf, TwistMm{50.0, 0.0, 0.0}, 0.35, 40.0);
  EXPECT_GT(ends.aep.x - lf.x, 5.0);
  EXPECT_LT(ends.pep.x - lf.x, -5.0);
  EXPECT_NEAR(ends.aep.y, lf.y, 1e-6);
  EXPECT_NEAR(ends.pep.y, lf.y, 1e-6);
}

TEST(FollowGait, VyPositiveMovesAepLeft) {
  const Vec3 lf = kDefaultPose[0];
  const auto ends = aep_pep(lf, TwistMm{0.0, 50.0, 0.0}, 0.35, 40.0);
  EXPECT_GT(ends.aep.y - lf.y, 5.0);
  EXPECT_LT(ends.pep.y - lf.y, -5.0);
  EXPECT_NEAR(ends.aep.x, lf.x, 1e-6);
}

TEST(FollowGait, WzPositiveLengthensRightStride) {
  // ωz > 0 is a left turn. The outside (right, y0 < 0) foot takes a
  // longer step: extra +X on AEP relative to a left-side foot.
  const Vec3 lf = kDefaultPose[0];  // y > 0
  const Vec3 rf = kDefaultPose[5];  // y < 0
  const TwistMm spin{0.0, 0.0, 0.4};
  const auto a_lf = aep_pep(lf, spin, 0.35, 40.0);
  const auto a_rf = aep_pep(rf, spin, 0.35, 40.0);
  EXPECT_GT(a_rf.aep.x - rf.x, a_lf.aep.x - lf.x);
}

TEST(FollowGait, TripodBDownAtPhiZero) {
  const auto s = sample(0.0, cmd_vx());
  EXPECT_TRUE(s.stance[1]);  // LM
  EXPECT_TRUE(s.stance[3]);  // RR
  EXPECT_TRUE(s.stance[5]);  // RF
  EXPECT_FALSE(s.stance[0]);
  EXPECT_FALSE(s.stance[2]);
  EXPECT_FALSE(s.stance[4]);
  EXPECT_TRUE(com_in_support(s, 0.0, 0.0));
}

TEST(FollowGait, TripodADownAtPhiPi) {
  const auto s = sample(kPi, cmd_vx());
  EXPECT_TRUE(s.stance[0]);
  EXPECT_TRUE(s.stance[2]);
  EXPECT_TRUE(s.stance[4]);
  EXPECT_FALSE(s.stance[1]);
  EXPECT_FALSE(s.stance[3]);
  EXPECT_FALSE(s.stance[5]);
  EXPECT_TRUE(com_in_support(s, 0.0, 0.0));
}

TEST(FollowGait, ComInTriangleEveryBead) {
  const auto cmd = cmd_vx();
  const int n = 35;
  for (int k = 0; k < n; ++k) {
    const double phi = kTwoPi * static_cast<double>(k) / static_cast<double>(n);
    const auto s = sample(phi, cmd);
    EXPECT_TRUE(com_in_support(s, 0.0, 0.0)) << "phi=" << phi;
    int down = 0;
    double zmin = 1e9;
    for (int i = 0; i < 6; ++i) {
      if (s.stance[static_cast<std::size_t>(i)]) {
        ++down;
        zmin = std::min(zmin, s.feet[static_cast<std::size_t>(i)].z);
      }
    }
    EXPECT_EQ(down, 3);
    for (int i = 0; i < 6; ++i) {
      if (s.stance[static_cast<std::size_t>(i)]) {
        EXPECT_NEAR(s.feet[static_cast<std::size_t>(i)].z, zmin, 3.0);
      }
    }
  }
}

TEST(FollowGait, SwingZDotZeroAtTouchdown) {
  // Finite-difference z-dot of a swinging A-foot near lift (φ=0+) and
  // land (φ=π−). Because sigma-dot is 0 at the ends, this must be ~0
  // even though the spatial parabola 4s(1−s) has a slope at s=0,1.
  const auto cmd = cmd_vx();
  const double dphi = 1.0e-4;
  const auto z = [&](double phi) { return sample(phi, cmd).feet[0].z; };
  const double T = cmd.period_s;
  const double zdot_lift = (z(dphi) - z(0.0)) / (dphi * T / kTwoPi);
  const double zdot_land = (z(kPi) - z(kPi - dphi)) / (dphi * T / kTwoPi);
  EXPECT_NEAR(zdot_lift, 0.0, 5.0);  // mm/s; 25 mm lift, quintic start
  EXPECT_NEAR(zdot_land, 0.0, 5.0);
}

TEST(FollowGait, StanceSpeedNonzeroAtWrap) {
  // A rest-to-rest cycle clock would park the stance feet at wrap.
  // Ours must keep sliding: d(xy)/dφ of a B-stance foot just after
  // φ = 0 is the no-slip retraction, not ~0.
  const auto cmd = cmd_vx();
  const double dphi = 0.02;
  const auto p0 = sample(0.0, cmd).feet[1];   // LM, stance in [0, π)
  const auto p1 = sample(dphi, cmd).feet[1];
  const double speed = std::hypot(p1.x - p0.x, p1.y - p0.y) / dphi;
  EXPECT_GT(speed, 1.0);
}

TEST(FollowGait, Se2ExpLinearMatchesSmallAngle) {
  const double t = 0.35;
  const auto g = se2_exp(50.0, 20.0, 1.0e-8, t);
  EXPECT_NEAR(g.tx, 50.0 * t, 1e-4);
  EXPECT_NEAR(g.ty, 20.0 * t, 1e-4);
}

TEST(FollowGait, AllSixOnGroundAtSwitch) {
  const auto cmd = cmd_vx();
  for (double phi : {0.0, kPi}) {
    const auto s = sample(phi, cmd);
    for (int i = 0; i < 6; ++i) {
      EXPECT_NEAR(s.feet[static_cast<std::size_t>(i)].z, kDefaultPose[static_cast<std::size_t>(i)].z,
                  1e-6)
          << "phi=" << phi << " leg=" << i;
    }
  }
}

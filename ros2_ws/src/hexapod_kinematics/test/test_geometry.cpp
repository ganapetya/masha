// Locks the constants in geometry.cpp to the numbers in PLAN.md.
// This is not a kinematics test. Coxa, the triangle, and the stand
// contract are later files. If a hip moves, this test fails on purpose.

#include "hexapod_kinematics/geometry.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace hexapod_kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;

TEST(Geometry, HipTableMatchesBuildInPose) {
  // Six entries, leg 1 at the front left, then counterclockwise.
  const Hip* lf = hip_or_null(LegId::Lf);
  ASSERT_NE(lf, nullptr);
  EXPECT_STREQ(lf->name, "LF");
  EXPECT_DOUBLE_EQ(lf->x_mm, 93.60);
  EXPECT_DOUBLE_EQ(lf->y_mm, 50.805);
  EXPECT_DOUBLE_EQ(lf->mount_yaw_rad, kPi / 4.0);

  const Hip& lm = hip(LegId::Lm);
  EXPECT_DOUBLE_EQ(lm.x_mm, 0.0);
  EXPECT_DOUBLE_EQ(lm.y_mm, 73.535);
  EXPECT_DOUBLE_EQ(lm.mount_yaw_rad, kPi / 2.0);

  const Hip& lr = hip(LegId::Lr);
  EXPECT_DOUBLE_EQ(lr.x_mm, -93.60);
  EXPECT_DOUBLE_EQ(lr.y_mm, 50.805);
  EXPECT_DOUBLE_EQ(lr.mount_yaw_rad, 3.0 * kPi / 4.0);

  const Hip& rr = hip(LegId::Rr);
  EXPECT_DOUBLE_EQ(rr.x_mm, -93.60);
  EXPECT_DOUBLE_EQ(rr.y_mm, -50.805);
  EXPECT_DOUBLE_EQ(rr.mount_yaw_rad, -3.0 * kPi / 4.0);

  const Hip& rm = hip(LegId::Rm);
  EXPECT_DOUBLE_EQ(rm.x_mm, 0.0);
  EXPECT_DOUBLE_EQ(rm.y_mm, -73.535);
  EXPECT_DOUBLE_EQ(rm.mount_yaw_rad, -kPi / 2.0);

  const Hip& rf = hip(LegId::Rf);
  EXPECT_DOUBLE_EQ(rf.x_mm, 93.60);
  EXPECT_DOUBLE_EQ(rf.y_mm, -50.805);
  EXPECT_DOUBLE_EQ(rf.mount_yaw_rad, -kPi / 4.0);
}

TEST(Geometry, RejectsALegIdOutsideOneToSix) {
  // 0 is not a leg. The enum's underlying value is the vendor number.
  EXPECT_EQ(hip_or_null(static_cast<LegId>(0)), nullptr);
  EXPECT_EQ(hip_or_null(static_cast<LegId>(7)), nullptr);
}

TEST(Geometry, LengthGuessesStaySeparate) {
  const LinkLengths& links = link_lengths();
  EXPECT_DOUBLE_EQ(links.coxa_mm, 45.0);
  EXPECT_DOUBLE_EQ(links.femur_mm, 77.1);
  EXPECT_DOUBLE_EQ(links.tibia_mm, 115.6);
  EXPECT_DOUBLE_EQ(links.hip_z_mm, 0.0);
  // The vertical gap is its own field. It is not added into the femur.
  EXPECT_DOUBLE_EQ(links.coxa_femur_z_mm, 1.13);
  EXPECT_NE(links.coxa_femur_z_mm, 0.0);
}

TEST(Geometry, JointZeroIsFittedAndStandTapeIsSeparate) {
  // The numbers LegIk.StandContract fitted. The tape below is the
  // vendor stand, not this offset.
  const JointZero& zero = joint_zero();
  EXPECT_DOUBLE_EQ(zero.coxa_rad, 0.0);
  EXPECT_DOUBLE_EQ(zero.femur_rad, 0.144427);
  EXPECT_DOUBLE_EQ(zero.tibia_rad, 1.481540);

  const StandTape& tape = stand_tape_lf();
  EXPECT_DOUBLE_EQ(tape.coxa_rad, 0.1207);
  EXPECT_DOUBLE_EQ(tape.femur_rad, 0.7555);
  EXPECT_DOUBLE_EQ(tape.tibia_rad, -0.650);
}

}  // namespace
}  // namespace hexapod_kinematics

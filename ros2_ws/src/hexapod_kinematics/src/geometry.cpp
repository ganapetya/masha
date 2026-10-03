// The numbers the rest of the leg math is allowed to use.
//
// This file does not solve a triangle. It writes down three kinds of
// constant that must not be mixed:
//   1. Where each yaw axis sits, and which way "coxa = 0" points.
//   2. How long the links are, including the small vertical gap between
//      the yaw axis and the femur hinge.
//   3. The still-unfitted shift from the drawing's zero to the servo's zero.
//
// Body frame: +X head, +Y left, +Z up, millimetres.
// The hip xy values are build_in_pose.py's X1, Y1, Y2. The URDF
// leg_center for the left-front leg is about (104.6, 50.6, 16.6) mm.
// That is the mesh frame. These hips are the simplified IK frame.
// The two points are not required to match, and this file does not
// solve the CAD chain.

#include "hexapod_kinematics/geometry.hpp"

#include <cstdlib>

namespace hexapod_kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Mount yaw is "the heading of coxa = 0", in radians.
// 45 degrees is a quarter turn, so pi/4, not a rounded decimal.
constexpr double kYaw45 = kPi / 4.0;
constexpr double kYaw90 = kPi / 2.0;
constexpr double kYaw135 = 3.0 * kPi / 4.0;

// Index 0 is leg 1 (LF). See leg_index in types.hpp.
constexpr Hip kHips[kLegCount] = {
    {LegId::Lf, "LF", 93.60, 50.805, kYaw45},
    {LegId::Lm, "LM", 0.0, 73.535, kYaw90},
    {LegId::Lr, "LR", -93.60, 50.805, kYaw135},
    {LegId::Rr, "RR", -93.60, -50.805, -kYaw135},
    {LegId::Rm, "RM", 0.0, -73.535, -kYaw90},
    {LegId::Rf, "RF", 93.60, -50.805, -kYaw45},
};

// Starting guesses from joint-origin distances in
// rospider_description/urdf/base.urdf.xacro, left-front leg.
//   coxa  ~ hypot(45.04, 0.53) mm, written 45.0
//   femur ~ 77.1 mm between the femur and tibia joint origins
//   tibia ~ 115.6 mm from the tibia joint origin to the foot
// hip_z starts at 0: the IK frame already shares z with the feet in
// build_in_pose. It is not the CAD origin height.
// coxa_femur_z is the femur joint's z in the coxa link, about 1.13 mm
// (the URDF value is 1.1277 mm). Sign: femur hinge minus the yaw-axis
// origin, along the yaw axis. A ruler may set this to 0. It must not
// be folded into femur_mm or tibia_mm.
constexpr LinkLengths kLinks = {
    45.0,
    77.1,
    115.6,
    0.0,
    1.13,
};

// Fitted by LegIk.StandContract in test_leg_ik.cpp. That test requires
// solve_leg on all six DEFAULT_POSE feet to land within 0.005 rad of
// the angles set_leg_position returns.
//
// Coxa stays 0. The largest coxa residual is about 0.0037 rad, on the
// corner legs, which is already inside 0.005. The middle legs match
// with no offset.
//
// Femur 0.144427 rad (about 8.28 deg) and tibia 1.481540 rad (about
// 84.89 deg) are one shared pair. They are the average gap, at the
// stand, between the drawing and the servo. The drawing measures the
// femur from horizontal and the tibia from straight. The servo measures
// both from the center of its travel. The tibia gap is that definition.
// It is not a second copy of a link length.
//
// Corner and middle feet are different triangles, about 4 mm apart in
// the horizontal plane. The vendor tibia differs by about 0.037 rad
// between those two shapes, and this model differs by about 0.036 rad,
// so one shared tibia zero covers both. A per-leg table was not needed.
// The URDF length guesses and coxa_femur_z were left as they are.
// Moving them is not what closed the contract.
constexpr JointZero kJointZero = {0.0, 0.144427, 1.481540};

// Left-front vendor stand, rounded the way PLAN.md writes it:
// 0.1207, 0.7555, -0.650 rad (6.91 deg, 43.29 deg, -37.24 deg).
// The stand test writes the fuller six-leg readings. Middle femur and
// tibia are not this pair. Coxa on the right negates the left, and the
// middles are 0. This tape is not added into a formula.
constexpr StandTape kStandLf = {0.1207, 0.7555, -0.650};

}  // namespace

const Hip* hip_or_null(LegId id) {
  if (!leg_id_ok(id)) {
    return nullptr;
  }
  return &kHips[leg_index(id)];
}

const Hip& hip(LegId id) {
  // Precondition: id is one of the six. Callers that hold an untrusted
  // integer use hip_or_null and do not publish when it returns null.
  // Abort rather than return leg 1: a wrong yaw axis would move a foot.
  const Hip* found = hip_or_null(id);
  if (found == nullptr) {
    std::abort();
  }
  return *found;
}

const LinkLengths& link_lengths() {
  return kLinks;
}

const JointZero& joint_zero() {
  return kJointZero;
}

const StandTape& stand_tape_lf() {
  return kStandLf;
}

}  // namespace hexapod_kinematics

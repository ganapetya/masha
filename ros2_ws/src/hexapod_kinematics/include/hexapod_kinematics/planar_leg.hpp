#pragma once

// Subproblem 2: the knee triangle.
//
// After the coxa faces the foot, femur and tibia lie in one vertical
// plane. This file folds that two-link ruler so the tip lands on the
// foot. It is Lynch and Park, Figure 6.1: femur is L1, tibia is L2,
// and the book's (x, y) is our (x_plane, z_plane).
//
// The coxa yaw is the previous file. Pulses and servo direction are
// Python. A foot outside the ring has no angle. A foot just inside the
// straight-leg edge has an angle and is still refused.

#include "hexapod_kinematics/geometry.hpp"

namespace hexapod_kinematics {

// The book's planar target, in millimetres.
// x_plane is horizontal distance from the femur hinge out to the foot.
// z_plane is the book's y: up is positive, and a foot below the femur
// hinge has a negative z_plane.
struct PlaneTarget {
  double x_plane_mm = 0.0;
  double z_plane_mm = 0.0;
};

// One fold of the triangle, radians.
// femur is the book's theta1, measured from horizontal out along +x_plane.
// tibia is the book's theta2, measured from the femur. Zero tibia is
// straight. A negative tibia bends the other way.
struct PlanarAngles {
  double femur_rad = 0.0;
  double tibia_rad = 0.0;
};

// The two legal folds. Figure 6.1 calls them righty and lefty.
// Jazar draws the same pair as elbow-down and elbow-up.
enum class KneeBranch {
  Righty,
  Lefty,
};

// Both folds, and the one a standing Masha keeps.
//
// ok is true only when reason is None. standing_angles may then be
// commanded. Those angles are the standing fold plus the femur and
// tibia joint zeros.
//
// near_singular still fills the angles: the triangle exists, and the
// knee would move too far for a 0.5 mm foot error. Callers read them
// for a trace and must not publish them.
//
// unreachable leaves the angles at 0. That 0 means "not computed".
// No clamped pose is written in their place.
struct PlanarResult {
  bool ok = false;
  IkReason reason = IkReason::NoReference;
  double alpha_rad = 0.0;
  double beta_rad = 0.0;
  double gamma_rad = 0.0;
  PlanarAngles righty{};
  PlanarAngles lefty{};
  KneeBranch standing = KneeBranch::Lefty;
  PlanarAngles standing_angles{};
};

// Body-frame foot to the book's (x, y).
//
// hip is that leg's yaw pivot. foot is millimetres in the body frame.
// links supplies coxa_mm, hip_z_mm, and coxa_femur_z_mm. The references
// are read for this call and not stored.
//
// This step does not decide whether the triangle exists. gtest calls it
// directly. leg_ik will call it once per leg, after the coxa yaw.
PlaneTarget plane_target(const Hip& hip, const Vec3& foot, const LinkLengths& links);

// The fold a standing Masha uses, for every foot.
//
// Fixed. It does not look at the foot and pick again. The left-front
// stand numbers that decided it are written above the definition.
KneeBranch standing_branch();

// Figure 6.1, then the joint-zero add.
//
// target is the book's (x, y) in millimetres. links.femur_mm is L1 and
// links.tibia_mm is L2. Both lengths must be positive millimetres.
// max_step_rad is the knee step that a 0.5 mm foot error is still
// allowed to ask for. Tests pass a stand-in. The measured constant
// belongs to the safety gate, which is a later file.
// zero is applied to the standing fold only, after theta1 and theta2
// exist. The coxa component of zero is not used here.
//
// gtest calls this directly. The 20 ms loop will call it later from
// leg_ik, once per leg per tick.
PlanarResult solve_planar(const PlaneTarget& target,
                          const LinkLengths& links,
                          double max_step_rad,
                          const JointZero& zero);

}  // namespace hexapod_kinematics

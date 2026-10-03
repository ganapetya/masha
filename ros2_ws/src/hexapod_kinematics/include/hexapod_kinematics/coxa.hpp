#pragma once

// Subproblem 1: which way the coxa faces.
//
// The coxa is the yaw hinge, a turret in the horizontal plane. This
// header names that one angle. Femur, tibia, link lengths, and joint
// zeros are other files. A foot the links cannot reach is refused
// later, by the triangle, not here.

#include "hexapod_kinematics/types.hpp"

namespace hexapod_kinematics {

// Coxa angle for one foot, radians, in the interval (-pi, pi].
// -pi is folded up to +pi, so each direction has one representative.
//
// foot is a body-frame point in millimetres (+X head, +Y left, +Z up).
// Only x_mm and y_mm are read. z_mm is the height, and a pure yaw does
// not depend on height.
//
// id must be one of the six legs. hip() aborts for any other value,
// because a guessed mount yaw would aim the foot along the wrong ray.
//
// The reference is read during this call and not stored.
// gtest calls this directly. solve_leg calls it once per leg.
// The 20 ms loop reaches it through solve_pose.
double solve_coxa(LegId id, const Vec3& foot);

}  // namespace hexapod_kinematics

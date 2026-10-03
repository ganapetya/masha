#pragma once

// Composition only. No new formula.
//
// solve_leg calls the coxa, then the knee triangle, and adds the coxa
// joint zero. The femur and tibia zeros are already inside the triangle's
// standing angles. solve_pose, the six-leg call for one 20 ms tick, is
// a later step.

#include "hexapod_kinematics/types.hpp"

namespace hexapod_kinematics {

// One foot, body frame millimetres, to three servo radians.
//
// id must be one of the six legs. foot is +X head, +Y left, +Z up.
// max_step_rad is the knee step a 0.5 mm foot error may still ask for.
// The measured value belongs to the safety gate. Tests pass a stand-in.
//
// ok is true only when reason is None. The angles may then be commanded.
// On a refusal the angles stay 0, which means "not computed", and the
// reason is the triangle's reason (unreachable, near_singular, or
// non_finite). This function does not raise.
//
// gtest calls this directly. The 20 ms loop will call it later, once
// per leg per tick, through the Python binding.
IkResult solve_leg(LegId id, const Vec3& foot, double max_step_rad);

}  // namespace hexapod_kinematics

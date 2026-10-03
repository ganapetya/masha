#pragma once

// Composition only. No new formula.
//
// solve_leg calls the coxa, then the knee triangle, and adds the coxa
// joint zero. The femur and tibia zeros are already inside the triangle's
// standing angles.
//
// solve_pose is the call one 20 ms tick makes: six feet in, one decision
// out. It loops legs 1..6 and calls solve_leg. A later Python binding
// returns this result as a value. A refusal stays a filled PoseResult,
// so the tick's handler keeps the generator instead of seeing an exception.

#include "hexapod_kinematics/types.hpp"

#include <array>

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
// gtest calls this directly. solve_pose calls it once per leg.
IkResult solve_leg(LegId id, const Vec3& foot, double max_step_rad);

// One tick's answer for all six legs.
//
// ok is true only when every leg solved. The eighteen angles may then
// be commanded. leg is the first leg that refused, and it is read only
// when ok is false. When ok is true, leg is unused.
//
// angles[0] is leg 1 and angles[5] is leg 6. A slot is publishable only
// as part of a pose whose ok is true. On a refusal the refusing leg's
// slot stays 0, and so does every leg the loop had not reached yet.
// Legs that already solved keep the angles solve_leg returned. Those
// angles are still not for a servo, because the pose itself was refused.
struct PoseResult {
  bool ok = false;
  IkReason reason = IkReason::NoReference;
  LegId leg = LegId::Lf;
  std::array<JointAngles, kLegCount> angles{};
};

// Six feet, body frame millimetres, to one pose decision.
//
// feet[0] is leg 1 (LF) and feet[5] is leg 6 (RF), the same order a
// gait sample uses. max_step_rad is passed through to every leg.
//
// The loop walks leg 1, then 2, and so on. The first refusal is the
// one this result names, and the loop stops there. A later bad foot
// does not replace that leg id. This function does not raise.
PoseResult solve_pose(const std::array<Vec3, kLegCount>& feet, double max_step_rad);

}  // namespace hexapod_kinematics

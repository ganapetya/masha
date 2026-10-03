// Turn feet into servo angles by calling the two subproblems in order.
// There is no third formula in this file.
//
// solve_leg, one foot:
//   1. Hip and mount yaw, from geometry.
//   2. solve_coxa.
//   3. plane_target builds (x_plane, z_plane).
//   4. solve_planar folds the triangle and adds the femur and tibia zeros.
//   5. Add the coxa zero. It is 0 today; the add stays so a later fit
//      has one place to land.
//   6. Return the three angles, or the triangle's refusal.
//
// solve_pose, one tick: call solve_leg for leg 1, then leg 2, and so on.
// The first refusal is the pose's refusal. The function returns that
// result. It does not raise, because a raised error in the later Python
// binding would drop the gait generator.

#include "hexapod_kinematics/leg_ik.hpp"

#include "hexapod_kinematics/coxa.hpp"
#include "hexapod_kinematics/geometry.hpp"
#include "hexapod_kinematics/planar_leg.hpp"

#include <array>
#include <cmath>

namespace hexapod_kinematics {

IkResult solve_leg(LegId id, const Vec3& foot, double max_step_rad) {
  IkResult result;
  result.leg = id;

  if (!std::isfinite(foot.x_mm) || !std::isfinite(foot.y_mm) ||
      !std::isfinite(foot.z_mm)) {
    result.reason = IkReason::NonFinite;
    return result;
  }

  // hip() aborts when id is not a real leg. A guessed yaw is not an angle.
  const double q_coxa = solve_coxa(id, foot);
  if (!std::isfinite(q_coxa)) {
    result.reason = IkReason::NonFinite;
    return result;
  }

  const LinkLengths& links = link_lengths();
  const JointZero& zero = joint_zero();
  const PlanarResult knee = solve_planar(plane_target(hip(id), foot, links), links,
                                         max_step_rad, zero);
  if (!knee.ok) {
    result.ok = false;
    result.reason = knee.reason;
    return result;
  }

  result.ok = true;
  result.reason = IkReason::None;
  result.angles.coxa_rad = q_coxa + zero.coxa_rad;
  result.angles.femur_rad = knee.standing_angles.femur_rad;
  result.angles.tibia_rad = knee.standing_angles.tibia_rad;
  return result;
}

PoseResult solve_pose(const std::array<Vec3, kLegCount>& feet, double max_step_rad) {
  // Written in leg-id order so the reported leg is the first refusal
  // a person would count: LF, then LM, and so on around the body.
  constexpr std::array<LegId, kLegCount> kOrder = {
      LegId::Lf, LegId::Lm, LegId::Lr, LegId::Rr, LegId::Rm, LegId::Rf};

  PoseResult pose;
  for (LegId id : kOrder) {
    const int index = leg_index(id);
    const IkResult leg = solve_leg(id, feet[static_cast<std::size_t>(index)], max_step_rad);
    if (!leg.ok) {
      // Leave this slot and every later slot at 0. Do not copy a
      // refused angle into the array. Legs already stored stay, so a
      // log can see how far the tick got, and ok false keeps them off
      // the servos.
      pose.ok = false;
      pose.reason = leg.reason;
      pose.leg = leg.leg;
      return pose;
    }
    pose.angles[static_cast<std::size_t>(index)] = leg.angles;
  }

  pose.ok = true;
  pose.reason = IkReason::None;
  return pose;
}

}  // namespace hexapod_kinematics

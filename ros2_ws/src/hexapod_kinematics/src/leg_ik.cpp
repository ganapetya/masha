// Turn one foot into three servo angles by calling the two subproblems
// in order. There is no third formula in this file.
//
//   1. Hip and mount yaw, from geometry.
//   2. solve_coxa.
//   3. plane_target builds (x_plane, z_plane).
//   4. solve_planar folds the triangle and adds the femur and tibia zeros.
//   5. Add the coxa zero. It is 0 today; the add stays so a later fit
//      has one place to land.
//   6. Return the three angles, or the triangle's refusal.

#include "hexapod_kinematics/leg_ik.hpp"

#include "hexapod_kinematics/coxa.hpp"
#include "hexapod_kinematics/geometry.hpp"
#include "hexapod_kinematics/planar_leg.hpp"

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

}  // namespace hexapod_kinematics

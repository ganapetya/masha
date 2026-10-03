// Which way the coxa must face so the leg points at the foot.
//
// The coxa only yaws. Once it faces the foot, the femur and the tibia
// fold in a vertical plane. This file answers the yaw. The two link
// lengths, the knee triangle, and the joint-zero offsets are later
// files. Every horizontal direction is an angle. There is no refusal
// in this step.
//
// The angle is measured from the leg's own zero, not from the body's +X.
// Mount yaw is the direction coxa = 0 already points: +45° on the
// front-left leg, +90° on the middle-left leg, and the mirrors on the
// right. Subtracting that fixed heading is the change of frame.
//
//   dx = x_foot - hip_x
//   dy = y_foot - hip_y
//   q_coxa = wrap( atan2(dy, dx) - mount_yaw )
//
// std::atan2 takes the y offset first and the x offset second.
// Left-front stand, the worked line. Hip (93.60, 50.805) mm, foot
// (163.6, 140.8) mm, so (dx, dy) = (70, 89.995).
// atan2(89.995, 70) is about 52.12°. The mount is 45°. The leftover is
// about 7.12° (0.1243 rad). The vendor tape for that foot is 6.91°
// (0.1207 rad). The test window for this file is 0.01 rad. No offset
// is added on top of the leftover.

#include "hexapod_kinematics/coxa.hpp"

#include "hexapod_kinematics/geometry.hpp"

#include <cmath>

namespace hexapod_kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

// One representative in (-pi, pi]. A value of -pi, or anything below it,
// is raised by one full turn, so -pi becomes +pi. +pi is already inside
// and stays.
//
// One turn is the whole range this robot can produce. atan2 returns a
// value in (-pi, pi], and every mount yaw here is within ±135°, so the
// difference lies in (-2pi, 2pi]. A second turn would mean the inputs
// were not finite. A loop is the wrong shape: +infinity compares greater
// than pi forever. A non-finite angle is returned as it is. The safety
// gate, later, is what refuses it.
double wrap_pm_pi(double angle) {
  if (angle > kPi) {
    return angle - kTwoPi;
  }
  if (angle <= -kPi) {
    return angle + kTwoPi;
  }
  return angle;
}

}  // namespace

double solve_coxa(LegId id, const Vec3& foot) {
  // Precondition: id is one of the six. hip() aborts otherwise.
  const Hip& mount = hip(id);
  const double dx = foot.x_mm - mount.x_mm;
  const double dy = foot.y_mm - mount.y_mm;
  // First argument is dy, second is dx. A foot whose (x, y) is the hip
  // has no direction: atan2(0, 0) is 0, and the mount yaw is then
  // subtracted. That result is not a solved heading. Whether the links
  // can reach the foot is the triangle's question.
  const double heading_rad = std::atan2(dy, dx);
  return wrap_pm_pi(heading_rad - mount.mount_yaw_rad);
}

}  // namespace hexapod_kinematics

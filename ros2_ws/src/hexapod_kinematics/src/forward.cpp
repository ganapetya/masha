// Angles back to a foot, by walking the three links.
//
// This is the opposite of solve_leg. No product of exponentials and no
// Denavit-Hartenberg table: the leg is a yaw, then two links in the
// vertical plane that yaw faces.
//
//   1. Start at the hip.
//   2. Step the coxa length along mount_yaw + q_coxa, and step
//      coxa_femur_z along body z. That point is the femur hinge.
//   3. Step the femur, then the tibia, in that vertical plane.
//
// The angles that arrive still carry the joint zeros. Those zeros are
// removed before step 3, because the drawing measures the femur from
// horizontal and the tibia from straight, and the servo does not.

#include "hexapod_kinematics/forward.hpp"

#include "hexapod_kinematics/geometry.hpp"

#include <cmath>

namespace hexapod_kinematics {

Vec3 forward_foot(LegId id, const JointAngles& angles) {
  const Hip& mount = hip(id);
  const LinkLengths& links = link_lengths();
  const JointZero& zero = joint_zero();

  // Drawing angles. Subtracting the zero is the whole conversion.
  const double q_coxa = angles.coxa_rad - zero.coxa_rad;
  const double theta1 = angles.femur_rad - zero.femur_rad;
  const double theta2 = angles.tibia_rad - zero.tibia_rad;

  // Heading of the coxa link in the body plane. cos/sin take this
  // heading; a wrap into (-pi, pi] would not move the foot.
  const double heading = mount.mount_yaw_rad + q_coxa;
  const double c = std::cos(heading);
  const double s = std::sin(heading);

  // Femur hinge. coxa_femur_z is added along body z and is not folded
  // into the two link lengths. hip_z is the yaw-axis height.
  const double hinge_x = mount.x_mm + links.coxa_mm * c;
  const double hinge_y = mount.y_mm + links.coxa_mm * s;
  const double hinge_z = links.hip_z_mm + links.coxa_femur_z_mm;

  // Figure 6.1 read forwards. theta1 is the femur from horizontal out.
  // theta2 is the tibia from the femur. Zero and zero is straight out.
  const double x_plane = links.femur_mm * std::cos(theta1) +
                         links.tibia_mm * std::cos(theta1 + theta2);
  const double z_plane = links.femur_mm * std::sin(theta1) +
                         links.tibia_mm * std::sin(theta1 + theta2);

  Vec3 foot;
  foot.x_mm = hinge_x + x_plane * c;
  foot.y_mm = hinge_y + x_plane * s;
  foot.z_mm = hinge_z + z_plane;
  return foot;
}

}  // namespace hexapod_kinematics

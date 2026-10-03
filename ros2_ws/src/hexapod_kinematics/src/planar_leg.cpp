// How the femur and the tibia fold so the foot lands on the target.
//
// The coxa has already turned to face the foot. What is left is a flat
// triangle in a vertical plane, the picture in Lynch and Park, Figure
// 6.1. Femur is L1. Tibia is L2. The line from the femur hinge to the
// foot has length r. Two folds reach the same tip. A standing Masha
// keeps one of them.
//
//   r^2     = x_plane^2 + z_plane^2
//   cos β   = (L1^2 + L2^2 - r^2) / (2 L1 L2)
//   cos α   = (r^2 + L1^2 - L2^2) / (2 L1 r)
//   γ       = atan2(z_plane, x_plane)
//   righty: θ1 = γ - α,  θ2 =  π - β
//   lefty:  θ1 = γ + α,  θ2 =  β - π
//
// β is the interior angle at the knee, opposite r. At full stretch β
// is π and the two folds meet. α is the angle between L1 and the line
// to the foot. γ is the heading of that line, up from horizontal.
// θ1 is our femur. θ2 is our tibia, relative to the femur.
//
// A cosine outside [-1, 1] means the foot is outside the ring: no
// closer than |L1 - L2|, no farther than L1 + L2, measured from the
// femur hinge. The angles stay unset. Nothing is clamped onto the ring.
//
// Just inside that ring the knee rate blows up. δ = 0.5 mm is the foot
// error the forward round trip is required to survive. If that error
// would move the knee by more than max_step_rad, the reason is
// near_singular. The cutoff is that product, not a fixed cosine such
// as 0.99. α grows at the same edge; the later rate gate is the
// backstop for α and for the coxa.

#include "hexapod_kinematics/planar_leg.hpp"

#include <cmath>

namespace hexapod_kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Same 0.5 mm the forward round trip will demand of a solved foot.
constexpr double kFootToleranceMm = 0.5;

bool finite_number(double value) {
  return std::isfinite(value);
}

PlanarAngles add_joint_zero(const PlanarAngles& book, const JointZero& zero) {
  // The book's zero is "femur along +x_plane, tibia straight on past
  // the femur." The servo's zero is the center of its travel. Those
  // offsets are a definition of zero, added after the triangle exists.
  // The coxa offset is not part of this plane. leg_ik adds it to the yaw.
  PlanarAngles servo;
  servo.femur_rad = book.femur_rad + zero.femur_rad;
  servo.tibia_rad = book.tibia_rad + zero.tibia_rad;
  return servo;
}

}  // namespace

KneeBranch standing_branch() {
  // Decided from the left-front stand foot (163.6, 140.8, -70) mm and
  // the length guesses in geometry.cpp (femur 77.1, tibia 115.6,
  // coxa_femur_z 1.13). The plane target is about (69.0, -71.1) mm.
  //
  //   lefty:  femur ≈ +0.61 rad (+35°), tibia ≈ -2.13 rad (-122°)
  //   righty: femur ≈ -2.21 rad,         tibia ≈ +2.13 rad
  //
  // A standing Masha holds the femur up and the tibia down, so the
  // kept fold is lefty. Raising that foot from z = -70 to z = -50
  // moves lefty to about +0.94 rad and -2.30 rad: the femur rises and
  // the tibia bends further down.
  //
  // The vendor tape for that pose is femur about +0.76 rad and tibia
  // about -0.65 rad. The gap is the shared joint zero in geometry.cpp,
  // added after these book angles by add_joint_zero. The cosines do
  // not contain it.
  return KneeBranch::Lefty;
}

PlaneTarget plane_target(const Hip& hip, const Vec3& foot, const LinkLengths& links) {
  const double dx = foot.x_mm - hip.x_mm;
  const double dy = foot.y_mm - hip.y_mm;
  PlaneTarget target;
  // Horizontal distance from the yaw axis, then back along the coxa
  // link to the femur hinge. What remains is the book's x.
  target.x_plane_mm = std::hypot(dx, dy) - links.coxa_mm;
  // The book's y. hip_z and coxa_femur_z move the origin from the body
  // frame up to the femur hinge. coxa_femur_z stays its own length so a
  // change in that gap moves only this vertical coordinate.
  target.z_plane_mm = foot.z_mm - links.hip_z_mm - links.coxa_femur_z_mm;
  return target;
}

PlanarResult solve_planar(const PlaneTarget& target,
                          const LinkLengths& links,
                          double max_step_rad,
                          const JointZero& zero) {
  PlanarResult result;
  result.standing = standing_branch();

  const double l1 = links.femur_mm;
  const double l2 = links.tibia_mm;
  const double x = target.x_plane_mm;
  const double z = target.z_plane_mm;
  // A length that is not a positive number of millimetres is a broken
  // constant. A step limit that is not a positive angle is not a
  // measurement yet. Either one is refused before acos runs.
  if (!finite_number(l1) || !finite_number(l2) || !(l1 > 0.0) || !(l2 > 0.0) ||
      !finite_number(x) || !finite_number(z) || !finite_number(max_step_rad) ||
      !(max_step_rad > 0.0) || !finite_number(zero.coxa_rad) ||
      !finite_number(zero.femur_rad) || !finite_number(zero.tibia_rad)) {
    result.reason = IkReason::NonFinite;
    return result;
  }

  const double r2 = x * x + z * z;
  const double r = std::sqrt(r2);
  // Interior angle at the knee, opposite the line to the foot.
  const double cos_beta = (l1 * l1 + l2 * l2 - r2) / (2.0 * l1 * l2);
  if (!finite_number(cos_beta)) {
    result.reason = IkReason::NonFinite;
    return result;
  }
  if (cos_beta < -1.0 || cos_beta > 1.0) {
    result.reason = IkReason::Unreachable;
    return result;
  }
  // r == 0 is the femur hinge. Unequal links already failed the cosine
  // above (the foot is in the hole). Equal links make this the folded
  // boundary, where the two solutions meet and the rate is unbounded.
  if (!(r > 0.0)) {
    result.reason = IkReason::NearSingular;
    return result;
  }

  const double cos_alpha = (r2 + l1 * l1 - l2 * l2) / (2.0 * l1 * r);
  if (!finite_number(cos_alpha)) {
    result.reason = IkReason::NonFinite;
    return result;
  }
  if (cos_alpha < -1.0 || cos_alpha > 1.0) {
    result.reason = IkReason::Unreachable;
    return result;
  }

  const double alpha = std::acos(cos_alpha);
  const double beta = std::acos(cos_beta);
  // atan2 takes the book's y first and the book's x second. z_plane is
  // that y. A foot below the hinge gives a negative gamma.
  const double gamma = std::atan2(z, x);

  PlanarAngles righty;
  righty.femur_rad = gamma - alpha;
  righty.tibia_rad = kPi - beta;
  PlanarAngles lefty;
  lefty.femur_rad = gamma + alpha;
  lefty.tibia_rad = beta - kPi;

  result.alpha_rad = alpha;
  result.beta_rad = beta;
  result.gamma_rad = gamma;
  result.righty = righty;
  result.lefty = lefty;
  const PlanarAngles& book =
      standing_branch() == KneeBranch::Righty ? righty : lefty;
  result.standing_angles = add_joint_zero(book, zero);

  // |dβ/dr| = r / (L1 L2 |sin β|). At full stretch β is π and the
  // denominator is 0, so any positive foot error asks for an unbounded
  // knee step. δ is kFootToleranceMm.
  const double sin_beta = std::fabs(std::sin(beta));
  bool near_singular = false;
  if (sin_beta == 0.0) {
    near_singular = true;
  } else {
    const double dbeta_dr = r / (l1 * l2 * sin_beta);
    near_singular = dbeta_dr * kFootToleranceMm > max_step_rad;
  }
  if (near_singular) {
    result.ok = false;
    result.reason = IkReason::NearSingular;
    return result;
  }

  result.ok = true;
  result.reason = IkReason::None;
  return result;
}

}  // namespace hexapod_kinematics

#pragma once

// Shapes shared by every later file in this package.
//
// This header has no ROS and no formula. It names the three things a leg
// calculation hands around: a point in millimetres, three joint angles in
// radians, and a yes-or-no result. The solver that fills an IkResult is
// not in this file.

namespace hexapod_kinematics {

// Vendor and build_in_pose number the legs 1..6, starting at the front-left
// and walking counterclockwise as seen from above: LF, LM, LR, RR, RM, RF.
// The values are those numbers, so a 1 here is leg 1 there.
enum class LegId : int {
  Lf = 1,
  Lm = 2,
  Lr = 3,
  Rr = 4,
  Rm = 5,
  Rf = 6,
};

// Six legs. An array in this package is indexed 0..5; LegId is 1..6.
// leg_index is the only place that subtraction happens.
constexpr int kLegCount = 6;

inline int leg_index(LegId id) {
  return static_cast<int>(id) - 1;
}

inline bool leg_id_ok(LegId id) {
  const int i = leg_index(id);
  return i >= 0 && i < kLegCount;
}

// A point or an offset in the body frame.
// +X toward the head, +Y to the robot's left, +Z up. Millimetres.
// Feet sit at negative z. This is the same frame as follow_gait and
// DEFAULT_POSE. It is not a ROS message and it has no frame_id.
struct Vec3 {
  double x_mm = 0.0;
  double y_mm = 0.0;
  double z_mm = 0.0;
};

// Three hinge angles, radians, in the sense set_leg_position returns.
// JointControl has not yet applied per-servo direction or offset.
// Coxa is the yaw. Femur and tibia are the two angles of the flat triangle.
struct JointAngles {
  double coxa_rad = 0.0;
  double femur_rad = 0.0;
  double tibia_rad = 0.0;
};

// Why a solve refused. None means the angles may be sent.
// The trace file spells these the same way, in English.
enum class IkReason {
  None,
  Unreachable,
  NearSingular,
  JointLimit,
  RateLimit,
  NonFinite,
  NoReference,
};

inline const char* ik_reason_name(IkReason reason) {
  switch (reason) {
    case IkReason::None:
      return "";
    case IkReason::Unreachable:
      return "unreachable";
    case IkReason::NearSingular:
      return "near_singular";
    case IkReason::JointLimit:
      return "joint_limit";
    case IkReason::RateLimit:
      return "rate_limit";
    case IkReason::NonFinite:
      return "non_finite";
    case IkReason::NoReference:
      return "no_reference";
  }
  return "non_finite";
}

// One leg's answer. ok is false unless reason is None.
// angles is meaningful only when ok is true. Callers must not publish
// a result whose ok is false.
struct IkResult {
  bool ok = false;
  IkReason reason = IkReason::NoReference;
  LegId leg = LegId::Lf;
  JointAngles angles{};
};

}  // namespace hexapod_kinematics

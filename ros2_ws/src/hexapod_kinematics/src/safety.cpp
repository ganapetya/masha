// The publish decision for one tick.
//
// No triangle is solved here. The four checks are the whole file:
// finite numbers, the solver's own yes, the driver travel window, and
// the step from the last sent pose. The first failure returns. A later
// check does not get a vote.
//
// The window numbers are the leg rows of
// src/driver/kinematics/kinematics/config.py, SERVOS ids 1..18.
// JointControl refuses a radian when
//   |direction * (offset + radians)| > |max_radians / 2|.
// This file uses that comparison. It does not invent a tighter one.
// Leg servos are 240° of travel around pulse 500, so the half-window
// is ±120° (±2.094 rad). Offsets in that table are 0. Directions are
// copied per joint, because a later non-zero offset would shift the
// window on the side the driver actually uses.

#include "hexapod_kinematics/safety.hpp"

#include <cmath>
#include <cstdlib>

namespace hexapod_kinematics {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Full travel of a leg servo. config.py writes math.radians(240).
constexpr double kLegTravelRad = 240.0 * kPi / 180.0;

// Measured step limit. scripts/calibrate_max_step.py, vendor backend,
// no servos (pseudo). One FollowGaitGenerator cycle from DEFAULT_POSE,
// hunter lift 35 mm, period 0.60 s, Twist clamps vx = +0.12 m/s,
// vy = +0.10 m/s, wz = +0.6 rad/s. Thirty frames. The largest |Δq|
// in one tick of that cycle is 0.17622116866774196 rad, on femur_LF
// (joint 2), the step into frame 5. The other seven sign combinations
// of those clamps land on the same peak, mirrored onto another femur.
// Each axis alone is smaller. This constant is that peak times 1.5.
//
// The first tick of that same command, from the stand into the intro
// splice, is about 0.328 rad. That is a catch-up, not a reason to
// raise this number. A knee flip is still about a radian.
//
// A 0.5 mm foot error asks the knee for about 0.0066 rad at the
// left-front stand, so the stand stays inside the soft edge. The same
// error asks for about 0.116 rad when the foot is 0.2 mm short of
// straight, which is under this limit, and about 0.37 rad when the
// foot is 0.02 mm short, which is over it. The near_singular tests
// use that closer foot.
constexpr double kMaxStepRad = 0.17622116866774196 * 1.5;

// Index 0 is kinematic joint 1. Brace order is name, direction,
// offset, full travel. Directions are the driver's, including the
// right-side femur and tibia signs.
constexpr ServoLimit kLegServos[kJointCount] = {
    {"coxa_joint_LF", -1, 0.0, kLegTravelRad},
    {"femur_joint_LF", -1, 0.0, kLegTravelRad},
    {"tibia_joint_LF", -1, 0.0, kLegTravelRad},
    {"coxa_joint_LM", -1, 0.0, kLegTravelRad},
    {"femur_joint_LM", -1, 0.0, kLegTravelRad},
    {"tibia_joint_LM", -1, 0.0, kLegTravelRad},
    {"coxa_joint_LR", -1, 0.0, kLegTravelRad},
    {"femur_joint_LR", -1, 0.0, kLegTravelRad},
    {"tibia_joint_LR", -1, 0.0, kLegTravelRad},
    {"coxa_joint_RR", -1, 0.0, kLegTravelRad},
    {"femur_joint_RR", 1, 0.0, kLegTravelRad},
    {"tibia_joint_RR", 1, 0.0, kLegTravelRad},
    {"coxa_joint_RM", -1, 0.0, kLegTravelRad},
    {"femur_joint_RM", 1, 0.0, kLegTravelRad},
    {"tibia_joint_RM", 1, 0.0, kLegTravelRad},
    {"coxa_joint_RF", -1, 0.0, kLegTravelRad},
    {"femur_joint_RF", 1, 0.0, kLegTravelRad},
    {"tibia_joint_RF", 1, 0.0, kLegTravelRad},
};

// joint_id is 1..18. The hinge inside the leg is 0 coxa, 1 femur, 2 tibia.
double joint_angle(const std::array<JointAngles, kLegCount>& pose, int joint_id) {
  const int zero_based = joint_id - 1;
  const JointAngles& leg = pose[static_cast<std::size_t>(zero_based / 3)];
  switch (zero_based % 3) {
    case 0:
      return leg.coxa_rad;
    case 1:
      return leg.femur_rad;
    default:
      return leg.tibia_rad;
  }
}

SafetyDecision refuse_joint(IkReason reason, int joint_id, double q_now, double q_previous) {
  SafetyDecision decision;
  decision.allow = false;
  decision.reason = reason;
  decision.joint_id = joint_id;
  decision.q_now_rad = q_now;
  decision.q_previous_rad = q_previous;
  return decision;
}

}  // namespace

const ServoLimit* leg_servo_or_null(int joint_id) {
  if (joint_id < 1 || joint_id > kJointCount) {
    return nullptr;
  }
  return &kLegServos[joint_id - 1];
}

double max_step_rad() {
  return kMaxStepRad;
}

SafetyDecision gate(const PoseResult& command,
                    const std::array<JointAngles, kLegCount>* previous) {
  // 1. Finite. A NaN in an earlier joint hides a later window problem
  // on purpose: the number is not an angle, so the window is meaningless.
  for (int joint_id = 1; joint_id <= kJointCount; ++joint_id) {
    const double q_now = joint_angle(command.angles, joint_id);
    if (!std::isfinite(q_now)) {
      const double q_previous = previous == nullptr ? 0.0 : joint_angle(*previous, joint_id);
      return refuse_joint(IkReason::NonFinite, joint_id, q_now, q_previous);
    }
  }
  if (previous != nullptr) {
    for (int joint_id = 1; joint_id <= kJointCount; ++joint_id) {
      const double q_previous = joint_angle(*previous, joint_id);
      if (!std::isfinite(q_previous)) {
        return refuse_joint(IkReason::NonFinite, joint_id, joint_angle(command.angles, joint_id),
                            q_previous);
      }
    }
  }

  // 2. The solver already refused this foot. Keep its reason. Do not
  // re-label it as a window or a step. joint_id stays 0: the failure
  // is a leg, and command.leg already names it.
  if (!command.ok) {
    SafetyDecision decision;
    decision.allow = false;
    decision.reason = command.reason;
    decision.joint_id = 0;
    return decision;
  }

  // 3. Driver window, before any step comparison. One joint past the
  // half-travel refuses the pose. The angle is not pulled back to ±120°.
  for (int joint_id = 1; joint_id <= kJointCount; ++joint_id) {
    const ServoLimit* servo = leg_servo_or_null(joint_id);
    if (servo == nullptr) {
      std::abort();
    }
    const double q_now = joint_angle(command.angles, joint_id);
    const double real_radians = servo->direction * (servo->offset_rad + q_now);
    if (std::abs(real_radians) > std::abs(servo->max_radians / 2.0)) {
      const double q_previous = previous == nullptr ? 0.0 : joint_angle(*previous, joint_id);
      return refuse_joint(IkReason::JointLimit, joint_id, q_now, q_previous);
    }
  }

  // 4. No last-sent pose. The step check would otherwise subtract zero
  // and treat an unsolved reference as a real stance.
  if (previous == nullptr) {
    SafetyDecision decision;
    decision.allow = false;
    decision.reason = IkReason::NoReference;
    decision.joint_id = 0;
    return decision;
  }

  // 5. Step from the pose this gate last allowed the caller to send.
  // Equal to the limit is allowed. The comparison is >, matching ≤.
  for (int joint_id = 1; joint_id <= kJointCount; ++joint_id) {
    const double q_now = joint_angle(command.angles, joint_id);
    const double q_previous = joint_angle(*previous, joint_id);
    if (std::abs(q_now - q_previous) > max_step_rad()) {
      return refuse_joint(IkReason::RateLimit, joint_id, q_now, q_previous);
    }
  }

  SafetyDecision decision;
  decision.allow = true;
  decision.reason = IkReason::None;
  decision.joint_id = 0;
  return decision;
}

}  // namespace hexapod_kinematics

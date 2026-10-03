#pragma once

// What is allowed to reach a servo.
//
// This file does not fold a triangle and does not pick a foot. solve_pose
// has already named the angles. The gate answers one question: given
// those angles and the angles last actually sent, may this pose be
// published?
//
// The answer is one decision for all eighteen joints. A refusal names
// the joint when one joint is the cause. The caller's angles are left
// as they arrived. Nothing here clamps a hinge or substitutes a shorter
// step. A shorter step is the caller's later repair, outside this file.

#include "hexapod_kinematics/leg_ik.hpp"

#include <array>

namespace hexapod_kinematics {

// Kinematic servo order: coxa, femur, tibia, leg 1 then leg 2, through
// leg 6. Joint 1 is the front-left coxa. Joint 18 is the front-right
// tibia. These are the ids in kinematics/config.py SERVOS, not the
// physical bus ids stored under each row's 'id' field. Head joints 19
// and 20 are not legs and are not in this gate.
constexpr int kJointCount = 18;

// One row of the driver's SERVOS table, for a leg joint.
// direction and offset are the driver's conversion from a solver radian
// to the servo's own radian. max_radians is the full travel, 240° for
// every leg servo. The half-window is half of that.
struct ServoLimit {
  const char* name = "";
  int direction = 1;
  double offset_rad = 0.0;
  double max_radians = 0.0;
};

// nullptr when joint_id is outside 1..18. The gate calls this only
// with ids it counted itself.
const ServoLimit* leg_servo_or_null(int joint_id);

// The step a hunter tick is still allowed to take, radians.
//
// safety.cpp stores 1.5 times the peak measured by
// scripts/calibrate_max_step.py. The knee's near_singular
// check uses this same number.
double max_step_rad();

// The gate's answer. allow is the gate's word. The solver's word is ok.
// A pose with allow false publishes nothing.
//
// joint_id is 1..18 when one joint caused the refusal, and 0 when the
// refusal is the whole pose (the solver said no, or there is no
// previous sent pose) or when allow is true.
// q_now_rad and q_previous_rad are the pair a rate_limit line prints.
// q_now_rad is also set when one joint is non-finite or past the window.
struct SafetyDecision {
  bool allow = false;
  IkReason reason = IkReason::NoReference;
  int joint_id = 0;
  double q_now_rad = 0.0;
  double q_previous_rad = 0.0;
};

// command is what solve_pose returned for this tick.
// previous is the last pose the caller sent after this gate allowed it.
// The caller owns that array. nullptr means there is no such pose:
// the bout has not sent one, or the reference solve of the body pose
// did not succeed. The controller's joints_state starts at zero and
// is not this argument.
//
// Checked in this order, and the first failure is the decision:
//   1. Any non-finite angle → non_finite, naming that joint.
//   2. command.ok is false → the solver's reason, joint_id 0.
//   3. A joint outside the driver window → joint_limit, naming it.
//   4. previous is nullptr → no_reference.
//   5. A joint step larger than max_step_rad → rate_limit, naming it
//      and both angles.
// Otherwise allow is true and reason is None.
//
// This function does not raise. gtest calls it directly. The 20 ms
// loop calls it once per tick, after solve_pose, through the binding.
SafetyDecision gate(const PoseResult& command,
                    const std::array<JointAngles, kLegCount>* previous);

}  // namespace hexapod_kinematics

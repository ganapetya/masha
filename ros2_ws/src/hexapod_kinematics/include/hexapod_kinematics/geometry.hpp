#pragma once

// Robot constants for the leg model. No formula lives here.
//
// A later file is allowed to read these numbers and is not allowed to
// hide a different copy. Lengths are millimetres. Angles are radians.
// Nothing in this header talks to a servo or to ROS.

#include "hexapod_kinematics/types.hpp"

namespace hexapod_kinematics {

// Where the coxa yaw axis meets the body, in the body frame, and the
// direction that axis's servo calls zero.
// mount_yaw_rad is the heading of the foot when the coxa angle is 0.
// x_mm and y_mm are the build_in_pose offsets X1, Y1, Y2. They are not
// the URDF leg_center. hip_z is shared; see LinkLengths.
struct Hip {
  LegId leg = LegId::Lf;
  const char* name = "";
  double x_mm = 0.0;
  double y_mm = 0.0;
  double mount_yaw_rad = 0.0;
};

// Shared by all six legs. Mirror is the mount yaw, not a second set of
// lengths. These are starting guesses. The round-trip test is allowed
// to change them, and the comment in geometry.cpp must name that test
// when a number moves.
struct LinkLengths {
  double coxa_mm = 0.0;          // yaw axis to femur hinge, horizontal
  double femur_mm = 0.0;         // L1 in Figure 6.1, femur axis to tibia axis
  double tibia_mm = 0.0;         // L2 in Figure 6.1, tibia axis to foot tip
  double hip_z_mm = 0.0;         // yaw-axis height in the IK frame
  double coxa_femur_z_mm = 0.0;  // femur hinge minus yaw-axis origin, along yaw
};

// Added to the geometric angles before they are compared with a servo.
// The stand contract filled this shared triple. Coxa is 0. Femur and
// tibia are the gap between the drawing's zero and the servo's zero.
// A per-leg table was not needed. geometry.cpp names the test and the
// residuals.
struct JointZero {
  double coxa_rad = 0.0;
  double femur_rad = 0.0;
  double tibia_rad = 0.0;
};

// Vendor stand of the left-front leg, radians. This is the measuring tape
// the stand contract compares against. It is not a joint-zero offset.
struct StandTape {
  double coxa_rad = 0.0;
  double femur_rad = 0.0;
  double tibia_rad = 0.0;
};

// The six hips, leg 1 at index 0. The list is fixed for the life of the
// process. Callers do not free it.
const Hip& hip(LegId id);

// nullptr when id is not one of the six. hip() requires a real LegId;
// hip_or_null is the check for a value that came from outside.
const Hip* hip_or_null(LegId id);

const LinkLengths& link_lengths();
const JointZero& joint_zero();
const StandTape& stand_tape_lf();

}  // namespace hexapod_kinematics

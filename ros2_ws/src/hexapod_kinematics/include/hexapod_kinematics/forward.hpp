#pragma once

// The reverse of the leg solve: three angles in, one foot out.
//
// Forward kinematics walks the links. Inverse kinematics, in coxa and
// planar_leg, answers the other way. This file is how a test checks
// that the two maps undo each other. The gait loop does not call it.

#include "hexapod_kinematics/types.hpp"

namespace hexapod_kinematics {

// Foot position, body frame, millimetres, for one leg's servo angles.
//
// angles are radians in the sense solve_leg returns: joint zeros are
// already included. This function subtracts those zeros first, then
// walks the three links with the drawing's angles.
//
// id must be one of the six legs. hip() aborts otherwise.
// The angles are read for this call and not stored.
// gtest calls this directly.
Vec3 forward_foot(LegId id, const JointAngles& angles);

}  // namespace hexapod_kinematics

#pragma once

#include "servo_controller_msgs/msg/servos_position.hpp"

namespace proud_up {

// Arm bus-servo pulses (ids 19-24). Defaults match ActionGroups/init_horizontal.d6a:
// arm straight, wrist level, camera looking forward.
//
// Bus protocol: 0–1000 ticks over ~240° of the servo horn.
// Hunter only *changes* 19 (pan, camera yaw) and 22 (tilt, camera pitch).
// The other four stay at rest so the arm does not fold while we hunt.
//
//   id19  pan     500 = centre. Hunt sweeps 200–800.
//   id20  shoulder-out  810 = arm extended
//   id21  shoulder-down 180
//   id22  tilt    yaml hunter pose is 117 (Saveli is a low car). Rest 150.
//   id23  wrist
//   id24  gripper
struct ArmPulses {
  float id19{500.0f};
  float id20{810.0f};
  float id21{180.0f};
  float id22{150.0f};
  float id23{500.0f};
  float id24{500.0f};
  double duration_s{1.5};  // seconds the bus interpolates toward these pulses
};

// Pack into servo_controller/ServosPosition. position_unit = "pulse"
// (not radians). Hunter publishes this on topic servo_controller.
servo_controller_msgs::msg::ServosPosition make_arm_command(const ArmPulses &pose);

}  // namespace proud_up

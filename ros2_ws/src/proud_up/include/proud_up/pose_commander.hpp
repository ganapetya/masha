#pragma once

#include "servo_controller_msgs/msg/servos_position.hpp"

namespace proud_up {

// Arm bus-servo pulses (ids 19-24). Defaults match ActionGroups/init_horizontal.d6a:
// arm straight, wrist level, camera looking forward.
struct ArmPulses {
  float id19{500.0f};
  float id20{810.0f};
  float id21{180.0f};
  float id22{150.0f};
  float id23{500.0f};
  float id24{500.0f};
  double duration_s{1.5};
};

servo_controller_msgs::msg::ServosPosition make_arm_command(const ArmPulses &pose);

}  // namespace proud_up

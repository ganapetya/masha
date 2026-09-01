#include "proud_up/pose_commander.hpp"

#include <cstdint>
#include <utility>

#include "servo_controller_msgs/msg/servo_position.hpp"

namespace proud_up {

servo_controller_msgs::msg::ServosPosition make_arm_command(const ArmPulses &pose) {
  servo_controller_msgs::msg::ServosPosition msg;
  msg.duration = pose.duration_s;
  msg.position_unit = "pulse";

  const std::pair<uint16_t, float> joints[] = {
      {19, pose.id19}, {20, pose.id20}, {21, pose.id21},
      {22, pose.id22}, {23, pose.id23}, {24, pose.id24},
  };
  for (const auto &joint : joints) {
    servo_controller_msgs::msg::ServoPosition servo;
    servo.id = joint.first;
    servo.position = joint.second;
    msg.position.push_back(servo);
  }
  return msg;
}

}  // namespace proud_up

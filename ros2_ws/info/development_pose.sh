#!/bin/bash
# Development pose: holding torque off, or back on, for every bus servo.
#
# This is not a foot pose. RELAX_POSE still commands angles, so the motors
# keep current in the windings. Here the board is told to drop that current.
# Support the body before "off". The legs will not hold Masha up.
# A later position command (a pose, a gait, init) turns torque back on,
# because a move packet is a hold.
#
# The board message uses two-element arrays. The first element means
# "this field is set" and must be 1. The second is the value.
#   present_id:    [1, <bus id>]
#   enable_torque: [1, 0]   off
#   enable_torque: [1, 1]   on, hold the angle the servo is at now
#
# Usage, from a terminal on Masha:
#   ~/ros2_ws/info/development_pose.sh off
#   ~/ros2_ws/info/development_pose.sh on

set -euo pipefail
export ROS_DOMAIN_ID=27

mode="${1:-off}"
case "$mode" in
  off) value=0 ;;
  on)  value=1 ;;
  *)
    echo "usage: $(basename "$0") [off|on]" >&2
    exit 1
    ;;
esac

if [[ "$value" == "0" ]]; then
  echo "Sending torque off on servos 1-24. Support the body. The legs go limp only after 'sent'." >&2
else
  echo "Sending torque on on servos 1-24. Each servo will hold the angle it has now." >&2
fi

# setup.bash reads AMENT_TRACE_SETUP_FILES, which is normally unset.
# set -u would abort there, before any servo command is published.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/ubuntu/ros2_ws/install/setup.bash
set -u

yaml="state:"$'\n'
for id in $(seq 1 24); do
  yaml+="- present_id: [1, ${id}]"$'\n'
  yaml+="  enable_torque: [1, ${value}]"$'\n'
done
yaml+="duration: 0.0"$'\n'

ros2 topic pub --once /ros_robot_controller/bus_servo/set_state \
  ros_robot_controller_msgs/msg/SetBusServoState \
  "$yaml"
echo "sent" >&2

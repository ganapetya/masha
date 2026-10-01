#!/bin/bash
# Read every bus servo's temperature and print three columns:
#   id, hinge name, degrees Celsius.
#
#   ~/ros2_ws/info/servo_temperatures.sh
#   5   left-front-body-coxa       34
#   16  right-rear-coxa-femur      33
#   2   right-front-femur-tibia    34
#
# The hinge name is side-place-joint. body-coxa is the yaw servo,
# coxa-femur is the femur servo, femur-tibia is the tibia servo.
# A '?' means that servo did not answer. This only reads.
#
# get_voltage and get_torque_state stay off. The running driver calls
# method names the board code does not have, and those calls throw.

set -euo pipefail
export ROS_DOMAIN_ID=27

# setup.bash reads AMENT_TRACE_SETUP_FILES, which is normally unset.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source /home/ubuntu/ros2_ws/install/setup.bash
set -u

python3 - << 'PY'
import rclpy
from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState

rclpy.init()
node = rclpy.create_node("servo_temperatures")
client = node.create_client(GetBusServoState, "/ros_robot_controller/bus_servo/get_state")
if not client.wait_for_service(timeout_sec=5.0):
    raise SystemExit("service /ros_robot_controller/bus_servo/get_state is not up")

request = GetBusServoState.Request()
for servo_id in range(1, 25):
    cmd = GetBusServoCmd()
    cmd.id = servo_id
    cmd.get_temperature = 1
    request.cmd.append(cmd)

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
if not future.done() or future.result() is None:
    raise SystemExit("temperature read did not return")

# Bus id -> hinge. Leg names follow servo_controller.yaml.
# body-coxa is the coxa servo, coxa-femur the femur servo, femur-tibia the tibia servo.
HINGE = {
    1: "left-front-femur-tibia",
    2: "right-front-femur-tibia",
    3: "left-front-coxa-femur",
    4: "right-front-coxa-femur",
    5: "left-front-body-coxa",
    6: "right-front-body-coxa",
    7: "left-middle-femur-tibia",
    8: "right-middle-femur-tibia",
    9: "left-middle-coxa-femur",
    10: "right-middle-coxa-femur",
    11: "left-middle-body-coxa",
    12: "right-middle-body-coxa",
    13: "left-rear-femur-tibia",
    14: "right-rear-femur-tibia",
    15: "left-rear-coxa-femur",
    16: "right-rear-coxa-femur",
    17: "left-rear-body-coxa",
    18: "right-rear-body-coxa",
    19: "arm-joint1",
    20: "arm-joint2",
    21: "arm-joint3",
    22: "arm-joint4",
    23: "arm-joint5",
    24: "arm-gripper",
}

result = future.result()
print(f"{'id':<4}{'hinge':<28}C")
for servo_id, state in zip(range(1, 25), result.state):
    degrees = str(int(state.temperature[0])) if state.temperature else "?"
    print(f"{servo_id:<4}{HINGE[servo_id]:<28}{degrees}")

node.destroy_node()
rclpy.shutdown()
PY

cmake_minimum_required(VERSION 3.10)
project(hexapod_kinematics)

# This revision installs the plan and nothing else.
# The leg solver, the safety gate, and the Python plug come in later
# commits. See PLAN.md, work order item 1.

find_package(ament_cmake REQUIRED)

install(FILES PLAN.md README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

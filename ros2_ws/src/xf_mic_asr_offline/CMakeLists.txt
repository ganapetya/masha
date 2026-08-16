cmake_minimum_required(VERSION 3.8)
project(xf_mic_asr_offline)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Voice is Python + sherpa-onnx. Do not add a vendor ASR binary here.

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/asr_node.py
  scripts/voice_control_move.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch feedback_voice
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

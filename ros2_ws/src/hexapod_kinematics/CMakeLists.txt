cmake_minimum_required(VERSION 3.10)
project(hexapod_kinematics)

# Pure leg geometry. No rclcpp. Coxa, the knee triangle, forward,
# solve_leg, solve_pose, the safety gate, and the pybind11 module are
# in this package. The controller plug is a later step. See PLAN.md.
# -O3 so hypot / atan2 / acos can inline. The Python module gets the
# same flag. No -ffast-math: the gate treats NaN, and the knee treats
# a cosine outside [-1, 1], as real failures. Fast-math is allowed to
# break both.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# Development.Module is the piece an extension links. pybind11 2.9's
# pybind11_add_module calls python3_add_library, and CMake 4 names that
# command Python3_add_library, so the module is added with that command.
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/geometry.cpp
  src/coxa.cpp
  src/planar_leg.cpp
  src/forward.cpp
  src/leg_ik.cpp
  src/safety.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

# Python module name is hexapod_kinematics, so `import hexapod_kinematics`
# loads this file. The C++ library keeps the lib prefix, so the two
# files do not share a name. The module is conversion only.
Python3_add_library(hexapod_kinematics_py MODULE WITH_SOABI src/bindings.cpp)
target_link_libraries(hexapod_kinematics_py PRIVATE ${PROJECT_NAME} pybind11::headers)
target_compile_options(hexapod_kinematics_py PRIVATE -O3)
set_target_properties(hexapod_kinematics_py PROPERTIES
  OUTPUT_NAME hexapod_kinematics
  # site-packages sits two directories under lib/, next to the C++ library.
  INSTALL_RPATH "$ORIGIN/../.."
)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS hexapod_kinematics_py
  DESTINATION ${PYTHON_INSTALL_DIR}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES PLAN.md README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry ${PROJECT_NAME})
  ament_add_gtest(test_coxa test/test_coxa.cpp)
  target_link_libraries(test_coxa ${PROJECT_NAME})
  ament_add_gtest(test_planar_leg test/test_planar_leg.cpp)
  target_link_libraries(test_planar_leg ${PROJECT_NAME})
  ament_add_gtest(test_forward test/test_forward.cpp)
  target_link_libraries(test_forward ${PROJECT_NAME})
  ament_add_gtest(test_leg_ik test/test_leg_ik.cpp)
  target_link_libraries(test_leg_ik ${PROJECT_NAME})
  ament_add_gtest(test_solve_pose test/test_solve_pose.cpp)
  target_link_libraries(test_solve_pose ${PROJECT_NAME})
  ament_add_gtest(test_safety test/test_safety.cpp)
  target_link_libraries(test_safety ${PROJECT_NAME})
  ament_add_pytest_test(test_bindings test/test_bindings.py)
endif()

ament_package()

cmake_minimum_required(VERSION 3.10)
project(hexapod_kinematics)

# Pure leg geometry. No rclcpp. The coxa yaw is in this library.
# The triangle, the safety gate, and the Python plug are later steps.
# See PLAN.md.
# -O3 so hypot / atan2 / acos can inline once those files exist.
# No -ffast-math: the later gate treats NaN and a cosine outside [-1, 1]
# as real failures, and fast-math is allowed to break both.

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/geometry.cpp
  src/coxa.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(${PROJECT_NAME} PRIVATE -O3)

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
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_link_libraries(test_geometry ${PROJECT_NAME})
  ament_add_gtest(test_coxa test/test_coxa.cpp)
  target_link_libraries(test_coxa ${PROJECT_NAME})
endif()

ament_package()

cmake_minimum_required(VERSION 3.10)
project(proud_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kinematics_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(servo_controller_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs objdetect dnn)
find_package(CURL REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Prefer the Jetson overlay (pose + size) over Humble's corner-only message
# so we match the apriltag_ros that actually runs on this board.
if(NOT apriltag_msgs_DIR AND EXISTS
    "/home/ubuntu/third_party/third_party_ws/install/apriltag_msgs/share/apriltag_msgs/cmake")
  set(apriltag_msgs_DIR
      "/home/ubuntu/third_party/third_party_ws/install/apriltag_msgs/share/apriltag_msgs/cmake")
endif()
find_package(apriltag_msgs REQUIRED)

add_library(proud_up_core STATIC
  src/camera_frame.cpp
  src/load_dotenv.cpp
  src/mailer.cpp
  src/pose_commander.cpp
)
target_include_directories(proud_up_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(proud_up_core
  rclcpp
  sensor_msgs
  servo_controller_msgs
)
target_link_libraries(proud_up_core
  ${OpenCV_LIBS}
  CURL::libcurl
)

# Geometry + detection only. No rclcpp — gtest links this without spinning a node.
add_library(follow_the_cat STATIC src/follow_the_cat.cpp)
target_include_directories(follow_the_cat PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(follow_the_cat PUBLIC ${OpenCV_LIBS})
if(TARGET Eigen3::Eigen)
  target_link_libraries(follow_the_cat PUBLIC Eigen3::Eigen)
endif()

# Variant-3 omnidirectional tripod map. No rclcpp — gtest links this alone.
add_library(follow_gait STATIC src/follow_gait.cpp)
target_include_directories(follow_gait PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Hunter policy + target plugs. No rclcpp in hunter.cpp.
add_library(hunter STATIC
  src/hunter.cpp
  src/target_saveli.cpp
  src/target_cat.cpp
)
target_include_directories(hunter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(hunter PUBLIC follow_gait follow_the_cat ${OpenCV_LIBS})

add_executable(proud_up_node src/proud_up_node.cpp)
target_include_directories(proud_up_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(proud_up_node
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
  servo_controller_msgs
)
target_link_libraries(proud_up_node
  proud_up_core
  ${OpenCV_LIBS}
)

# Stage manager: "who is your master" → spoken line → master_dance.d6a.
# Header-only phrases live in include/proud_up/interaction_phrases.hpp
# (no extra library; the node and the gtest both include it).
add_executable(masha_interaction_node src/masha_interaction_node.cpp)
target_include_directories(masha_interaction_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
ament_target_dependencies(masha_interaction_node
  rclcpp
  geometry_msgs
  kinematics_msgs
  interfaces
  std_msgs
  std_srvs
)

add_executable(follow_the_cat_node src/follow_the_cat_node.cpp)
target_include_directories(follow_the_cat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(follow_the_cat_node
  rclcpp
  sensor_msgs
  geometry_msgs
  kinematics_msgs
  std_srvs
  cv_bridge
  servo_controller_msgs
  ament_index_cpp
)
target_link_libraries(follow_the_cat_node
  follow_the_cat
  proud_up_core
  ${OpenCV_LIBS}
)

add_executable(masha_hunter_node src/masha_hunter_node.cpp)
target_include_directories(masha_hunter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(masha_hunter_node
  rclcpp
  sensor_msgs
  geometry_msgs
  kinematics_msgs
  std_srvs
  cv_bridge
  servo_controller_msgs
  ament_index_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  apriltag_msgs
)
target_link_libraries(masha_hunter_node
  hunter
  follow_gait
  follow_the_cat
  proud_up_core
  ${OpenCV_LIBS}
)

install(TARGETS proud_up_node follow_the_cat_node masha_interaction_node masha_hunter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config models sounds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_frame test/test_camera_frame.cpp)
  target_include_directories(test_camera_frame PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
  )
  ament_target_dependencies(test_camera_frame rclcpp servo_controller_msgs)
  target_link_libraries(test_camera_frame proud_up_core ${OpenCV_LIBS} CURL::libcurl)

  ament_add_gtest(test_follow_the_cat test/test_follow_the_cat.cpp)
  target_include_directories(test_follow_the_cat PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
  )
  target_link_libraries(test_follow_the_cat follow_the_cat ${OpenCV_LIBS})

  ament_add_gtest(test_interaction_phrases test/test_interaction_phrases.cpp)
  target_include_directories(test_interaction_phrases PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_follow_gait test/test_follow_gait.cpp)
  target_include_directories(test_follow_gait PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_follow_gait follow_gait)

  ament_add_gtest(test_hunter test/test_hunter.cpp)
  target_include_directories(test_hunter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(test_hunter hunter follow_gait follow_the_cat ${OpenCV_LIBS})
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(fm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
# Vision 1:1 mirroring: host moveit_servo::PoseTracking (pose_tracking_node).
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(control_node src/control_node.cpp)
target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(control_node PUBLIC cxx_std_17)
ament_target_dependencies(control_node rclcpp)

# G1-D right-arm Servo -> arm_sdk bridge (the `real` backend; replaces ros2_control).
add_executable(g1_arm_sdk_bridge src/g1_arm_sdk_bridge.cpp)
target_include_directories(g1_arm_sdk_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(g1_arm_sdk_bridge PUBLIC cxx_std_17)
ament_target_dependencies(g1_arm_sdk_bridge rclcpp trajectory_msgs unitree_hg)

# G1-D base teleop: Twist -> Unitree AGV Move RPC (separate from Servo).
add_executable(g1_base_teleop src/g1_base_teleop.cpp)
target_include_directories(g1_base_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(g1_base_teleop PUBLIC cxx_std_17)
ament_target_dependencies(g1_base_teleop rclcpp geometry_msgs unitree_api)

# G1-D Dex3 hand bridge (the `real` backend; replaces ros2_control for the hands).
add_executable(g1_hand_sdk_bridge src/g1_hand_sdk_bridge.cpp)
target_include_directories(g1_hand_sdk_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(g1_hand_sdk_bridge PUBLIC cxx_std_17)
ament_target_dependencies(g1_hand_sdk_bridge rclcpp trajectory_msgs unitree_hg)

install(TARGETS control_node g1_arm_sdk_bridge g1_base_teleop g1_hand_sdk_bridge
  DESTINATION lib/${PROJECT_NAME})

# Vision 1:1 mirroring: hosts moveit_servo::PoseTracking, servoing the EE to an absolute
# /target_pose and holding. Replaces servo_node on the mirror teleop path.
# PoseTracking exists only in Humble's moveit_servo; Jazzy's rewrite removed it, so the
# node is skipped there until it is ported to moveit_servo::Servo.
if("$ENV{ROS_DISTRO}" STREQUAL "humble")
  add_executable(pose_tracking_node src/pose_tracking_node.cpp)
  target_compile_features(pose_tracking_node PUBLIC cxx_std_17)
  ament_target_dependencies(pose_tracking_node
    rclcpp
    geometry_msgs
    moveit_servo
    moveit_ros_planning
    moveit_core
    Eigen3)
  install(TARGETS pose_tracking_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "fm_control: skipping pose_tracking_node on $ENV{ROS_DISTRO} "
    "(moveit_servo PoseTracking is Humble-only)")
endif()
install(DIRECTORY include/ DESTINATION include)

# Backend-selectable ros2_control xacro consumed by the sim/teleop launches.
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
# Sim-only MoveIt Servo for the OpenArm on Jazzy: SRDF, kinematics, params, launch.
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_g1_arm_sdk_bridge test/test_g1_arm_sdk_bridge.cpp)
  target_include_directories(test_g1_arm_sdk_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_g1_arm_sdk_bridge unitree_hg)

  ament_add_gtest(test_g1_base_teleop test/test_g1_base_teleop.cpp)
  target_include_directories(test_g1_base_teleop PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_g1_base_teleop unitree_api)

  ament_add_gtest(test_g1_hand_sdk_bridge test/test_g1_hand_sdk_bridge.cpp)
  target_include_directories(test_g1_hand_sdk_bridge PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_g1_hand_sdk_bridge unitree_hg)

  # Process each installed robot xacro for every sim_backend value.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_openarm_xacro test/test_openarm_xacro.py
    TIMEOUT 120)
  ament_add_pytest_test(test_axol_xacro test/test_axol_xacro.py
    TIMEOUT 120)
  # Checks the sim Servo SRDF and params against the generated URDF and moveit_servo.
  ament_add_pytest_test(test_openarm_sim_servo test/test_openarm_sim_servo.py
    TIMEOUT 120)
  ament_add_pytest_test(test_so101_xacro test/test_so101_xacro.py
    TIMEOUT 120)
  ament_add_pytest_test(test_g1_xacro test/test_g1_xacro.py
    TIMEOUT 120)
endif()

ament_package()

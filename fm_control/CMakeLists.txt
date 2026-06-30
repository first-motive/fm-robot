cmake_minimum_required(VERSION 3.8)
project(fm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mujoco_ros2_control_plugins REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)

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

add_library(fm_control_mujoco_plugins SHARED
  src/so101_task_env_plugin.cpp
)
target_include_directories(fm_control_mujoco_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(fm_control_mujoco_plugins PUBLIC cxx_std_17)
ament_target_dependencies(fm_control_mujoco_plugins
  mujoco_ros2_control_plugins
  pluginlib
  rclcpp
  realtime_tools
  visualization_msgs
)
pluginlib_export_plugin_description_file(
  mujoco_ros2_control_plugins
  fm_control_mujoco_plugins.xml
)

install(TARGETS
  control_node
  g1_arm_sdk_bridge
  g1_base_teleop
  g1_hand_sdk_bridge
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS fm_control_mujoco_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES fm_control_mujoco_plugins.xml DESTINATION share/${PROJECT_NAME})

# Backend-selectable ros2_control xacro consumed by the sim/teleop launches.
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

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

  ament_add_gtest(test_so101_task_env_markers test/test_so101_task_env_markers.cpp)
  target_include_directories(test_so101_task_env_markers PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_so101_task_env_markers rclcpp visualization_msgs)
endif()

ament_package()

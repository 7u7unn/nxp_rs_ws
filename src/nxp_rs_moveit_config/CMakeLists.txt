cmake_minimum_required(VERSION 3.8)
project(nxp_rs_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(floor_scene src/floor_scene.cpp)
target_compile_features(floor_scene PRIVATE cxx_std_17)
ament_target_dependencies(floor_scene geometry_msgs moveit_msgs rclcpp shape_msgs)

install(TARGETS floor_scene
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(joint_trajectory_controller REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(moveit_core REQUIRED)
  ament_add_gtest(test_hardware_execution test/test_hardware_execution.cpp
    TIMEOUT 60 ENV ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/nxp_rs_execution_test_logs)
  target_compile_features(test_hardware_execution PRIVATE cxx_std_17)
  target_compile_definitions(test_hardware_execution PRIVATE
    PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_target_dependencies(test_hardware_execution joint_trajectory_controller rclcpp_action)
  ament_add_gtest(test_collision_geometry test/test_collision_geometry.cpp TIMEOUT 60)
  target_compile_features(test_collision_geometry PRIVATE cxx_std_17)
  target_compile_definitions(test_collision_geometry PRIVATE
    PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  ament_target_dependencies(test_collision_geometry moveit_core)
endif()

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

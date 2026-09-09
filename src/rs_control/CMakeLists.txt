cmake_minimum_required(VERSION 3.8)
project(rs_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(rs_control_core SHARED
  src/motor_config.cpp
  src/robstride_bus.cpp
)
target_include_directories(rs_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rs_control_core yaml-cpp)
ament_target_dependencies(rs_control_core rclcpp)

add_library(rs_control_hardware SHARED
  src/robstride_system.cpp
)
target_include_directories(rs_control_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rs_control_hardware rs_control_core)
ament_target_dependencies(rs_control_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(rs_encoder_reader src/rs_encoder_reader.cpp)
target_link_libraries(rs_encoder_reader rs_control_core)
ament_target_dependencies(rs_encoder_reader
  diagnostic_msgs
  rclcpp
  sensor_msgs
)

add_executable(rs_scan src/rs_scan.cpp)
target_link_libraries(rs_scan rs_control_core)

pluginlib_export_plugin_description_file(hardware_interface rs_control_plugins.xml)

install(
  TARGETS
    rs_control_core
    rs_control_hardware
    rs_encoder_reader
    rs_scan
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY python/rs_control_py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    python/rs_python_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rs_python_control
)

install(
  PROGRAMS
    python/rs_python_scan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rs_python_scan
)

install(
  PROGRAMS
    python/rs_python_set_id.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rs_python_set_id
)

install(
  PROGRAMS
    python/rs_jog_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rs_jog_controller
)

install(
  FILES rs_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_pytest_test(test_python_protocol test/test_python_protocol.py)
  ament_add_pytest_test(test_python_bus test/test_python_bus.py)
  ament_add_pytest_test(test_jog test/test_jog.py)

  ament_add_gtest(test_motor_config test/test_motor_config.cpp)
  target_link_libraries(test_motor_config rs_control_core)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol rs_control_core)

  ament_add_gtest(test_hardware_plugin test/test_hardware_plugin.cpp)
  target_link_libraries(test_hardware_plugin rs_control_hardware)
  ament_target_dependencies(test_hardware_plugin hardware_interface rclcpp rclcpp_lifecycle)
endif()

ament_export_include_directories(include)
ament_export_libraries(rs_control_core rs_control_hardware)
ament_export_dependencies(
  diagnostic_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  yaml_cpp_vendor
)

ament_package()

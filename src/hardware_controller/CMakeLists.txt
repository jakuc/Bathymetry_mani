cmake_minimum_required(VERSION 3.8)
project(hardware_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(hardware_controller SHARED
  src/dynamixel_system.cpp
  src/echosounder_sensor.cpp
  src/temperature_broadcaster.cpp
  src/gnss_sensor.cpp
  src/gnss_broadcaster.cpp
  src/imu_sensor.cpp
  src/laser_sensor.cpp
  src/laser_broadcaster.cpp
)
target_include_directories(hardware_controller PRIVATE
  include
)
ament_target_dependencies(hardware_controller
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dynamixel_sdk
  controller_interface
  realtime_tools
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
)

pluginlib_export_plugin_description_file(hardware_interface hardware_controller_plugins.xml)
pluginlib_export_plugin_description_file(controller_interface hardware_controller_controller_plugins.xml)

install(
  TARGETS hardware_controller
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/goal_position_bridge.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME goal_position_bridge
)
install(
  PROGRAMS scripts/spherical_sweep_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME spherical_sweep_node
)
install(
  PROGRAMS scripts/cloud_collector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cloud_collector_node
)

ament_export_include_directories(include)
ament_export_libraries(hardware_controller)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle dynamixel_sdk controller_interface realtime_tools sensor_msgs geometry_msgs diagnostic_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(xm540_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetOrientation.srv"
  "srv/StartSweep.srv"
  "srv/SetBoatPose.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

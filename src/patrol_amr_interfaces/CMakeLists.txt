cmake_minimum_required(VERSION 3.8)
project(patrol_amr_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # AMR PC 내부 감지 연계(설계도 03 페이지 DetectEvent). 공용 계약 아님.
  "action/DetectEvent.action"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

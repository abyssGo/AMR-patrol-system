cmake_minimum_required(VERSION 3.8)
project(patrol_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # patrol_interfaces 2.1.0: 2.0.0 계약 + SetEStop.srv, Patrol REASON_LOW_BATTERY,
  # 사용하지 않는 DetectionEvidence·MissionExecutionEvent 제거.
  "action/Patrol.action"
  "msg/PatrolCommand.msg"
  "msg/DriveToken.msg"
  "msg/EStop.msg"
  "msg/CameraState.msg"
  "srv/ReportDetection.srv"
  "srv/SetEStop.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

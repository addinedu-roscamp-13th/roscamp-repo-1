cmake_minimum_required(VERSION 3.8)
project(automato_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ===== 시나리오 1: 주간 순찰 =====
  # E0 상시 텔레메트리 (ROS2 Topic)
  "msg/ServoStatus.msg"       # DdagiTelemetry.servo_health 원소
  "msg/DdagoTelemetry.msg"    # DdaGo Control Service -> HQ  /dg_01/ddago/telemetry
  "msg/DdagiTelemetry.msg"    # Ddagi Control Service -> HQ  /dg_01/ddagi/telemetry
  "msg/FleetTelemetry.msg"    # HQ -> Automato Control Service / QT  (취합)
  # E1/E2 순찰 waypoint 보조 타입
  "msg/WaypointGoal.msg"
  # E2 촬영/저장 (ROS2 Service)
  "srv/AnalyzeFrame.srv"      # DdaGo Control Service -> HQ  /dg/analyze_frame
  "srv/SaveDetection.srv"     # HQ -> Automato Control Service  /automato/save_detection
  # E1/E2 순찰 (ROS2 Action) — 단일 waypoint. ACS→HQ, HQ→DdaGo 공통 재사용
  "action/Patrol.action"      # ACS->HQ /{robot_id}/patrol, HQ->DdaGo /{robot_id}/ddago/patrol
  DEPENDENCIES std_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

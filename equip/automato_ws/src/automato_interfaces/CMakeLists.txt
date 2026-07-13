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
  "msg/DdagoTelemetry.msg"    # DdaGo Control Service -> DG  /dg_01/ddago/telemetry
  "msg/DdagiTelemetry.msg"    # Ddagi Control Service -> DG  /dg_01/ddagi/telemetry
  "msg/FleetTelemetry.msg"    # DG -> Automato Control Service / QT  (취합)
  # E1/E2 순찰 waypoint 보조 타입
  "msg/WaypointGoal.msg"      # (구) Patrol 보조 — Navigate 마이그레이션 후 삭제 예정
  "msg/Waypoint.msg"          # (신) Navigate 보조, capture 필드 추가
  # E2 촬영/저장 (ROS2 Service)
  "srv/CaptureFrame.srv"      # DdaGo Control Service -> 카메라 노드  /{robot_id}/ddago/capture_frame
  "srv/AnalyzeFrame.srv"      # DdaGo Control Service -> DG  /dg/analyze_frame
  "srv/SaveDetection.srv"     # DG -> Automato Control Service  /automato/save_detection
  # E1/E2 순찰 (ROS2 Action) — ACS→DG, DG→DdaGo 공통 재사용
  "action/Patrol.action"      # (구) 단일 waypoint — Navigate 마이그레이션 후 삭제 예정
  "action/Navigate.action"    # (신) 경로(Waypoint[]) 하달  /{robot_id}/navigate, /{robot_id}/ddago/navigate
  DEPENDENCIES std_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

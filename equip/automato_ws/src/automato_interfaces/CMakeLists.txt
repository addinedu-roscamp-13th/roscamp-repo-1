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
  "msg/DdagoTelemetry.msg"    # DdaGo Control Service -> DG  /ddago/telemetry (로봇 쪽 네임스페이스 없음)
  "msg/DdagiTelemetry.msg"    # Ddagi Control Service -> DG  /ddagi/telemetry (로봇 쪽 네임스페이스 없음)
  "msg/RobotTelemetry.msg"    # DG -> Automato Control Service  /{robot_id}/telemetry (자기 세트분)
  "msg/FleetMember.msg"       # FleetTelemetry.robots 원소 (ACS 가 robot_id 를 붙인다)
  "msg/FleetTelemetry.msg"    # Automato Control Service -> QT  /automato/dashboard/fleet_telemetry (취합)
  # E1/E2 순찰 waypoint 보조 타입
  "msg/WaypointGoal.msg"      # (구) Patrol 보조 — Navigate 마이그레이션 후 삭제 예정
  "msg/Waypoint.msg"          # (신) Navigate 보조, capture 필드 추가
  # E2 촬영/저장 (ROS2 Service)
  "srv/CaptureFrame.srv"      # DdaGo Control Service -> 카메라 노드  /ddago/capture_frame
  "srv/AnalyzeFrame.srv"      # DdaGo Control Service -> DG  /dg/analyze_frame
  "srv/SaveDetection.srv"     # DG -> Automato Control Service  /automato/save_detection
  # E1/E2 순찰 (ROS2 Action) — ACS→DG, DG→DdaGo 공통 재사용
  "action/Patrol.action"      # (구) 단일 waypoint — Navigate 마이그레이션 후 삭제 예정
  "action/Navigate.action"    # (신) 경로(Waypoint[]) 하달  /{robot_id}/navigate, /{robot_id}/ddago/navigate
  # E4-6/E2 22-1 충전소 정밀 후진 도킹
  "action/Dock.action"        # ACS→DG /{robot_id}/dock, DG→DdaGo /ddago/dock
  # 시나리오2 E3~E5 수확 사이클 (Ddagi 주관, DG 중계)
  "action/Harvest.action"     # ACS→DG /{robot_id}/harvest, DG→Ddagi /ddagi/harvest
  DEPENDENCIES std_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

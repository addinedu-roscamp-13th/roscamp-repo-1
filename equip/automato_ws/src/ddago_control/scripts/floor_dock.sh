#!/bin/bash
# ============================================================================
# floor_dock.sh — RP-126 바닥 H 마커 후진 도킹 실행 헬퍼 (DdaGo / ddago03)
#
#   floor_dock.sh server        실주행 서버 (라이다 충돌방지·스트림·디버그)
#   floor_dock.sh dry           dry_run 서버 (모터 미발행 — 검출·라이다 확인만)
#   floor_dock.sh goal          단발 도킹 goal 1회 (다른 터미널에서)
#   floor_dock.sh repeat [N]    N회 반복 도킹 (기본 3, 마지막 회차는 도킹 상태로 종료)
#
#   추가 launch 인자는 그대로 전달됨:
#     floor_dock.sh server obstacle_avoid:=false wall_gap_target:=0.035
#     ROBOT=dg_01 floor_dock.sh server            # 다른 로봇
#
# ⚠️ bringup(odom·cmd_vel·scan) 먼저 떠 있어야 함. cmd_vel 워치독 없음 → Ctrl+C 준비.
# ============================================================================
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-12}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
WS=${WS:-~/roscamp-repo-1/equip/automato_ws}

source /opt/ros/jazzy/setup.bash 2>/dev/null
source "${WS/#\~/$HOME}/install/setup.bash" 2>/dev/null || {
  echo "워크스페이스 소싱 실패: $WS/install/setup.bash"; exit 1; }

ROBOT=${ROBOT:-dg_03}
ACTION=/ddago/floor_dock

case "$1" in
  server)
    echo "[floor_dock] 실주행 서버  robot=$ROBOT  (라이다·스트림·디버그, Ctrl+C 준비)"
    ros2 launch ddago_control ddago_floor_dock.launch.py \
      robot_id:="$ROBOT" debug:=true stream:=true obstacle_avoid:=true "${@:2}"
    ;;
  dry)
    echo "[floor_dock] DRY-RUN 서버  robot=$ROBOT  (모터 미발행 — 검출/라이다 확인)"
    ros2 launch ddago_control ddago_floor_dock.launch.py \
      robot_id:="$ROBOT" debug:=true stream:=true obstacle_avoid:=true dry_run:=true "${@:2}"
    ;;
  goal)
    echo "[floor_dock] 단발 도킹 goal 전송"
    ros2 action send_goal "$ACTION" automato_interfaces/action/FloorDock \
      "{task_id: 1, task_point_id: 'CHARGE_03', wall_gap_m: 0.0, lateral_offset_m: 0.0}" --feedback
    ;;
  repeat)
    N=${2:-3}
    echo "[floor_dock] 반복 도킹 ${N}회 (마지막은 도킹 상태로 종료)"
    ros2 run ddago_control floor_dock_repeat_client --ros-args \
      -p count:="$N" -p wall_gap_m:=0.03 -p post_advance_m:=0.30 -p pause_s:=2.0
    ;;
  *)
    echo "사용법: floor_dock.sh {server|dry|goal|repeat [N]}"
    echo "  server      실주행 서버(라이다 충돌방지·스트림·디버그)"
    echo "  dry         dry_run 서버(모터 미발행, 검출/라이다 확인)"
    echo "  goal        단발 도킹 goal 1회"
    echo "  repeat [N]  N회 반복 도킹(기본 3, 마지막은 도킹 유지)"
    echo ""
    echo "  추가 인자 전달: floor_dock.sh server obstacle_avoid:=false"
    echo "  웹 스트림/원격제어: http://<로봇ip>:8001 (도킹 미진행 시 방향패드)"
    ;;
esac

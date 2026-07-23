#!/usr/bin/env bash
# DG Control Service(DCS) 테스트 스택 올리기/내리기.
#   ./dashboard.sh up            # dcs + 시뮬4종 + rosbridge + web 전부 기동
#   ./dashboard.sh up --no-sim   # 시뮬4종 빼고 기동 (실장비 연동 시)
#                                #   실제 로봇/ACS 가 붙을 때 시뮬이 같이 떠 있으면
#                                #   같은 토픽·액션 이름에 발행자·서버가 둘씩 생겨
#                                #   goal 이 엉뚱한 쪽으로 가거나 텔레메트리가 섞인다.
#   ./dashboard.sh down          # 전부 종료
#   ./dashboard.sh status        # 상태 확인
#   ./dashboard.sh restart       # 재기동
#   ./dashboard.sh dock          # E4 도킹 goal 하달 (ACS 역할: /{ROBOT_ID}/dock)
#   ./dashboard.sh stop  <노드>  # 특정 노드만 내림  (dcs|acs|ddago|ddagi|dg_ai|rosbridge|web)
#   ./dashboard.sh start <노드>  # 특정 노드만 올림
#
# 노드:
#   dcs   = 실제 DG Control Service (dg_control dcs_node)  ← 본인 담당 구현
#   acs   = Automato Control Service 시뮬 (dg_sim)
#   ddago = DdaGo 시뮬 (dg_sim)
#   ddagi = Ddagi 시뮬 (dg_sim)
#   dg_ai = DG AI Service 시뮬 (TCP :9100).  끄면 AI 접속대상 active=real, 켜면 active=sim.
#
# 로봇 식별자는 환경변수 ROBOT_ID 하나로 통제한다 (~/.bashrc: export ROBOT_ID=dg_01).
#   ROBOT_ID=dg_02 ./dashboard.sh up   # 다른 로봇으로 스택 기동
# 주의: ROS setup.bash 는 set -u 와 충돌하므로 set -u 를 쓰지 않는다.
# 로봇 식별자: 환경변수 ROBOT_ID(~/.bashrc) 가 단일 출처. 없으면 dg_01.
# DCS·시뮬의 robot_id 파라미터로 함께 주입해 토픽/액션 이름(/{robot_id}/...)이 어긋나지 않게 한다.
ROBOT_ID="${ROBOT_ID:-dg_01}"
RID_ARG="--ros-args -p robot_id:=$ROBOT_ID"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="source /home/ane/dev_ws/.venv/bin/activate; source /opt/ros/jazzy/setup.bash; cd $WS; source install/setup.bash"
TARGET="$WS/dg_web/dg_ai_target.json"

# dg_ai_target.json 의 active 를 sim/real 로 전환 (dg_ai 시뮬 on/off 연동)
set_ai_active() {
  python3 - "$TARGET" "$1" <<'PY'
import json, sys
p, active = sys.argv[1], sys.argv[2]
try:
    with open(p) as f: d = json.load(f)
except Exception:
    d = {"real": "", "sim": "127.0.0.1:9100"}
d["active"] = active
with open(p, "w") as f: json.dump(d, f, indent=2, ensure_ascii=False)
print("  AI 접속대상 active =", active)
PY
}

status() {
  _proc() { pgrep -f "$1" >/dev/null 2>&1 && echo UP || echo DOWN; }
  _port() { ss -ltn 2>/dev/null | grep -q ":$1 " && echo UP || echo DOWN; }
  echo "  ROBOT_ID          : $ROBOT_ID"
  echo "  dcs (DG 본체)     : $(_proc 'dg_control/lib/dg_control/dcs_node')"
  echo "  acs   (시뮬)      : $(_proc 'dg_sim/lib/dg_sim/acs_sim')"
  echo "  ddago (시뮬)      : $(_proc 'dg_sim/lib/dg_sim/ddago_sim')"
  echo "  ddagi (시뮬)      : $(_proc 'dg_sim/lib/dg_sim/ddagi_sim')"
  echo "  dg_ai (시뮬 :9100): $(_port 9100)"
  echo "  rosbridge (:9090) : $(_port 9090)"
  echo "  web (:8000)       : $(_port 8000)"
}

up() {
  # --no-sim: 시뮬 4종(acs·ddago·ddagi·dg_ai)을 띄우지 않는다. 실장비가 그 자리를
  # 대신하므로, 같이 띄우면 이름이 겹쳐 충돌한다(위 헤더 설명 참조).
  local no_sim=0
  [ "${1:-}" = "--no-sim" ] && no_sim=1
  echo "▶ DG 테스트 스택 시작... (ROBOT_ID=$ROBOT_ID)"
  [ "$no_sim" = 1 ] && echo "  --no-sim: 시뮬(acs·ddago·ddagi·dg_ai) 제외 — 실장비가 붙어야 동작"
  start_one dcs
  if [ "$no_sim" = 0 ]; then
    start_one dg_ai
    start_one ddagi
    start_one ddago
    start_one acs
  else
    # 시뮬 dg_ai 를 안 띄우므로 AI 접속대상은 실서비스로 돌린다.
    set_ai_active real
  fi
  setsid bash -c "$SRC; exec ros2 run rosbridge_server rosbridge_websocket" >/tmp/dash_rb.log  2>&1 & disown
  setsid bash -c "exec python3 $WS/dg_web/control_server.py"                  >/tmp/dash_http.log 2>&1 & disown
  sleep 2
  status
  echo "  → 브라우저: http://localhost:8000"
}

start_one() {
  local n="${1:-}"
  case "$n" in
    dcs)       setsid bash -c "$SRC; exec ros2 run dg_control dcs_node $RID_ARG"   >/tmp/dash_dcs.log   2>&1 & disown ;;
    acs)       setsid bash -c "$SRC; exec ros2 run dg_sim acs_sim $RID_ARG"        >/tmp/dash_acs.log   2>&1 & disown ;;
    ddago)     setsid bash -c "$SRC; exec ros2 run dg_sim ddago_sim $RID_ARG"      >/tmp/dash_ddago.log 2>&1 & disown ;;
    ddagi)     setsid bash -c "$SRC; exec ros2 run dg_sim ddagi_sim $RID_ARG"      >/tmp/dash_ddagi.log 2>&1 & disown ;;
    dg_ai)     setsid bash -c "$SRC; exec ros2 run dg_sim dg_ai_sim"      >/tmp/dash_ai.log    2>&1 & disown
               set_ai_active sim ;;
    rosbridge) setsid bash -c "$SRC; exec ros2 run rosbridge_server rosbridge_websocket" >/tmp/dash_rb.log 2>&1 & disown ;;
    web)       setsid bash -c "exec python3 $WS/dg_web/control_server.py"  >/tmp/dash_http.log 2>&1 & disown ;;
    *) echo "  알 수 없는 노드: '$n' (dcs|acs|ddago|ddagi|dg_ai|rosbridge|web)"; return 1 ;;
  esac
  echo "  '$n' 기동"
}

stop_one() {
  local n="${1:-}"
  case "$n" in
    dcs)       pkill -f 'dg_control/lib/dg_control/dcs_node' ;;
    acs)       pkill -f 'dg_sim/lib/dg_sim/acs_sim' ;;
    ddago)     pkill -f 'dg_sim/lib/dg_sim/ddago_sim' ;;
    ddagi)     pkill -f 'dg_sim/lib/dg_sim/ddagi_sim' ;;
    dg_ai)     pkill -f 'dg_sim/lib/dg_sim/dg_ai_sim'
               set_ai_active real ;;   # 시뮬 끄면 실서버로 자동 전환
    rosbridge) pkill -f 'rosbridge_server/rosbridge_websocket' ;;
    web)       pkill -f 'dg_web/control_server.py' ;;
    *) echo "  알 수 없는 노드: '$n' (dcs|acs|ddago|ddagi|dg_ai|rosbridge|web)"; return 1 ;;
  esac
  echo "  '$n' 종료 시도 (다시: ./dashboard.sh start $n)"
}

down() {
  echo "■ 스택 종료..."
  pkill -f 'automato_ws/install/.*/lib'           2>/dev/null
  pkill -f 'dg_sim/lib/dg_sim/dg_ai_sim'          2>/dev/null
  pkill -f 'rosbridge_server/rosbridge_websocket' 2>/dev/null
  pkill -f 'dg_web/control_server.py'             2>/dev/null
  sleep 1
  echo "  종료 완료"
}

# 순찰 테스트 1회 실행 (acs_sim 의 start_patrol 서비스 호출).
# dcs + acs + ddago + ddagi + dg_ai 가 떠 있어야 흐름이 끝까지 돈다.
run_test() {
  bash -c "$SRC; ros2 service call /acs_sim/start_patrol std_srvs/srv/Trigger" 2>&1
}

# E0 상시 모니터링: 시뮬 텔레메트리 발행 시작(상시)/중지.
run_telemetry() {
  bash -c "$SRC; ros2 service call /ddago_sim/start_telemetry std_srvs/srv/Trigger; ros2 service call /ddagi_sim/start_telemetry std_srvs/srv/Trigger" 2>&1
}
run_telemetry_stop() {
  bash -c "$SRC; ros2 service call /ddago_sim/stop_telemetry std_srvs/srv/Trigger; ros2 service call /ddagi_sim/stop_telemetry std_srvs/srv/Trigger" 2>&1
}

# E4-6 정밀 도킹: ACS 역할로 Dock goal 을 DCS 에 하달한다.
#   ACS → DCS(/{ROBOT_ID}/dock) → DdaGo(/ddago/dock) 중계 사슬을 한 번에 태운다.
# 마커 정보는 실제로는 ACS 가 DB(작업 지점의 ChArUco 보드)에서 조회해 채운다.
# 아래 값은 현장 보드(mid24 스테이션 A): 6x5칸, 칸 24mm / 마커 18mm, 시작 ID 500.
run_dock() {
  bash -c "$SRC; ros2 action send_goal /$ROBOT_ID/dock automato_interfaces/action/Dock \
    '{task_id: 1024, task_point_id: CHARGE_01, marker_id: \"500\",
      dictionary: DICT_5X5_1000, squares_x: 6, squares_y: 5,
      square_size_m: 0.024, marker_size_m: 0.018}' --feedback" 2>&1
}

# 최근 흐름 로그(분석→저장→순찰결과)만 추려 출력.
logs() {
  echo "== DCS =="
  grep -hE '순찰 경로 수신|DdaGo 하달|분석결과|구간 결과 전달|도킹 지시 수신|도킹 하달|도킹 종료|도킹 결과 전달' /tmp/dash_dcs.log 2>/dev/null | tail -12
  echo "== ACS =="
  grep -hE '순찰 시작|구간 하달|SaveDetection 저장|구간 결과|순찰 완료|Fleet 수신' /tmp/dash_acs.log 2>/dev/null | tail -12
}

case "${1:-}" in
  up)      up "${2:-}" ;;
  down)    down ;;
  stop)    stop_one "${2:-}" ;;
  start)   start_one "${2:-}" ;;
  status)  status ;;
  restart) down; sleep 1; up "${2:-}" ;;
  test)           run_test ;;
  telemetry)      run_telemetry ;;
  telemetry-stop) run_telemetry_stop ;;
  dock)           run_dock ;;
  logs)           logs ;;
  *) echo "usage: $0 {up [--no-sim]|down|status|restart [--no-sim]|test|dock|telemetry|telemetry-stop|logs|stop <node>|start <node>}  # node: dcs|acs|ddago|ddagi|dg_ai|rosbridge|web"; exit 1 ;;
esac

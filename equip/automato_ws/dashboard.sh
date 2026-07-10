#!/usr/bin/env bash
# DG Control Service(HQ) 테스트 스택 올리기/내리기.
#   ./dashboard.sh up            # hq + 시뮬4종 + rosbridge + web 전부 기동
#   ./dashboard.sh down          # 전부 종료
#   ./dashboard.sh status        # 상태 확인
#   ./dashboard.sh restart       # 재기동
#   ./dashboard.sh stop  <노드>  # 특정 노드만 내림  (hq|acs|ddago|ddagi|dg_ai|rosbridge|web)
#   ./dashboard.sh start <노드>  # 특정 노드만 올림
#
# 노드:
#   hq    = 실제 DG Control Service (dg_control hq_node)  ← 본인 담당 구현
#   acs   = Automato Control Service 시뮬 (dg_sim)
#   ddago = DdaGo 시뮬 (dg_sim)
#   ddagi = Ddagi 시뮬 (dg_sim)
#   dg_ai = DG AI Service 시뮬 (TCP :9100).  끄면 AI 접속대상 active=real, 켜면 active=sim.
# 주의: ROS setup.bash 는 set -u 와 충돌하므로 set -u 를 쓰지 않는다.
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
  echo "  hq  (DG 본체)     : $(_proc 'dg_control/lib/dg_control/hq_node')"
  echo "  acs   (시뮬)      : $(_proc 'dg_sim/lib/dg_sim/acs_sim')"
  echo "  ddago (시뮬)      : $(_proc 'dg_sim/lib/dg_sim/ddago_sim')"
  echo "  ddagi (시뮬)      : $(_proc 'dg_sim/lib/dg_sim/ddagi_sim')"
  echo "  dg_ai (시뮬 :9100): $(_port 9100)"
  echo "  rosbridge (:9090) : $(_port 9090)"
  echo "  web (:8000)       : $(_port 8000)"
}

up() {
  echo "▶ DG 테스트 스택 시작..."
  start_one hq
  start_one dg_ai
  start_one ddagi
  start_one ddago
  start_one acs
  setsid bash -c "$SRC; exec ros2 run rosbridge_server rosbridge_websocket" >/tmp/dash_rb.log  2>&1 & disown
  setsid bash -c "exec python3 $WS/dg_web/control_server.py"                  >/tmp/dash_http.log 2>&1 & disown
  sleep 2
  status
  echo "  → 브라우저: http://localhost:8000"
}

start_one() {
  local n="${1:-}"
  case "$n" in
    hq)        setsid bash -c "$SRC; exec ros2 run dg_control hq_node"    >/tmp/dash_hq.log    2>&1 & disown ;;
    acs)       setsid bash -c "$SRC; exec ros2 run dg_sim acs_sim"        >/tmp/dash_acs.log   2>&1 & disown ;;
    ddago)     setsid bash -c "$SRC; exec ros2 run dg_sim ddago_sim"      >/tmp/dash_ddago.log 2>&1 & disown ;;
    ddagi)     setsid bash -c "$SRC; exec ros2 run dg_sim ddagi_sim"      >/tmp/dash_ddagi.log 2>&1 & disown ;;
    dg_ai)     setsid bash -c "$SRC; exec ros2 run dg_sim dg_ai_sim"      >/tmp/dash_ai.log    2>&1 & disown
               set_ai_active sim ;;
    rosbridge) setsid bash -c "$SRC; exec ros2 run rosbridge_server rosbridge_websocket" >/tmp/dash_rb.log 2>&1 & disown ;;
    web)       setsid bash -c "exec python3 $WS/dg_web/control_server.py"  >/tmp/dash_http.log 2>&1 & disown ;;
    *) echo "  알 수 없는 노드: '$n' (hq|acs|ddago|ddagi|dg_ai|rosbridge|web)"; return 1 ;;
  esac
  echo "  '$n' 기동"
}

stop_one() {
  local n="${1:-}"
  case "$n" in
    hq)        pkill -f 'dg_control/lib/dg_control/hq_node' ;;
    acs)       pkill -f 'dg_sim/lib/dg_sim/acs_sim' ;;
    ddago)     pkill -f 'dg_sim/lib/dg_sim/ddago_sim' ;;
    ddagi)     pkill -f 'dg_sim/lib/dg_sim/ddagi_sim' ;;
    dg_ai)     pkill -f 'dg_sim/lib/dg_sim/dg_ai_sim'
               set_ai_active real ;;   # 시뮬 끄면 실서버로 자동 전환
    rosbridge) pkill -f 'rosbridge_server/rosbridge_websocket' ;;
    web)       pkill -f 'dg_web/control_server.py' ;;
    *) echo "  알 수 없는 노드: '$n' (hq|acs|ddago|ddagi|dg_ai|rosbridge|web)"; return 1 ;;
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
# hq + acs + ddago + ddagi + dg_ai 가 떠 있어야 흐름이 끝까지 돈다.
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

# 최근 흐름 로그(분석→저장→순찰결과)만 추려 출력.
logs() {
  echo "== HQ =="
  grep -hE 'DdaGo 하달|분석결과|순찰 완료|순찰 시작' /tmp/dash_hq.log 2>/dev/null | tail -12
  echo "== ACS =="
  grep -hE '순찰 발행|순찰 진행|SaveDetection 저장|순찰 결과|Fleet 수신' /tmp/dash_acs.log 2>/dev/null | tail -12
}

case "${1:-}" in
  up)      up ;;
  down)    down ;;
  stop)    stop_one "${2:-}" ;;
  start)   start_one "${2:-}" ;;
  status)  status ;;
  restart) down; sleep 1; up ;;
  test)           run_test ;;
  telemetry)      run_telemetry ;;
  telemetry-stop) run_telemetry_stop ;;
  logs)           logs ;;
  *) echo "usage: $0 {up|down|status|restart|test|telemetry|telemetry-stop|logs|stop <node>|start <node>}  # node: hq|acs|ddago|ddagi|dg_ai|rosbridge|web"; exit 1 ;;
esac

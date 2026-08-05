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
#   ./dashboard.sh test          # S1 순찰 1회 (ACS: /acs_sim/start_patrol)
#   ./dashboard.sh harvest-move  # S2 E2 수확 이동+도킹 1회 (ACS: /acs_sim/start_harvest_move)
#   ./dashboard.sh harvest       # S2 E3 수확 시작 1회 (도킹 성공 task 로 Harvest 하달)
#   ./dashboard.sh unload        # S2 E6 하역 시작 1회 (도킹 성공 task 로 Unload 하달)
#   ./dashboard.sh dock          # charuco 도킹 단발 하달 (휴면 방식, 수동 확인용)
#   ./dashboard.sh floor-dock      # H마커 도킹 단발 하달 (/{ROBOT_ID}/floor_dock)
#   ./dashboard.sh reflective-dock # 반사테이프 도킹 단발 하달 (/{ROBOT_ID}/reflective_dock, E4 충전소)
#   ./dashboard.sh return-dock   # E4 복귀 주행+충전소 도킹 2단계 (ACS: /acs_sim/start_return)
#   ./dashboard.sh stop  <노드>  # 특정 노드만 내림  (dcs|acs|ddago|ddagi|dg_ai|rosbridge|web)
#   ./dashboard.sh start <노드>  # 특정 노드만 올림
#
# 실행되는 명령은 dg_web/cmdcfg.py 가 갖고 있고, 대시보드 화면에서 고칠 수 있다
# (편집분은 dg_web/commands.local.json). 여기 스크립트에는 명령을 직접 쓰지 않는다.
#   ./dashboard.sh start-key <키>      # 실서버 프로파일 기동 (real-dcs·real-ai·… )
#   ./dashboard.sh run-cmdline <꼬리표> <명령줄>   # 명령줄을 직접 받아 1회 실행
#   ./dashboard.sh stop-pattern <패턴> # 프로세스 패턴으로 종료
# 실서버(실장비)는 시뮬과 화면을 나눴다 → http://localhost:8000/real
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
export ROBOT_ID   # 명령 정의 안의 ${ROBOT_ID} 를 실행 시점에 펼치기 위해 export 한다
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WS   # 실서버 명령 정의(real-ai 의 ${WS}/.venv)가 실행 시점에 펼친다
SRC="source /home/ane/dev_ws/.venv/bin/activate; source /opt/ros/jazzy/setup.bash; cd $WS; source install/setup.bash"
TARGET="$WS/dg_web/dg_ai_target.json"

# 실행할 명령줄은 dg_web/cmdcfg.py 가 만든다(대시보드 화면에서 편집 가능).
# 여기에 명령을 직접 쓰지 않는 이유: 웹과 CLI 가 서로 다른 명령을 돌리면 안 된다.
# 되돌리기는 대시보드의 '기본값' 버튼 또는 dg_web/commands.local.json 삭제.
CMD() {
  python3 "$WS/dg_web/cmdcfg.py" "$1" || {
    echo "  ✗ 명령 정의를 읽지 못했습니다: $1" >&2
    return 1
  }
}

# 노드 이름 → /tmp/dash_*.log 꼬리표 (이름이 그대로가 아닌 것만 예외)
_logtag() {
  case "$1" in
    dg_ai)     echo ai ;;
    rosbridge) echo rb ;;
    web)       echo http ;;
    *)         echo "$1" ;;
  esac
}

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
  start_one rosbridge
  start_one web
  sleep 2
  status
  echo "  → 브라우저: http://localhost:8000"
}

start_one() {
  local n="${1:-}" c
  case "$n" in
    dcs|acs|ddago|ddagi|dg_ai|rosbridge)
      c="$(CMD "$n")" || return 1
      setsid bash -c "$SRC; exec $c" >"/tmp/dash_$(_logtag "$n").log" 2>&1 & disown
      # 시뮬 dg_ai 를 띄웠으면 AI 접속대상도 시뮬로 돌린다(끌 때는 real 로 되돌린다).
      if [ "$n" = dg_ai ]; then set_ai_active sim; fi ;;
    # web 은 이 스크립트가 아니라 대시보드 서버 자신이라 편집 대상에서 뺀다
    # (자기가 실행하는 명령을 자기 화면에서 고치면 되살릴 길이 없어진다).
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
run_test() { run_cmd test; }

# 시나리오·도킹 명령 1회 실행. 무엇을 실행할지는 cmdcfg 가 정한다(대시보드에서 편집 가능).
# stdout·stderr 를 한데 묶어 /tmp/dash_cmd_<키>.log 에도 남긴다 — 대시보드가 이 파일을
# 읽어 실행 결과를 그대로 보여준다(tee 라서 CLI 로 돌렸을 때 화면 출력은 그대로다).
# 맨 앞에 실행한 명령줄 자체를 찍어 둔다: 편집된 명령이 무엇이었는지 나중에 알 수 있어야 한다.
run_cmd() {
  local c f rc
  c="$(CMD "$1")" || return 1
  f="/tmp/dash_cmd_$1.log"
  {
    echo "\$ $c"
    echo "--------"
    bash -c "$SRC; $c" 2>&1
    rc=$?
    echo "--------"
    echo "(종료코드 $rc)"
  } | tee "$f"
}

# S2 E2 수확 위치 이동+도킹 1회 실행 (acs_sim 의 start_harvest_move 서비스 호출).
#   ACS: 수확 위치까지 전 구간 capture=false 로 주행 → 도착 후 FloorDock(H마커) 하달
#        → DCS 가 /{ROBOT_ID}/{navigate,dock} 중계 → DdaGo 시뮬이 응답.
# 도킹 실패를 보려면 먼저 DdaGo 시뮬 모드를 바꾼다(성공 복귀는 dock_mode:=success):
#   ros2 param set /ddago_sim dock_mode no_marker   # 또는 error_exceeded / hang
run_harvest_move() {
  run_cmd harvest-move
}

# E4 순찰 종료 후 복귀 및 충전 — 2단계(복귀 주행 → 충전소 도킹) 1회 실행.
#   ACS: 충전소까지 전 구간 capture=false 로 주행 → 도착 후 ReflectiveDock 하달
#   (충전소 마커가 반사테이프. 지점 id CHARGE_* 에서 방식이 자동으로 정해진다.)
#   서비스 호출은 즉시 반환하고 시나리오는 백그라운드로 돈다(완주는 대시보드 eval 이 로그로 폴링).
run_return_dock() {
  run_cmd return-dock
}

# S2 E3 수확 시작 1회 실행 (acs_sim 의 start_harvest 서비스 호출).
#   도킹 성공(E2)으로 게이트가 열린 마지막 task 로 Harvest 를 하달한다.
#   ACS→DCS(/{ROBOT_ID}/harvest)→Ddagi(/ddagi/harvest) 중계. Ddagi 시뮬이 라운드
#   Feedback 을 흘리고 종료 사유(exit_reason)로 마감 → DCS 가 그대로 되돌린다.
# 종료 사유를 바꾸려면 먼저 Ddagi 시뮬 모드를 바꾼다(기본 depleted):
#   ros2 param set /ddagi_sim harvest_mode full   # 또는 max_rounds / hang
run_harvest() {
  run_cmd harvest
}

# S2 E6 하역 시작 1회 실행 (acs_sim 의 start_unload 서비스 호출).
#   예냉실 도킹 성공으로 게이트가 열린 마지막 task 로 Unload 를 하달한다.
#   ACS→DCS(/{ROBOT_ID}/unload)→Ddagi(/ddagi/unload) 중계. Ddagi 시뮬이 하역 phase
#   (GRIP_HANDLE→LIFT→WAIT→SHAKE→RETURN) Feedback 을 흘리고 result_code 로 마감 →
#   DCS 가 그대로 되돌린다.
# 결과를 바꾸려면 먼저 Ddagi 시뮬 모드를 바꾼다(기본 success):
#   ros2 param set /ddagi_sim unload_mode grip_fail   # 또는 hang
run_unload() {
  run_cmd unload
}

# E0 상시 모니터링: 시뮬 텔레메트리 발행 시작(상시)/중지.
run_telemetry() {
  run_cmd telemetry
}
run_telemetry_stop() {
  run_cmd telemetry-stop
}

# charuco 정밀 도킹: ACS 역할로 Dock goal 을 DCS 에 하달한다.
#   ACS → DCS(/{ROBOT_ID}/dock) → DdaGo(/ddago/dock) 중계 사슬을 한 번에 태운다.
# 현재 어느 지점도 charuco 를 쓰지 않는다(E4 충전소 복귀는 reflective-dock). 휴면 경로 확인용.
# 마커 정보는 실제로는 ACS 가 DB(작업 지점의 ChArUco 보드)에서 조회해 채운다.
# 아래 값은 현장 보드(mid24 스테이션 A): 6x5칸, 칸 24mm / 마커 18mm, 시작 ID 500.
run_dock() {
  run_cmd dock
}

# 도킹 방식 나머지 둘(RP-131) — 바닥 H 마커(수확지·예냉실) / 반사테이프(충전소).
# charuco 와 달리 **마커리스**라 goal 에 마커 규격이 없다. 목표 정차값을 0 으로 두면
# 로봇 노드(반사테이프는 로봇별 config yaml)의 기본값을 쓴다 — 실제 ACS 도 그렇게 보낸다.
run_floor_dock() {
  run_cmd floor-dock
}
run_reflective_dock() {
  run_cmd reflective-dock
}

# ── 실서버 프로파일 (/real 대시보드가 쓴다) ─────────────────────────────
# 시뮬 스택(start_one)과 함수를 나눠 둔다. 여기 오는 것은 실제로 도는 서비스라
# 실수로 시뮬과 섞이면 안 된다 — 화면도 명령도 정의도 따로 간다.
# 로그는 /tmp/dash_<키>.log (키에 real-/robot- 접두가 붙어 시뮬 로그와 안 겹친다).
# exec 를 쓰지 않는다: 실서버 명령은 'source ...' 로 시작할 수 있는데(venv·워크스페이스),
# source 는 셸 내장이라 exec 의 대상이 될 수 없다("exec: source: not found").
# setsid 로 새 프로세스 그룹이 되므로, 종료는 그룹째 신호를 주면 자식까지 함께 정리된다.
start_key() {
  local k="${1:-}" c tag
  c="$(CMD "$k")" || return 1
  # 로그 파일 이름은 cmdcfg 가 정한다. 시뮬과 같은 프로세스를 쓰는 항목(real-dcs)은
  # 같은 파일에 쌓아야 메시지 시계열이 갈라지지 않는다.
  tag="$(python3 "$WS/dg_web/cmdcfg.py" --log "$k" 2>/dev/null)" || tag="$k"
  [ -z "$tag" ] && tag="$k"
  setsid bash -c "$SRC; $c" >"/tmp/dash_$tag.log" 2>&1 & disown
  echo "  '$k' 기동 (로그: /tmp/dash_$tag.log)"
}

# 대시보드가 만든 명령줄을 ROS 환경에서 그대로 실행한다.
# run_cmd 와 달리 **키가 아니라 명령줄**을 받는다 — 복귀 좌표처럼 실행 시점에만 정해지는
# 값이 있어서다. 명령을 짓는 곳은 여전히 cmdcfg 하나이고, 여기서는 짓지 않는다.
# ⚠ 파이프라인이라 이 함수의 종료코드는 tee 의 것이다. 호출자는 종료코드를 믿지 말고
#    출력 내용(예: 'Goal finished with status:')으로 판정할 것.
run_line() {
  local tag="${1:-cmd}" c="${2:-}" rc f
  [ -z "$c" ] && { echo "  명령줄이 비어 있습니다"; return 1; }
  f="/tmp/dash_cmd_${tag}.log"
  {
    echo "\$ $c"
    echo "--------"
    bash -c "$SRC; $c" 2>&1
    rc=$?
    echo "--------"
    echo "(종료코드 $rc)"
  } | tee "$f"
}

# 프로세스 패턴으로 종료. 패턴은 cmdcfg 의 check 값을 대시보드가 그대로 넘긴다.
# launch 가 띄운 자식까지 걷으려면 그룹째 신호를 줘야 한다.
# ⚠ pgrep -f 는 **이 스크립트 자신**도 잡는다. 패턴이 인자라 명령줄에 그대로 들어 있기
# 때문이다. 자기(와 부모)를 빼지 않으면 스크립트가 제 프로세스 그룹에 신호를 보내
# 스스로 죽고, 정작 대상은 그대로 남는다(전체 종료가 성공했다고 보고하면서 아무것도
# 안 끄는 상태가 된다). 같은 이유로 대상이 우리와 같은 그룹이면 그룹이 아니라 PID 로 죽인다.
#
# 신호는 단계적으로 올린다: INT(정상 종료 요청) → TERM → KILL.
# INT 한 번만 보내고 끝내면 그걸 무시하는 프로세스가 살아남는다(analysis_server 가 그랬다).
# 그렇다고 처음부터 KILL 하면 ROS 노드가 정리 없이 죽어 다음 기동에서 이름이 겹친다.
stop_pattern() {
  local pat="${1:-}" pids p pgid mypgid sig i
  [ -z "$pat" ] && { echo "  패턴이 비어 있습니다"; return 1; }
  mypgid="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ')"

  # ⚠ pgrep -f 는 이 스크립트 자신은 물론, 스크립트가 fork 한 **서브셸**까지 잡는다
  # (서브셸은 부모의 명령줄을 그대로 물려받고, 패턴은 인자로 명령줄에 들어 있다).
  # $$·$PPID 만 빼면 서브셸이 남아 "아직 안 죽었다"고 영원히 보고한다.
  # 우리 프로세스 그룹 전체를 제외한다 — 진짜 대상은 setsid 로 띄워 다른 그룹에 있다.
  _targets() {
    local q g
    for q in $(pgrep -f "$pat" 2>/dev/null); do
      g="$(ps -o pgid= -p "$q" 2>/dev/null | tr -d ' ')"
      [ -n "$g" ] && [ "$g" != "$mypgid" ] && echo "$q"
    done
  }

  [ -z "$(_targets)" ] && { echo "  해당 프로세스 없음: $pat"; return 0; }

  for sig in INT TERM KILL; do
    pids="$(_targets)"
    [ -z "$pids" ] && break
    for p in $pids; do
      # _targets 가 우리 그룹을 이미 걸렀으므로 여기 오는 것은 전부 남의 그룹이다.
      # 그룹째 보내야 ros2 launch 가 띄운 자식 노드까지 함께 정리된다.
      pgid="$(ps -o pgid= -p "$p" 2>/dev/null | tr -d ' ')"
      [ -n "$pgid" ] && kill -"$sig" "-$pgid" 2>/dev/null || kill -"$sig" "$p" 2>/dev/null
    done
    echo "  SIG$sig 보냄: $pat"
    [ "$sig" = KILL ] && { sleep 0.5; break; }
    for i in 1 2 3 4 5 6 7 8 9 10; do   # 최대 5초 기다렸다 다음 단계로
      sleep 0.5
      [ -z "$(_targets)" ] && break
    done
  done

  if [ -z "$(_targets)" ]; then
    echo "  종료 확인: $pat"
  else
    echo "  ⚠ 아직 남아 있음: $pat"
    return 1
  fi
}

# 최근 흐름 로그(분석→저장→순찰결과)만 추려 출력.
logs() {
  echo "== DCS =="
  grep -hE '경로 수신|순찰 경로 수신|DdaGo 하달|분석결과|구간 결과 전달|도킹 지시 수신|도킹 하달|도킹 종료|도킹 결과 전달|E3 진입 가능' /tmp/dash_dcs.log 2>/dev/null | tail -12
  echo "== ACS =="
  grep -hE '순찰 시작|수확 이동 시작|구간 하달|수확 이동 구간|SaveDetection 저장|구간 결과|순찰 완료|도킹 하달|도킹 진행|도킹 결과|Fleet 수신' /tmp/dash_acs.log 2>/dev/null | tail -12
}

case "${1:-}" in
  up)      up "${2:-}" ;;
  down)    down ;;
  stop)    stop_one "${2:-}" ;;
  start)   start_one "${2:-}" ;;
  status)  status ;;
  restart) down; sleep 1; up "${2:-}" ;;
  test)           run_test ;;
  harvest-move)   run_harvest_move ;;
  harvest)        run_harvest ;;
  unload)         run_unload ;;
  telemetry)      run_telemetry ;;
  telemetry-stop) run_telemetry_stop ;;
  dock)           run_dock ;;
  floor-dock)      run_floor_dock ;;
  reflective-dock) run_reflective_dock ;;
  return-dock)     run_return_dock ;;
  logs)           logs ;;
  start-key)      start_key "${2:-}" ;;      # 실서버 프로파일 기동 (/real 대시보드용)
  run-cmdline)    run_line "${2:-}" "${3:-}" ;;  # <꼬리표> <명령줄> — 실행 시점 값이 섞인 명령
  stop-pattern)   stop_pattern "${2:-}" ;;   # 프로세스 패턴으로 종료
  *) echo "usage: $0 {up [--no-sim]|down|status|restart [--no-sim]|test|harvest-move|harvest|unload|dock|floor-dock|reflective-dock|return-dock|telemetry|telemetry-stop|logs|stop <node>|start <node>|start-key <key>|run-cmdline <tag> <line>|stop-pattern <pat>}  # node: dcs|acs|ddago|ddagi|dg_ai|rosbridge|web"; exit 1 ;;
esac

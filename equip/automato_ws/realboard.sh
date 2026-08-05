#!/usr/bin/env bash
# 실서버(실장비) 대시보드 — 시뮬 스택(dashboard.sh)과 **완전히 따로** 돈다.
#   프로세스도 포트도 화면도 따로다. 시뮬을 통째로 내려도 이 화면은 살아 있고 반대도 같다.
#   실장비가 붙은 채 시뮬 버튼을 누르는 사고를 막는 것이 분리의 목적이다.
#
#   ./realboard.sh up        # 실서버 대시보드(:8010) 기동 → 브라우저에서 전체 기동
#   ./realboard.sh down      # 대시보드 + 실서버 스택 전부 종료
#   ./realboard.sh status    # 대시보드·로봇 에이전트·서비스 상태
#   ./realboard.sh agent     # 로봇에 에이전트 설치·기동 (토큰은 agents.local.json 에서 읽음)
#   ./realboard.sh logs <키> # 그 서비스 로그 꼬리 (real-dcs·robot-ddago 등)
#
# 서비스 자체(real-dcs·robot-bringup…)의 기동·종료·로그는 화면에서 한다.
# 명령 정의는 dg_web/cmdcfg.py(profile=real), 편집분은 dg_web/commands.local.json.
# 로봇 인벤토리·토큰은 dg_web/agents.local.json (커밋 제외).
#
# 주의: ROS setup.bash 는 set -u 와 충돌하므로 set -u 를 쓰지 않는다.
ROBOT_ID="${ROBOT_ID:-dg_01}"
export ROBOT_ID
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WS
WEB="$WS/dg_web"
PORT="${DG_REAL_PORT:-8010}"

_agent_field() {   # $1 = 필드명 (addr|port|token) — 첫 번째 로봇 것
  python3 - "$WEB/agents.local.json" "$1" <<'PY'
import json, sys
try:
    d = json.load(open(sys.argv[1]))
except Exception:
    d = {"ddago02": {"addr": "ddago02", "port": 8500, "token": ""}}
first = next(iter(d.values()), {})
print(first.get(sys.argv[2], ''))
PY
}

up() {
  echo "▶ 실서버 대시보드 기동 (ROBOT_ID=$ROBOT_ID)"
  if ss -ltn 2>/dev/null | grep -q ":$PORT "; then
    echo "  이미 떠 있음 (:$PORT)"
  else
    DG_REAL_PORT="$PORT" setsid bash -c "exec python3 $WEB/real_server.py" \
      >/tmp/dash_realweb.log 2>&1 & disown
    sleep 1.5
  fi
  status
  echo "  → 브라우저: http://localhost:$PORT   ('전체 기동' 버튼으로 스택을 올린다)"
}

down() {
  echo "■ 실서버 스택 종료..."
  # 화면과 같은 순서(기동의 역순)로 내리기 위해 서버에 맡긴다. 서버가 죽어 있으면 건너뛴다.
  curl -s -X POST "http://127.0.0.1:$PORT/api/stack/down" >/dev/null 2>&1 \
    && echo "  서비스 전체 종료 요청 보냄" \
    || echo "  (대시보드가 안 떠 있어 서비스 종료는 건너뜀)"
  pkill -f 'dg_web/real_server.py' 2>/dev/null && echo "  대시보드 종료"
  echo "  완료"
}

status() {
  local addr port
  addr="$(_agent_field addr)"; port="$(_agent_field port)"
  echo "  ROBOT_ID          : $ROBOT_ID"
  echo -n "  실서버 대시보드   : "
  ss -ltn 2>/dev/null | grep -q ":$PORT " && echo "UP (:$PORT)" || echo "DOWN"
  echo -n "  로봇 에이전트     : "
  if curl -s -m 3 "http://$addr:$port/health" >/dev/null 2>&1; then
    echo "UP ($addr:$port)"
  else
    echo "DOWN ($addr:$port) — ./realboard.sh agent 로 설치·기동"
  fi
  # 서비스별 UP/DOWN 은 대시보드가 로봇까지 물어봐야 알 수 있으므로 그쪽에 묻는다.
  curl -s -m 20 "http://127.0.0.1:$PORT/api/status" 2>/dev/null | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(0)
print('  서비스:')
for k in d.get('order', []):
    print('    %-16s %s' % (k, d.get('services', {}).get(k, '?')))
" 2>/dev/null
}

# 로봇에 에이전트를 올려 두고 띄운다. 토큰은 agents.local.json 에서 읽어 쓴다
# (명령줄에 직접 적으면 셸 히스토리에 남는다).
agent() {
  local addr token user
  addr="$(_agent_field addr)"; token="$(_agent_field token)"
  user="${ROBOT_USER:-pinky}"
  [ -z "$token" ] && echo "  ⚠ agents.local.json 의 token 이 비어 있습니다 — 인증 없이 뜹니다"
  echo "▶ $user@$addr 에 에이전트 설치"
  scp -q "$WEB/dg_agent.py" "$user@$addr:~/" || { echo "  ✗ scp 실패"; return 1; }
  # ⚠ 종료와 기동을 **다른 ssh 로 나눠서** 한다.
  # 한 명령에 합치면 그 원격 셸의 명령줄에 'dg_agent.py'(기동 쪽)가 들어가고,
  # pkill -f 가 그걸 잡아 **자기 자신을 죽인다** → 뒤 명령이 실행되지 않고 ssh 가 255 로 끝난다.
  # 대괄호 트릭도 같은 줄에 진짜 경로가 있으면 소용이 없다.
  ssh "$user@$addr" "pkill -f '[d]g_agent[.]py' || true" >/dev/null 2>&1
  sleep 0.5
  ssh "$user@$addr" "DG_AGENT_TOKEN='$token' setsid python3 \$HOME/dg_agent.py >/tmp/dg_agent.log 2>&1 & sleep 1; echo '  로봇에서 기동됨'" \
    || echo "  ✗ ssh 실패"
  sleep 1
  curl -s -m 3 "http://$addr:$(_agent_field port)/health" && echo
}

logs() {
  local k="${1:-}"
  [ -z "$k" ] && { echo "usage: $0 logs <키>  (real-dcs|real-ai|real-nav2|robot-bringup|robot-ddago…)"; return 1; }
  curl -s -m 15 "http://127.0.0.1:$PORT/api/svc/$k/log" | python3 -c "
import json, sys
d = json.load(sys.stdin)
print('== %s (%s) ==' % (d.get('key'), d.get('where')))
print('\n'.join(d.get('lines', [])))
"
}

case "${1:-}" in
  up)     up ;;
  down)   down ;;
  status) status ;;
  agent)  agent ;;
  logs)   logs "${2:-}" ;;
  *) echo "usage: $0 {up|down|status|agent|logs <키>}"; exit 1 ;;
esac

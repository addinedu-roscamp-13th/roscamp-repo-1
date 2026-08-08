#!/usr/bin/env bash
# 실서버(실장비) 대시보드 — 시뮬 스택(dashboard.sh)과 **완전히 따로** 돈다.
#   프로세스도 포트도 화면도 따로다. 시뮬을 통째로 내려도 이 화면은 살아 있고 반대도 같다.
#   실장비가 붙은 채 시뮬 버튼을 누르는 사고를 막는 것이 분리의 목적이다.
#
#   ./realboard.sh up        # 실서버 대시보드(:8010) 기동 → 브라우저에서 전체 기동
#                            #  이미 떠 있으면 **띄우지 않는다**(코드 수정 미반영) — restart 를 쓸 것
#   ./realboard.sh restart   # 대시보드만 새 프로세스로 (스택은 그대로) — 코드 수정 반영용
#   ./realboard.sh down      # 대시보드 + 실서버 스택 전부 종료
#   ./realboard.sh status    # Discovery Server 점검 + 대시보드·로봇 에이전트·서비스 상태
#   ./realboard.sh dds       # Discovery Server·양쪽 설정 점검(보고만)
#   ./realboard.sh dds fix   #   ″  + 서버 기동·로봇 .bashrc 정정까지
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
# 2026-08-06: 이 스크립트가 dg_web/ 안으로 들어왔다(팀 요청). WEB 은 스크립트 폴더 자신,
#  WS(워크스페이스)는 그 상위다.
WEB="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(dirname "$WEB")"
export WS
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

_dash_pid() { pgrep -f 'dg_web/real_server.py' 2>/dev/null | head -1; }

# ── FastDDS Discovery Server 점검 ──────────────────────────────────────────
# 2026-08-06: 로봇망을 5GHz AP 로 옮기면서 **무선→유선 멀티캐스트가 막혔다**
#   (실측: 로봇→노트북 `ros2 multicast send` 0 수신 / 반대 방향은 정상).
#   그래서 SIMPLE 디스커버리로는 노트북이 로봇 노드를 영영 못 본다 —
#   Discovery Server 가 이 망의 **전제 조건**이다. 서버가 죽어 있으면 대시보드가
#   서비스를 띄워도 아무것도 안 보이고, 원인이 화면에 드러나지 않아 한참 헤맨다.
#   → up/restart 전에 자동 점검하고, 서버가 없으면 띄운다.
DDS_SH="$HOME/dds.sh"

_ds_addr() {   # ~/dds.sh 를 단일 출처로 삼는다(주소를 두 군데 적지 않게)
  local ip port
  ip="$(awk -F'"' '/^SRV_LAN=/{print $2; exit}' "$DDS_SH" 2>/dev/null)"
  port="$(awk -F'=' '/^PORT=/{print $2; exit}' "$DDS_SH" 2>/dev/null | tr -d ' \t')"
  [ -n "$ip" ] && [ -n "$port" ] && echo "$ip:$port"
}

_ds_running() { ss -lun 2>/dev/null | grep -q ":${1##*:} "; }

# $1: fix 면 고칠 수 있는 것은 고친다(서버 기동·로봇 설정 정정). 아니면 보고만.
dds_check() {
  local mode="${1:-report}" want ip robot_addr robot_ds robot_wl ok=1
  echo "▶ Discovery Server 점검"
  if [ ! -f "$DDS_SH" ]; then echo "  ✖ $DDS_SH 없음 — 점검 불가"; return 1; fi
  want="$(_ds_addr)"; ip="${want%%:*}"
  if [ -z "$want" ]; then echo "  ✖ dds.sh 에서 주소를 못 읽음"; return 1; fi
  echo "  기준 주소(dds.sh): $want"

  # ① 노트북에 그 IP 가 실제로 있는가 (없으면 서버가 뜨지도, 로봇이 닿지도 못한다)
  if ip -4 -o addr show 2>/dev/null | grep -q " ${ip}/"; then
    echo "  ✔ 노트북 인터페이스에 $ip 있음"
  else
    echo "  ✖ 노트북에 $ip 가 없다 — 망이 바뀌었다. dds.sh SRV_LAN 과 fastdds_lan.xml 을 실제 IP 로 고칠 것"
    ip -4 -o addr show | awk '{print "      현재:", $2, $4}'; ok=0
  fi

  # ② FastDDS 화이트리스트(인터페이스 제한)에 그 IP 가 있는가 — 없으면 UDP 전송이 안 생긴다
  if grep -q "$ip" "${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/fastdds_lan.xml}" 2>/dev/null; then
    echo "  ✔ 노트북 화이트리스트에 $ip 있음"
  else
    echo "  ✖ 노트북 fastdds 화이트리스트에 $ip 없음 → UDPv4 전송 자체가 안 생긴다"; ok=0
  fi

  # ③ 서버가 떠 있는가
  if _ds_running "$want"; then
    echo "  ✔ 서버 기동 중 (:${want##*:})"
  elif [ "$mode" = fix ]; then
    echo "  … 서버가 없다 → 기동"
    "$DDS_SH" server start 2>&1 | sed 's/^/    /'
    _ds_running "$want" || { echo "  ✖ 서버 기동 실패"; ok=0; }
  else
    echo "  ✖ 서버가 떠 있지 않다 — './realboard.sh dds fix' 또는 '~/dds.sh server start'"; ok=0
  fi

  # ④ 이 셸의 환경 — 대시보드가 띄우는 노트북 서비스들이 이 값을 물려받는다
  if [ "${ROS_DISCOVERY_SERVER:-}" = "$want" ]; then
    echo "  ✔ 이 셸 ROS_DISCOVERY_SERVER=$want"
  else
    echo "  ✖ 이 셸 ROS_DISCOVERY_SERVER='${ROS_DISCOVERY_SERVER:-<미설정>}' ≠ $want"
    echo "      → 이대로 대시보드를 띄우면 노트북 서비스가 로봇을 못 본다. ~/.bashrc 확인 후 새 터미널에서 재기동"
    ok=0
  fi

  # ⑤ 로봇 쪽 설정 (.bashrc 와 화이트리스트)
  robot_addr="$(_agent_field addr)"
  robot_ds="$(timeout 12 ssh -o BatchMode=yes -o ConnectTimeout=6 "${ROBOT_USER:-pinky}@$robot_addr" \
                'grep -m1 "^export ROS_DISCOVERY_SERVER=" ~/.bashrc | sed "s/^export ROS_DISCOVERY_SERVER=//;s/ *#.*//"' 2>/dev/null)"
  if [ -z "$robot_ds" ]; then
    echo "  ✖ 로봇 .bashrc 에 ROS_DISCOVERY_SERVER 가 없다(또는 ssh 실패)"; ok=0
    [ "$mode" = fix ] && _ds_fix_robot "$robot_addr" "$want" && ok=1
  elif [ "$robot_ds" != "$want" ]; then
    echo "  ✖ 로봇 설정 $robot_ds ≠ $want"
    if [ "$mode" = fix ]; then _ds_fix_robot "$robot_addr" "$want" && ok=1
    else ok=0; fi
  else
    echo "  ✔ 로봇 .bashrc ROS_DISCOVERY_SERVER=$robot_ds"
  fi

  # 로봇 화이트리스트가 자기 IP 와 맞는지 (여기가 어긋나면 로봇 쪽 UDP 가 안 생긴다)
  robot_wl="$(timeout 12 ssh -o BatchMode=yes -o ConnectTimeout=6 "${ROBOT_USER:-pinky}@$robot_addr" \
      'wl=$(grep -oE "[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+" ~/fastdds_lan.xml | head -1);
       ipv=$(ip -4 -o addr show wlan0 | awk "{print \$4}" | cut -d/ -f1);
       [ "$wl" = "$ipv" ] && echo "OK $wl" || echo "MISMATCH wl=$wl ip=$ipv"' 2>/dev/null)"
  case "$robot_wl" in
    OK*)       echo "  ✔ 로봇 화이트리스트 = 로봇 IP (${robot_wl#OK })" ;;
    MISMATCH*) echo "  ✖ 로봇 fastdds 화이트리스트 불일치 — $robot_wl"
               echo "      → 로봇 ~/fastdds_lan.xml 의 주소를 실제 IP 로 고치고 bringup 재기동"; ok=0 ;;
    *)         echo "  ? 로봇 화이트리스트 확인 실패(ssh)"; ok=0 ;;
  esac

  [ "$ok" = 1 ] && echo "  → 디스커버리 준비됨" || echo "  → ⚠️ 위 ✖ 를 먼저 해결할 것"
  return 0
}

_ds_fix_robot() {   # $1=로봇주소  $2=원하는 값
  echo "    … 로봇 .bashrc 를 $2 로 맞춘다"
  timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=6 "${ROBOT_USER:-pinky}@$1" "
    cp ~/.bashrc ~/.bashrc.bak_realboard 2>/dev/null
    if grep -q '^#\?export ROS_DISCOVERY_SERVER=' ~/.bashrc; then
      sed -i 's|^#\?export ROS_DISCOVERY_SERVER=.*|export ROS_DISCOVERY_SERVER=$2|' ~/.bashrc
    else
      echo 'export ROS_DISCOVERY_SERVER=$2' >> ~/.bashrc
    fi
    grep -m1 '^export ROS_DISCOVERY_SERVER=' ~/.bashrc" 2>/dev/null | sed 's/^/      /'
  echo "      ⚠️ 반영하려면 로봇에서 bringup 재기동 필요: ~/imu.sh on"
}

up() {
  # 디스커버리가 준비되지 않으면 대시보드로 서비스를 띄워도 서로 못 본다 —
  #  화면에는 'down' 으로만 보여 원인을 알 수 없다. 그래서 **띄우기 전에** 점검·기동한다.
  dds_check fix
  echo
  echo "▶ 실서버 대시보드 기동 (ROBOT_ID=$ROBOT_ID)"
  local pid
  pid="$(_dash_pid)"
  if [ -n "$pid" ]; then
    # ⚠ 여기서 아무것도 하지 않는다. 예전엔 '이미 떠 있음' 한 줄만 찍고 바로 status 가
    #  UP 을 출력해서 **재기동된 것처럼 보였다** — real_server.py 를 고쳐도 반영이 안 된
    #  채로 계속 쓰게 된다. 그래서 '안 띄웠다'는 사실과 근거(PID·경과)를 분명히 남긴다.
    echo "  ✖ 재기동하지 않았습니다 — 이미 떠 있는 프로세스를 그대로 둡니다"
    echo "     PID $pid · $(ps -o etime= -p "$pid" 2>/dev/null | tr -d ' ') 경과 · 시작 $(ps -o lstart= -p "$pid" 2>/dev/null | sed 's/^ *//')"
    echo "     → real_server.py·dgcommon.py 를 고쳤다면 이 프로세스엔 반영되지 않습니다."
    echo "       코드를 반영하려면: ./realboard.sh restart   (스택은 그대로 두고 대시보드만 새로)"
  else
    DG_REAL_PORT="$PORT" setsid bash -c "exec python3 $WEB/real_server.py" \
      >/tmp/dash_realweb.log 2>&1 & disown
    sleep 1.5
    pid="$(_dash_pid)"
    if [ -n "$pid" ]; then
      echo "  ✔ 새로 기동됨 — PID $pid (이 번호가 이전과 다르면 새 코드로 뜬 것)"
    else
      echo "  ✗ 기동 실패 — /tmp/dash_realweb.log 확인"
    fi
  fi
  status
  echo "  → 브라우저: http://localhost:$PORT   ('전체 기동' 버튼으로 스택을 올린다)"
}

# 대시보드 프로세스만 새로 띄운다(코드 수정 반영용). 로봇 스택은 건드리지 않는다 —
#  down 은 /api/stack/down 까지 보내 서비스를 전부 내리므로 이 용도로 쓰면 안 된다.
restart() {
  local old
  old="$(_dash_pid)"
  echo "▶ 실서버 대시보드 재기동"
  if [ -n "$old" ]; then
    echo "  이전 PID $old 종료"
    pkill -f 'dg_web/real_server.py' 2>/dev/null
    sleep 1
  else
    echo "  (떠 있지 않았음 — 새로 띄웁니다)"
  fi
  up
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
  # 재기동 확인용으로 이전 PID 를 먼저 받아 둔다. /health 의 uptime_s 는 **머신 부팅**
  #  경과라 재기동해도 그대로여서 판단 근거가 못 된다 — pid 가 바뀌었는지로 본다.
  local port before after
  port="$(_agent_field port)"
  before="$(curl -s -m 3 "http://$addr:$port/health" | _json_field pid)"
  ssh "$user@$addr" "pkill -f '[d]g_agent[.]py' || true" >/dev/null 2>&1
  sleep 0.5
  ssh "$user@$addr" "DG_AGENT_TOKEN='$token' setsid python3 \$HOME/dg_agent.py >/tmp/dg_agent.log 2>&1 & sleep 1; echo '  로봇에서 기동됨'" \
    || echo "  ✗ ssh 실패"
  sleep 1
  # /health 왕복을 감싸 시각을 재둔다 — 편차 계산에 왕복 지연의 절반을 보정하려면
  #  요청 전후 시각이 필요하다(ssh date 로 재면 왕복이 통째로 섞여 ±0.3s 오차가 난다).
  local health t0 t1
  t0="$(date +%s.%N)"
  health="$(curl -s -m 3 "http://$addr:$port/health")"
  t1="$(date +%s.%N)"
  after="$(printf '%s' "$health" | _json_field pid)"
  printf '%s\n' "$health"
  if [ -z "$after" ]; then
    echo "  ✗ 에이전트 응답 없음 — 기동 실패 (로봇에서 /tmp/dg_agent.log 확인)"
  elif [ -n "$before" ] && [ "$before" = "$after" ]; then
    echo "  ✖ PID 가 $before 에서 바뀌지 않았습니다 — **재기동되지 않았습니다**"
    echo "     (pkill 이 안 먹었거나 옛 프로세스가 그대로입니다. 새 코드가 반영되지 않은 상태)"
  else
    echo "  ✔ 재기동 확인 — PID ${before:-없음} → $after (agent_uptime_s=$(printf '%s' "$health" | _json_field agent_uptime_s))"
  fi
  [ -n "$after" ] && _clock_check "$user" "$addr" "$health" "$t0" "$t1"
}

# 로봇 시계 확인. **에이전트를 띄운 직후가 확인하기 가장 좋은 시점**이다 —
#  여기서 어긋난 걸 놓치면 나중에 주행 중 TF 구멍·라이다 드롭으로 나타나고,
#  그때는 원인이 시계라는 게 화면에 전혀 드러나지 않는다.
_clock_check() {
  local user="$1" addr="$2" health="$3" t0="$4" t1="$5" rtime skew src
  echo "▶ 시계 동기 확인"
  rtime="$(printf '%s' "$health" | _json_field time)"
  if [ -n "$rtime" ]; then
    # 편차 = 로봇시각 − (요청 중간시각). 왕복의 절반을 보정한다.
    skew="$(python3 -c "print('%+.3f' % ($rtime - ($t0 + $t1) / 2))" 2>/dev/null)"
    printf '  편차: %s 초  ' "${skew:-?}"
    case "$skew" in
      "") echo "(계산 실패)" ;;
      *) python3 - "$skew" <<'PY'
import sys
v = abs(float(sys.argv[1]))
# ROS2 는 메시지 타임스탬프로 TF 를 찾는다. 0.1s 를 넘기면 costmap 이 스캔을 버리기 시작한다
#  (nav2 transform_tolerance 가 0.2s 라 여유가 그 안쪽이어야 한다).
print('✔ 정상' if v < 0.05 else ('⚠️ 큼 — 주행 중 TF 구멍·라이다 드롭 위험' if v < 0.3
      else '✖ 심각 — ROS2 통신이 정상 동작하지 않는다'))
PY
      ;;
    esac
  else
    echo "  ? /health 에 time 이 없다(에이전트가 구버전)"
  fi

  # 판정은 chronyc sources 의 '^*' 와 Reach 로 한다.
  #  tracking 의 Reference ID 는 **소스가 죽어도 마지막 값이 남아** 정상처럼 보인다(08-04 오판).
  # agent() 가 직전에 ssh 를 연달아 쓰므로(pkill·기동) 곧바로 붙으면 느릴 때가 있다.
  #  여유를 주고 한 번 더 시도한다 — 여기서 실패하면 '시계 확인 못 함'으로 끝나 버린다.
  local try
  for try in 1 2; do
    src="$(timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=8 "$user@$addr" \
          "chronyc sources 2>/dev/null | awk '/^\^/{print \$1, \$2, \$5, \$6}'" 2>/dev/null)"
    [ -n "$src" ] && break
    sleep 1
  done
  if [ -n "$src" ]; then
    echo "  chrony 소스 (선택=^* / Reach 377=최근 8회 성공):"
    printf '%s\n' "$src" | sed 's/^/    /'
    printf '%s' "$src" | grep -q '^\^\*' || \
      echo "    ⚠️ 선택된 소스(^*)가 없다 — NTP 서버에 못 닿는다. ~/dds.sh 와 같은 주소인지 확인"
  else
    echo "  ? chrony 상태를 못 읽었다(ssh 실패 또는 chrony 미설치)"
  fi
}

# JSON 한 줄에서 최상위 필드 하나만 꺼낸다(jq 미설치 환경 대비).
_json_field() {
  python3 -c "
import json, sys
try:
    print(json.load(sys.stdin).get(sys.argv[1], '') or '')
except Exception:
    print('')
" "$1" 2>/dev/null
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
  restart) restart ;;
  status) dds_check report; echo; status ;;
  dds)    dds_check "${2:-report}" ;;
  agent)  agent ;;
  logs)   logs "${2:-}" ;;
  *) echo "usage: $0 {up|restart|down|status|dds [fix]|agent|logs <키>}"; exit 1 ;;
esac

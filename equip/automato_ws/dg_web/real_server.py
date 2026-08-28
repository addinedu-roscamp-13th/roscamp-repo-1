#!/usr/bin/env python3
"""dg_web — 실서버(실장비) 대시보드 서버.  포트 8010, localhost 전용.

시뮬 대시보드(control_server.py, :8000)와 **프로세스·포트·화면이 완전히 따로**다.
시뮬 스택을 통째로 내려도 이 화면은 살아 있고, 반대도 마찬가지다. 실장비가 붙은 채
시뮬 버튼을 누르는 사고를 막는 것이 분리의 목적이라, 여기에는 시뮬 명령이 아예 없다.

다루는 것 (cmdcfg 의 profile='real'):
    노트북  real-dcs · real-ai · real-nav2 · real-rviz · real-rosbridge
    로봇    robot-bringup · robot-ddago   (dg_agent 를 거쳐 실행)

API
    GET  /                          → real.html
    GET  /api/status                → 서비스별 UP/DOWN + 로봇 도달·시계편차
    GET  /api/system?host=          → 로봇 CPU·온도·전원·WiFi·DDS 설정
    GET  /api/commands              → 실서버 명령 정의(편집용)
    POST /api/commands/<key>        → 파라미터/명령줄 편집
    POST /api/commands/<key>/reset  → 기본값 복귀
    POST /api/svc/<key>/<start|stop>→ 개별 기동·종료
    GET  /api/svc/<key>/log         → 그 서비스의 stdout+stderr
    POST /api/stack/<up|down>       → 전체 기동·전체 종료 (정해진 순서대로)
    POST /api/agent/restart         → 로봇 에이전트 SSH 배포·기동(에이전트가 죽었을 때)
    GET  /api/logs/index            → 통합 로그 뷰어가 고를 수 있는 로그 목록
    GET  /api/wire?limit=           → DCS 시점 메시지 시계열(@@WIRE@@)
    POST /api/wire/clear            → 시계열 지우기(기준 시각 갱신)
  ── 충전소 복귀 ──
    GET  /api/map                   → 맵 이미지(raw PGM base64) + origin/resolution
    GET|POST /api/home              → 충전소 위치 조회·저장(robot_id 별)
    POST /api/home/clear            → 저장된 충전소 위치 삭제
    GET  /api/pose/once             → 로봇 위치 1회 (map→base_footprint TF, /amcl_pose 폴백)
  ── 웨이포인트 경로 ──
    GET|POST /api/waypoints         → 지도에서 찍은 웨이포인트 조회·저장(통째로)
    POST /api/route/start           → 선택한 순서대로 주행 → (확인) → 도킹
    POST /api/return/start          → 2단계 복귀 시작(주행 → **확인** → 도킹)
    GET  /api/return/status         → 진행 상황 폴링
    POST /api/return/confirm        → 접근점 도착 확인 → 도킹 진행 승인
    POST /api/return/stop           → 취소 요청 + 프로세스 종료 + cmd_vel 0
"""
import base64
import json
import math
import os
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import quote, urlparse, parse_qs

import cmdcfg
from dgcommon import (DASH, WEB_DIR, agent_call, clear_wire, first_robot, is_up,
                      load_map, map_fingerprint, occupancy_at, read_agents,
                      read_home, read_waypoints, read_wire, robot_id,
                      single_instance, tail_bytes, write_home, write_waypoints)

PORT = int(os.environ.get('DG_REAL_PORT', '8010'))

# 에이전트 설치·기동만은 **SSH 로** 한다. 에이전트가 곧 통신 채널이라, 그게 죽어 있으면
# 에이전트를 통해 살릴 수 없다(로봇을 재부팅하면 늘 이 상황이 된다).
REALBOARD = os.path.join(WEB_DIR, 'realboard.sh')   # 08-06: dg_web/ 안으로 이동

# 전체 기동 순서. 로봇이 먼저 서야 노트북 쪽이 붙을 대상이 생긴다.
#   ① 로봇 bringup(라이다·모터·odom) → ② rosbridge → ③ DdaGo Control(navigate·도킹) →
#   ④ Nav2(로봇 주행) → ⑤ RViz(2D Pose Estimate) → ⑥ AI → ⑦ DCS
# rosbridge 를 앞으로 뺀 이유: 다른 노드에 의존하지 않고 혼자 뜨므로, 일찍 올려 두면
# 뒤에 올라오는 것들을 처음부터 웹에서 들여다볼 수 있다.
# DCS 를 맨 뒤에 두는 이유: 뜨자마자 상대 액션 서버를 찾으므로, 상대가 먼저 있어야
# '서버 없음' 로그를 헛되이 남기지 않는다.
STACK_ORDER = ['robot-bringup', 'real-rosbridge', 'robot-ddago', 'real-nav2',
               'real-rviz', 'real-ai', 'real-dcs']


# ── 충전소 복귀 ─────────────────────────────────────────────────────────
RETURN_KEYS = ('real-return-nav', 'real-return-dock')
ROUTE_KEYS = ('real-route-nav', 'real-route-floor-dock')
# 웨이포인트 저장 직전 스냅샷. 실수로 덮었을 때 되돌릴 마지막 수단.
WAYPOINTS_BAK = os.path.join(WEB_DIR, 'waypoints.local.bak.json')
DOCK_KIND = {'floor': ('real-route-floor-dock', 'H마커 도킹 (FloorDock)'),
             'reflective': ('real-return-dock', '반사테이프 도킹 (ReflectiveDock)')}
NAV_TIMEOUT_S, DOCK_TIMEOUT_S = 240.0, 180.0
POSE_MAX_AGE_S = 15.0          # 이보다 오래된 위치로는 주행을 시작하지 않는다
# 복귀 진행 로그. 예전엔 버퍼 400줄에 응답은 60줄뿐이라 화면에서 스크롤할 게 거의 없었다
#  — 실패 원인은 앞쪽 줄에 있는 경우가 많은데 그게 잘려 나갔다.
RETURN_LOG_KEEP = 3000         # 서버가 스텝별로 들고 있는 줄 수
RETURN_LOG_DEFAULT = 500       # 화면이 안 지정하면 보내는 줄 수
# 접근점 도착 후 도킹 확인을 기다리는 상한. 무한 대기하면 화면을 닫았을 때 프로세스가 남는다.
DOCK_CONFIRM_TIMEOUT_S = 300.0
NAV2_PARAMS = os.path.expanduser(
    '~/pinky_pro/install/pinky_navigation/share/pinky_navigation/params/nav2_params_pinky2.yaml')

_RETURN = {'active': False, 'run_id': 0, 'phase': '', 'seq': 0, 'steps': [],
           'target': None, 'error': None, 'cancel': False, 'procs': [],
           # 접근점 도착 후 도킹 확인 대기 상태. await_dock=True 면 화면이 확인 버튼을 띄운다.
           'await_dock': False, 'dock_ok': False, 'awaited_at': 0.0}
_RETURN_LOCK = threading.Lock()


def real_keys():
    """서비스 카드·상태 조회 대상. oneshot(한 번 돌고 끝나는 동작)은 뺀다 —
    켜고 끄는 토글 모델에 맞지 않는다."""
    return [c['key'] for c in cmdcfg.describe_all('real')
            if not cmdcfg.DEFAULTS[c['key']].get('oneshot')]


def is_robot(key):
    return cmdcfg.DEFAULTS[key].get('host') == 'robot'


def log_path(key):
    return '/tmp/dash_%s.log' % cmdcfg.log_tag(key)


def robots_health():
    """로봇 도달 여부 + 시계 편차. 시계가 어긋나면 ROS2 통신 자체가 안 되므로 맨 앞에 둔다.
    에이전트가 자기 시각을 실어 보내므로 ssh-date 처럼 왕복 지연이 통째로 섞이지 않는다."""
    out = {}
    for name in read_agents():
        t0 = time.time()
        h = agent_call(name, '/health', timeout=3)
        rtt = time.time() - t0
        if h.get('error'):
            out[name] = {'reachable': False, 'error': h['error']}
            continue
        out[name] = {'reachable': True, 'host': h.get('host'),
                     'uptime_s': h.get('uptime_s'),
                     'clock_skew_s': round(h.get('time', 0) - (t0 + rtt / 2.0), 3),
                     'rtt_ms': round(rtt * 1000)}
    return out


def service_status(robots=None):
    """로컬은 pgrep, 로봇은 에이전트 /procs.

    로봇이 안 잡히면 'unknown' 이다 — 'down' 이 아니다. 로봇이 꺼진 것과 서비스가 안 뜬 것을
    같은 색으로 칠하면 원인을 잘못 짚는다. 못 닿는 로봇에 /procs 를 또 던지면 폴링마다
    타임아웃만큼 멎으므로 건너뛴다."""
    if robots is None:
        robots = robots_health()
    out, cache = {}, {}
    host = first_robot()
    for key in real_keys():
        chk = cmdcfg.DEFAULTS[key].get('check', '')
        if not is_robot(key):
            out[key] = 'up' if (chk and is_up('proc', chk)) else 'down'
            continue
        if not robots.get(host, {}).get('reachable'):
            out[key] = 'unknown'
            continue
        if chk not in cache:
            cache[chk] = agent_call(host, '/procs?match=' + quote(chk or '.'), timeout=5)
        r = cache[chk]
        out[key] = 'unknown' if r.get('error') else ('up' if r.get('procs') else 'down')
    return out


_cache = {'t': 0.0, 'data': None}


def snapshot(ttl=3.0):
    """화면이 4초마다 묻는데 로봇 왕복이 그보다 오래 걸릴 수 있어 짧게 캐시한다.
    없으면 느린 폴링이 서로 겹쳐 쌓인다."""
    now = time.time()
    if _cache['data'] and now - _cache['t'] < ttl:
        return _cache['data']
    robots = robots_health()
    data = {'services': service_status(robots), 'robots': robots,
            'robot_id': robot_id(), 'order': STACK_ORDER}
    _cache['t'], _cache['data'] = now, data
    return data


def check_requires(key):
    """cmdcfg 의 requires 를 확인한다. 못 갖췄으면 사유 문자열, 괜찮으면 None.

    선행 서비스 없이 띄우면 '떴다가 아무것도 안 되는' 상태가 되는데, 화면에는 그냥
    기동 실패로만 보여 원인을 짚기 어렵다(Nav2 가 대표적 — 로봇 TF·스캔이 없으면
    lifecycle_manager 가 무한 대기로 멎는다). 그래서 시작 전에 걸러 사유를 알려준다.
    """
    need = cmdcfg.DEFAULTS[key].get('requires') or []
    if not need:
        return None

    # '전체 기동'은 bringup 을 띄운 직후 곧바로 다음 것으로 넘어간다. 로봇 노드가 실제로
    #  보이기까지 몇 초 걸리므로, 바로 판정하면 멀쩡한 순서인데도 막힌다 → 잠깐 기다린다.
    #  정말 안 떠 있으면 이 시간만큼 늦게 사유가 나올 뿐, 결과는 같다.
    deadline = time.time() + 12.0
    st = service_status()
    while any(st.get(d) != 'up' for d in need) and time.time() < deadline:
        time.sleep(2.0)
        _cache['data'] = None      # 캐시를 비워야 새로 읽는다
        st = service_status()

    for dep in need:
        s = st.get(dep)
        if s == 'up':
            continue
        label = cmdcfg.DEFAULTS.get(dep, {}).get('label', dep)
        if s == 'unknown':
            return ('%s(%s) 상태를 확인할 수 없습니다 — 로봇에 닿지 않습니다. '
                    '로봇 전원·네트워크를 확인한 뒤 다시 시도하세요.' % (label, dep))
        return ('%s(%s) 가 먼저 떠 있어야 합니다 — 로봇의 odom TF 와 /scan 이 없으면 '
                'Nav2 는 활성화되지 못하고 멎습니다. %s 를 먼저 기동하세요.'
                % (label, dep, dep))
    return None


def svc_start(key):
    why = check_requires(key)
    if why:
        return {'key': key, 'where': 'local' if not is_robot(key) else first_robot(),
                'ok': False, 'error': why, 'lines': ['[선행 조건 미충족] ' + why]}
    if not is_robot(key):
        r = subprocess.run(['bash', DASH, 'start-key', key],
                           capture_output=True, text=True, timeout=30)
        time.sleep(1.2)   # 뜨자마자 죽는 경우를 로그에 담기 위한 짧은 대기
        return {'key': key, 'where': 'local', 'ok': True,
                'lines': (r.stdout + r.stderr).splitlines() + svc_log(key)['lines'][-40:]}
    # 로봇: 편집된 명령줄을 그대로 에이전트에 넘긴다(로봇 명령도 화면에서 고칠 수 있게).
    res = agent_call(first_robot(), '/run', 'POST',
                     {'name': key, 'cmdline': cmdcfg.effective(key)}, timeout=15)
    time.sleep(1.2)
    return {'key': key, 'where': first_robot(), 'ok': not res.get('error'),
            'error': res.get('error'), 'lines': svc_log(key)['lines'][-40:]}


def svc_stop(key):
    """종료하고 **정말 죽었는지 다시 확인해서** 보고한다.

    신호를 보냈다는 사실만으로 ok 를 돌려주면, 상대가 SIGINT 를 무시하거나 패턴이 안 맞아
    하나도 못 죽였을 때에도 화면에는 성공으로 뜬다. 그러면 실장비가 살아 있는 줄 모르고
    다음 작업으로 넘어간다 — 여기서는 그게 제일 위험하다."""
    pat = cmdcfg.DEFAULTS[key].get('check', '')
    if not is_robot(key):
        r = subprocess.run(['bash', DASH, 'stop-pattern', pat],
                           capture_output=True, text=True, timeout=30)
        time.sleep(1.0)
        still = bool(pat and is_up('proc', pat))
        return {'key': key, 'where': 'local', 'ok': not still,
                'error': '아직 떠 있습니다 — 종료 신호를 무시했을 수 있습니다' if still else None,
                'lines': (r.stdout + r.stderr).splitlines()}
    host = first_robot()
    res = agent_call(host, '/stop', 'POST', {'name': key, 'pattern': pat}, timeout=15)
    lines = [json.dumps(res, ensure_ascii=False)]
    if res.get('error'):
        return {'key': key, 'where': host, 'ok': False, 'error': res['error'], 'lines': lines}
    time.sleep(1.0)
    chk = agent_call(host, '/procs?match=' + quote(pat or '.'), timeout=5)
    still = bool(not chk.get('error') and chk.get('procs'))
    return {'key': key, 'where': host, 'ok': not still,
            'error': '아직 떠 있습니다 — 종료 신호를 무시했을 수 있습니다' if still else None,
            'lines': lines}


def extra_logs(key, n=400):
    """이 서비스가 **따로 남기는** 로그들. 스크립트가 백그라운드로 띄우면 대시보드가 잡는
    출력에는 스크립트 몇 줄만 남고 정작 알맹이는 그 파일에 있다(nav.sh 가 그렇다)."""
    out = []
    for path in cmdcfg.DEFAULTS[key].get('extra_logs') or []:
        out.append({'title': path, 'lines': tail_bytes(path, 262144)[-n:]})
    return out


def svc_log(key, n=400):
    if not is_robot(key):
        # @@WIRE@@ 는 사람이 읽을 로그가 아니라 메시지 시계열용 데이터라 걷어낸다.
        lines = [ln for ln in tail_bytes(log_path(key), 262144) if '@@WIRE@@' not in ln]
        return {'key': key, 'where': 'local', 'lines': lines[-n:],
                'extra': extra_logs(key, n)}
    res = agent_call(first_robot(), '/log?name=%s&n=%d' % (quote(key), n), timeout=8)
    if res.get('error'):
        return {'key': key, 'where': first_robot(),
                'lines': ['(로봇 로그를 못 읽었습니다: %s)' % res['error']]}
    return {'key': key, 'where': first_robot(),
            'lines': res.get('lines') or ['(아직 로그 없음 — 이 화면에서 기동한 적이 없습니다)']}


def agent_restart():
    """realboard.sh agent — scp 로 dg_agent.py 를 올리고 ssh 로 다시 띄운다.
    토큰은 스크립트가 agents.local.json 에서 읽으므로 여기서 다루지 않는다(로그에도 안 남는다)."""
    try:
        r = subprocess.run(['bash', REALBOARD, 'agent'],
                           capture_output=True, text=True, timeout=120)
        lines = (r.stdout + r.stderr).splitlines()
    except subprocess.TimeoutExpired:
        return {'ok': False, 'lines': ['(120초 안에 끝나지 않았습니다 — 로봇 전원·네트워크 확인)']}
    except OSError as e:
        return {'ok': False, 'lines': ['실행 실패: %s' % e]}
    time.sleep(1.0)
    h = agent_call(first_robot(), '/health', timeout=5)
    ok = not h.get('error')
    if not ok:
        lines.append('— 확인: ' + str(h.get('error')))
    else:
        # '응답 OK' 만으로는 재기동 여부를 알 수 없다(옛 프로세스도 똑같이 응답한다).
        #  pid·agent_uptime_s 를 같이 보여줘 새로 뜬 것인지 눈으로 확인되게 한다.
        age = h.get('agent_uptime_s')
        fresh = isinstance(age, (int, float)) and age < 30
        lines.append('— 확인: 에이전트 응답 OK (%s) · PID %s · 기동 후 %s초'
                     % (h.get('host'), h.get('pid', '?'),
                        age if age is not None else '?'))
        lines.append('— 판정: ' + ('새 프로세스로 재기동됨' if fresh else
                                   '⚠ 기동 후 시간이 오래됐습니다 — 재기동되지 않았을 수 있습니다'
                                   ' (에이전트가 구버전이면 이 값이 안 나옵니다)'))
    _cache['data'] = None      # 상태 캐시 무효화 — 방금 살아났을 수 있다
    return {'ok': ok, 'lines': lines}


def stack_up():
    """전체 기동. 순서대로 올리고 각 단계 결과를 함께 돌려준다.
    앞 단계가 실패해도 멈추지 않는다 — 어디까지 되고 어디서 막혔는지 한 번에 보는 게 낫다."""
    steps = []
    for key in STACK_ORDER:
        r = svc_start(key)
        steps.append({'key': key, 'ok': r['ok'], 'where': r['where'],
                      'error': r.get('error'), 'tail': r['lines'][-6:]})
    _cache['data'] = None   # 상태 캐시 무효화 — 방금 바꿔 놨으니 다시 읽어야 한다
    return {'steps': steps}


def stack_down():
    """전체 종료. 기동의 역순으로 내린다(의존하는 쪽을 먼저 끊는다)."""
    steps = []
    for key in reversed(STACK_ORDER):
        r = svc_stop(key)
        steps.append({'key': key, 'ok': r['ok'], 'where': r['where'],
                      'error': r.get('error'), 'tail': r['lines'][-4:]})
    _cache['data'] = None
    return {'steps': steps}


# ── 충전소 복귀: 좌표·사전점검 ──────────────────────────────────────────
def nav2_initial_pose():
    """nav2_params.yaml 의 set_initial_pose/initial_pose 를 읽는다.

    amcl 이 `set_initial_pose: true` 라 **로봇이 어디 있든 그럴듯한 pose 를 낸다.**
    화면은 '위치 확인됨'으로 보이는데 실제와 다른 좌표계로 주행하는, 조용히 틀리는
    실패가 이 기능의 최대 위험이다. 현재 pose 가 이 초기값과 같으면 경고해야 한다."""
    txt = ''
    try:
        with open(NAV2_PARAMS, 'r', encoding='utf-8') as f:
            txt = f.read()
    except OSError:
        return None
    if not re.search(r'set_initial_pose:\s*true', txt):
        return None
    out = {}
    for k in ('x', 'y', 'yaw'):
        m = re.search(r'^\s*%s:\s*(-?[\d.eE+]+)' % k, txt, re.M)
        if m:
            try:
                out[k] = float(m.group(1))
            except ValueError:
                pass
    return out if len(out) == 3 else None


def nav_target(home):
    """저장된 **충전소 진입점 그 자체**가 주행 목표다. 어떤 오프셋도 더하지 않는다.

    08-06 정정: 예전에는 '저장값 = 도킹된 충전소 자리'로 보고 헤딩 방향으로
    approach_offset_m(0.3m) 앞을 계산했다. 실제 운용은 **저장하는 위치가 이미 진입점**이라
    (사용자가 도킹 시작 자리에 로봇을 세우고 저장한다) 오프셋을 또 더해 엉뚱한 곳으로 주행했다.
    오프셋 개념 자체를 없앴다 — 자리를 바꾸려면 **저장 위치를 다시 잡는다**.
    화면에서 눈으로 보고 저장하므로 그게 더 직관적이고 어긋날 여지가 없다.
    """
    return {'x': float(home['x']), 'y': float(home['y']), 'yaw': float(home['yaw'])}


def return_precheck(body):
    """하나라도 걸리면 **아무것도 실행하지 않고** 사유를 돌려준다.
    검사 없이 시작한 주행은 실장비에서 되돌릴 수 없다."""
    rid = robot_id()
    if _RETURN['active']:
        return {'ok': False, 'code': 409, 'error': '이미 복귀가 진행 중입니다'}

    home = read_home().get(rid)
    if not home:
        return {'ok': False, 'error': '충전소 위치가 저장돼 있지 않습니다 — 먼저 저장하세요'}

    m = load_map()
    if not m.get('ok'):
        return {'ok': False, 'error': '맵을 읽지 못했습니다: %s' % m.get('error')}
    now_fp, saved_fp = map_fingerprint(m), home.get('map')
    if saved_fp and (saved_fp.get('origin') != now_fp['origin']
                     or saved_fp.get('resolution') != now_fp['resolution']
                     or saved_fp.get('size') != now_fp['size']
                     or saved_fp.get('mtime') != now_fp['mtime']):
        return {'ok': False, 'error': '맵이 바뀌었습니다 — 저장된 좌표는 딴 곳을 가리킵니다. '
                                      '충전소 위치를 다시 저장하세요'}

    svc = snapshot()['services']
    for key, why in (('real-nav2', 'Nav2 가 안 떠 있습니다'),
                     ('real-dcs', 'DCS 가 안 떠 있습니다 — /%s/navigate 액션 서버가 없습니다' % rid),
                     ('robot-bringup', '로봇 bringup 이 안 떠 있습니다'),
                     ('robot-ddago', 'DdaGo Control 이 안 떠 있습니다')):
        st = svc.get(key)
        if st != 'up':
            # 'unknown'(로봇 미도달)도 막는다 — 모르는 상태로 주행을 시작하지 않는다.
            extra = ' (상태 확인불가 — 로봇에 닿지 않습니다)' if st == 'unknown' else ''
            return {'ok': False, 'error': why + extra, 'need': key}

    pose = body.get('pose') or {}
    try:
        px, py = float(pose['x']), float(pose['y'])
    except (KeyError, TypeError, ValueError):
        return {'ok': False, 'error': '로봇 위치가 없습니다'}
    if pose.get('frame_id') and pose['frame_id'] != 'map':
        return {'ok': False, 'error': "위치 frame 이 map 이 아닙니다: %s" % pose['frame_id']}
    age = float(pose.get('age_s') or 0.0)
    if age > POSE_MAX_AGE_S:
        # 화면이 보낸 값이 낡았다고 바로 막지 않는다 — 그 값은 /amcl_pose 기반이라
        #  서 있는 로봇에서는 낡는 것이 정상이다(update_min_d). TF 로 다시 읽어 본다.
        tf = pose_tf()
        if tf.get('ok') and float(tf.get('age_s') or 0.0) <= POSE_MAX_AGE_S:
            px, py = tf['x'], tf['y']
            pose = dict(pose, x=px, y=py, yaw=tf['yaw'],
                        age_s=tf['age_s'], source='tf')
            age = float(tf['age_s'])
        else:
            return {'ok': False,
                    'error': '로봇 위치를 확인할 수 없습니다 — map→base_footprint TF 가 '
                             '흐르지 않습니다(%s). Nav2·bringup 기동과 초기 위치를 확인하세요'
                             % (tf.get('error') or '%.0f초 전' % float(tf.get('age_s') or 0))}

    for k in RETURN_KEYS:
        if cmdcfg.cmdline_of(k):
            return {'ok': False, 'error': "'%s' 의 명령줄 전체가 편집돼 있어 좌표 주입이 "
                                          "무시됩니다 — 설정에서 기본값으로 되돌리세요" % k}

    tgt = nav_target(home)
    occ = occupancy_at(m, tgt['x'], tgt['y'])
    if occ is None:
        return {'ok': False, 'error': '목표점(%.2f, %.2f)이 맵 밖입니다 — '
                                      '진입점 위치를 다시 저장하세요' % (tgt['x'], tgt['y'])}
    if occ['state'] == 'occupied':
        return {'ok': False, 'error': '목표점이 점유 셀(벽)입니다 — 진입점 위치를 다시 저장하세요'}

    diag = math.hypot(m['width'] * m['resolution'], m['height'] * m['resolution'])
    dist = math.hypot(px - float(home['x']), py - float(home['y']))
    if dist > diag * 1.5:
        return {'ok': False, 'error': '로봇 위치와 충전소가 비정상적으로 멉니다(%.2f m) — '
                                      'amcl 이 엉뚱한 곳에 수렴했을 수 있습니다' % dist}

    ip = nav2_initial_pose()
    if ip and not body.get('force_pose'):
        if (abs(px - ip['x']) < 0.05 and abs(py - ip['y']) < 0.05):
            return {'ok': False, 'need_force': True,
                    'error': 'AMCL 초기 pose 와 같습니다 — 실제 위치가 아닐 수 있습니다. '
                             '로봇을 조금 움직여 수렴을 확인하거나, 확인했다면 체크박스를 켜세요'}
    return {'ok': True, 'home': home, 'target': tgt, 'occ': occ}


# ── 충전소 복귀: 실행 ───────────────────────────────────────────────────
_NAV_OK = re.compile(r'Goal finished with status:\s*SUCCEEDED')
_NAV_BAD = re.compile(r'Goal was rejected|status:\s*(ABORTED|CANCELED)|'
                      r'Unable to find action server')


def _bump(**kw):
    with _RETURN_LOCK:
        _RETURN.update(kw)
        _RETURN['seq'] += 1


def _run_step(step, cmdline, timeout):
    """한 단계를 돌리며 출력을 실시간으로 모은다.

    ⚠ PYTHONUNBUFFERED 가 없으면 ros2 CLI 가 블록 버퍼링이라 **끝날 때까지 한 줄도**
    안 나온다 — 진행 표시가 통째로 죽는다.
    ⚠ start_new_session 으로 프로세스 그룹을 확보한다(정지에서 그룹째 신호를 준다).
    종료코드는 믿지 않는다: run_line 이 파이프라인이라 tee 의 코드가 나오고,
    send_goal 은 ABORTED 여도 0 을 내는 경우가 있다. 출력으로 판정한다."""
    env = dict(os.environ, PYTHONUNBUFFERED='1')
    try:
        p = subprocess.Popen(['bash', DASH, 'run-cmdline', step['key'], cmdline],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                             text=True, bufsize=1, start_new_session=True, env=env)
    except OSError as e:
        step['state'], step['error'] = 'fail', '실행 실패: %s' % e
        _bump()
        return False
    with _RETURN_LOCK:
        _RETURN['procs'].append(p)
    t0, ok, bad = time.monotonic(), False, None
    try:
        for ln in p.stdout:
            ln = ln.rstrip('\n')
            step['lines'].append(ln)
            del step['lines'][:-RETURN_LOG_KEEP]
            if _NAV_OK.search(ln):
                ok = True
            m = _NAV_BAD.search(ln)
            if m:
                bad = ln.strip()
            step['progress'] = _progress_of(step['key'], ln) or step['progress']
            step['elapsed_s'] = round(time.monotonic() - t0, 1)
            _bump()
            if time.monotonic() - t0 > timeout:
                bad = '시간 초과(%.0f초)' % timeout
                break
    except Exception as e:
        bad = '출력을 읽는 중 오류: %s' % e
    finally:
        try:
            p.stdout.close()
        except Exception:
            pass
        try:
            p.wait(timeout=5)
        except Exception:
            pass
        with _RETURN_LOCK:
            if p in _RETURN['procs']:
                _RETURN['procs'].remove(p)

    if _RETURN['cancel']:
        step['state'], step['error'] = 'canceled', '사용자 정지'
    elif step['key'] == 'real-return-dock':
        # 도킹은 액션 성공만으로 부족하다 — result_code 0 이어야 실제로 붙은 것이다.
        joined = '\n'.join(step['lines'][-40:])
        if ok and re.search(r'result_code[=:\s]+0\b', joined):
            step['state'] = 'ok'
        else:
            step['state'] = 'fail'
            step['error'] = bad or '도킹 실패 — result_code 가 0 이 아닙니다'
    else:
        step['state'] = 'ok' if (ok and not bad) else 'fail'
        if step['state'] == 'fail':
            step['error'] = _translate(bad or '성공 표시를 찾지 못했습니다')
    step['elapsed_s'] = round(time.monotonic() - t0, 1)
    _bump()
    return step['state'] == 'ok'


def _translate(msg):
    if 'Unable to find action server' in msg:
        return '액션 서버가 없습니다 — Nav2/DCS 가 떠 있는지 확인하세요'
    if 'Goal was rejected' in msg:
        return 'goal 이 거부됐습니다 — 로그와 /tmp/nav_single.log 를 확인하세요'
    return msg


def _progress_of(key, ln):
    if key == 'real-return-nav':
        m = re.search(r'waypoint_index[=:\s]+(\d+)', ln)
        if m:
            return '구간 %s 진행 중' % m.group(1)
        m = re.search(r'current_x[=:\s]+(-?[\d.]+).*current_y[=:\s]+(-?[\d.]+)', ln)
        if m:
            return '현재 (%.2f, %.2f)' % (float(m.group(1)), float(m.group(2)))
    else:
        m = re.search(r"phase[=:\s]+'?(\w+)'?", ln)
        if m:
            d = re.search(r'distance_to_marker_m[=:\s]+([\d.]+)', ln)
            return '%s%s' % (m.group(1), '  (마커까지 %.2f m)' % float(d.group(1)) if d else '')
    return None


def _new_step(key, title):
    return {'key': key, 'title': title, 'state': 'pending', 'progress': '',
            'elapsed_s': 0.0, 'lines': [], 'error': None}


def _await_dock_confirm():
    """① 진입점 주행이 끝나면 **여기서 멈추고 사용자 확인을 기다린다**(사용자 지시).

    후진 도킹은 되돌릴 수 없는 기동이다 — 접근점에 제대로 섰는지 눈으로 보고 시작하는 것과
    주행이 끝나자마자 이어서 후진하는 것은 사고의 대가가 다르다(08-05 에 벽 충돌로 로봇을
    손으로 집어낸 적이 있다). 그래서 자동 연결을 끊고 확인을 받는다.

    True=진행 / False=취소·타임아웃. 무한 대기는 하지 않는다 — 화면을 닫아 버리면
    프로세스가 영영 남는다.
    """
    _bump(phase='await_dock', await_dock=True, awaited_at=time.time())
    deadline = time.time() + DOCK_CONFIRM_TIMEOUT_S
    while time.time() < deadline:
        with _RETURN_LOCK:
            if _RETURN['cancel']:
                _RETURN['await_dock'] = False
                return False
            if _RETURN.get('dock_ok'):
                _RETURN['await_dock'] = False
                _RETURN['dock_ok'] = False
                return True
        time.sleep(0.3)
    _bump(await_dock=False,
          error='도킹 확인을 %d초 안에 받지 못해 중단했습니다' % int(DOCK_CONFIRM_TIMEOUT_S))
    return False


def _return_worker(home, target):
    """① 주행 → (확인) → ② 도킹. **앞 단계가 실패하면 즉시 멈춘다.**
    stack_up() 은 '실패해도 계속'이지만 여기는 정반대다 — 잘못된 위치에서 후진 도킹을
    시작하는 것과, 진단용으로 끝까지 훑어보는 것은 실패의 대가가 다르다."""
    steps = _RETURN['steps']
    plans = [
        (steps[0], {'x': '%.4f' % target['x'], 'y': '%.4f' % target['y'],
                    'yaw': '%.4f' % target['yaw']}, NAV_TIMEOUT_S, 'nav'),
        (steps[1], {'task_point_id': home.get('task_point_id') or 'CHARGE_01'},
         DOCK_TIMEOUT_S, 'dock'),
    ]
    try:
        for step, extra, timeout, phase in plans:
            if _RETURN['cancel']:
                step['state'] = 'canceled'
                break
            # 도킹 직전에만 확인을 받는다. 주행은 확인 없이 그대로 간다.
            if phase == 'dock':
                steps[0]['progress'] = '진입점 도착 — 도킹 확인 대기'
                if not _await_dock_confirm():
                    step['state'] = 'canceled'
                    break
            step['state'] = 'running'
            _bump(phase=phase)
            line = cmdcfg.effective(step['key'], extra=extra)
            step['lines'].append('$ ' + line)
            if not _run_step(step, line, timeout):
                break
    except Exception as e:
        _bump(error='복귀 중 오류: %s' % e)
    finally:
        _bump(active=False, phase='done', await_dock=False)


def return_start(body):
    chk = return_precheck(body)
    if not chk.get('ok'):
        return chk
    with _RETURN_LOCK:
        if _RETURN['active']:
            return {'ok': False, 'code': 409, 'error': '이미 복귀가 진행 중입니다'}
        _RETURN.update({
            'active': True, 'run_id': _RETURN['run_id'] + 1, 'phase': 'start',
            'cancel': False, 'error': None, 'procs': [],
            'target': dict(chk['target'], occ=chk['occ']),
            'steps': [_new_step('real-return-nav', '① 복귀 주행'),
                      _new_step('real-return-dock', '② 충전소 도킹')],
        })
        _RETURN['seq'] += 1
    threading.Thread(target=_return_worker, args=(chk['home'], chk['target']),
                     daemon=True).start()
    return {'ok': True, 'run_id': _RETURN['run_id'], 'target': chk['target']}


def return_status(n=RETURN_LOG_DEFAULT):
    n = max(50, min(int(n or RETURN_LOG_DEFAULT), RETURN_LOG_KEEP))
    with _RETURN_LOCK:
        steps = [dict(st, lines=st['lines'][-n:]) for st in _RETURN['steps']]
        out = {k: _RETURN[k] for k in ('active', 'run_id', 'phase', 'seq', 'target',
                                       'error', 'await_dock')}
        # 남은 확인 시간을 같이 준다 — 화면이 '언제까지 눌러야 하는지' 보여줄 수 있게.
        if _RETURN['await_dock']:
            left = DOCK_CONFIRM_TIMEOUT_S - (time.time() - (_RETURN['awaited_at'] or 0))
            out['await_left_s'] = max(0, round(left))
    out['steps'] = steps
    # Nav2 는 백그라운드로 돌며 자기 로그에 쌓는다 — goal 을 거절한 진짜 이유는
    # CLI 출력이 아니라 거기 있다.
    out['extra'] = extra_logs('real-nav2', 120)
    return out


def return_confirm_dock():
    """접근점 도착 확인 → 도킹 진행 승인. 대기 중이 아닐 때 눌리면 아무 일도 하지 않는다
    (폴링 지연으로 버튼이 잠깐 남아 있을 수 있어, 늦게 눌러도 사고가 나지 않게)."""
    with _RETURN_LOCK:
        if not _RETURN['active']:
            return {'ok': False, 'error': '복귀가 진행 중이 아닙니다'}
        if not _RETURN['await_dock']:
            return {'ok': False, 'error': '지금은 도킹 확인 단계가 아닙니다'}
        _RETURN['dock_ok'] = True
        _RETURN['seq'] += 1
    return {'ok': True, 'lines': ['도킹 진행 승인 — 후진 도킹을 시작합니다']}



# ── 지도에서 찍은 웨이포인트 ─────────────────────────────────────────────
def wp_list():
    return (read_waypoints().get(robot_id()) or {}).get('points') or []


def wp_save(points):
    """통째로 덮어쓴다. 화면이 목록 전체를 들고 있으므로 부분 갱신 API 를 따로 두지 않는다
    (부분 갱신은 화면과 서버의 순서가 어긋날 때 조용히 깨진다)."""
    out, seen = [], set()
    for i, p in enumerate(points or []):
        try:
            x, y = float(p['x']), float(p['y'])
        except (KeyError, TypeError, ValueError):
            return {'ok': False, 'error': '%d번째 점의 좌표가 잘못됐습니다' % (i + 1)}
        dock = p.get('dock') or 'none'
        if dock not in ('none', 'floor', 'reflective'):
            return {'ok': False, 'error': '알 수 없는 도킹 종류: %s' % dock}
        wid = int(p.get('id') or (i + 1))
        while wid in seen:          # id 는 Feedback 추적용이라 겹치면 안 된다
            wid += 1
        seen.add(wid)
        out.append({'id': wid, 'name': (p.get('name') or 'WP%d' % wid)[:40],
                    'x': x, 'y': y, 'yaw': float(p.get('yaw') or 0.0),
                    # yaw_fixed: 사람이 정한 방향(지도에서 끌기·각도 입력·로봇 자세 복사).
                    #  켜져 있으면 '다음 점 바라보기' 자동 계산이 덮지 않는다.
                    'yaw_fixed': bool(p.get('yaw_fixed')),
                    'dock': dock, 'task_point_id': (p.get('task_point_id') or '')[:40]})
    data = read_waypoints()
    # 덮어쓰기 전에 직전 내용을 남긴다. 지도를 클릭해 하나하나 찍어 만든 값이라 날리면
    #  복구할 방법이 없다(08-07 에 실제로 테스트가 사용자 경로를 덮어썼다).
    prev = (data.get(robot_id()) or {}).get('points')
    if prev:
        try:
            with open(WAYPOINTS_BAK, 'w', encoding='utf-8') as f:
                json.dump({'robot_id': robot_id(), 'saved_at': time.time(),
                           'points': prev}, f, ensure_ascii=False, indent=2)
        except OSError:
            pass          # 백업 실패가 저장을 막을 이유는 없다
    data[robot_id()] = {'points': out, 'saved_at': time.time(),
                        'map': map_fingerprint(load_map())}
    write_waypoints(data)
    return {'ok': True, 'points': out}


def _is_dock_target(p, i, last):
    """이 점에서 실제로 도킹을 할 것인가. **경로의 마지막 점**일 때만 참이다.
    중간에 낀 도킹 지점은 통과 노드로 다룬다."""
    return i == last and p.get('dock', 'none') != 'none'


def _holds_yaw(p, i, last):
    """이 점에서 목표 방향으로 **고쳐 서야** 하는가 — 마지막 점에서만 참이다.

    중간 지점은 도킹 속성이 붙어 있든 방향을 직접 지정했든 **그냥 지나간다**(사용자 지시).
    통과 노드마다 고개를 돌리면 지점마다 멈칫거리고, 경로가 길수록 손해만 커진다
    (Waypoint.msg: "통과 노드 전반에 켜면 안 된다 — 두리번거린다").
    """
    if i != last:
        return False
    return p.get('dock', 'none') != 'none' or bool(p.get('yaw_fixed'))


def _wp_yaw_chain(pts):
    """yaw 는 **다음 점을 바라보게** 자동 계산한다(사용자 선택).
    마지막 점은 직전 방향을 유지한다 — 바라볼 다음 점이 없다.

    자동에서 빠지는 두 경우:
      · 도킹이 걸린 점 — 마커를 정면에서 봐야 도킹이 시작된다. '가는 방향'으로 서면 실패한다
        (Waypoint.msg 의 hold_yaw 설명과 같은 이유)
      · yaw_fixed 인 점 — 사람이 지도에서 끌거나 각도를 입력해 정한 값이다. 덮으면 안 된다
    """
    out = []
    last = len(pts) - 1
    for i, p in enumerate(pts):
        yaw = float(p.get('yaw') or 0.0)
        # 도킹 속성은 **마지막 점에서만** 의미가 있다. 중간에 낀 도킹 지점은 그냥 지나가는
        #  통과 노드로 다룬다(사용자 지시) — 안 그러면 지날 때마다 마커 방향으로 고쳐 서느라
        #  멈칫거리고, 경로가 길수록 손해만 커진다.
        if not _holds_yaw(p, i, last):
            if i + 1 < len(pts):
                yaw = math.atan2(pts[i + 1]['y'] - p['y'], pts[i + 1]['x'] - p['x'])
            elif out:
                yaw = out[-1]['yaw']
        out.append(dict(p, yaw=yaw))
    return out


def _wp_goal_yaml(pts):
    """Navigate goal 의 waypoints 배열 문자열. hold_yaw 는 **도킹 지점만** 켠다 —
    통과 노드마다 켜면 지점마다 고개를 돌리느라 두리번거린다(Waypoint.msg 주석)."""
    items = []
    last = len(pts) - 1
    for i, p in enumerate(pts):
        # hold_yaw 는 **도킹할 마지막 점**과 사용자가 방향을 직접 정한 점에만 켠다.
        #  통과 노드 전반에 켜면 지점마다 고개를 돌리느라 두리번거린다(Waypoint.msg 주석).
        hold = _holds_yaw(p, i, last)
        items.append(
            '{waypoint_id: %d, x: %.4f, y: %.4f, yaw: %.4f, capture: false, hold_yaw: %s}'
            % (int(p['id']), p['x'], p['y'], p['yaw'], 'true' if hold else 'false'))
    return ', '.join(items)


def route_start(body):
    """선택한 웨이포인트를 **순서대로** 주행하고, 마지막 점에 도킹이 걸려 있으면
    도착 확인을 받은 뒤 도킹한다. 복귀와 같은 진행/취소/확인 machinery 를 그대로 쓴다."""
    if _RETURN['active']:
        return {'ok': False, 'code': 409, 'error': '이미 주행이 진행 중입니다'}
    ids = body.get('ids') or []
    if not ids:
        return {'ok': False, 'error': '주행할 웨이포인트를 선택하세요'}
    by = {int(p['id']): p for p in wp_list()}
    pts = [by[int(i)] for i in ids if int(i) in by]
    if len(pts) != len(ids):
        return {'ok': False, 'error': '저장되지 않은 웨이포인트가 섞여 있습니다 — 다시 저장하세요'}

    m = load_map()
    if not m.get('ok'):
        return {'ok': False, 'error': '맵을 읽지 못해 시작할 수 없습니다'}
    for p in pts:
        # ⚠️ occupancy_at 은 **dict** 를 돌려준다({'p','state','col','row'}) — 숫자로 비교하면
        #    TypeError 로 응답이 통째로 죽는다(08-07 실제 발생). return_precheck 과 같은 방식으로 본다.
        occ = occupancy_at(m, p['x'], p['y'])
        if occ is None:
            return {'ok': False, 'error': "'%s' 가 맵 범위 밖입니다" % p['name']}
        if occ['state'] == 'occupied':
            return {'ok': False, 'error': "'%s' 가 점유 셀(벽)입니다 — 위치를 다시 잡으세요" % p['name']}

    pts = _wp_yaw_chain(pts)
    last = pts[-1]
    dock_kind = last.get('dock', 'none')
    if dock_kind != 'none' and not last.get('task_point_id'):
        return {'ok': False,
                'error': "'%s' 에 도킹이 걸려 있는데 task_point_id 가 비었습니다" % last['name']}

    steps = [_new_step('real-route-nav',
                       '① 경로 주행 (%d개 지점)' % len(pts))]
    if dock_kind != 'none':
        steps.append(_new_step(DOCK_KIND[dock_kind][0],
                               '② %s — %s' % (DOCK_KIND[dock_kind][1], last['task_point_id'])))
    with _RETURN_LOCK:
        _RETURN.update({
            'active': True, 'run_id': _RETURN['run_id'] + 1, 'phase': 'start',
            'seq': _RETURN['seq'] + 1, 'steps': steps, 'error': None, 'cancel': False,
            'procs': [], 'await_dock': False, 'dock_ok': False,
            'target': {'x': last['x'], 'y': last['y'], 'yaw': last['yaw']}})
    # auto_dock: 시작할 때 이미 '도킹까지 진행'을 승인받았으면 도착 확인을 건너뛴다.
    #  결정 시점을 앞으로 당긴 것일 뿐, 확인 없이 도킹하는 경로는 만들지 않는다.
    auto_dock = bool(body.get('auto_dock'))
    threading.Thread(target=_route_worker, args=(pts, dock_kind, auto_dock),
                     daemon=True).start()
    return {'ok': True, 'run_id': _RETURN['run_id'], 'points': pts,
            'auto_dock': auto_dock}


def _route_worker(pts, dock_kind, auto_dock=False):
    steps = _RETURN['steps']
    try:
        steps[0]['state'] = 'running'
        _bump(phase='nav')
        line = cmdcfg.effective('real-route-nav', extra={'waypoints': _wp_goal_yaml(pts)})
        steps[0]['lines'].append('$ ' + line)
        if not _run_step(steps[0], line, NAV_TIMEOUT_S * max(1, len(pts))):
            return
        if dock_kind == 'none':
            return
        if auto_dock:
            steps[0]['progress'] = '도착 — 자동 도킹(시작 시 승인됨)'
            if _RETURN['cancel']:
                steps[1]['state'] = 'canceled'
                return
        else:
            steps[0]['progress'] = '도착 — 도킹 확인 대기'
            if not _await_dock_confirm():
                steps[1]['state'] = 'canceled'
                return
        steps[1]['state'] = 'running'
        _bump(phase='dock')
        key = DOCK_KIND[dock_kind][0]
        line = cmdcfg.effective(key, extra={'task_point_id': pts[-1]['task_point_id']})
        steps[1]['lines'].append('$ ' + line)
        _run_step(steps[1], line, DOCK_TIMEOUT_S)
    except Exception as e:
        _bump(error='경로 주행 중 오류: %s' % e)
    finally:
        _bump(active=False, phase='done', await_dock=False)


def _cancel_all():
    """액션 goal 취소 — 이게 유일하게 '확실한' 정지 수단이다.
    CancelGoal 규약상 goal_id 가 전부 0 이면 그 서버의 **모든 goal** 을 취소한다."""
    rid = robot_id()
    blank = ("'{goal_info: {goal_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}, "
             "stamp: {sec: 0, nanosec: 0}}}'")
    out = []
    for act in ('/navigate_to_pose', '/%s/navigate' % rid,
                '/%s/reflective_dock' % rid, '/ddago/reflective_dock'):
        cmd = ('timeout 8 ros2 service call %s/_action/cancel_goal '
               'action_msgs/srv/CancelGoal %s' % (act, blank))
        try:
            r = subprocess.run(['bash', DASH, 'run-cmdline', 'cancel', cmd],
                               capture_output=True, text=True, timeout=20)
            txt = (r.stdout + r.stderr)
            m = re.search(r'return_code=(\d+)', txt)
            out.append('%s → %s' % (act, ('return_code=%s' % m.group(1)) if m
                                    else ('응답 없음' if 'timeout' not in txt else '시간 초과')))
        except (OSError, subprocess.SubprocessError) as e:
            out.append('%s → 호출 실패: %s' % (act, e))
    return out


def _zero_cmd_vel():
    """마지막 뒷정리. bringup 에 cmd_vel 워치독이 없어 **마지막 Twist 가 영원히 유지**되므로
    발행자를 멈춘 뒤 0 을 한 번 박아 준다. 단독으로는 정지 수단이 아니다 —
    Nav2 나 도킹 FSM 이 아직 20Hz 로 쏘고 있으면 즉시 덮어써진다."""
    cmd = ("timeout 6 ros2 topic pub --times 5 -r 10 /cmd_vel "
           "geometry_msgs/msg/Twist '{}'")
    try:
        subprocess.run(['bash', DASH, 'run-cmdline', 'zerovel', cmd],
                       capture_output=True, text=True, timeout=15)
        return 'cmd_vel 0 발행'
    except (OSError, subprocess.SubprocessError) as e:
        return 'cmd_vel 0 실패: %s' % e


def return_stop():
    """조건 없이 3계층을 순서대로. 복귀가 안 돌고 있어도 실행한다
    (이전 goal 이 아직 살아 있을 수 있다)."""
    _bump(cancel=True)
    lines = ['[1] 액션 goal 취소 요청']
    lines += ['    ' + x for x in _cancel_all()]

    lines.append('[2] 실행 중이던 CLI 종료 (⚠ 클라이언트 종료는 정지가 아니다 — '
                 '서버측 goal 은 계속 돈다)')
    import signal
    with _RETURN_LOCK:
        procs = list(_RETURN['procs'])
    for p in procs:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            if p.poll() is not None:
                break
            try:
                os.killpg(os.getpgid(p.pid), sig)
            except OSError:
                break
            time.sleep(0.7)
        lines.append('    pid %d → %s' % (p.pid, '종료' if p.poll() is not None else '남음'))

    lines.append('[3] ' + _zero_cmd_vel())
    time.sleep(2.0)
    lines.append('[확인] 로봇이 실제로 멈췄는지는 맵의 위치 갱신으로 확인하세요 — '
                 '취소했다는 사실만으로 정지를 단정하지 않습니다.')
    _bump(active=False, phase='stopped')
    return {'ok': True, 'lines': lines}


def pose_tf():
    """map→base_footprint TF 로 로봇 위치를 읽는다. **위치의 정본은 이쪽이다.**

    ⚠️ 나이 판정에 /amcl_pose 를 쓰면 안 된다 — amcl 은 update_min_d(10cm)·update_min_a
       를 넘겨 움직였을 때만 발행한다. 그래서 **서 있는 로봇에서는 그 값이 무한정 낡는다**
       (08-06: '위치가 오래됐습니다(216초 전) — 갱신을 기다리세요' 로 복귀 시작이 막혔다.
       그런데 움직이지 않는 한 영원히 갱신되지 않으니, 안내가 실행 불가능한 조건을 요구했다).
       TF 는 amcl 이 갱신하지 않아도 계속 브로드캐스트되고, Nav2 의 costmap·planner 도
       위치를 TF 로 본다 → 판정 근거를 Nav2 와 같은 것으로 맞춘다.

    tf2_echo 는 1초 주기로 계속 찍으므로 잠깐 돌리고 **마지막 블록**을 쓴다.
    """
    cmd = 'timeout 6 ros2 run tf2_ros tf2_echo map base_footprint'
    try:
        r = subprocess.run(['bash', DASH, 'run-cmdline', 'posetf', cmd],
                           capture_output=True, text=True, timeout=25)
    except (OSError, subprocess.SubprocessError) as e:
        return {'ok': False, 'error': 'TF 조회 실행 실패: %s' % e}
    txt = r.stdout + r.stderr
    for blk in reversed(txt.split('At time ')[1:]):
        try:
            stamp = float(blk.split('\n', 1)[0].strip())
        except (ValueError, IndexError):
            continue
        tr = re.search(r'Translation:\s*\[\s*(-?[\d.eE+]+),\s*(-?[\d.eE+]+)', blk)
        yw = re.search(r'RPY \(radian\)\s*\[\s*-?[\d.eE+]+,\s*-?[\d.eE+]+,\s*(-?[\d.eE+]+)\]', blk)
        if tr and yw:
            return {'ok': True, 'source': 'tf', 'frame_id': 'map',
                    'x': float(tr.group(1)), 'y': float(tr.group(2)), 'yaw': float(yw.group(1)),
                    'stamp_sec': stamp, 'age_s': round(time.time() - stamp, 1)}
    return {'ok': False,
            'error': 'map→base_footprint TF 를 읽지 못했습니다 — Nav2(amcl)와 로봇 bringup 이 '
                     '떠 있고 초기 위치(2D Pose Estimate)가 잡혔는지 확인하세요',
            'lines': txt.splitlines()[-15:]}


def pose_once():
    """/amcl_pose 를 1회 읽는다. rosbridge 없이도 위치를 볼 수 있어야 하고,
    amcl 은 로봇이 10cm 이상 움직여야 발행하므로(update_min_d) 서 있는 로봇에서는
    **latched 마지막 값**을 꺼내야 한다 → QoS 를 transient_local 로 명시한다.
    ros2 CLI 기동이 2~3초라 폴링용이 아니다(모달 열 때 1회)."""
    cmd = ('timeout 8 ros2 topic echo --once --qos-durability transient_local '
           '--qos-reliability reliable --qos-depth 1 /amcl_pose '
           'geometry_msgs/msg/PoseWithCovarianceStamped')
    try:
        r = subprocess.run(['bash', DASH, 'run-cmdline', 'poseonce', cmd],
                           capture_output=True, text=True, timeout=25)
    except (OSError, subprocess.SubprocessError) as e:
        return {'ok': False, 'error': '실행 실패: %s' % e}
    txt = r.stdout + r.stderr

    def _f(pat):
        m = re.search(pat, txt)
        return float(m.group(1)) if m else None

    pos = re.search(r'position:\s*\n\s*x:\s*(-?[\d.eE+]+)\s*\n\s*y:\s*(-?[\d.eE+]+)', txt)
    ori = re.search(r'orientation:\s*\n\s*x:\s*(-?[\d.eE+]+)\s*\n\s*y:\s*(-?[\d.eE+]+)'
                    r'\s*\n\s*z:\s*(-?[\d.eE+]+)\s*\n\s*w:\s*(-?[\d.eE+]+)', txt)
    if not pos or not ori:
        return {'ok': False, 'error': '/amcl_pose 를 읽지 못했습니다 — Nav2(amcl)가 떠 있는지 확인하세요',
                'lines': txt.splitlines()[-15:]}
    qx, qy, qz, qw = (float(ori.group(i)) for i in (1, 2, 3, 4))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    sec = _f(r'stamp:\s*\n\s*sec:\s*(\d+)')
    return {'ok': True, 'source': 'once', 'x': float(pos.group(1)), 'y': float(pos.group(2)),
            'yaw': yaw, 'frame_id': 'map', 'stamp_sec': sec,
            'age_s': round(time.time() - sec, 1) if sec else None}


def logs_index():
    """통합 로그 뷰어가 고를 수 있는 목록. 서비스 로그 + 로봇 에이전트 자체 로그."""
    out = []
    for c in cmdcfg.describe_all('real'):
        out.append({'key': c['key'], 'label': c['label'],
                    'where': first_robot() if is_robot(c['key']) else 'local'})
    return out


class Handler(BaseHTTPRequestHandler):
    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        n = int(self.headers.get('Content-Length', 0) or 0)
        if not n:
            return {}
        try:
            return json.loads(self.rfile.read(n).decode())
        except ValueError:
            return {}

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == '/api/status':
            return self._json(snapshot())
        if u.path == '/api/system':
            host = (q.get('host', [''])[0]) or first_robot()
            # 배터리는 여기 없다 — /battery/percent·voltage 토픽이 정본이고,
            # 화면이 rosbridge 로 직접 구독한다(중계를 거칠수록 값이 늦고 갈린다).
            return self._json(agent_call(host, '/system', timeout=10))
        if u.path == '/api/commands':
            return self._json(cmdcfg.describe_all('real'))
        if u.path == '/api/map':
            m = load_map()
            if not m.get('ok'):
                return self._json(m)
            out = {k: v for k, v in m.items() if k != 'data'}
            out['data'] = base64.b64encode(m['data']).decode()
            out['initial_pose'] = nav2_initial_pose()   # amcl 초기값(오인 경고용)
            return self._json(out)
        if u.path == '/api/home':
            rid = robot_id()
            m = load_map()
            home = read_home().get(rid)
            out = {'robot_id': rid, 'home': home, 'map_ok': bool(m.get('ok')),
                   'map': map_fingerprint(m)}
            if home:
                out['target'] = nav_target(home)
                out['map_changed'] = bool(
                    home.get('map') and m.get('ok')
                    and (home['map'].get('origin') != m['origin']
                         or home['map'].get('resolution') != m['resolution']
                         or home['map'].get('size') != [m['width'], m['height']]
                         or home['map'].get('mtime') != m['mtime']))
            return self._json(out)
        if u.path == '/api/pose/once':
            # TF 를 먼저 본다 — 서 있는 로봇에서도 항상 최신이다(/amcl_pose 는 낡는다).
            #  TF 가 없을 때만 latched /amcl_pose 로 폴백.
            p = pose_tf()
            return self._json(p if p.get('ok') else pose_once())
        if u.path == '/api/waypoints':
            return self._json({'ok': True, 'robot_id': robot_id(), 'points': wp_list()})
        if u.path == '/api/return/status':
            # 화면이 보고 싶은 줄 수를 정한다(기본 500). 서버 버퍼 상한까지만.
            return self._json(return_status(q.get('lines', [''])[0] or RETURN_LOG_DEFAULT))
        if u.path == '/api/wire':
            try:
                limit = max(1, min(5000, int(q.get('limit', ['500'])[0])))
            except ValueError:
                limit = 500
            return self._json(read_wire(limit))
        if u.path == '/api/logs/index':
            return self._json(logs_index())
        parts = u.path.strip('/').split('/')
        if len(parts) == 4 and parts[:2] == ['api', 'svc'] and parts[3] == 'log':
            if parts[2] not in cmdcfg.DEFAULTS:
                return self._json({'error': 'unknown'}, 404)
            n = int(q.get('n', ['400'])[0] or 400)
            return self._json(svc_log(parts[2], n))
        return self._static(u.path)

    def do_POST(self):
        path = urlparse(self.path).path
        if path == '/api/home':
            b = self._body()
            try:
                x, y, yaw = float(b['x']), float(b['y']), float(b['yaw'])
            except (KeyError, TypeError, ValueError):
                return self._json({'ok': False, 'error': '좌표(x·y·yaw)가 필요합니다'}, 400)
            m = load_map()
            if not m.get('ok'):
                return self._json({'ok': False, 'error': '맵을 읽지 못해 저장할 수 없습니다'}, 400)
            if occupancy_at(m, x, y) is None:
                return self._json({'ok': False, 'error': '맵 범위 밖 좌표입니다'}, 400)
            rid = robot_id()
            data = read_home()
            data[rid] = {
                'x': x, 'y': y, 'yaw': yaw, 'frame_id': b.get('frame_id') or 'map',
                'task_point_id': b.get('task_point_id') or 'CHARGE_01',
                'saved_at': time.time(), 'source': b.get('source') or 'amcl_pose',
                'cov': b.get('cov') or {},
                # 맵 지문을 함께 박는다 — 맵을 다시 뜨면 저장 좌표가 조용히 딴 곳을 가리킨다
                'map': map_fingerprint(m),
            }
            write_home(data)
            return self._json({'ok': True, 'home': data[rid],
                               'target': nav_target(data[rid])})
        if path == '/api/home/clear':
            rid = robot_id()
            data = read_home()
            data.pop(rid, None)
            write_home(data)
            return self._json({'ok': True})
        if path == '/api/return/start':
            res = return_start(self._body())
            return self._json(res, res.get('code', 200 if res.get('ok') else 400))
        if path == '/api/waypoints':
            return self._json(wp_save((self._body() or {}).get('points')))
        if path == '/api/route/start':
            r = route_start(self._body() or {})
            return self._json(r, r.get('code', 200 if r.get('ok') else 400))
        if path == '/api/return/confirm':
            return self._json(return_confirm_dock())
        if path == '/api/return/stop':
            return self._json(return_stop())
        if path == '/api/agent/restart':
            return self._json(agent_restart())
        if path == '/api/wire/clear':
            clear_wire()
            return self._json({'ok': True})
        parts = path.strip('/').split('/')
        if len(parts) == 3 and parts[:2] == ['api', 'stack']:
            if parts[2] == 'up':
                return self._json(stack_up())
            if parts[2] == 'down':
                return self._json(stack_down())
            return self._json({'error': 'bad action'}, 400)
        if len(parts) >= 3 and parts[:2] == ['api', 'commands']:
            key = parts[2]
            if key not in cmdcfg.DEFAULTS or cmdcfg.DEFAULTS[key]['profile'] != 'real':
                return self._json({'error': '실서버 명령이 아님: %s' % key}, 404)
            if len(parts) == 4 and parts[3] == 'reset':
                return self._json(cmdcfg.reset(key))
            b = self._body()
            return self._json(cmdcfg.set_command(
                key, params=b.get('params'),
                cmdline=b.get('cmdline') if 'cmdline' in b else None))
        if len(parts) == 4 and parts[:2] == ['api', 'svc']:
            key, act = parts[2], parts[3]
            if key not in cmdcfg.DEFAULTS or cmdcfg.DEFAULTS[key]['profile'] != 'real':
                return self._json({'error': '실서버 명령이 아님: %s' % key}, 404)
            if act == 'start':
                return self._json(svc_start(key))
            if act == 'stop':
                return self._json(svc_stop(key))
        return self._json({'error': 'bad request'}, 400)

    def _static(self, path):
        if path in ('/', '/real', '/real/'):
            path = '/real.html'
        fp = os.path.normpath(os.path.join(WEB_DIR, path.lstrip('/')))
        if not fp.startswith(WEB_DIR) or not os.path.isfile(fp):
            self.send_error(404)
            return
        with open(fp, 'rb') as f:
            data = f.read()
        self.send_response(200)
        self.send_header('Content-Type',
                         'text/html; charset=utf-8' if fp.endswith('.html')
                         else 'application/octet-stream')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store, must-revalidate')
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *a):
        pass


if __name__ == '__main__':
    _lock = single_instance('real_server')   # 살려 둬야 락이 유지된다
    print('[dg_web real_server] http://127.0.0.1:%d  (실서버 전용)' % PORT)
    try:
        ThreadingHTTPServer(('127.0.0.1', PORT), Handler).serve_forever()
    except OSError as e:
        raise SystemExit('[real_server] 포트 %d 를 열 수 없습니다: %s' % (PORT, e))

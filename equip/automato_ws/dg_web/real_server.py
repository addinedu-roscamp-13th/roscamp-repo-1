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
    GET  /api/pose/once             → /amcl_pose 를 1회 읽음(rosbridge 폴백)
    POST /api/return/start          → 2단계 복귀 시작(주행 → 도킹)
    GET  /api/return/status         → 진행 상황 폴링
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
                      read_home, read_wire, robot_id, single_instance,
                      tail_bytes, write_home)

PORT = int(os.environ.get('DG_REAL_PORT', '8010'))

# 에이전트 설치·기동만은 **SSH 로** 한다. 에이전트가 곧 통신 채널이라, 그게 죽어 있으면
# 에이전트를 통해 살릴 수 없다(로봇을 재부팅하면 늘 이 상황이 된다).
REALBOARD = os.path.join(os.path.dirname(WEB_DIR), 'realboard.sh')

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
NAV_TIMEOUT_S, DOCK_TIMEOUT_S = 240.0, 180.0
POSE_MAX_AGE_S = 15.0          # 이보다 오래된 위치로는 주행을 시작하지 않는다
NAV2_PARAMS = os.path.expanduser(
    '~/dev_ws/pinky_nav_ws/src/pinky_navigation/params/nav2_params.yaml')

_RETURN = {'active': False, 'run_id': 0, 'phase': '', 'seq': 0, 'steps': [],
           'target': None, 'error': None, 'cancel': False, 'procs': []}
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


def svc_start(key):
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


def battery_from_telemetry(max_age=40.0):
    """배터리의 **정본은 ROS 토픽** /battery/percent·/battery/voltage 다.
    그 값은 이미 여기까지 와 있다 — battery_publisher(5초 주기) → telemetry_publisher 가
    DdagoTelemetry 에 실어 DCS 로 보내고, DCS 가 @@WIRE@@ 로 로그에 남긴다.
    새로 구독할 필요 없이 그 마지막 값을 읽는다(구독자를 하나 더 붙이면 DDS 만 더 시끄러워진다).

    단 이 경로는 bringup+DCS 가 떠 있어야 산다. 스택이 내려가 있으면 값이 없고,
    그때는 에이전트가 I2C 를 직접 읽은 값을 쓴다(/system 의 battery)."""
    for ln in reversed(tail_bytes('/tmp/dash_dcs.log', 262144)):
        i = ln.find('@@WIRE@@ ')
        if i < 0:
            continue
        try:
            rec = json.loads(ln[i + 9:])
        except ValueError:
            continue
        if rec.get('iface') != 'DdagoTelemetry':
            continue
        pl = rec.get('payload') or {}
        if 'battery_percent' not in pl:
            continue
        age = time.time() - (rec.get('ts') or 0)
        if age > max_age:
            return None      # 낡은 값을 지금 값인 양 보여주면 안 된다
        return {'source': 'topic', 'percent': pl.get('battery_percent'),
                'voltage': pl.get('battery_voltage'), 'age_s': round(age, 1)}
    return None


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
    lines.append('— 확인: ' + ('에이전트 응답 OK (%s)' % h.get('host') if ok else h.get('error')))
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


def approach_point(home):
    """저장된 충전소 pose 는 보통 **도킹된 상태**에서 찍은 것이다. 반사테이프 도킹은
    후진 접붙이기라 그 자리를 목표로 주면 충전소에 처박는다. 헤딩 방향으로
    approach_offset_m 만큼 앞을 목표로 삼는다(도킹 FSM 이 나머지 정렬을 한다)."""
    d = float(home.get('approach_offset_m') or 0.0)
    yaw = float(home['yaw'])
    return {'x': float(home['x']) + d * math.cos(yaw),
            'y': float(home['y']) + d * math.sin(yaw), 'yaw': yaw}


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
        return {'ok': False, 'error': '로봇 위치가 오래됐습니다(%.0f초 전) — 갱신을 기다리세요' % age}

    for k in RETURN_KEYS:
        if cmdcfg.cmdline_of(k):
            return {'ok': False, 'error': "'%s' 의 명령줄 전체가 편집돼 있어 좌표 주입이 "
                                          "무시됩니다 — 설정에서 기본값으로 되돌리세요" % k}

    tgt = approach_point(home)
    occ = occupancy_at(m, tgt['x'], tgt['y'])
    if occ is None:
        return {'ok': False, 'error': '목표점(%.2f, %.2f)이 맵 밖입니다 — '
                                      'approach_offset_m 을 줄이세요' % (tgt['x'], tgt['y'])}
    if occ['state'] == 'occupied':
        return {'ok': False, 'error': '목표점이 점유 셀(벽)입니다 — approach_offset_m 을 줄이세요'}

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
            del step['lines'][:-400]
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


def _return_worker(home, target):
    """① 주행 → ② 도킹. **앞 단계가 실패하면 즉시 멈춘다.**
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
            step['state'] = 'running'
            _bump(phase=phase)
            line = cmdcfg.effective(step['key'], extra=extra)
            step['lines'].append('$ ' + line)
            if not _run_step(step, line, timeout):
                break
    except Exception as e:
        _bump(error='복귀 중 오류: %s' % e)
    finally:
        _bump(active=False, phase='done')


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


def return_status():
    with _RETURN_LOCK:
        steps = [dict(st, lines=st['lines'][-60:]) for st in _RETURN['steps']]
        out = {k: _RETURN[k] for k in ('active', 'run_id', 'phase', 'seq', 'target', 'error')}
    out['steps'] = steps
    # Nav2 는 백그라운드로 돌며 자기 로그에 쌓는다 — goal 을 거절한 진짜 이유는
    # CLI 출력이 아니라 거기 있다.
    out['extra'] = extra_logs('real-nav2', 120)
    return out


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
            d = agent_call(host, '/system', timeout=10)
            # 배터리는 ROS 토픽 값(정본)을 우선한다. 스택이 내려가 있어 그 값이 없을 때만
            # 에이전트가 I2C 로 직접 읽은 값을 쓴다.
            if isinstance(d, dict) and not d.get('error'):
                tb = battery_from_telemetry()
                if tb:
                    d['battery'] = tb
            return self._json(d)
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
                out['target'] = approach_point(home)
                out['map_changed'] = bool(
                    home.get('map') and m.get('ok')
                    and (home['map'].get('origin') != m['origin']
                         or home['map'].get('resolution') != m['resolution']
                         or home['map'].get('size') != [m['width'], m['height']]
                         or home['map'].get('mtime') != m['mtime']))
            return self._json(out)
        if u.path == '/api/pose/once':
            return self._json(pose_once())
        if u.path == '/api/return/status':
            return self._json(return_status())
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
                'approach_offset_m': float(b.get('approach_offset_m') or 0.30),
                'saved_at': time.time(), 'source': b.get('source') or 'amcl_pose',
                'cov': b.get('cov') or {},
                # 맵 지문을 함께 박는다 — 맵을 다시 뜨면 저장 좌표가 조용히 딴 곳을 가리킨다
                'map': map_fingerprint(m),
            }
            write_home(data)
            return self._json({'ok': True, 'home': data[rid],
                               'target': approach_point(data[rid])})
        if path == '/api/home/clear':
            rid = robot_id()
            data = read_home()
            data.pop(rid, None)
            write_home(data)
            return self._json({'ok': True})
        if path == '/api/return/start':
            res = return_start(self._body())
            return self._json(res, res.get('code', 200 if res.get('ok') else 400))
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

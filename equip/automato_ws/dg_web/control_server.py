#!/usr/bin/env python3
"""dg_web — DG Control Service(DCS) 테스트용 제어 대시보드 서버.

- 정적 파일(index.html) 제공
- REST API 로 노드 상태 조회 및 start/stop (dashboard.sh 호출)
    GET  /api/status                 → {nodes:{dcs,acs,...}, ai_target:{real,sim,active}}
    POST /api/node/<name>/<action>   → dashboard.sh <action> <name>  (action: start|stop)
- DG AI Service TCP 접속 대상(실서버/시뮬 IP) 저장·조회
    GET  /api/ai-target              → {real, sim, active}
    POST /api/ai-target  {real,sim}  → 저장(active 유지)
- 실행 명령 조회·편집 (정의는 cmdcfg.py, 편집분은 commands.local.json)
    GET  /api/commands                       → [{key,label,params,cmdline,effective,...}]
    POST /api/commands/<key> {params,cmdline}→ 저장 후 그 항목 반환
    POST /api/commands/<key>/reset           → 기본값 복귀
- E0/E1/E2 항목별 테스트 실행·판정 (PASS/FAIL + 근거 로그)
    POST /api/test/e0                → 상시 모니터링(FleetTelemetry 취합) 판정
    POST /api/test/e1                → 순찰 시작(Navigate 경로 접수→DdaGo 하달) 판정
    POST /api/test/e2                → 체크·저장(capture 노드 분석→SaveDetection→순찰완료) 판정
    POST /api/test/e4                → 복귀·도킹(Dock 중계 ACS→DCS→DdaGo) 판정
    GET  /api/logs                   → 최근 흐름 로그 추림(참고용)

dg_ai 시뮬을 껐다 켜면 dashboard.sh 가 active 를 real/sim 으로 자동 전환하고,
DCS 는 dg_ai_target.json 의 active 를 읽어 해당 엔드포인트로 자동 재접속한다.

포트 8000, localhost 전용. rosbridge(9090) 와 별개.
"""
import json
import os
import re
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from urllib.parse import quote

import cmdcfg   # 실행 명령 정의·편집 (dashboard.sh 도 같은 것을 읽는다)
from dgcommon import (DASH, WEB_DIR, agent_call, first_robot, is_up, robot_id,
                      single_instance, tail_bytes as _tail_bytes)

TARGET = os.path.join(WEB_DIR, 'dg_ai_target.json')

# 제어 가능한 컴포넌트: 이름 → (검사종류, 대상)
CHECKS = {
    'dcs':       ('proc', 'dg_control/lib/dg_control/dcs_node'),
    'acs':       ('proc', 'dg_sim/lib/dg_sim/acs_sim'),
    'ddago':     ('proc', 'dg_sim/lib/dg_sim/ddago_sim'),
    'ddagi':     ('proc', 'dg_sim/lib/dg_sim/ddagi_sim'),
    'dg_ai':     ('port', '9100'),
    'rosbridge': ('port', '9090'),
    'web':       ('port', '8000'),   # 이 서버 자신(상태 표시용). start/stop 버튼은 없음.
}
SIMS = ('acs', 'ddago', 'ddagi', 'dg_ai')   # 시뮬 4종

# 로봇 식별자: dashboard.sh 와 같은 출처(환경변수 ROBOT_ID, ~/.bashrc)를 읽는다.
# 노드들의 토픽/액션 이름(/{robot_id}/...)이 이 값으로 만들어지므로 화면에도 그대로 보여준다.


def status():
    return {k: ('up' if is_up(*v) else 'down') for k, v in CHECKS.items()}


def read_target():
    try:
        with open(TARGET, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return {'real': '', 'sim': '127.0.0.1:9100', 'active': 'sim'}


def write_target(data):
    with open(TARGET, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def _tail_filtered(path, keywords, n=12):
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            lines = [ln.rstrip('\n') for ln in f if any(k in ln for k in keywords)]
        return lines[-n:]
    except OSError:
        return []


_LOG_TS_RE = re.compile(r'\[(\d+\.\d+)\]')


def _log_ts(line):
    """ROS 로그 라인의 [<epoch>] 타임스탬프(초)를 추출. 없으면 None."""
    m = _LOG_TS_RE.search(line)
    return float(m.group(1)) if m else None


def _tail_since(path, keywords, since, n=12):
    """_tail_filtered 와 같되, ROS 로그 타임스탬프가 since(초) 이후인 INFO 라인만 반환한다.

    /tmp/dash_*.log 는 실행 사이에 초기화되지 않으므로, 단순 keyword 매칭은 **이전 실행이
    남긴 낡은 로그**(예: 지난 도킹의 'E3 진입 가능')를 잡아 게이트/판정이 오작동한다.
    각 eval 이 시작 시각(t0)을 잡아 이 함수로 그 이후 라인만 보게 하면, 반복 실행에도
    이번 실행의 로그만 판정에 쓴다. 타임스탬프 없는 라인(@@WIRE@@ 등)은 제외한다."""
    out = []
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            for ln in f:
                if not any(k in ln for k in keywords):
                    continue
                ts = _log_ts(ln)
                if ts is None or ts < since:
                    continue
                out.append(ln.rstrip('\n'))
    except OSError:
        return []
    return out[-n:]




WIRE_SINCE_FILE = '/tmp/dash_wire_since'   # clear 기준 시각을 파일에 저장(서버 재시작에도 유지)


def clear_wire():
    """메시지 시계열 지우기: 기준 시각을 파일에 기록 → 이후 메시지만 표시.
    파일 백업이라 web 서버가 재시작돼도, 새 브라우저로 접속해도 지운 상태가 유지된다."""
    try:
        with open(WIRE_SINCE_FILE, 'w') as f:
            f.write(repr(time.time()))
    except OSError:
        pass


def _wire_since():
    """clear 기준 시각(파일). 없으면 0.0. read_wire 가 매 요청마다 읽어 필터에 사용."""
    try:
        with open(WIRE_SINCE_FILE) as f:
            return float(f.read().strip() or 0)
    except (OSError, ValueError):
        return 0.0


WIRE_MAX = 500   # 시계열 보관 기본 개수(대시보드에서 조정 가능)


def read_wire(limit=WIRE_MAX):
    """DCS 로그의 @@WIRE@@ 라인을 파싱해 DCS 시점의 '전체 시계열'을 반환.
    반환: [ {ts, dir, iface, payload}, ... ]  (WIRE_SINCE 이후, 시각 오름차순, 최근 limit건)."""
    marker = '@@WIRE@@ '
    since = _wire_since()   # 파일에 저장된 clear 기준 시각
    out = []
    for ln in _tail_bytes('/tmp/dash_dcs.log', 1048576):
        i = ln.find(marker)
        if i < 0:
            continue
        try:
            rec = json.loads(ln[i + len(marker):])
        except ValueError:
            continue
        if rec.get('ts', 0) < since:
            continue
        if not rec.get('iface'):
            continue
        # text: 파이썬이 만든 payload JSON 문자열을 그대로 실어 보낸다.
        # 브라우저에서 JSON.stringify 로 다시 만들면 0.0 → 0 처럼 float 의 소수점이 사라진다.
        payload = rec.get('payload')
        out.append({'ts': rec.get('ts'), 'dir': rec.get('dir'),
                    'iface': rec.get('iface'), 'payload': payload,
                    'text': json.dumps(payload, ensure_ascii=False)})
    out.sort(key=lambda r: r.get('ts') or 0)
    return out[-limit:]


def read_logs():
    return {
        'dcs': _tail_filtered('/tmp/dash_dcs.log',
                              ['경로 수신', 'DdaGo 하달', '분석결과', '구간 결과 전달',
                               '도킹 결과 전달', 'E3 진입 가능', '수확 시작 수신',
                               'Ddagi 수확 하달', '수확 결과 전달']),
        'acs': _tail_filtered('/tmp/dash_acs.log',
                              ['순찰 시작', '구간 하달', 'SaveDetection 저장', '구간 결과',
                               '순찰 완료', 'Fleet 수신', '수확 시작 하달', '수확 진행',
                               '수확 결과']),
    }


def read_exec_log(key, n=400):
    """단발 명령의 마지막 실행 출력(stdout+stderr). dashboard.sh 의 run_cmd 가 남긴다."""
    lines = _tail_bytes('/tmp/dash_cmd_%s.log' % key, 262144)
    return lines[-n:] if lines else []


def read_node_log(name, n=400):
    """노드 프로세스의 stdout+stderr 원문 꼬리. start_one 이 리다이렉트해 둔 파일을 읽는다.
    /api/logs 는 흐름만 추려 주는 반면 이쪽은 걸러내지 않은 원문이라, 노드가 뜨다 죽었을 때
    (import 실패·파라미터 오류 등) 이유가 여기에만 남는다."""
    tag = {'dg_ai': 'ai', 'rosbridge': 'rb', 'web': 'http'}.get(name, name)
    lines = _tail_bytes('/tmp/dash_%s.log' % tag, 262144)
    # 시계열 표시용 @@WIRE@@ 는 사람이 읽을 로그가 아니라 뺀다(메시지 로그 패널이 따로 본다).
    return [ln for ln in lines if '@@WIRE@@' not in ln][-n:]


_GUARD_CACHE = {'t': 0.0, 'up': False}


def robot_ddago_up(ttl=5.0):
    """실장비 DdaGo(로봇 온보드)가 떠 있는지 로봇 에이전트에 물어본다.

    시뮬 서버는 실서버 화면과 완전히 분리돼 있지만, **이 판정 하나만은 남겨 둔다** —
    실장비가 붙은 채 같은 이름의 시뮬 액션 서버를 띄우면 goal 이 어디로 갈지 알 수 없다.
    로봇이 안 잡히면 False(=막지 않음)로 둔다. 로봇이 꺼져 있을 때까지 시뮬을 못 켜면
    개발이 막히기 때문이다. 짧게 캐시해 버튼마다 왕복하지 않는다."""
    now = time.time()
    if now - _GUARD_CACHE['t'] < ttl:
        return _GUARD_CACHE['up']
    chk = cmdcfg.DEFAULTS.get('robot-ddago', {}).get('check', '')
    r = agent_call(first_robot(), '/procs?match=' + quote(chk or '.'), timeout=3)
    up = bool(not r.get('error') and r.get('procs'))
    _GUARD_CACHE['t'], _GUARD_CACHE['up'] = now, up
    return up


def stack_up(no_sim=False):
    """시뮬 스택 전체 기동 (dashboard.sh up). 화면의 '전체 기동' 버튼이 쓴다."""
    args = ['bash', DASH, 'up'] + (['--no-sim'] if no_sim else [])
    r = subprocess.run(args, capture_output=True, text=True, timeout=120)
    return {'ok': r.returncode == 0, 'lines': (r.stdout + r.stderr).splitlines()}


def stack_down():
    r = subprocess.run(['bash', DASH, 'down'], capture_output=True, text=True, timeout=60)
    return {'ok': r.returncode == 0, 'lines': (r.stdout + r.stderr).splitlines()}


def _trigger_patrol():
    """acs_sim 의 순찰 트리거(dashboard.sh test) 실행."""
    subprocess.run(['bash', DASH, 'test'], capture_output=True, text=True, timeout=40)


def eval_e0():
    """E0 상시 모니터링: 실행 시 시뮬 텔레메트리를 트리거하고,
    DdaGo/Ddagi → DCS → ACS 로 FleetTelemetry(ddago,ddagi 취합)가 흐르는지 확인."""
    clear_wire()   # 실행 시 메시지 초기화
    subprocess.run(['bash', DASH, 'telemetry'], capture_output=True, text=True, timeout=30)

    def done():
        for ln in reversed(_tail_filtered('/tmp/dash_acs.log', ['Fleet 수신'], n=10)):
            m = re.search(r'ddago=(\d+).*ddagi=(\d+)', ln)
            if m and int(m.group(1)) >= 1 and int(m.group(2)) >= 1:
                return True
        return False

    ok = _wait_until(done, 12)
    for ln in reversed(_tail_filtered('/tmp/dash_acs.log', ['Fleet 수신'], n=10)):
        m = re.search(r'ddago=(\d+).*ddagi=(\d+)', ln)
        if m and int(m.group(1)) >= 1 and int(m.group(2)) >= 1:
            return ok, [ln]
    ev = _tail_filtered('/tmp/dash_acs.log', ['Fleet 수신'], n=3)
    return ok, ev or ['(Fleet 수신 없음 — dcs·ddago·ddagi·acs UP 확인)']


def _wait_until(check, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if check():
            return True
        time.sleep(1.0)
    return check()


def eval_e1():
    """E1 순찰 시작: ACS→DCS 로 경로(Waypoint[]) 접수 후 DCS→DdaGo 로 구간 하달까지."""
    clear_wire()   # 실행 시 메시지 초기화
    _trigger_patrol()

    def done():
        dcs = _tail_filtered('/tmp/dash_dcs.log', ['경로 수신', 'DdaGo 하달'], n=8)
        return (any('경로 수신' in l for l in dcs)
                and any('DdaGo 하달' in l for l in dcs))

    ok = _wait_until(done, 10)
    dcs = _tail_filtered('/tmp/dash_dcs.log', ['경로 수신', 'DdaGo 하달'], n=6)
    return ok, dcs or ['(DCS 로그 없음 — dcs UP 및 acs 트리거 확인)']


def eval_e2():
    """E2 체크·저장: capture 노드 분석→SaveDetection 저장→순찰 완료(code=0)까지.
    시뮬 처리 지연(이동·분석 각 3초)으로 완주에 시간이 걸리므로 완료까지 폴링(최대 60초)."""
    clear_wire()   # 실행 시 메시지 초기화
    _trigger_patrol()

    def done():
        dcs = _tail_filtered('/tmp/dash_dcs.log', ['분석결과'], n=5)
        acs = _tail_filtered('/tmp/dash_acs.log', ['순찰 완료', 'SaveDetection 저장'], n=8)
        return (any('분석결과' in l for l in dcs)
                and any('SaveDetection 저장' in l for l in acs)
                and any('순찰 완료' in l and 'code=0' in l for l in acs))

    ok = _wait_until(done, 60)
    dcs = _tail_filtered('/tmp/dash_dcs.log', ['분석결과', '구간 결과 전달'], n=6)
    acs = _tail_filtered('/tmp/dash_acs.log', ['SaveDetection 저장', '순찰 완료'], n=6)
    return ok, dcs[-4:] + acs[-4:]


def _eval_dock(cmd):
    """정밀 도킹 중계 판정 — ACS→DCS 접수 → DCS→DdaGo 하달 → 결과 반환까지.

    DCS 는 중계자다. 실제 도킹 기동(마커 탐색→정렬→접근→회전→후진)은 DdaGo 가 하고,
    여기서 보는 것은 **중계가 값을 잃지 않는가**다. 특히 final_lateral_m(중심선/법선
    이탈)·final_yaw_error(스큐)는 ACS 가 도킹 품질을 판정하는 근거라 빠지면 안 된다.

    방식(charuco/floor/reflective)이 셋이지만 DCS 로그 문구는 같아서(방식=… 만 다름)
    판정 절차를 공유한다. cmd 는 dashboard.sh 서브커맨드(dock/floor-dock/…).
    """
    clear_wire()   # 실행 시 메시지 초기화
    t0 = time.time()   # 이번 실행 기준 시각 — 낡은 로그를 판정에서 배제(_tail_since)
    subprocess.run(['bash', DASH, cmd], capture_output=True, text=True, timeout=90)

    keys = ['도킹 지시 수신', '도킹 하달', '도킹 종료', '도킹 결과 전달']

    def done():
        dcs = _tail_since('/tmp/dash_dcs.log', keys, t0, n=12)
        return (any('도킹 지시 수신' in l for l in dcs)      # ACS → DCS 접수
                and any('도킹 하달' in l for l in dcs)        # DCS → DdaGo 중계
                and any('도킹 종료' in l for l in dcs)        # DdaGo → DCS 결과
                and any('도킹 결과 전달' in l for l in dcs))  # DCS → ACS 반환

    ok = _wait_until(done, 30)
    dcs = _tail_since('/tmp/dash_dcs.log', keys, t0, n=8)
    if ok:
        # 성공이라면 code=0 이어야 한다(중계는 됐는데 도킹이 실패한 경우를 가른다).
        ok = any('code=0' in l for l in dcs if '도킹 결과 전달' in l)
    return ok, dcs or ['(DCS 로그 없음 — dcs/ddago UP 확인)']


def eval_e4():
    """E4 순찰 종료 후 복귀 및 충전 — **복귀 주행 → 충전소 도킹 2단계**.

    1) ACS 가 충전소까지 경로를 전 구간 capture=false 로 하달 → DCS→DdaGo 중계 (촬영 없음)
    2) 도착 후 ReflectiveDock 하달 → DCS→DdaGo 중계 → 결과(뒤끝~마커 갭·법선이탈·스큐) 반환

    도킹만 단발로 찔러 보던 것을 시나리오 전체로 바꿨다. 충전소 마커는 반사테이프이고
    ACS 라우팅(docking.method_for: CHARGE_*→reflective)도 그렇게 정해져 있어, 지점 id 만
    충전소로 주면 방식은 자동으로 정해진다. charuco 는 어느 지점도 쓰지 않는 휴면 방식이라
    버튼에서 뺐다(`dashboard.sh dock` 으로 손수 하달할 수 있고, 중계 자체는 test_e2e 가
    3방식 모두 검증한다).

    판정: 복귀 주행 경로 접수 → 도킹 지시 접수 → 도킹 결과 code=0 이 DCS 로그에 모두 남는지.
    이동 중 촬영이 없어야 하지만(capture=false) 그건 s2e2 와 같은 경로라 여기서 또 보지 않는다.
    """
    clear_wire()   # 실행 시 메시지 초기화
    t0 = time.time()   # 이번 실행 기준 시각 — 낡은 로그를 판정에서 배제(_tail_since)
    subprocess.run(['bash', DASH, 'return-dock'], capture_output=True, text=True, timeout=40)

    keys = ['경로 수신', '도킹 지시 수신', '도킹 종료', '도킹 결과 전달']

    def done():
        dcs = _tail_since('/tmp/dash_dcs.log', keys, t0, n=20)
        return (any('경로 수신' in l for l in dcs)                 # 1) 복귀 주행 접수
                and any('도킹 지시 수신' in l for l in dcs)         # 2) 도킹 접수
                and any('도킹 결과 전달' in l and 'code=0' in l for l in dcs))

    ok = _wait_until(done, 40)
    dcs = _tail_since('/tmp/dash_dcs.log', keys, t0, n=10)
    return ok, dcs or ['(DCS 로그 없음 — dcs·acs·ddago UP 확인)']


def eval_e4_floor():
    """바닥 H 마커 도킹 중계 — 수확지·예냉실(S2 E2·E5)에서 쓰는 방식."""
    return _eval_dock('floor-dock')


def _trigger_harvest_move():
    """ACS 역할로 수확 이동+도킹 하달(dashboard.sh harvest-move). 서비스 호출은 즉시 반환하고
    시나리오는 백그라운드로 돈다(완주는 아래 eval 이 로그로 폴링)."""
    subprocess.run(['bash', DASH, 'harvest-move'], capture_output=True, text=True, timeout=40)


def eval_s2e2():
    """S2 E2 수확 위치 이동+도킹: ACS→DCS 로 수확지점까지 경로(전 구간 capture=false) 접수 →
    DCS→DdaGo 중계 → 도착 후 FloorDock(H마커) 하달·중계 → 도킹 성공 시 E3 진입 게이트 오픈까지.

    순찰(E1·E2)과 달리 이동 중 촬영·분석이 없다(capture=false → 분석 경로 미진입). 판정은
    '경로 수신 → 도킹 지시 수신 → 도킹 결과 전달(code=0) → E3 진입 가능(도킹 성공)' 4단계가
    DCS 로그에 모두 남는지로 한다. 도킹이 실패하면 게이트가 열리지 않아 FAIL 로 갈린다."""
    clear_wire()   # 실행 시 메시지 초기화
    t0 = time.time()   # 이번 실행 기준 시각 — 낡은 로그를 판정에서 배제(_tail_since)
    _trigger_harvest_move()

    open_key = 'E3 진입 가능(도킹 성공'   # 게이트 오픈(성공)만. '해제'(clear)와 구분된다.
    keys = ['경로 수신', '도킹 지시 수신', '도킹 결과 전달', open_key]

    def done():
        dcs = _tail_since('/tmp/dash_dcs.log', keys, t0, n=20)
        return (any('경로 수신' in l for l in dcs)
                and any('도킹 지시 수신' in l for l in dcs)
                and any('도킹 결과 전달' in l and 'code=0' in l for l in dcs)
                and any(open_key in l for l in dcs))

    ok = _wait_until(done, 40)
    dcs = _tail_since('/tmp/dash_dcs.log', keys, t0, n=10)
    return ok, dcs or ['(DCS 로그 없음 — dcs·acs·ddago UP 확인)']


def _trigger_harvest():
    """ACS 역할로 수확 시작(E3) 하달(dashboard.sh harvest). 도킹 성공한 task 로
    Harvest 를 하달한다. 즉시 반환하고 수확 진행은 백그라운드(로그 폴링으로 판정)."""
    subprocess.run(['bash', DASH, 'harvest'], capture_output=True, text=True, timeout=60)


def eval_s2e3():
    """S2 E3 수확 대상 인식(DG Harvest 중계): 도킹 성공(is_docked)한 task 로 수확을 시작해
    ACS→DCS→Ddagi 로 Harvest 가 중계되고, 라운드 Feedback·종료 사유가 그대로 되돌아오는지.

    수확은 E2 도킹이 선행돼야 하므로(게이트) 이 테스트는 **E2(이동+도킹) → E3(수확)** 를
    이어서 실행한다. 판정: 도킹 성공(E3 진입 가능) → 수확 시작 수신 → Ddagi 수확 하달 →
    수확 결과 전달(exit_reason=DEPLETED/FULL/MAX_ROUNDS_EXCEEDED) 이 DCS 로그에 남는지.
    도킹 안 된 task 로는 goal 이 거부(reject)돼 수확 하달이 없으므로 FAIL 로 갈린다."""
    clear_wire()   # 실행 시 메시지 초기화
    t0 = time.time()   # 이번 실행 기준 시각 — 낡은 로그를 판정에서 배제(_tail_since)

    # 1) E2 이동+도킹으로 게이트를 연다
    _trigger_harvest_move()
    open_key = 'E3 진입 가능(도킹 성공'

    def docked():
        return any(open_key in l for l in
                   _tail_since('/tmp/dash_dcs.log', [open_key], t0, n=5))

    if not _wait_until(docked, 40):
        dcs = _tail_since('/tmp/dash_dcs.log',
                          ['경로 수신', '도킹 결과 전달', open_key], t0, n=8)
        return False, ['(E2 도킹 실패 — E3 는 도킹 성공이 선행돼야 함)'] + dcs

    # 2) E3 수확 시작. 게이트 오픈 로그(DCS)는 ACS 의 _last_docked_task 갱신보다 살짝
    #    앞서므로, 곧바로 발사하면 ACS 가 낡은 task 로 수확을 보낼 수 있다. 짧게 정착 대기.
    time.sleep(1.0)
    t1 = time.time()   # 수확 단계 기준 시각(도킹 단계 로그와도 분리)
    _trigger_harvest()
    exits = ('exit=DEPLETED', 'exit=FULL', 'exit=MAX_ROUNDS_EXCEEDED')
    keys = ['수확 시작 수신', 'Ddagi 수확 하달', 'Ddagi 수확 종료', '수확 결과 전달']

    def done():
        dcs = _tail_since('/tmp/dash_dcs.log', keys, t1, n=20)
        return (any('수확 시작 수신' in l for l in dcs)          # ACS → DCS 접수
                and any('Ddagi 수확 하달' in l for l in dcs)      # DCS → Ddagi 중계
                and any('수확 결과 전달' in l and any(e in l for e in exits)
                        for l in dcs))                            # DCS → ACS 정상 종료 반환

    ok = _wait_until(done, 40)
    dcs = _tail_since('/tmp/dash_dcs.log', keys, t1, n=10)
    return ok, dcs or ['(DCS 로그 없음 — dcs·acs·ddago·ddagi UP 확인)']


def _trigger_unload():
    """ACS 역할로 하역 시작(E6) 하달(dashboard.sh unload). 도킹 성공한 task 로
    Unload 를 하달한다. 즉시 반환하고 하역 진행은 백그라운드(로그 폴링으로 판정)."""
    subprocess.run(['bash', DASH, 'unload'], capture_output=True, text=True, timeout=60)


def eval_s2e6():
    """S2 E6 예냉실 하역(DG Unload 중계): 도킹 성공(is_docked)한 task 로 하역을 시작해
    ACS→DCS→Ddagi 로 Unload 가 중계되고, phase Feedback·result 가 그대로 되돌아오는지.

    하역도 도킹(게이트)이 선행돼야 하므로 이 테스트는 **E2(이동+도킹) → E6(하역)** 를
    이어서 실행한다(E3 없이 도킹만으로 게이트가 열린다). 판정: 도킹 성공(E3 진입 가능) →
    하역 시작 수신 → Ddagi 하역 하달 → 하역 결과 전달(code=0) 이 DCS 로그에 남는지.
    도킹 안 된 task 로는 goal 이 거부(reject)돼 하역 하달이 없으므로 FAIL 로 갈린다."""
    clear_wire()   # 실행 시 메시지 초기화
    t0 = time.time()   # 이번 실행 기준 시각 — 낡은 로그를 판정에서 배제(_tail_since)

    # 1) E2 이동+도킹으로 게이트를 연다(하역은 예냉실 도킹 성공이 선행)
    _trigger_harvest_move()
    open_key = 'E3 진입 가능(도킹 성공'

    def docked():
        return any(open_key in l for l in
                   _tail_since('/tmp/dash_dcs.log', [open_key], t0, n=5))

    if not _wait_until(docked, 40):
        dcs = _tail_since('/tmp/dash_dcs.log',
                          ['경로 수신', '도킹 결과 전달', open_key], t0, n=8)
        return False, ['(도킹 실패 — E6 하역은 도킹 성공이 선행돼야 함)'] + dcs

    # 2) E6 하역 시작. 게이트 오픈 로그(DCS)는 ACS 의 _last_docked_task 갱신보다 살짝
    #    앞서므로, 곧바로 발사하면 ACS 가 낡은 task 로 하역을 보낼 수 있다. 짧게 정착 대기.
    time.sleep(1.0)
    t1 = time.time()   # 하역 단계 기준 시각(도킹 단계 로그와도 분리)
    _trigger_unload()
    keys = ['하역 시작 수신', 'Ddagi 하역 하달', 'Ddagi 하역 종료', '하역 결과 전달']

    def done():
        dcs = _tail_since('/tmp/dash_dcs.log', keys, t1, n=20)
        return (any('하역 시작 수신' in l for l in dcs)          # ACS → DCS 접수
                and any('Ddagi 하역 하달' in l for l in dcs)      # DCS → Ddagi 중계
                and any('하역 결과 전달' in l and 'code=0' in l
                        for l in dcs))                            # DCS → ACS 성공 반환

    ok = _wait_until(done, 40)
    dcs = _tail_since('/tmp/dash_dcs.log', keys, t1, n=10)
    return ok, dcs or ['(DCS 로그 없음 — dcs·acs·ddago·ddagi UP 확인)']


EVALS = {'e0': eval_e0, 'e1': eval_e1, 'e2': eval_e2, 'e4': eval_e4,
         'e4floor': eval_e4_floor,
         's2e2': eval_s2e2, 's2e3': eval_s2e3, 's2e6': eval_s2e6}
EVAL_NAMES = {'e0': 'E0 상시 모니터링', 'e1': 'E1 순찰 시작', 'e2': 'E2 체크·저장',
              'e4': 'E4 순찰 종료 후 복귀 및 충전', 'e4floor': 'H마커 도킹 중계',
              's2e2': 'S2 E2 수확 이동·도킹',
              's2e3': 'S2 E3 수확 대상 인식', 's2e6': 'S2 E6 예냉실 하역'}


class Handler(BaseHTTPRequestHandler):
    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self):
        length = int(self.headers.get('Content-Length', 0))
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode())
        except ValueError:
            return {}

    # 노드로 뜨는 명령(백그라운드·로그는 파일) vs 한 번 돌고 끝나는 명령(출력을 바로 돌려줌)
    NODE_KEYS = ('dcs', 'acs', 'ddago', 'ddagi', 'dg_ai', 'rosbridge')

    def _run_command(self, key):
        """명령 하나를 실행하고 stdout+stderr 를 그대로 돌려준다.
        노드는 백그라운드로 떠서 출력이 파일로 가므로, 잠깐 기다렸다 그 파일을 읽어 준다
        (뜨자마자 죽는 경우가 가장 흔한 실패라 그 흔적이 남아야 한다)."""
        try:
            if key in self.NODE_KEYS:
                subprocess.run(['bash', DASH, 'start', key],
                               capture_output=True, text=True, timeout=30)
                time.sleep(1.5)
                return {'key': key, 'kind': 'node', 'ok': is_up(*CHECKS[key]),
                        'lines': read_node_log(key, 200)}
            r = subprocess.run(['bash', DASH, key],
                               capture_output=True, text=True, timeout=200)
            out = (r.stdout or '') + (r.stderr or '')
            return {'key': key, 'kind': 'oneshot', 'ok': r.returncode == 0,
                    'lines': out.splitlines() or read_exec_log(key, 200)}
        except subprocess.TimeoutExpired:
            # 도킹처럼 오래 걸리는 명령이 상한을 넘긴 경우. 그때까지의 출력은 파일에 남아 있다.
            return {'key': key, 'kind': 'oneshot', 'ok': False,
                    'lines': read_exec_log(key, 200) + ['(대시보드 대기 상한 초과 — 명령은 계속 돌고 있을 수 있음)']}

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/api/status':
            return self._json({'nodes': status(), 'ai_target': read_target(),
                               'robot_id': robot_id()})
        if path == '/api/ai-target':
            return self._json(read_target())
        if path == '/api/logs':
            return self._json(read_logs())
        if path == '/api/commands':
            # 이 서버는 시뮬 전용이다. 실서버 명령은 여기서 아예 보이지 않는다
            # (실장비 조작은 :8010 real_server 가 맡는다).
            return self._json(cmdcfg.describe_all('sim'))
        # /api/commands/<key>/log , /api/node/<name>/log
        parts = path.strip('/').split('/')
        if len(parts) == 4 and parts[0] == 'api' and parts[3] == 'log':
            if parts[1] == 'commands' and parts[2] in cmdcfg.DEFAULTS:
                return self._json({'key': parts[2], 'lines': read_exec_log(parts[2])})
            if parts[1] == 'node' and parts[2] in CHECKS:
                return self._json({'name': parts[2], 'lines': read_node_log(parts[2])})
        if path == '/api/wire':
            limit = WIRE_MAX
            if '?' in self.path:
                for kv in self.path.split('?', 1)[1].split('&'):
                    if kv.startswith('limit='):
                        try:
                            limit = max(1, min(5000, int(kv[6:])))
                        except ValueError:
                            pass
            return self._json(read_wire(limit))
        return self._serve_static()

    def do_POST(self):
        path = self.path.split('?')[0]
        # 메시지 시계열 지우기 (WIRE_SINCE 갱신 → 이후 메시지만 표시)
        if path == '/api/wire/clear':
            clear_wire()
            return self._json({'ok': True})
        # E0 상시 모니터링 중지 (시뮬 텔레메트리 발행 정지)
        if path == '/api/telemetry/stop':
            subprocess.run(['bash', DASH, 'telemetry-stop'], capture_output=True,
                           text=True, timeout=30)
            return self._json({'ok': True})
        # E0/E1/E2 항목별 테스트 실행·판정
        parts = path.strip('/').split('/')
        if len(parts) == 3 and parts[0] == 'api' and parts[1] == 'test' and parts[2] in EVALS:
            key = parts[2]
            ok, evidence = EVALS[key]()
            return self._json({'item': key, 'name': EVAL_NAMES[key],
                               'ok': ok, 'evidence': evidence})
        # 스택 전체 기동·종료 — /api/stack/<up|down>[?no-sim]
        if path == '/api/stack/up':
            return self._json(stack_up('no-sim' in self.path))
        if path == '/api/stack/down':
            return self._json(stack_down())

        # 실행 명령 편집·실행 — /api/commands/<key>[/reset|/run]
        parts = path.strip('/').split('/')
        if len(parts) >= 3 and parts[0] == 'api' and parts[1] == 'commands':
            key = parts[2]
            if key not in cmdcfg.DEFAULTS:
                return self._json({'error': 'unknown command: %s' % key}, 404)
            action = parts[3] if len(parts) == 4 else ''
            if action == 'reset':
                return self._json(cmdcfg.reset(key))
            if action == 'run':
                return self._json(self._run_command(key))
            if action:
                return self._json({'error': 'bad action'}, 400)
            body = self._read_body()
            # cmdline 은 키가 있을 때만 건드린다(빈 문자열 = 전체편집 해제).
            return self._json(cmdcfg.set_command(
                key, params=body.get('params'),
                cmdline=body.get('cmdline') if 'cmdline' in body else None))

        if path == '/api/ai-target':
            body = self._read_body()
            cfg = read_target()
            if 'real' in body:
                cfg['real'] = str(body['real']).strip()
            if 'sim' in body:
                cfg['sim'] = str(body['sim']).strip()
            write_target(cfg)
            return self._json(cfg)

        parts = path.strip('/').split('/')
        # /api/node/<name>/<action>
        if len(parts) == 4 and parts[0] == 'api' and parts[1] == 'node':
            name, action = parts[2], parts[3]
            if name in CHECKS and action in ('start', 'stop'):
                # 실장비가 떠 있는데 같은 역할의 시뮬을 켜면 액션 서버가 둘이 되어
                # goal 이 어디로 갈지 알 수 없다. 사람 기억이 아니라 여기서 막는다.
                if action == 'start' and name in ('ddago', 'ddagi'):
                    if robot_ddago_up():
                        return self._json(
                            {'error': '로봇 DdaGo(실장비)가 떠 있어 시뮬을 켤 수 없습니다. '
                                      '같은 이름의 액션 서버가 둘이 되면 goal 이 엉뚱한 쪽으로 갑니다.',
                             'nodes': status(), 'ai_target': read_target(),
                             'robot_id': robot_id()}, 409)
                subprocess.run(['bash', DASH, action, name], timeout=30)
                return self._json({'name': name, 'action': action,
                                   'nodes': status(), 'ai_target': read_target(),
                                   'robot_id': robot_id()})
        return self._json({'error': 'bad request'}, 400)

    def _serve_static(self):
        path = self.path.split('?')[0]
        if path == '/':
            path = '/index.html'
        fp = os.path.normpath(os.path.join(WEB_DIR, path.lstrip('/')))
        if not fp.startswith(WEB_DIR) or not os.path.isfile(fp):
            self.send_error(404)
            return
        ctype = 'text/html; charset=utf-8' if fp.endswith('.html') else 'application/octet-stream'
        with open(fp, 'rb') as f:
            data = f.read()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data)))
        # 브라우저가 옛 HTML/JS 를 캐시해 대시보드 변경이 안 보이는 문제 방지
        self.send_header('Cache-Control', 'no-store, must-revalidate')
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    _lock = single_instance('control_server')   # 살려 둬야 락이 유지된다
    print('[dg_web control_server] http://127.0.0.1:8000  (정적 + /api)')
    try:
        ThreadingHTTPServer(('127.0.0.1', 8000), Handler).serve_forever()
    except OSError as e:
        raise SystemExit('[control_server] 포트 8000 을 열 수 없습니다: %s' % e)

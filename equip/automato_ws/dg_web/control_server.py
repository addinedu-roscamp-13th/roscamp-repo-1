#!/usr/bin/env python3
"""dg_web — DG Control Service(DCS) 테스트용 제어 대시보드 서버.

- 정적 파일(index.html) 제공
- REST API 로 노드 상태 조회 및 start/stop (dashboard.sh 호출)
    GET  /api/status                 → {nodes:{dcs,acs,...}, ai_target:{real,sim,active}}
    POST /api/node/<name>/<action>   → dashboard.sh <action> <name>  (action: start|stop)
- DG AI Service TCP 접속 대상(실서버/시뮬 IP) 저장·조회
    GET  /api/ai-target              → {real, sim, active}
    POST /api/ai-target  {real,sim}  → 저장(active 유지)
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

WEB_DIR = os.path.dirname(os.path.abspath(__file__))       # .../automato_ws/dg_web
WS_DIR = os.path.dirname(WEB_DIR)                           # .../automato_ws
DASH = os.path.join(WS_DIR, 'dashboard.sh')
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
def robot_id():
    return os.environ.get('ROBOT_ID', 'dg_01')


def is_up(kind, target):
    if kind == 'proc':
        return subprocess.run(['pgrep', '-f', target],
                              stdout=subprocess.DEVNULL).returncode == 0
    out = subprocess.run(['ss', '-ltn'], capture_output=True, text=True).stdout
    return (':' + target + ' ') in out


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


def _tail_bytes(path, nbytes=131072):
    try:
        with open(path, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - nbytes))
            data = f.read()
        return data.decode('utf-8', 'replace').splitlines()
    except OSError:
        return []


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


def _trigger_dock():
    """ACS 역할로 Dock goal 하달(dashboard.sh dock). 완주까지 기다린다."""
    subprocess.run(['bash', DASH, 'dock'], capture_output=True, text=True, timeout=90)


def eval_e4():
    """E4-6 복귀 후 정밀 도킹: ACS→DCS 로 Dock goal 접수 → DCS→DdaGo 중계 →
    결과가 ACS 로 되돌아오는지까지.

    DCS 는 중계자다. 실제 도킹 기동(마커 탐색→중심선 정렬→접근→회전→후진)은 DdaGo 가
    하고, 여기서 보는 것은 **중계가 값을 잃지 않는가**다. 특히 final_lateral_m(중심선
    이탈)·final_yaw_error(스큐)는 ACS 가 도킹 품질을 판정하는 근거라 빠지면 안 된다.
    """
    clear_wire()   # 실행 시 메시지 초기화
    _trigger_dock()

    keys = ['도킹 지시 수신', '도킹 하달', '도킹 종료', '도킹 결과 전달']

    def done():
        dcs = _tail_filtered('/tmp/dash_dcs.log', keys, n=12)
        return (any('도킹 지시 수신' in l for l in dcs)      # ACS → DCS 접수
                and any('도킹 하달' in l for l in dcs)        # DCS → DdaGo 중계
                and any('도킹 종료' in l for l in dcs)        # DdaGo → DCS 결과
                and any('도킹 결과 전달' in l for l in dcs))  # DCS → ACS 반환

    ok = _wait_until(done, 30)
    dcs = _tail_filtered('/tmp/dash_dcs.log', keys, n=8)
    if ok:
        # 성공이라면 code=0 이어야 한다(중계는 됐는데 도킹이 실패한 경우를 가른다).
        ok = any('code=0' in l for l in dcs if '도킹 결과 전달' in l)
    return ok, dcs or ['(DCS 로그 없음 — dcs/ddago UP 확인)']


def _trigger_harvest_move():
    """ACS 역할로 수확 이동+도킹 하달(dashboard.sh harvest-move). 서비스 호출은 즉시 반환하고
    시나리오는 백그라운드로 돈다(완주는 아래 eval 이 로그로 폴링)."""
    subprocess.run(['bash', DASH, 'harvest-move'], capture_output=True, text=True, timeout=40)


def eval_s2e2():
    """S2 E2 수확 위치 이동+도킹: ACS→DCS 로 수확지점까지 경로(전 구간 capture=false) 접수 →
    DCS→DdaGo 중계 → 도착 후 Dock 하달·중계 → 도킹 성공 시 E3 진입 게이트 오픈까지.

    순찰(E1·E2)과 달리 이동 중 촬영·분석이 없다(capture=false → 분석 경로 미진입). 판정은
    '경로 수신 → 도킹 지시 수신 → 도킹 결과 전달(code=0) → E3 진입 가능(도킹 성공)' 4단계가
    DCS 로그에 모두 남는지로 한다. 도킹이 실패하면 게이트가 열리지 않아 FAIL 로 갈린다."""
    clear_wire()   # 실행 시 메시지 초기화
    _trigger_harvest_move()

    open_key = 'E3 진입 가능(도킹 성공'   # 게이트 오픈(성공)만. '해제'(clear)와 구분된다.
    keys = ['경로 수신', '도킹 지시 수신', '도킹 결과 전달', open_key]

    def done():
        dcs = _tail_filtered('/tmp/dash_dcs.log', keys, n=20)
        return (any('경로 수신' in l for l in dcs)
                and any('도킹 지시 수신' in l for l in dcs)
                and any('도킹 결과 전달' in l and 'code=0' in l for l in dcs)
                and any(open_key in l for l in dcs))

    ok = _wait_until(done, 40)
    dcs = _tail_filtered('/tmp/dash_dcs.log', keys, n=10)
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

    # 1) E2 이동+도킹으로 게이트를 연다
    _trigger_harvest_move()
    open_key = 'E3 진입 가능(도킹 성공'

    def docked():
        return any(open_key in l for l in
                   _tail_filtered('/tmp/dash_dcs.log', [open_key], n=5))

    if not _wait_until(docked, 40):
        dcs = _tail_filtered('/tmp/dash_dcs.log',
                             ['경로 수신', '도킹 결과 전달', open_key], n=8)
        return False, ['(E2 도킹 실패 — E3 는 도킹 성공이 선행돼야 함)'] + dcs

    # 2) E3 수확 시작
    _trigger_harvest()
    exits = ('exit=DEPLETED', 'exit=FULL', 'exit=MAX_ROUNDS_EXCEEDED')
    keys = ['수확 시작 수신', 'Ddagi 수확 하달', 'Ddagi 수확 종료', '수확 결과 전달']

    def done():
        dcs = _tail_filtered('/tmp/dash_dcs.log', keys, n=20)
        return (any('수확 시작 수신' in l for l in dcs)          # ACS → DCS 접수
                and any('Ddagi 수확 하달' in l for l in dcs)      # DCS → Ddagi 중계
                and any('수확 결과 전달' in l and any(e in l for e in exits)
                        for l in dcs))                            # DCS → ACS 정상 종료 반환

    ok = _wait_until(done, 40)
    dcs = _tail_filtered('/tmp/dash_dcs.log', keys, n=10)
    return ok, dcs or ['(DCS 로그 없음 — dcs·acs·ddago·ddagi UP 확인)']


EVALS = {'e0': eval_e0, 'e1': eval_e1, 'e2': eval_e2, 'e4': eval_e4,
         's2e2': eval_s2e2, 's2e3': eval_s2e3}
EVAL_NAMES = {'e0': 'E0 상시 모니터링', 'e1': 'E1 순찰 시작', 'e2': 'E2 체크·저장',
              'e4': 'E4 복귀·도킹', 's2e2': 'S2 E2 수확 이동·도킹',
              's2e3': 'S2 E3 수확 대상 인식'}


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

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/api/status':
            return self._json({'nodes': status(), 'ai_target': read_target(),
                               'robot_id': robot_id()})
        if path == '/api/ai-target':
            return self._json(read_target())
        if path == '/api/logs':
            return self._json(read_logs())
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
    print('[dg_web control_server] http://127.0.0.1:8000  (정적 + /api)')
    ThreadingHTTPServer(('127.0.0.1', 8000), Handler).serve_forever()

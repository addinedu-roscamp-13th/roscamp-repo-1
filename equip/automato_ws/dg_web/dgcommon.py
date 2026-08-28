#!/usr/bin/env python3
"""dg_web 공용 유틸 — 시뮬 서버(control_server)와 실서버(real_server)가 함께 쓴다.

두 서버는 **프로세스도 포트도 화면도 따로**다(시뮬 :8000 / 실서버 :8010). 섞이면
실장비가 붙은 채로 시뮬을 켜는 사고가 나기 때문이다. 다만 '프로세스가 떠 있나',
'로그 꼬리를 읽는다' 같은 것까지 두 벌로 두면 한쪽만 고치는 일이 생기므로 여기 모은다.
"""
import json
import os
import re
import subprocess

WEB_DIR = os.path.dirname(os.path.abspath(__file__))
WS_DIR = os.path.dirname(WEB_DIR)
# 2026-08-06: dashboard.sh·realboard.sh 가 워크스페이스 루트에서 **dg_web/ 안으로** 옮겨졌다
#  (팀 요청). 즉 스크립트는 이 파일과 같은 폴더에 있다.
DASH = os.path.join(WEB_DIR, 'dashboard.sh')


def robot_id():
    """로봇 식별자. dashboard.sh 와 같은 출처(환경변수 ROBOT_ID)를 읽는다.
    이 값은 DCS→ACS 이름(/{robot_id}/...)에만 쓰인다 — 로봇 쪽 이름에는 안 붙는다."""
    return os.environ.get('ROBOT_ID', 'dg_01')


def is_up(kind, target):
    if kind == 'proc':
        return subprocess.run(['pgrep', '-f', target],
                              stdout=subprocess.DEVNULL).returncode == 0
    out = subprocess.run(['ss', '-ltn'], capture_output=True, text=True).stdout
    return (':' + target + ' ') in out


def tail_bytes(path, nbytes=131072):
    """파일 끝에서 nbytes 만 읽어 줄 단위로 돌려준다.

    바이트 위치로 자르므로 첫 줄은 잘린 조각이다. 화면에 반토막 로그가 뜨고
    @@WIRE@@ 표시도 사라져 필터를 빠져나가므로 버린다."""
    try:
        with open(path, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - nbytes))
            data = f.read()
        lines = data.decode('utf-8', 'replace').splitlines()
        if size > nbytes and lines:
            lines = lines[1:]
        return lines
    except OSError:
        return []


def read_json(path, default):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except (OSError, ValueError):
        return default


def write_json(path, data):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


# ── 로봇 에이전트(dg_agent) 호출 ────────────────────────────────────────
AGENTS_FILE = os.path.join(WEB_DIR, 'agents.local.json')
DEFAULT_AGENTS = {'ddago02': {'addr': 'ddago02', 'port': 8500, 'token': ''}}


def read_agents():
    """로봇 인벤토리. 접속은 /etc/hosts 별칭으로 한다(IP 직접 지정은 망이 바뀌면 못 찾는다)."""
    data = read_json(AGENTS_FILE, None)
    return data if isinstance(data, dict) and data else dict(DEFAULT_AGENTS)


def agent_call(host, path, method='GET', body=None, timeout=6):
    """로봇이 꺼져 있거나 망이 끊긴 게 정상 상황이라, 예외를 밖으로 던지지 않고
    {'error': ...} 로 돌려준다. 화면이 통째로 죽으면 원인을 볼 수 없다."""
    import urllib.error
    import urllib.request
    ag = read_agents().get(host)
    if not ag:
        return {'error': '등록되지 않은 로봇: %s' % host}
    url = 'http://%s:%s%s' % (ag['addr'], ag.get('port', 8500), path)
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(url, data=data, method=method)
    req.add_header('Content-Type', 'application/json')
    if ag.get('token'):
        req.add_header('Authorization', 'Bearer ' + ag['token'])
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        # 401 은 십중팔구 토큰 불일치다. 원인을 바로 알 수 있게 말로 적어 준다.
        if e.code == 401:
            return {'error': '401 인증 실패 — agents.local.json 의 token 과 '
                             '로봇 DG_AGENT_TOKEN 이 다릅니다'}
        return {'error': 'HTTP %s' % e.code}
    except Exception as e:
        return {'error': '%s: %s' % (type(e).__name__, e)}


def first_robot():
    return (list(read_agents()) or [''])[0]


# ── DCS 메시지 시계열(@@WIRE@@) ─────────────────────────────────────────
# DCS 는 주고받은 메시지를 한 줄 JSON(@@WIRE@@)으로 로그에 남긴다. 시뮬 화면과 실서버
# 화면이 **같은 파일 하나**를 본다 — DCS 는 어느 쪽에서 띄우든 하나뿐인 프로세스라,
# 로그가 갈리면 시계열이 반토막 난다(cmdcfg 의 real-dcs 가 log='dcs' 로 맞춰 준다).
DCS_LOG = '/tmp/dash_dcs.log'
WIRE_SINCE_FILE = '/tmp/dash_wire_since'   # '지우기' 기준 시각(파일이라 서버 재시작에도 유지)
WIRE_MAX = 500


def clear_wire():
    try:
        with open(WIRE_SINCE_FILE, 'w') as f:
            f.write(repr(__import__('time').time()))
    except OSError:
        pass


def wire_since():
    try:
        with open(WIRE_SINCE_FILE) as f:
            return float(f.read().strip() or 0)
    except (OSError, ValueError):
        return 0.0


def read_wire(limit=WIRE_MAX):
    """[{ts, dir, iface, payload, text}, ...] 시각 오름차순, 최근 limit 건."""
    marker = '@@WIRE@@ '
    since = wire_since()
    out = []
    for ln in tail_bytes(DCS_LOG, 1048576):
        i = ln.find(marker)
        if i < 0:
            continue
        try:
            rec = json.loads(ln[i + len(marker):])
        except ValueError:
            continue
        if rec.get('ts', 0) < since or not rec.get('iface'):
            continue
        # text: 파이썬이 만든 JSON 문자열을 그대로 실어 보낸다. 브라우저에서 다시
        # JSON.stringify 하면 0.0 → 0 처럼 float 의 소수점이 사라진다.
        out.append({'ts': rec.get('ts'), 'dir': rec.get('dir'),
                    'iface': rec.get('iface'), 'payload': rec.get('payload'),
                    'text': json.dumps(rec.get('payload'), ensure_ascii=False)})
    out.sort(key=lambda r: r.get('ts') or 0)
    return out[-limit:]


# ── 중복 실행 방지 ──────────────────────────────────────────────────────
# 포트가 하나뿐이라 두 번째 인스턴스는 어차피 바인드에 실패한다. 그런데 그때 남는 것이
# **traceback 한 줄과 죽은 듯 살아 있는 프로세스**라, 나중에 pgrep 으로 보면 서버가 둘로
# 보이고 "어느 쪽이 진짜인가"를 따지게 된다(실제로 그렇게 헷갈렸다).
# 그래서 바인드 이전에 파일 락으로 먼저 막고, 사람이 읽을 문장을 남기고 끝낸다.
def single_instance(name):
    """이미 같은 이름으로 돌고 있으면 메시지를 남기고 종료. 아니면 락 핸들을 돌려준다.
    ⚠ 돌려받은 핸들을 **살려 두어야** 락이 유지된다(가비지 컬렉션되면 풀린다)."""
    import fcntl
    path = '/tmp/dg_%s.lock' % re.sub(r'[^A-Za-z0-9_.-]', '_', name)
    f = open(path, 'a+')
    try:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.seek(0)
        pid = (f.read() or '').strip()
        raise SystemExit('[%s] 이미 실행 중입니다%s — 중복 기동을 중단합니다.\n'
                         '  종료: kill %s   (락: %s)'
                         % (name, (' (pid %s)' % pid) if pid else '', pid or '<pid>', path))
    f.seek(0)
    f.truncate()
    f.write(str(os.getpid()))
    f.flush()
    return f


# ── 맵 파일 읽기 (PGM + YAML) ───────────────────────────────────────────
# 맵을 rosbridge 로 받지 않고 **파일에서 직접** 읽는다. /map 은 latched(transient_local)
# 라 늦게 붙은 구독자가 못 받는 일이 있는데, 맵은 애초에 정적이라 파일이면 충분하다.
# ⚠ 반드시 nav2 가 실제로 로드하는 파일을 봐야 한다. 이 저장소에는 낡은 사본이 여럿 있고,
#    다른 사본을 그리면 좌표가 조용히 어긋난다(그게 이 기능의 최악의 실패다).
DEFAULT_MAP_YAML = os.path.expanduser('~/maps/automato_map.yaml')
_MAP_CACHE = {'path': None, 'mtime': 0.0, 'data': None}


def map_yaml_path():
    """nav2 에 넘어가는 맵 경로를 그대로 따라간다.
    ① real-nav2 명령의 `map:=...`  ② $DG_MAP_YAML  ③ 기본값
    화면이 그리는 맵과 nav2 가 쓰는 맵이 갈리지 않게, 해석 결과를 API 로 함께 내보낸다."""
    try:
        import cmdcfg
        m = re.search(r'map:=(\S+)', cmdcfg.effective('real-nav2'))
        if m:
            return os.path.expanduser(m.group(1).strip('"\''))
    except Exception:
        pass
    return os.path.expanduser(os.environ.get('DG_MAP_YAML') or DEFAULT_MAP_YAML)


def parse_map_yaml(path):
    """map_server 용 yaml 최소 파서. PyYAML 없이 `key: value` 와 `[a, b, c]` 만 읽는다
    (이 대시보드는 표준 라이브러리만 쓴다)."""
    out = {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
    except OSError as e:
        return {'error': '맵 yaml 을 못 읽었습니다: %s' % e}
    for ln in lines:
        ln = ln.split('#', 1)[0].strip()
        if not ln or ':' not in ln:
            continue
        k, _, v = ln.partition(':')
        k, v = k.strip(), v.strip()
        if v.startswith('[') and v.endswith(']'):
            try:
                out[k] = [float(x) for x in v[1:-1].split(',') if x.strip()]
            except ValueError:
                out[k] = v
        else:
            try:
                out[k] = float(v) if re.fullmatch(r'-?\d+(\.\d+)?([eE][-+]?\d+)?', v) else v
            except ValueError:
                out[k] = v
    return out


def parse_pgm(path):
    """P5(바이너리 그레이스케일) PGM 파서.

    예외를 던지지 않고 {'error': ...} 로 돌려준다 — 맵이 없는 것도 정상 상황이고,
    화면이 통째로 죽으면 원인을 볼 수 없다(agent_call 과 같은 규약).
    주의: maxval 뒤의 **공백문자 하나만** 소비한다. strip 계열로 잘라내면 첫 픽셀이
    0x0A 일 때 한 칸 밀려 맵 전체가 대각선으로 어긋난다."""
    try:
        with open(path, 'rb') as f:
            raw = f.read()
    except OSError as e:
        return {'error': '맵 pgm 을 못 읽었습니다: %s' % e}
    if not raw.startswith(b'P5'):
        return {'error': 'P5(바이너리) PGM 이 아닙니다: %s' % raw[:2]}

    i, toks = 2, []
    while len(toks) < 3 and i < len(raw):
        c = raw[i:i + 1]
        if c.isspace():
            i += 1
            continue
        if c == b'#':                       # 주석은 줄 끝까지 버린다(GIMP 가 넣는다)
            while i < len(raw) and raw[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(raw) and not raw[j:j + 1].isspace():
            j += 1
        toks.append(raw[i:j])
        i = j
    if len(toks) < 3:
        return {'error': 'PGM 헤더가 불완전합니다'}
    try:
        w, h, maxval = (int(t) for t in toks)
    except ValueError:
        return {'error': 'PGM 헤더 숫자를 읽지 못했습니다'}
    if maxval > 255:
        return {'error': '16bit PGM 은 지원하지 않습니다 (maxval=%d)' % maxval}
    data = raw[i + 1:]                      # 공백 정확히 1개만 건너뛴다
    if len(data) != w * h:
        return {'error': 'PGM 픽셀 수 불일치: %d 바이트, 기대 %d(%dx%d)'
                         % (len(data), w * h, w, h)}
    return {'width': w, 'height': h, 'maxval': maxval, 'data': data}


def load_map(yaml_path=None):
    """{ok, path, image_path, width, height, resolution, origin, negate,
        occupied_thresh, free_thresh, mtime, data(bytes)} 또는 {ok:False, error}."""
    path = yaml_path or map_yaml_path()
    meta = parse_map_yaml(path)
    if 'error' in meta:
        return {'ok': False, 'path': path, 'error': meta['error']}
    img = meta.get('image') or ''
    img_path = img if os.path.isabs(img) else os.path.join(os.path.dirname(path), img)
    try:
        mtime = max(os.path.getmtime(path), os.path.getmtime(img_path))
    except OSError as e:
        return {'ok': False, 'path': path, 'image_path': img_path,
                'error': '맵 파일 시각을 못 읽었습니다: %s' % e}

    c = _MAP_CACHE
    if c['path'] == path and c['mtime'] == mtime and c['data']:
        return c['data']

    pgm = parse_pgm(img_path)
    if 'error' in pgm:
        return {'ok': False, 'path': path, 'image_path': img_path, 'error': pgm['error']}
    origin = meta.get('origin') or [0.0, 0.0, 0.0]
    while len(origin) < 3:
        origin.append(0.0)
    out = {'ok': True, 'path': path, 'image_path': img_path,
           'width': pgm['width'], 'height': pgm['height'],
           'resolution': float(meta.get('resolution') or 0.05),
           'origin': [float(v) for v in origin[:3]],
           'negate': int(meta.get('negate') or 0),
           'occupied_thresh': float(meta.get('occupied_thresh') or 0.65),
           'free_thresh': float(meta.get('free_thresh') or 0.196),
           'mtime': mtime, 'data': pgm['data']}
    c['path'], c['mtime'], c['data'] = path, mtime, out
    return out


def map_fingerprint(m):
    """저장된 좌표가 이 맵의 것인지 판별하는 지문. 맵을 다시 뜨면 origin 이 바뀌고
    저장 좌표는 **조용히 딴 곳**을 가리키므로, 저장할 때 같이 박아 둔다."""
    if not m or not m.get('ok'):
        return None
    return {'path': m['path'], 'mtime': m['mtime'], 'resolution': m['resolution'],
            'origin': m['origin'], 'size': [m['width'], m['height']]}


def occupancy_at(m, x, y):
    """월드 좌표의 점유 확률(0~1)과 판정. 맵 밖이면 None.
    origin 은 이미지 **왼쪽-아래** 모서리이고 PGM 0행은 **맨 위**라 y 를 뒤집는다."""
    if not m or not m.get('ok'):
        return None
    r, (ox, oy, _oyaw) = m['resolution'], m['origin']
    col = int((x - ox) / r)
    row = int(m['height'] - (y - oy) / r)
    if col < 0 or col >= m['width'] or row < 0 or row >= m['height']:
        return None
    v = m['data'][row * m['width'] + col]
    p = (v / 255.0) if m['negate'] else ((255 - v) / 255.0)
    state = ('occupied' if p > m['occupied_thresh']
             else 'free' if p < m['free_thresh'] else 'unknown')
    return {'p': round(p, 3), 'state': state, 'col': col, 'row': row}


# ── 충전소 위치 (로봇별) ────────────────────────────────────────────────
# 로봇에는 충전소 좌표를 담은 파일이 없다(DB 시드에만 있다). 복귀를 지휘하는 주체가
# 노트북이고 Nav2 도 노트북에서 돌므로, 좌표도 노트북에 둔다.
HOME_FILE = os.path.join(WEB_DIR, 'home.local.json')


def read_home():
    d = read_json(HOME_FILE, None)
    return d if isinstance(d, dict) else {}


def write_home(data):
    write_json(HOME_FILE, data)


# ── 웨이포인트 (로봇별) ──────────────────────────────────────────────────
# 화면에서 지도를 클릭해 찍어 두는 경로점. 충전소 진입점과 같은 이유로 노트북에 둔다.
#   {robot_id: {'points': [{'id','name','x','y','yaw','dock','task_point_id'}, ...]}}
#   dock: 'none' | 'floor'(H마커) | 'reflective'(반사테이프)
#   H 마커는 별도 목록을 두지 않는다 — dock='floor' 인 점이 곧 H 마커 지점이다.
#   따로 관리하면 지도 위 표시와 실제 도킹 대상이 어긋날 수 있다.
WAYPOINTS_FILE = os.path.join(WEB_DIR, 'waypoints.local.json')


def read_waypoints():
    d = read_json(WAYPOINTS_FILE, None)
    return d if isinstance(d, dict) else {}


def write_waypoints(data):
    write_json(WAYPOINTS_FILE, data)

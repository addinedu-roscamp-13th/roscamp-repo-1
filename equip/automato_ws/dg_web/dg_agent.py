#!/usr/bin/env python3
"""dg_agent — 로봇 온보드 관리 에이전트. 실서버 대시보드(/real)가 이걸 통해 로봇을 본다.

로봇에 파일 하나만 복사해 띄운다. **표준 라이브러리만** 쓰고 ROS 도 venv 도 건드리지
않는다 — 빌드가 깨졌거나 ROS 가 안 뜨는 로봇에서도 에이전트는 떠야 원인을 볼 수 있다.
같은 이유로 진단 채널을 DDS 에 태우지 않는다(DDS 가 죽으면 같이 눈이 먼다).

    로봇에서:  python3 dg_agent.py            # 기본 0.0.0.0:8500
    토큰 지정: DG_AGENT_TOKEN=xxxx python3 dg_agent.py
    포트 지정: DG_AGENT_PORT=8500

API (전부 Authorization: Bearer <토큰> 필요, /health 만 예외)
    GET  /health                  → {ok, host, time, uptime}
    GET  /system                  → cpu·mem·온도·스로틀·네트워크·시계·배터리
    GET  /procs?match=<정규식>    → 프로세스 목록(등록 서비스 확인용)
    GET  /log?name=<이름>&n=200   → 이 에이전트가 띄운 프로세스의 stdout+stderr
    POST /run   {name, cmdline}   → 백그라운드 실행(로그는 /tmp/dg_agent_<name>.log)
                                    셸은 bash -ic (로봇 .bashrc 를 그대로 태움).
                                    바꾸려면 ~/.dg_agent_prelude 또는 DG_AGENT_PRELUDE
    POST /stop  {name, pattern}   → 그 프로세스 그룹 종료(PID 파일 우선, 없으면 패턴)

⚠️ /run 은 받은 명령줄을 그대로 셸에 넘긴다. 대시보드에서 로봇 명령까지 편집할 수 있게
하려면 이 방법뿐이다. 그래서 **토큰 필수**이고, 폐쇄망(로봇 랜선망)에서만 쓴다는 전제다.
인터넷에 닿는 인터페이스에 올리지 말 것.
"""
import json
import os
import re
import signal
import socket
import subprocess
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

PORT = int(os.environ.get('DG_AGENT_PORT', '8500'))
TOKEN = os.environ.get('DG_AGENT_TOKEN', '')
LOG_DIR = '/tmp'


def _read(path, default=''):
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            return f.read()
    except OSError:
        return default


def _cpu_times():
    """/proc/stat 첫 줄. 두 번 읽어 차분을 내야 사용률이 나온다."""
    line = _read('/proc/stat').split('\n', 1)[0].split()
    vals = [int(x) for x in line[1:]] if len(line) > 1 else []
    return vals


def cpu_percent(interval=0.25):
    a = _cpu_times()
    time.sleep(interval)
    b = _cpu_times()
    if not a or not b:
        return None
    idle_a, idle_b = a[3] + (a[4] if len(a) > 4 else 0), b[3] + (b[4] if len(b) > 4 else 0)
    tot_a, tot_b = sum(a), sum(b)
    dt = tot_b - tot_a
    if dt <= 0:
        return None
    return round(100.0 * (1.0 - (idle_b - idle_a) / dt), 1)


def meminfo():
    d = {}
    for ln in _read('/proc/meminfo').splitlines():
        k, _, v = ln.partition(':')
        d[k.strip()] = int(v.split()[0]) if v.split() else 0
    total = d.get('MemTotal', 0) / 1024.0
    avail = d.get('MemAvailable', 0) / 1024.0
    swap_t = d.get('SwapTotal', 0) / 1024.0
    swap_f = d.get('SwapFree', 0) / 1024.0
    return {'total_mb': round(total), 'available_mb': round(avail),
            'used_pct': round(100.0 * (1 - avail / total), 1) if total else None,
            'swap_used_mb': round(swap_t - swap_f)}


def temperature_c():
    raw = _read('/sys/class/thermal/thermal_zone0/temp').strip()
    try:
        return round(int(raw) / 1000.0, 1)
    except ValueError:
        return None


def throttled():
    """Pi 전용. 저전압·과열 스로틀은 '원인 불명 이상 동작'의 단골이라 꼭 본다.
    0x0 이면 정상. 없는 기기(비 Pi)면 None."""
    try:
        out = subprocess.run(['vcgencmd', 'get_throttled'], capture_output=True,
                             text=True, timeout=3).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None
    m = re.search(r'0x([0-9a-fA-F]+)', out)
    if not m:
        return None
    bits = int(m.group(1), 16)
    return {
        'raw': '0x%x' % bits,
        'ok': bits == 0,
        'under_voltage_now': bool(bits & 0x1),
        'throttled_now': bool(bits & 0x4),
        'under_voltage_since_boot': bool(bits & 0x10000),
        'throttled_since_boot': bool(bits & 0x40000),
    }


def net_bytes():
    """인터페이스별 누적 바이트. 대시보드가 두 번 받아 차분으로 실효 대역폭을 낸다.
    카메라 스트림이 WiFi 를 밀어내는지 보려면 이 값이 필요하다."""
    out = {}
    for ln in _read('/proc/net/dev').splitlines()[2:]:
        name, _, rest = ln.partition(':')
        f = rest.split()
        if len(f) >= 9:
            out[name.strip()] = {'rx': int(f[0]), 'tx': int(f[8])}
    return out


def wifi_link():
    """신호 세기·비트레이트. brcmfmac 이 내려앉기 전에 보통 여기가 먼저 나빠진다."""
    try:
        out = subprocess.run(['iw', 'dev'], capture_output=True, text=True, timeout=3).stdout
        ifaces = re.findall(r'Interface (\S+)', out)
    except (OSError, subprocess.SubprocessError):
        return None
    for ifc in ifaces:
        try:
            o = subprocess.run(['iw', 'dev', ifc, 'link'], capture_output=True,
                               text=True, timeout=3).stdout
        except (OSError, subprocess.SubprocessError):
            continue
        if 'Not connected' in o:
            continue
        sig = re.search(r'signal:\s*(-?\d+)', o)
        rate = re.search(r'tx bitrate:\s*([\d.]+)', o)
        return {'iface': ifc,
                'signal_dbm': int(sig.group(1)) if sig else None,
                'tx_bitrate_mbps': float(rate.group(1)) if rate else None}
    return None


def dmesg_wifi():
    """WiFi 드라이버 최근 에러. '로봇이 죽었다'의 상당수가 여기 남는다."""
    try:
        out = subprocess.run(['dmesg', '--level=err,warn', '-T'],
                             capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [ln for ln in out.splitlines() if 'brcmfmac' in ln or 'ieee80211' in ln][-8:]


# ── 명령을 실행할 셸 ────────────────────────────────────────────────────
# 에이전트 자신의 환경을 그대로 쓰면 안 된다. ssh 로 띄우든 systemd 로 띄우든
# **비대화형**이라 Ubuntu 기본 ~/.bashrc 가 맨 앞에서 return 해 버리고, 그 뒤에 있는
# ROS setup·ROS_DOMAIN_ID·RMW·FastDDS 프로파일이 하나도 실행되지 않는다
# (ros2 조차 PATH 에 없다). 그 상태로 bringup 을 띄우면 그냥 실패하거나, 더 나쁘게는
# 기본 도메인으로 떠서 노트북과 서로 안 보인다.
#
# 그래서 기본값은 `bash -ic` 다 — 로봇의 .bashrc 를 통째로 태워서, **사람이 터미널에서
# 얻는 환경과 똑같은 것**을 쓴다. 설정을 여기에 또 적어 두면 .bashrc 와 갈라져서,
# 예전에 RMW 가 어긋나 며칠을 태운 그 실패가 그대로 재현된다.
# 결정적으로 다른 값이 필요하면 ~/.dg_agent_prelude 나 DG_AGENT_PRELUDE 로 덮어쓴다.
def exec_prelude():
    p = os.environ.get('DG_AGENT_PRELUDE', '').strip()
    if p:
        return p
    f = os.path.expanduser('~/.dg_agent_prelude')
    if os.path.isfile(f):
        return _read(f).strip() or None
    return None


def build_argv(cmdline):
    pre = exec_prelude()
    if pre:
        return ['bash', '-c', pre + '\n' + cmdline]
    return ['bash', '-ic', cmdline]


_ENV_CACHE = {'t': 0.0, 'env': None}
ROS_KEYS = ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
            'FASTRTPS_DEFAULT_PROFILES_FILE', 'PINKY_NS', 'ROBOT_ID')


def ros_env(ttl=300.0):
    """DDS 설정. **명령이 실제로 실행될 환경**을 보고한다(에이전트 자신의 환경이 아니다).

    셸을 한 번 띄워 물어보므로 Pi4 에서는 공짜가 아니다 → 오래 캐시한다.
    이 값이 노트북과 하나라도 어긋나면 서로 안 보인다 — 실전에서 제일 많이 물린 곳이라
    화면 맨 앞에 띄운다."""
    now = time.time()
    if _ENV_CACHE['env'] is not None and now - _ENV_CACHE['t'] < ttl:
        return _ENV_CACHE['env']
    env = {k: '' for k in ROS_KEYS}
    script = '; '.join('echo "%s=${%s}"' % (k, k) for k in ROS_KEYS)
    try:
        out = subprocess.run(build_argv(script), capture_output=True,
                             text=True, timeout=40).stdout
        for ln in out.splitlines():
            k, _, v = ln.partition('=')
            if k in env:
                env[k] = v.strip()
    except (OSError, subprocess.SubprocessError):
        pass
    env['_shell'] = 'prelude 파일' if exec_prelude() else 'bash -ic (.bashrc)'
    _ENV_CACHE['t'], _ENV_CACHE['env'] = now, env
    return env


def clock():
    """벽시계와 단조시계를 같이 준다. 대시보드가 자기 시각과 견줘 편차를 낸다.
    ssh 로 date 를 찍어 비교하면 왕복 지연이 섞이므로(±0.3s) 이렇게 실어 보내야 한다."""
    return {'realtime': time.time(), 'monotonic': time.monotonic(),
            'tz': time.strftime('%Z%z')}


# ── 배터리 ─────────────────────────────────────────────────────────────
# 로봇 배터리는 I2C ADC(버스1 / 0x08 / 레지스터 0xF8)에 물려 있다. 값 환산은
# pinky_bringup 의 Battery 클래스와 같은 식을 쓴다(분압비 13/28, 4.096V 기준).
BATT_BUS, BATT_ADDR, BATT_REG = 1, 0x08, 0xF8
BATT_FULL_V, BATT_EMPTY_V = 7.6, 6.8


def _proc_running(pat):
    try:
        return subprocess.run(['pgrep', '-f', pat],
                              stdout=subprocess.DEVNULL, timeout=5).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def battery():
    """배터리 전압·잔량. ROS 없이 I2C 를 직접 읽는다 — 스택이 안 떠 있을 때도 봐야 하니까
    (충전 안 된 걸 모르고 나갔다가 로봇이 방전돼 배포를 못 한 적이 있다).

    ⚠ 다만 bringup 의 battery_publisher 가 **같은 장치를 폴링**한다. 읽기가
    '채널 선택 → 값 읽기' 두 단계라 두 프로세스가 끼어들면 엉뚱한 채널 값이 나온다.
    그래서 그쪽이 돌고 있으면 아예 읽지 않고, 대시보드가 DCS 텔레메트리(같은 값)를 쓰게 한다."""
    if _proc_running('battery_publisher'):
        return {'source': 'ros', 'percent': None, 'voltage': None,
                'note': 'bringup 이 I2C 사용 중 — 텔레메트리 값을 쓰세요'}
    try:
        import smbus2
    except ImportError:
        return {'source': 'none', 'percent': None, 'voltage': None,
                'error': 'smbus2 미설치'}
    try:
        bus = smbus2.SMBus(BATT_BUS)
    except OSError as e:
        return {'source': 'none', 'percent': None, 'voltage': None, 'error': str(e)}
    vals = []
    try:
        for _ in range(20):
            bus.write_byte(BATT_ADDR, BATT_REG)
            time.sleep(0.001)
            d = bus.read_i2c_block_data(BATT_ADDR, 0, 2)
            vals.append((d[0] << 4) | (d[1] >> 4))
    except OSError as e:
        return {'source': 'none', 'percent': None, 'voltage': None, 'error': str(e)}
    finally:
        try:
            bus.close()
        except Exception:
            pass
    if not vals:
        return {'source': 'none', 'percent': None, 'voltage': None, 'error': '읽기 실패'}
    v = (sum(vals) / len(vals) / 4096.0) * 4.096 / (13.0 / 28.0)
    pct = max(0.0, min(100.0, (v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100))
    return {'source': 'i2c', 'voltage': round(v, 3), 'percent': round(pct, 1)}


def procs(match=''):
    try:
        out = subprocess.run(['ps', '-eo', 'pid,pcpu,pmem,rss,etimes,args', '--sort=-pcpu'],
                             capture_output=True, text=True, timeout=8).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    rx = re.compile(match) if match else None
    rows = []
    for ln in out.splitlines()[1:]:
        f = ln.split(None, 5)
        if len(f) < 6:
            continue
        if rx and not rx.search(f[5]):
            continue
        rows.append({'pid': int(f[0]), 'cpu': float(f[1]), 'mem': float(f[2]),
                     'rss_mb': round(int(f[3]) / 1024.0, 1), 'uptime_s': int(f[4]),
                     'cmd': f[5][:220]})
    return rows[:40]


def _logfile(name):
    safe = re.sub(r'[^A-Za-z0-9_.-]', '_', name)[:64]
    return os.path.join(LOG_DIR, 'dg_agent_%s.log' % safe)


def _pidfile(name):
    return _logfile(name)[:-4] + '.pid'


def run_bg(name, cmdline):
    """새 프로세스 그룹으로 띄우고 PID 를 남긴다.

    그룹으로 띄우는 이유: ros2 launch 는 자식 노드를 여럿 만들고, 부모만 죽이면 노드가
    남아 다음 기동과 충돌한다. 종료는 그룹째 신호를 줘야 한다.
    PID 를 파일로 남기는 이유: 실행되는 명령줄에는 우리가 붙인 이름(키)이 안 들어가므로
    이름으로 pgrep 해서는 절대 못 찾는다."""
    log = _logfile(name)
    with open(log, 'wb') as f:
        p = subprocess.Popen(build_argv(cmdline), stdout=f, stderr=subprocess.STDOUT,
                             stdin=subprocess.DEVNULL, start_new_session=True)
    try:
        with open(_pidfile(name), 'w') as f:
            f.write(str(p.pid))
    except OSError:
        pass
    return {'ok': True, 'name': name, 'pid': p.pid, 'log': log}


def _kill_group(pid, killed):
    try:
        os.killpg(os.getpgid(pid), signal.SIGINT)
        killed.append(pid)
    except OSError:
        pass


def stop_bg(name, pattern=''):
    """① 이 에이전트가 띄운 것은 PID 파일로 정확히 잡는다.
    ② 사람이 ssh 로 직접 띄운 경우엔 PID 파일이 없으므로, 대시보드가 함께 보내 준
       프로세스 패턴으로 찾는다(그래야 화면에서 UP 으로 보이는 것을 끌 수 있다)."""
    killed = []
    try:
        with open(_pidfile(name)) as f:
            pid = int((f.read() or '0').strip())
        if pid > 0:
            _kill_group(pid, killed)
    except (OSError, ValueError):
        pass
    if not killed and pattern:
        try:
            out = subprocess.run(['pgrep', '-f', pattern], capture_output=True,
                                 text=True, timeout=5).stdout
        except (OSError, subprocess.SubprocessError):
            out = ''
        for pid in [int(x) for x in out.split() if x.isdigit()]:
            if pid != os.getpid():
                _kill_group(pid, killed)
    return {'ok': bool(killed), 'name': name, 'killed': killed, 'log': _logfile(name)}


# tty 없이 대화형 셸을 띄우면 bash 가 늘 뱉는 두 줄. 무해하지만 로그마다 끼면
# 진짜 오류를 가리므로 걷어낸다.
_SHELL_NOISE = ('cannot set terminal process group', 'no job control in this shell')


def tail(name, n=200):
    lines = _read(_logfile(name)).splitlines()
    lines = [ln for ln in lines if not any(x in ln for x in _SHELL_NOISE)]
    return lines[-n:]


class Handler(BaseHTTPRequestHandler):
    server_version = 'dg_agent'

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _authed(self):
        if not TOKEN:
            return True   # 토큰 미설정 = 개발 편의. 운용에서는 반드시 설정한다.
        return self.headers.get('Authorization', '') == 'Bearer ' + TOKEN

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
        if u.path == '/health':
            return self._json({'ok': True, 'host': socket.gethostname(),
                               'time': time.time(), 'uptime_s': _uptime()})
        if not self._authed():
            return self._json({'error': 'unauthorized'}, 401)
        if u.path == '/system':
            return self._json({
                'host': socket.gethostname(), 'uptime_s': _uptime(),
                'cpu_pct': cpu_percent(), 'loadavg': os.getloadavg(),
                'mem': meminfo(), 'temp_c': temperature_c(), 'throttled': throttled(),
                'net': net_bytes(), 'wifi': wifi_link(), 'wifi_errors': dmesg_wifi(),
                'battery': battery(),
                'ros': ros_env(), 'clock': clock(),
            })
        if u.path == '/procs':
            return self._json({'procs': procs(q.get('match', [''])[0])})
        if u.path == '/log':
            name = q.get('name', [''])[0]
            n = int(q.get('n', ['200'])[0] or 200)
            return self._json({'name': name, 'lines': tail(name, n)})
        return self._json({'error': 'not found'}, 404)

    def do_POST(self):
        u = urlparse(self.path)
        if not self._authed():
            return self._json({'error': 'unauthorized'}, 401)
        b = self._body()
        if u.path == '/run':
            name, cmd = b.get('name', ''), b.get('cmdline', '')
            if not name or not cmd:
                return self._json({'error': 'name·cmdline 필요'}, 400)
            return self._json(run_bg(name, cmd))
        if u.path == '/stop':
            name = b.get('name', '')
            if not name:
                return self._json({'error': 'name 필요'}, 400)
            return self._json(stop_bg(name, b.get('pattern', '')))
        return self._json({'error': 'not found'}, 404)

    def log_message(self, *a):
        pass


def _uptime():
    try:
        return int(float(_read('/proc/uptime').split()[0]))
    except (IndexError, ValueError):
        return None


if __name__ == '__main__':
    print('[dg_agent] 0.0.0.0:%d  토큰=%s' % (PORT, '설정됨' if TOKEN else '없음(개발용)'))
    ThreadingHTTPServer(('0.0.0.0', PORT), Handler).serve_forever()

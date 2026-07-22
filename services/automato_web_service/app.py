"""
Automato GUI — 실시간 공유 상태 서버 (Flask)
  - 공유 상태(mode/follow/zones/todayKg)를 서버가 보관 + state.json 에 영속화
  - GET  /api/state?cid=...   현재 상태 + version + viewers(접속자 수)
  - POST /api/action          {type, payload} 로 상태 변경 → version 증가
로컬 실행:  python3 app.py   →  http://localhost:5000
배포(WSGI): from app import app as application
"""
import os
import time
import json
import base64
import threading
from flask import Flask, request, jsonify, send_from_directory, send_file, Response

BASE = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(BASE, "state.json")

app = Flask(__name__, static_folder="static", static_url_path="")
LOCK = threading.Lock()

DEFAULT_ZONES = ["승인 대기", "대기", "대기", "관찰"]


def default_state():
    return {
        "mode": "day",
        "follow": True,
        "zones": [{"st": s} for s in DEFAULT_ZONES],
        "todayKg": 128.6,        # 오늘 수확량(전체 합계) — 수동 등록분 합산
        "version": 0,
    }


def load_state():
    try:
        with open(STATE_FILE, encoding="utf-8") as f:
            d = json.load(f)
        base = default_state()
        base.update({k: d[k] for k in base if k in d})
        if not isinstance(base.get("zones"), list) or len(base["zones"]) != len(DEFAULT_ZONES):
            base["zones"] = [{"st": s} for s in DEFAULT_ZONES]
        return base
    except Exception:
        return default_state()


def save_state():
    try:
        with open(STATE_FILE, "w", encoding="utf-8") as f:
            json.dump(STATE, f, ensure_ascii=False)
    except Exception:
        pass


STATE = load_state()

# ---- 실시간 카메라/텔레메트리 (로봇·노트북 → 서버로 push, 팀원은 서버에서 pull) ----
INGEST_TOKEN = os.environ.get("INGEST_TOKEN", "automato-live-2026")

# ===== Automato Web Service ↔ Control Service 연동 (Sequence Diagram E1/E2/E3, 2026-07 개정판) =====
#   CONTROL_SERVICE_URL 설정 시: 순찰 available/request 를 Control(ACS, 보연님)로 중계.
#   비어 있으면(pythonanywhere 기본): 기존 로컬 데모로 동작 → 라이브 웹앱 그대로 유지.
try:
    import requests as _rq
except Exception:
    _rq = None
CONTROL_SERVICE_URL = os.environ.get("CONTROL_SERVICE_URL", "").rstrip("/")

def wlog(*a):
    """Web Service 터미널 로그 + 공유 버퍼(파일). web_log_tail 이 어느 워커에서 찍힌 로그든 다 본다."""
    line = "[Web] " + " ".join(str(x) for x in a)
    print(line, flush=True)
    try:
        with _relay_tx() as d:
            d["weblog_seq"] += 1
            d["weblog"].append({"seq": d["weblog_seq"], "text": line})
            d["weblog"] = d["weblog"][-500:]
    except Exception:
        pass

def _control_on():
    return bool(CONTROL_SERVICE_URL and _rq)

def _control_get(path, timeout=4):
    return _rq.get(CONTROL_SERVICE_URL + path, timeout=timeout)

def _control_post(path, body, timeout=6):
    return _rq.post(CONTROL_SERVICE_URL + path, json=body, timeout=timeout)

# ============================================================================
#  E0-5) ACS 텔레메트리 WebSocket — Web Service가 '클라이언트'로 ACS에 접속(1Hz 수신)
#    Endpoint: ws://<acs-host>:8000/ws/telemetry  (ACS가 서버)
#    ROBOT_OFFLINE(3초 이상 미수신)은 Web이 자체 판정.
# ============================================================================
def _derive_telemetry_ws():
    v = os.environ.get("ACS_TELEMETRY_WS", "").strip()
    if v:
        return v
    if CONTROL_SERVICE_URL:                      # http://host:8200 → ws://host:8000/ws/telemetry
        import urllib.parse as _up
        host = _up.urlparse(CONTROL_SERVICE_URL).hostname
        if host:
            return "ws://%s:8000/ws/telemetry" % host
    return ""

TELEMETRY_WS_URL = _derive_telemetry_ws()
ROBOT_OFFLINE_SEC = 3.0
_TELEMETRY = {"robots": [], "recv_at": 0.0, "seq": None, "connected": False}
_TELEMETRY_STARTED = [False]

# ============================================================================
#  자체 함대 (ACS 없이 라이브 단독 구동용) — CONTROL_SERVICE_URL 없으면 Web이 ACS 대역 겸함.
#  값·역할·임계값을 mock_control_service._ROBOTS 와 동일하게 유지 → localhost(mock)와
#  라이브(pythonanywhere)가 같은 화면·같은 동작(dg_01/02/03, 스펙 준수)이 되도록.
#  dg_01·dg_02 = 수확(harvest), dg_03 = 순찰(patrol). 순찰은 전역 1대만.
# ============================================================================
_DEMO_FLEET = {
    "dg_01": {"robot_id": "dg_01", "role": "harvest", "battery_percent": 85.2, "status": "IDLE",
              "current_position": {"x": 3.21, "y": 1.05}, "operational_status": "NORMAL"},
    "dg_02": {"robot_id": "dg_02", "role": "harvest", "battery_percent": 62.0, "status": "IDLE",
              "current_position": {"x": 5.10, "y": 2.30}, "operational_status": "NORMAL"},
    "dg_03": {"robot_id": "dg_03", "role": "patrol",  "battery_percent": 78.0, "status": "IDLE",
              "current_position": {"x": 1.50, "y": 4.00}, "operational_status": "NORMAL"},
}
_DEMO_TASK_TYPE = {"PATROLLING": "PATROL", "HARVESTING": "HARVEST", "TRANSFERRING": "TRANSFER"}
_DEMO_SEQ = [1040]          # 자체 함대 task_id 카운터

def _demo_set_status(robot_id, status):
    r = _DEMO_FLEET.get(robot_id)
    if r:
        r["status"] = status

def _demo_reason(r):
    """E1 가용 판정 (mock._unavailable_reason 과 동일 우선순위·역할별 배터리 임계값)."""
    if r.get("operational_status") != "NORMAL":
        return "IMMOBILIZED"
    if r["status"] != "IDLE":
        return "ROBOT_BUSY"
    thr = MIN_BAT_HARVEST if r.get("role") == "harvest" else MIN_BAT_PATROL
    if r["battery_percent"] < thr:
        return "BATTERY_TOO_LOW"
    return None

def _demo_telemetry_robots():
    """E0-5 축약 텔레메트리 형식 (task_type/nav_status/position/available/unavailable_reason)."""
    out = []
    for r in _DEMO_FLEET.values():
        reason = _demo_reason(r)
        out.append({
            "robot_id": r["robot_id"],
            "task_type": _DEMO_TASK_TYPE.get(r["status"]),
            "nav_status": "NAVIGATING" if r["status"] in ("PATROLLING", "HARVESTING") else "IDLE",
            "position": {"x": r["current_position"]["x"], "y": r["current_position"]["y"], "yaw": 0.0},
            "battery_percent": r["battery_percent"], "role": r["role"],
            "available": reason is None, "unavailable_reason": reason,
        })
    return out

def _demo_available(kind):
    """kind='patrol'|'harvest'. 스펙 E1-0 형식(current_position 포함). 역할 맞는 로봇만."""
    out = []
    for r in _DEMO_FLEET.values():
        if r["role"] != kind:
            continue
        reason = _demo_reason(r)
        rr = {"robot_id": r["robot_id"], "status": r["status"], "battery_percent": r["battery_percent"],
              "current_position": r["current_position"], "available": reason is None}
        if reason:
            rr["unavailable_reason"] = reason
        out.append(rr)
    return out

def _telemetry_ws_loop():
    import simple_websocket
    while True:
        try:
            wlog("▶ ACS 텔레메트리 WS 접속: %s" % TELEMETRY_WS_URL)
            ws = simple_websocket.Client(TELEMETRY_WS_URL)
            _TELEMETRY["connected"] = True
            wlog("🟢 ACS 텔레메트리 WS 연결됨 — 1Hz 수신 시작")
            while True:
                msg = ws.receive(timeout=5)          # 5초 안에 못 받으면 None
                if msg is None:
                    continue
                try:
                    d = json.loads(msg)
                except Exception:
                    continue
                if d.get("event") == "telemetry":
                    _TELEMETRY["robots"] = (d.get("data") or {}).get("robots", []) or []
                    _TELEMETRY["recv_at"] = time.time()
                    _TELEMETRY["seq"] = d.get("seq")
        except Exception as e:                        # noqa: BLE001 (끊김/거부 → 재접속)
            wlog("⚠ 텔레메트리 WS 끊김/실패, 2초 후 재접속: %s" % e)
        _TELEMETRY["connected"] = False
        time.sleep(2)

def _start_telemetry_ws():
    if TELEMETRY_WS_URL and not _TELEMETRY_STARTED[0]:
        _TELEMETRY_STARTED[0] = True
        threading.Thread(target=_telemetry_ws_loop, daemon=True).start()

@app.get("/api/v1/telemetry")
def fleet_telemetry_v1():
    """E0-5: ACS 텔레메트리 WS로 받은 최신 fleet 상태. 3초 이상 미수신이면 ROBOT_OFFLINE(Web 자체 판정).
       텔레메트리 WS가 설정되지 않은 경우(=라이브 단독 구동)에만 자체 함대(dg_01/02/03)를 발행.
       ACS_TELEMETRY_WS 또는 CONTROL_SERVICE_URL 이 있으면 → 실제 ACS 텔레메트리를 그대로 전달."""
    if not TELEMETRY_WS_URL:
        return jsonify({"event": "telemetry", "connected": True, "seq": None, "age_sec": 0.0,
                        "ws_url": "", "robots": _demo_telemetry_robots()})
    age = (time.time() - _TELEMETRY["recv_at"]) if _TELEMETRY["recv_at"] else None
    stale = (age is None) or (age > ROBOT_OFFLINE_SEC)
    robots = [dict(r) for r in _TELEMETRY["robots"]]
    if stale:                                        # 미수신 → 전부 오프라인 판정
        for r in robots:
            r["available"] = False
            r["unavailable_reason"] = "ROBOT_OFFLINE"
    return jsonify({"event": "telemetry", "connected": bool(_TELEMETRY["connected"] and not stale),
                    "seq": _TELEMETRY["seq"], "age_sec": round(age, 1) if age is not None else None,
                    "ws_url": TELEMETRY_WS_URL, "robots": robots})

# Farm Admin App 실시간 채널.
#   스펙 = WebSocket `/ws/farm-admin` (아래 flask-sock 으로 구현).
#   WebSocket 을 못 쓰는 환경(pythonanywhere 등)에서는 폴링 피드(GET /api/v1/patrol/events)로 자동 폴백.
# 이벤트 payload 필드는 두 경로 모두 스펙과 동일(patrol_progress/patrol_completed/disease_alert).
import queue as _queue
try:
    from flask_sock import Sock
    _sock = Sock(app)
except Exception:                     # flask-sock 없으면 WS 비활성 → 폴링만
    _sock = None
_WS_CLIENTS = set()                   # 연결된 WebSocket 클라이언트별 큐

# ── 워커 간 공유 릴레이 상태 (파일 + 파일락) ─────────────────────────────
#   pythonanywhere는 워커를 여러 개 돌려 '메모리를 공유하지 않음' → 명령큐/이벤트/농장상태/로그를
#   메모리에 두면 워커마다 달라 유실됨. 파일에 두고 fcntl 락으로 원자적 read-modify-write 해야 안전.
import fcntl
RELAY_FILE = os.path.join(BASE, "relay.json")
RELAY_LOCK = os.path.join(BASE, "relay.lock")

def _relay_default():
    return {"cmd_queue": [], "cmd_seq": 0, "task_seq": 1000,
            "farm": {"last_poll": 0, "robots": None, "farm_id": None}, "report_req": False,
            "events": [], "event_seq": 0, "weblog": [], "weblog_seq": 0}

def _relay_read():
    try:
        with open(RELAY_FILE, encoding="utf-8") as f:
            d = json.load(f)
    except Exception:
        d = {}
    base = _relay_default()
    for k, v in base.items():
        d.setdefault(k, v)
    d["farm"] = {**base["farm"], **(d.get("farm") or {})}
    return d

class _relay_tx:
    """모든 워커 간 원자적 read-modify-write (fcntl 파일락). with _relay_tx() as d: d 수정."""
    def __enter__(self):
        self._f = open(RELAY_LOCK, "w")
        fcntl.flock(self._f, fcntl.LOCK_EX)
        self.d = _relay_read()
        return self.d
    def __exit__(self, exc_type, exc, tb):
        try:
            if exc_type is None:
                tmp = RELAY_FILE + ".tmp"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(self.d, f)
                os.replace(tmp, RELAY_FILE)
        finally:
            fcntl.flock(self._f, fcntl.LOCK_UN)
            self._f.close()
        return False

def _push_event(ev):
    ev = dict(ev)
    with _relay_tx() as d:            # 파일락 → 어느 워커가 콜백을 받아도 같은 이벤트 큐에 쌓임
        d["event_seq"] += 1
        ev["seq"] = d["event_seq"]
        d["events"].append(ev)
        d["events"] = d["events"][-300:]
    for q in list(_WS_CLIENTS):       # WebSocket 클라이언트에 즉시 브로드캐스트(있으면)
        try:
            q.put_nowait(ev)
        except Exception:
            pass
    return ev


if _sock:
    @_sock.route("/ws/farm-admin")
    def ws_farm_admin(ws):
        """스펙 E2-10/E3-2: Farm Admin App 실시간 이벤트 WebSocket."""
        q = _queue.Queue(maxsize=200)
        with LOCK:
            _WS_CLIENTS.add(q)
        try:
            ws.send(json.dumps({"event": "connected"}))
            while True:
                try:
                    ev = q.get(timeout=15)
                except _queue.Empty:
                    ws.send(json.dumps({"event": "ping"}))   # keepalive
                    continue
                ws.send(json.dumps(ev))
        except Exception:
            pass
        finally:
            with LOCK:
                _WS_CLIENTS.discard(q)


# ---- 텔레그램 알림 (병해충/순찰완료 시 Web Service가 서버측에서 발송 · 봇토큰 안전) ----
TELEGRAM_FILE = os.path.join(BASE, "telegram.json")

def _telegram_cfg():
    tok = os.environ.get("TELEGRAM_BOT_TOKEN", "")
    cid = os.environ.get("TELEGRAM_CHAT_ID", "")
    try:
        with open(TELEGRAM_FILE, encoding="utf-8") as f:
            j = json.load(f)
        tok = j.get("bot_token") or tok
        cid = j.get("chat_id") or cid
    except Exception:
        pass
    return tok, cid

def _send_telegram(text):
    """설정돼 있으면 텔레그램으로 발송. 미설정/실패해도 조용히 통과(best-effort)."""
    tok, cid = _telegram_cfg()
    if not (tok and cid and _rq):
        return False, "not_configured"
    try:
        r = _rq.post("https://api.telegram.org/bot%s/sendMessage" % tok,
                     json={"chat_id": cid, "text": text, "parse_mode": "HTML",
                           "disable_web_page_preview": True}, timeout=6)
        return (r.status_code == 200), ("HTTP %s" % r.status_code)
    except Exception as e:
        return False, str(e)


# 검출(병해충 레이블) 이미지 위치. ACS가 파일저장하므로 실연동 땐 ACS_IMAGE_BASE_URL 로 원본을 가져와 첨부.
DETECTION_IMG_DIR = os.environ.get("DETECTION_IMG_DIR", os.path.join(BASE, "detections"))
ACS_IMAGE_BASE_URL = os.environ.get("ACS_IMAGE_BASE_URL", "").rstrip("/")

def _resolve_detection_image(image_path, image_url=None):
    """실제 사진 소스 해석: 직접 URL > ACS 이미지서버 URL > Web 로컬 파일. (url, local_path) 반환."""
    if image_url:
        return image_url, None
    if image_path and ACS_IMAGE_BASE_URL:
        return ACS_IMAGE_BASE_URL + "/" + image_path.lstrip("/"), None
    if image_path:
        lp = os.path.join(DETECTION_IMG_DIR, image_path.lstrip("/"))
        if os.path.isfile(lp):
            return None, lp
    return None, None

def _send_telegram_photo(caption, image_path=None, image_url=None, image_data=None):
    """실제 레이블 사진을 sendPhoto 로 첨부. 우선순위: image_data(bytes) > url > 로컬파일 > 텍스트 폴백.
       실연동: ACS(보연)가 로봇 D435로 찍은 레이블 프레임을 image_data(base64)나 image_url 로 실어주면 그대로 첨부됨."""
    tok, cid = _telegram_cfg()
    if not (tok and cid and _rq):
        return False, "not_configured"
    if image_data:                                   # 로봇 카메라가 찍은 실제 이미지 바이트
        try:
            import base64 as _b64, io as _io
            raw = _b64.b64decode(image_data)
            r = _rq.post("https://api.telegram.org/bot%s/sendPhoto" % tok,
                         data={"chat_id": cid, "caption": caption, "parse_mode": "HTML"},
                         files={"photo": ("detection.jpg", _io.BytesIO(raw), "image/jpeg")}, timeout=15)
            return (r.status_code == 200), ("photo_bytes HTTP %s" % r.status_code)
        except Exception as e:
            return False, "photo_bytes_err:%s" % e
    url, local = _resolve_detection_image(image_path, image_url)
    try:
        if url:
            r = _rq.post("https://api.telegram.org/bot%s/sendPhoto" % tok,
                         json={"chat_id": cid, "photo": url, "caption": caption, "parse_mode": "HTML"}, timeout=10)
            return (r.status_code == 200), ("photo_url HTTP %s" % r.status_code)
        if local:
            with open(local, "rb") as f:
                r = _rq.post("https://api.telegram.org/bot%s/sendPhoto" % tok,
                             data={"chat_id": cid, "caption": caption, "parse_mode": "HTML"},
                             files={"photo": ("detection.jpg", f, "image/jpeg")}, timeout=15)
            return (r.status_code == 200), ("photo_file HTTP %s" % r.status_code)
    except Exception as e:
        return False, "photo_err:%s" % e
    return _send_telegram(caption + ("\n사진: " + image_path if image_path else ""))   # 사진 못 찾으면 텍스트

FRAME_FILE = os.path.join(BASE, "latest.jpg")
LIVE_FILE = os.path.join(BASE, "live.json")
CAMURL_FILE = os.path.join(BASE, "camurl.json")   # 노트북이 보고한 부드러운 영상(MJPEG) 터널 URL + 서버시간


def load_live():
    try:
        with open(LIVE_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def load_camurl():
    try:
        with open(CAMURL_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


ROBOTS_FILE = os.path.join(BASE, "robots.json")   # 노트북이 실측한 로봇별 상태(연결·지연·배터리)


def load_robots():
    try:
        with open(ROBOTS_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


CMD_FILE = os.path.join(BASE, "command.json")     # 제어 명령 릴레이 (GUI→서버→노트북→로봇)


def load_cmd():
    try:
        with open(CMD_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {"id": 0, "cmd": "", "status": "idle", "result": "", "ts": 0, "rts": 0}


def save_cmd(c):
    try:
        with open(CMD_FILE, "w", encoding="utf-8") as f:
            json.dump(c, f, ensure_ascii=False)
    except Exception:
        pass


# 접속자 표시용: client id -> 마지막 폴링 시각
PRESENCE = {}
PRESENCE_TTL = 5.0


def count_viewers():
    now = time.time()
    for k in list(PRESENCE.keys()):
        if now - PRESENCE[k] > PRESENCE_TTL:
            del PRESENCE[k]
    return len(PRESENCE)


def with_viewers():
    resp = dict(STATE)
    resp["viewers"] = count_viewers()
    return resp


@app.get("/api/state")
def get_state():
    cid = request.args.get("cid")
    if cid:
        PRESENCE[cid] = time.time()
    return jsonify(with_viewers())


@app.post("/api/action")
def do_action():
    data = request.get_json(force=True, silent=True) or {}
    t = data.get("type")
    payload = data.get("payload") or {}
    with LOCK:
        if t == "toggleMode":
            STATE["mode"] = "night" if STATE["mode"] == "day" else "day"
        elif t == "toggleFollow":
            STATE["follow"] = not STATE["follow"]
        elif t == "approve":
            i = payload.get("i")
            if isinstance(i, int) and 0 <= i < len(STATE["zones"]):
                STATE["zones"][i]["st"] = "수확 진행"
            else:
                return jsonify({"error": "invalid zone index"}), 400
        elif t == "addHarvest":
            try:
                kg = float(payload.get("kg") or 0)
            except (TypeError, ValueError):
                kg = 0
            if kg > 0:
                STATE["todayKg"] = round(STATE["todayKg"] + kg, 1)
        elif t == "reset":
            STATE["mode"] = "day"
            STATE["follow"] = True
            for z, st in zip(STATE["zones"], DEFAULT_ZONES):
                z["st"] = st
            STATE["todayKg"] = 128.6
        else:
            return jsonify({"error": "unknown action"}), 400
        STATE["version"] += 1
        save_state()
    return jsonify(with_viewers())


# ===== 실시간 카메라/텔레메트리 =====
@app.post("/api/ingest")
def ingest():
    """로봇 쪽(노트북)이 압축 프레임 + 텔레메트리를 올린다. 토큰 필요."""
    data = request.get_json(force=True, silent=True) or {}
    if data.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    fr = data.get("frame")
    if fr:
        try:
            raw = base64.b64decode(fr)
            tmp = FRAME_FILE + ".tmp"
            with open(tmp, "wb") as f:
                f.write(raw)
            os.replace(tmp, FRAME_FILE)        # 원자적 교체
        except Exception:
            pass
    meta = data.get("meta") or {}
    meta["server_ts"] = time.time()
    try:
        with open(LIVE_FILE, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False)
    except Exception:
        pass
    return jsonify({"ok": True})


@app.get("/api/frame.jpg")
def frame():
    if os.path.exists(FRAME_FILE):
        resp = send_file(FRAME_FILE, mimetype="image/jpeg")
        resp.headers["Cache-Control"] = "no-store"
        return resp
    return ("", 404)


@app.get("/api/now")
def server_now():
    # 서버 기준 한국시간(KST=UTC+9) + 실제 시간에 따른 주간/야간 자동 판정.
    lt = time.gmtime(time.time() + 9 * 3600)
    mode = "day" if 6 <= lt.tm_hour < 18 else "night"    # 06:00~18:00 주간, 그 외 야간
    return {"time": time.strftime("%Y-%m-%d %H:%M:%S", lt), "hour": lt.tm_hour, "mode": mode}


COMM_SEQ = [0]
COMM_CLIENTS = {}    # cid -> last_seen (통신테스트 클라이언트 추적)


@app.route("/api/echo", methods=["GET", "POST"])
def api_echo():
    """Farm Admin App ↔ Automato Web Service 통신 테스트 에코.
    클라이언트가 보낸 메시지/시퀀스를 서버가 받아 서버시각·서버시퀀스와 함께 되돌려준다."""
    data = request.get_json(force=True, silent=True) or {}
    COMM_SEQ[0] += 1
    now = time.time()
    cid = str(data.get("cid") or request.args.get("cid") or "?")
    COMM_CLIENTS[cid] = now
    active = sum(1 for t in COMM_CLIENTS.values() if now - t < 6)
    return jsonify({
        "ok": True,
        "service": "Automato Web Service",
        "server_seq": COMM_SEQ[0],           # 서버가 처리한 총 요청 수
        "client_seq": data.get("seq"),        # 클라이언트가 보낸 시퀀스(왕복 확인)
        "echo": data.get("msg", ""),          # 받은 메시지 그대로 반향
        "server_time": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(now + 9 * 3600)),
        "recv_epoch": round(now, 3),
        "clients_active": active,             # 최근 6초 내 통신한 클라이언트 수
    })


@app.post("/api/liveurl")
def set_liveurl():
    """노트북(live_smooth.sh)이 현재 MJPEG 터널 URL + 서버시간을 주기적으로 보고."""
    data = request.get_json(force=True, silent=True) or {}
    if data.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    rec = {"url": data.get("url", ""), "time": data.get("time", ""), "ts": time.time()}
    try:
        with open(CAMURL_FILE, "w", encoding="utf-8") as f:
            json.dump(rec, f, ensure_ascii=False)
    except Exception:
        pass
    return jsonify({"ok": True})


@app.post("/api/robots")
def set_robots():
    """노트북이 실측한 로봇별 상태(연결/지연ms/상태/배터리)를 보고."""
    data = request.get_json(force=True, silent=True) or {}
    if data.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    rec = {"list": data.get("list", []), "ts": time.time()}
    try:
        with open(ROBOTS_FILE, "w", encoding="utf-8") as f:
            json.dump(rec, f, ensure_ascii=False)
    except Exception:
        pass
    return jsonify({"ok": True})


# ===== 온디맨드 카메라: 실시간 카메라 페이지를 볼 때만 로봇 D435를 켠다 =====
CAMWANT_FILE = os.path.join(BASE, "camwant.json")   # 마지막 "보고 싶다" 시각(뷰어)
CAMLIVE_FILE = os.path.join(BASE, "camlive.json")   # 노트북이 보고하는 카메라 데몬 상태
WANT_TTL = 8        # 이 초 안에 요청 있으면 "보는 중"


@app.post("/api/camera/want")
def camera_want():
    """실시간 카메라 페이지를 열고 있는 뷰어가 1초마다 보내는 하트비트.
    이게 최근에 왔으면 노트북이 D435 스트림을 켠다(안 오면 자동으로 끔)."""
    try:
        with open(CAMWANT_FILE, "w", encoding="utf-8") as f:
            json.dump({"ts": time.time()}, f)
    except Exception:
        pass
    cam = {}
    try:
        with open(CAMLIVE_FILE, encoding="utf-8") as f:
            cam = json.load(f)
    except Exception:
        pass
    fresh = (time.time() - cam.get("ts", 0)) < 12
    # 뷰어에게: 카메라가 지금 어떤 상태인지 알려줌(켜는 중/켜짐/꺼짐)
    return jsonify({"ok": True, "cam": cam.get("state", "off") if fresh else "off"})


@app.get("/api/camera/state")
def camera_state():
    """노트북 감시 데몬이 폴링: 지금 누가 실시간 카메라를 보고 있나?"""
    want = False
    try:
        with open(CAMWANT_FILE, encoding="utf-8") as f:
            want = (time.time() - json.load(f).get("ts", 0)) < WANT_TTL
    except Exception:
        pass
    return jsonify({"want": bool(want)})


@app.post("/api/camera/report")
def camera_report():
    """노트북 데몬이 카메라 상태(off/starting/on)를 서버에 보고 → 뷰어 화면에 표시."""
    data = request.get_json(force=True, silent=True) or {}
    if data.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    try:
        with open(CAMLIVE_FILE, "w", encoding="utf-8") as f:
            json.dump({"state": data.get("state", "off"), "ts": time.time()}, f)
    except Exception:
        pass
    return jsonify({"ok": True})


OPERATION = {"state": "idle", "started_ts": 0}    # 전체 운영 상태 (RP-53)


@app.route("/api/v1/operation/start", methods=["POST", "GET"])
def operation_start():
    """[Automato Web Service] 운영 시작 API (Jira RP-53).
    관리자가 전체 시스템 가동을 트리거하는 HTTP 진입점.
      요청:  POST /api/v1/operation/start   Body(JSON, 선택): {"by": "<관리자>"}
      처리:  '운영 시작' 로그 출력 → 후속 중계(명령 큐)로 시작 신호 전달
      응답:  200 { ok, operation, message, state, relayed_cmd_id, server_time, ts }
    """
    data = request.get_json(force=True, silent=True) or {}
    by = str(data.get("by") or request.args.get("by") or "admin")
    print("운영 시작 (operation start) — requested by %s" % by, flush=True)  # 완료조건: 로그
    OPERATION["state"] = "running"
    OPERATION["started_ts"] = time.time()
    # 후속 중계 단계: 시작 신호를 명령 큐에 넣어 다음 단계(Control Service/브리지→ROS2)로 넘김
    c = load_cmd()
    c["id"] = c.get("id", 0) + 1
    c.update({"cmd": "mode:operation", "status": "pending", "result": "", "ts": time.time()})
    save_cmd(c)
    return jsonify({
        "ok": True,
        "operation": "start",
        "message": "운영 시작",
        "state": OPERATION["state"],
        "relayed_cmd_id": c["id"],
        "requested_by": by,
        "server_time": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(time.time() + 9 * 3600)),
        "ts": round(time.time(), 3),
    }), 200


@app.get("/api/v1/operation/status")
def operation_status():
    """운영 상태 조회(GET — 브라우저로 확인용)."""
    return jsonify({"state": OPERATION["state"], "started_ts": OPERATION["started_ts"]})


@app.post("/api/command")
def post_command():
    """GUI(핸드폰)에서 로봇 제어 명령 전송."""
    data = request.get_json(force=True, silent=True) or {}
    cmd = (data.get("cmd") or "").strip()
    if not cmd:
        return jsonify({"error": "no cmd"}), 400
    c = load_cmd()
    c["id"] = c.get("id", 0) + 1
    c.update({"cmd": cmd, "status": "pending", "result": "", "ts": time.time()})
    save_cmd(c)
    return jsonify({"ok": True, "id": c["id"]})


@app.get("/api/cmd_next")
def cmd_next():
    """노트북 실행기가 대기중 명령을 가져감(토큰 필요)."""
    if request.args.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    c = load_cmd()
    if c.get("status") == "pending":
        c["status"] = "running"
        save_cmd(c)
        return jsonify({"id": c["id"], "cmd": c["cmd"]})
    return jsonify({"id": 0, "cmd": ""})


@app.post("/api/cmd_result")
def cmd_result():
    """노트북이 실행 결과 보고(토큰 필요)."""
    data = request.get_json(force=True, silent=True) or {}
    if data.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    c = load_cmd()
    if c.get("id") == data.get("id"):
        c.update({"status": "done", "result": str(data.get("result", ""))[:200], "rts": time.time()})
        save_cmd(c)
    return jsonify({"ok": True})


@app.get("/api/cmd_status")
def cmd_status():
    """GUI가 마지막 명령/결과를 폴링."""
    c = load_cmd()
    c["age"] = round(time.time() - c.get("ts", 0), 1)
    return jsonify(c)


@app.get("/api/live")
def live():
    cid = request.args.get("cid")
    if cid:
        PRESENCE[cid] = time.time()
    d = load_live()
    age = time.time() - d.get("server_ts", 0) if d else 1e9
    d["online"] = age < 10          # 10초 내 갱신 있으면 ON
    d["age"] = round(age, 1)
    d["viewers"] = count_viewers()
    cam = load_camurl()                              # 부드러운 영상(MJPEG) URL + 서버시간
    cam_fresh = (time.time() - cam.get("ts", 0)) < 45
    d["mjpeg_url"] = cam.get("url", "") if cam_fresh else ""
    d["srv_time"] = cam.get("time", "") if cam_fresh else ""
    rb = load_robots()                               # 실측 로봇 상태
    d["robots_fresh"] = (time.time() - rb.get("ts", 0)) < 20
    d["robots"] = rb.get("list", []) if d["robots_fresh"] else []
    return jsonify(d)


LIVE_HTML = """<!DOCTYPE html><html lang=ko><head><meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>Automato 실시간 모니터</title><style>
body{margin:0;font-family:'Pretendard','Malgun Gothic',system-ui,sans-serif;background:#0f1713;color:#eaf2ec}
.wrap{max-width:1000px;margin:0 auto;padding:16px}
.top{display:flex;align-items:center;gap:10px;margin-bottom:12px;flex-wrap:wrap}
.dot{width:11px;height:11px;border-radius:50%;background:#888}.on{background:#34d27b;box-shadow:0 0 8px #34d27b}.off{background:#e2483a}
h1{font-size:18px;margin:0}.muted{color:#8fa89b;font-size:13px}
.cam{position:relative;background:#000;border-radius:12px;overflow:hidden;aspect-ratio:16/10}
.cam img{width:100%;height:100%;object-fit:contain;display:block}
.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:10px;margin-top:12px}
.card{background:#16241d;border:1px solid #25382e;border-radius:12px;padding:12px 14px}
.card .k{font-size:12px;color:#8fa89b}.card .v{font-size:22px;font-weight:800;margin-top:3px}
.badge{margin-left:auto;font-size:12px;color:#8fa89b}
.noimg{display:grid;place-items:center;height:100%;color:#6d8077;font-size:14px}
</style></head><body><div class=wrap>
<div class=top><span id=dot class=dot></span><h1>🍅 Automato 실시간 모니터</h1>
<span id=stat class=muted>연결 확인 중…</span><span id=viewers class=badge></span></div>
<div class=cam><img id=cam alt="" onerror="this.style.display='none';document.getElementById('noimg').style.display='grid'">
<div id=noimg class=noimg style=display:none>카메라 신호 없음 (로봇 스트리머 꺼짐)</div></div>
<div class=cards>
<div class=card><div class=k>검출 토마토</div><div class=v id=count>–</div></div>
<div class=card><div class=k>익음/덜익음</div><div class=v id=ripe>–</div></div>
<div class=card><div class=k>폐기/병충해</div><div class=v id=bad>–</div></div>
<div class=card><div class=k>최근접 거리</div><div class=v id=dist>–</div></div>
<div class=card><div class=k>스트리머 FPS</div><div class=v id=fps>–</div></div>
<div class=card><div class=k>마지막 갱신</div><div class=v id=age>–</div></div>
</div>
<p class=muted style=margin-top:14px>이 화면은 로봇 노트북이 클라우드로 올린 압축 영상을 보는 것입니다. (학원 와이파이 부하 없음 · 접속자 수 무관)</p>
</div><script>
var cid="v"+Math.random().toString(36).slice(2);
var _buf=new Image();_buf.onload=function(){var i=document.getElementById('cam');if(i){i.style.display='block';i.src=_buf.src;}};
function refreshCam(){if(_buf.complete||!_buf.src)_buf.src='/api/frame.jpg?t='+Date.now();}
function poll(){fetch('/api/live?cid='+cid).then(r=>r.json()).then(d=>{
 var on=d.online;document.getElementById('dot').className='dot '+(on?'on':'off');
 document.getElementById('stat').textContent=on?'실시간 연결됨':'오프라인 ('+(d.age||'?')+'s 전 마지막 신호)';
 document.getElementById('viewers').textContent='👥 '+(d.viewers||1)+'명 보는 중';
 var c=d.classes||{};
 document.getElementById('count').textContent=(d.count!=null?d.count:'–');
 document.getElementById('ripe').textContent=((c.ripe||0)+'/'+(c.unripe||0));
 document.getElementById('bad').textContent=((c.rotten||0)+'/'+(c.disease||0));
 document.getElementById('dist').textContent=(d.nearest_mm!=null?Math.round(d.nearest_mm)+'mm':'–');
 document.getElementById('fps').textContent=(d.fps!=null?d.fps:'–');
 document.getElementById('age').textContent=(d.age!=null?d.age+'s 전':'–');
}).catch(e=>{document.getElementById('stat').textContent='서버 응답 없음';});}
refreshCam();setInterval(refreshCam,130);poll();setInterval(poll,1000);
</script></body></html>"""


@app.get("/live")
def live_page():
    return Response(LIVE_HTML, mimetype="text/html")


# ===== 농장 작업 협업 보드 (작업자들이 서로의 업무를 공유) =====
FARMTASK_FILE = os.path.join(BASE, "farmtasks.json")


def load_farmtasks():
    try:
        with open(FARMTASK_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {"seq": 0, "tasks": []}


def save_farmtasks(d):
    try:
        with open(FARMTASK_FILE, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False)
    except Exception:
        pass


@app.get("/api/farmtasks")
def farmtasks_list():
    return jsonify(load_farmtasks())


@app.post("/api/farmtasks/add")
def farmtasks_add():
    """작업자가 새 농장 업무 등록. {title, zone, worker, ordered_by}"""
    data = request.get_json(force=True, silent=True) or {}
    title = (data.get("title") or "").strip()
    if not title:
        return jsonify({"error": "no title"}), 400
    with LOCK:
        d = load_farmtasks()
        d["seq"] = d.get("seq", 0) + 1
        d["tasks"].append({
            "id": d["seq"], "title": title[:120],
            "zone": (data.get("zone") or "").strip()[:30],
            "worker": (data.get("worker") or "").strip()[:20],
            "ordered_by": (data.get("ordered_by") or "").strip()[:20],
            "status": "todo", "created": time.time(), "updated": time.time(),
        })
        save_farmtasks(d)
    return jsonify({"ok": True, "id": d["seq"]})


@app.post("/api/farmtasks/move")
def farmtasks_move():
    """상태 변경. {id, status: todo|doing|done}"""
    data = request.get_json(force=True, silent=True) or {}
    tid, st = data.get("id"), data.get("status")
    if st not in ("todo", "doing", "done"):
        return jsonify({"error": "bad status"}), 400
    with LOCK:
        d = load_farmtasks()
        for t in d["tasks"]:
            if t["id"] == tid:
                t["status"] = st
                t["updated"] = time.time()
                break
        save_farmtasks(d)
    return jsonify({"ok": True})


@app.post("/api/farmtasks/delete")
def farmtasks_delete():
    data = request.get_json(force=True, silent=True) or {}
    tid = data.get("id")
    with LOCK:
        d = load_farmtasks()
        d["tasks"] = [t for t in d["tasks"] if t["id"] != tid]
        save_farmtasks(d)
    return jsonify({"ok": True})


# ===== E1 순찰 로봇 배정 (Farm Admin App / Automato Web Service · RP-66) =====
PATROL_FILE = os.path.join(BASE, "patrol.json")
MIN_BAT_PATROL = 70
PATROL_DEFAULT = {"seq": 1000, "robots": [
    {"robot_id": "DG1", "robot_type": "DDAGO", "compose": "arm+pinky", "status": "IDLE", "battery_percent": 85.2, "pos": "A-03"},
    {"robot_id": "DG2", "robot_type": "DDAGO", "compose": "arm+pinky", "status": "IDLE", "battery_percent": 74.0, "pos": "B-01"},
    {"robot_id": "DG3", "robot_type": "DDAGO", "compose": "pinky", "status": "IDLE", "battery_percent": 62.0, "pos": "C-05"},
], "tasks": []}


def load_patrol():
    try:
        with open(PATROL_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return json.loads(json.dumps(PATROL_DEFAULT))


def save_patrol(d):
    try:
        with open(PATROL_FILE, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False)
    except Exception:
        pass


def _avail_reason(r):
    if r["status"] != "IDLE":
        return False, "ROBOT_BUSY"
    if r["battery_percent"] < MIN_BAT_PATROL:
        return False, "BATTERY_TOO_LOW"
    return True, None


# ============================================================================
#  원격 농장 릴레이 — edge(농장 ACS)가 아웃바운드로 붙는 방식
#  농장은 공개주소·인바운드가 없어도(NAT 뒤) 이 공개 Web으로 "먼저 연결(폴링)"만 하면
#  ① 브라우저가 낸 명령을 가져가 실행하고 ② 결과/알람을 되쏠 수 있다.
#  Web은 바깥으로 전화하지 않으므로(무료 호스팅 OK) 전국·해외 어디 농장이든 지휘 가능.
# ============================================================================
FARM_ONLINE_WINDOW = 15.0    # 최근 이 시간(초) 내 폴링이 있으면 '농장 온라인'

def _farm_online(d=None):
    if d is None:
        d = _relay_read()
    return (time.time() - (d.get("farm", {}).get("last_poll") or 0)) < FARM_ONLINE_WINDOW

@app.post("/internal/v1/farm/poll")
def farm_poll():
    """원격 농장 ACS가 주기적으로 호출(아웃바운드). 살아있음 표시 + 로봇상태 보고 + 다음 명령 1건 수령."""
    j = request.get_json(force=True, silent=True) or {}
    with _relay_tx() as d:
        first = (time.time() - (d["farm"].get("last_poll") or 0)) >= FARM_ONLINE_WINDOW
        d["farm"]["last_poll"] = time.time()
        if j.get("farm_id"):
            d["farm"]["farm_id"] = j.get("farm_id")
        if j.get("robots") is not None:
            d["farm"]["robots"] = j.get("robots")
        cmd = d["cmd_queue"].pop(0) if d["cmd_queue"] else None
        report_now = d["report_req"]
        d["report_req"] = False
        farm_id = d["farm"].get("farm_id")
    if first:
        wlog("🟢 원격 농장 접속: farm_id=%s (아웃바운드 폴링 시작)" % farm_id)
    if cmd:
        wlog("◀ 농장 폴링 → 명령 전달: %s task_id=%s (농장이 실행)" % (cmd.get("type"), cmd.get("task_id")))
    return jsonify({"command": cmd, "report_now": report_now})

@app.get("/api/v1/farm/status")
def farm_status():
    """웹앱/운영자용: 원격 농장 연결 상태."""
    d = _relay_read()
    return jsonify({"farm_online": _farm_online(d), "farm_id": d["farm"].get("farm_id"),
                    "queued": len(d["cmd_queue"]),
                    "last_poll_ago_sec": round(time.time() - (d["farm"].get("last_poll") or 0), 1)})

@app.get("/api/v1/weblog")
def weblog():
    """웹서비스 로그를 즉시 tail (파일 공유 버퍼). since=<seq> 이후만 반환."""
    try:
        since = int(request.args.get("since", "0"))
    except (TypeError, ValueError):
        since = 0
    d = _relay_read()
    out = [x for x in d["weblog"] if x["seq"] > since]
    return jsonify({"lines": out, "last": d["weblog_seq"]})


@app.get("/api/v1/robots/patrol/available")
def patrol_available():
    """순찰 가능한 로봇 목록. (E1-0) Control 연동 시 /internal/v1/robots/patrol/available 로 중계."""
    if _control_on():
        try:
            wlog("▶ App 요청: 순찰 가능 로봇 조회 → ACS 중계 GET /internal/v1/robots/patrol/available")
            r = _control_get("/internal/v1/robots/patrol/available")
            js = r.json()
            wlog("◀ ACS 응답:", [(x.get("robot_id"), "가능" if x.get("available") else x.get("unavailable_reason"))
                                 for x in js.get("robots", [])], "→ App 반환")
            return (jsonify(js), r.status_code)
        except Exception as e:
            wlog("⚠ ACS 조회 실패, 로컬 폴백:", e)
            app.logger.warning("control available 실패, 로컬 폴백: %s", e)
    fd = _relay_read()
    if _farm_online(fd) and fd["farm"].get("robots"):   # 원격 농장이 보고한 실제 로봇 상태
        robots = [dict(r) for r in fd["farm"]["robots"]]
        if any(r.get("status") == "PATROLLING" for r in robots):   # 순찰 1대만 → 나머지도 배정 불가 표시
            for r in robots:
                if r.get("available"):
                    r["available"] = False
                    r["unavailable_reason"] = "ALREADY_PATROLLING"
        wlog("◀ App 요청: 순찰 가능 로봇 조회 → 농장에 상태 요청 + 최신값 App 반환",
             [(r["robot_id"], "가능" if r.get("available") else r.get("unavailable_reason")) for r in robots])
        with _relay_tx() as d:    # 다음 농장 폴링 때 컨트롤이 '상태 보고'를 찍도록 신호 → 버튼A에 컨트롤도 반응
            d["report_req"] = True
        return jsonify({"min_battery_percent": MIN_BAT_PATROL, "robots": robots, "farm_online": True,
                        "available_count": sum(1 for r in robots if r.get("available"))})
    # ACS·원격농장 없음(라이브 단독 구동) → 자체 함대(dg_01/02/03)로 스펙 E1-0 형식 반환
    robots = _demo_available("patrol")
    return jsonify({"requested_at": _sim_now(), "min_battery_percent": MIN_BAT_PATROL,
                    "robots": robots, "available_count": sum(1 for r in robots if r["available"])})


@app.post("/api/v1/patrol/requests")
def patrol_request():
    """순찰 요청. {robot_selection: auto|manual, robot_id?, mode?}
       Control 연동 시 /internal/v1/tasks/patrol 로 중계. 아니면 로컬 데모. (E1-2/3)"""
    data = request.get_json(force=True, silent=True) or {}
    sel = data.get("robot_selection", "auto")
    if _control_on():
        try:
            wlog("▶ App 요청: 순찰 요청(robot_selection=%s robot_id=%s) → ACS 중계 POST /internal/v1/tasks/patrol"
                 % (sel, data.get("robot_id")))
            r = _control_post("/internal/v1/tasks/patrol", data)
            js = r.json()
            wlog("◀ ACS 응답:", js.get("status"),
                 ("task_id=%s robot=%s" % (js.get("task_id"), js.get("assigned_robot_id")))
                 if js.get("status") == "ACCEPTED" else ("reason=%s" % js.get("reason")), "→ App 반환")
            return (jsonify(js), r.status_code)   # ACCEPTED(200) / REJECTED(409) 그대로 전달
        except Exception as e:
            wlog("⚠ ACS 요청 실패, 로컬 폴백:", e)
            app.logger.warning("control tasks/patrol 실패, 로컬 폴백: %s", e)
    fd = _relay_read()
    if _farm_online(fd):                            # 원격 농장 연결됨 → 농장 보고 상태로 즉시 수락/거절, 수락분은 큐로
        robots = fd["farm"].get("robots") or []
        wlog("▶ App 요청: 순찰 요청(robot_selection=%s robot_id=%s)" % (sel, data.get("robot_id")))
        if any(r.get("status") == "PATROLLING" for r in robots):     # 순찰은 한 번에 1대만
            wlog("  → 거절 ALREADY_PATROLLING (이미 순찰 중) → App 409")
            return jsonify({"status": "REJECTED", "reason": "ALREADY_PATROLLING",
                            "message": "이미 순찰 중인 로봇이 있습니다. 순찰은 한 번에 1대만 가능합니다."}), 409
        if sel == "manual":
            rr = next((r for r in robots if r.get("robot_id") == data.get("robot_id")), None)
            if not rr or not rr.get("available"):
                wlog("  → 거절 ROBOT_NOT_AVAILABLE (%s 불가) → App 409" % data.get("robot_id"))
                return jsonify({"status": "REJECTED", "reason": "ROBOT_NOT_AVAILABLE",
                                "message": "선택한 로봇을 지금 쓸 수 없습니다."}), 409
        elif not any(r.get("available") for r in robots):
            wlog("  → 거절 NO_AVAILABLE_ROBOT (가능 로봇 없음) → App 409")
            return jsonify({"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                            "message": "순찰 가능한 로봇이 없습니다."}), 409
        with _relay_tx() as d:                      # 파일 큐에 명령 적재 → 어느 워커가 폴링받아도 유실 없이 전달
            d["task_seq"] += 1
            task_id = d["task_seq"]
            d["cmd_seq"] += 1
            d["cmd_queue"].append({"cmd_id": d["cmd_seq"], "type": "patrol", "task_id": task_id,
                                   "robot_selection": sel, "robot_id": data.get("robot_id")})
        wlog("▶ 명령 큐 적재: patrol (task_id=%s) — 원격 농장이 가져가길 대기" % task_id)
        return jsonify({"task_id": task_id, "assigned_robot_id": None, "status": "ACCEPTED",
                        "mode": "remote_farm",
                        "message": "원격 농장으로 순찰 명령을 전송했습니다. 농장에서 로봇을 배정합니다."})
    # ACS·원격농장 없음(라이브 단독 구동) → 자체 함대(dg_03=순찰)로 처리. 스펙 E1 즉시요청.
    with LOCK:
        if any(r["status"] == "PATROLLING" for r in _DEMO_FLEET.values()):   # 순찰은 전역 1대만
            return jsonify({"status": "REJECTED", "reason": "ALREADY_PATROLLING",
                            "message": "이미 순찰 중인 로봇이 있습니다. 순찰은 한 번에 1대만 가능합니다."}), 409
        avail = [r for r in _DEMO_FLEET.values() if r["role"] == "patrol" and _demo_reason(r) is None]
        if not avail:
            return jsonify({"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                            "message": "순찰 가능한 로봇이 없습니다."}), 409
        if sel == "manual":
            rid = data.get("robot_id")
            chosen = next((r for r in avail if r["robot_id"] == rid), None)
            if not chosen:
                return jsonify({"status": "REJECTED", "reason": "ROBOT_NOT_AVAILABLE",
                                "message": "선택한 로봇을 지금 쓸 수 없습니다."}), 409
        else:                                                    # auto: 배터리 최고, 동률 robot_id 오름차순
            chosen = max(avail, key=lambda r: (r["battery_percent"], [-ord(c) for c in r["robot_id"]]))
        _DEMO_SEQ[0] += 1
        task_id = _DEMO_SEQ[0]
        chosen["status"] = "PATROLLING"
    _evolve_heat()          # 순찰 나갈 때마다 카메라가 새로 스캔 → 밀집 히트맵·작물 상태 갱신
    # Web이 ACS 역할까지 겸해 순찰 시뮬을 스스로 돌린다 → 버튼 한 번으로 E2(진행)·E3(병해충)·완료 이벤트 흐름
    threading.Thread(target=_simulate_local_patrol,
                     args=(task_id, chosen["robot_id"]), daemon=True).start()
    return jsonify({"task_id": task_id, "assigned_robot_id": chosen["robot_id"],
                    "status": "ACCEPTED", "message": "순찰 요청이 접수되었습니다."})


@app.post("/api/v1/patrol/reset")
def patrol_reset():
    """데모용: 로봇 상태 + 히트맵 + 수확 실적을 초기값으로 되돌림."""
    for r in _DEMO_FLEET.values():                  # 자체 함대 전부 대기 복귀
        r["status"] = "IDLE"
    save_patrol(json.loads(json.dumps(PATROL_DEFAULT)))
    save_heat(json.loads(json.dumps(HEAT_DEFAULT)))
    save_harvest_stats(harvest_default())
    with _relay_tx() as d:
        d["events"] = []
        d["cmd_queue"] = []
        d["report_req"] = False
    return jsonify({"ok": True})


# ============================================================================
#  Control Service(ACS) → Web Service 내부 콜백 (E2/E3) + Web → App 이벤트 피드
#  개정 스펙(2026-07): 검출은 퍼센트(ripe/unripe/rotten/disease_percent), 병해충 disease_percent>=5
# ============================================================================
@app.post("/internal/v1/detections/notify")
def internal_detections_notify():
    """E2-9: ACS가 waypoint마다 순찰 현황(퍼센트) 전달 → App 으로 patrol_progress 푸시."""
    d = request.get_json(force=True, silent=True) or {}
    wlog("◀ ACS 콜백: 검출현황 WP%s (익음%s/안익음%s/썩음%s/병해충%s%%) → App patrol_progress 푸시" % (
        d.get("waypoint_id"), d.get("ripe_percent"), d.get("unripe_percent"),
        d.get("rotten_percent"), d.get("disease_percent")))
    _push_event({"event": "patrol_progress",
                 "task_id": d.get("task_id"), "waypoint_id": d.get("waypoint_id"),
                 "robot_id": d.get("robot_id"),
                 "ripe_percent": d.get("ripe_percent"), "unripe_percent": d.get("unripe_percent"),
                 "rotten_percent": d.get("rotten_percent"), "disease_percent": d.get("disease_percent"),
                 "detected_at": d.get("detected_at")})
    return jsonify({"success": True})


@app.post("/internal/v1/patrol/completed")
def internal_patrol_completed():
    """마지막 waypoint 완료 → App 으로 patrol_completed 푸시 + 로봇 IDLE 복귀(데모 상태 동기화)."""
    d = request.get_json(force=True, silent=True) or {}
    wlog("◀ ACS 콜백: 순찰 완료 task_id=%s robot=%s → App patrol_completed 푸시 + 텔레그램" % (
        d.get("task_id"), d.get("robot_id")))
    ev = _push_event({"event": "patrol_completed", "task_id": d.get("task_id"),
                      "robot_id": d.get("robot_id"),
                      "status": d.get("status", "COMPLETED"),                      # 스펙 E2-9-1/10
                      "unvisited_waypoint_ids": d.get("unvisited_waypoint_ids", []),
                      "completed_at": d.get("completed_at"),
                      "summary": d.get("summary")})
    try:
        with LOCK:
            pd = load_patrol()
            for r in pd["robots"]:
                if r["status"] == "PATROLLING":
                    r["status"] = "IDLE"
            save_patrol(pd)
    except Exception:
        pass
    _schedule_telegram_fallback(ev)   # App 이 열려있으면 App 이 발송(스펙), 닫혀있으면 서버가 대신 발송
    return jsonify({"success": True})


@app.post("/internal/v1/alerts/disease")
def internal_alerts_disease():
    """E3-1: ACS가 disease_percent>=5 확인 후 알림 전달 → App 으로 disease_alert 푸시(사진경로 포함)."""
    d = request.get_json(force=True, silent=True) or {}
    wlog("◀ ACS 콜백: 병해충 알림 WP%s %s%% (≥5) → App disease_alert 푸시 + 텔레그램" % (
        d.get("waypoint_id"), d.get("disease_percent")))
    ev_disease = _push_event({"event": "disease_alert",
                 "task_id": d.get("task_id"), "waypoint_id": d.get("waypoint_id"),
                 "robot_id": d.get("robot_id"), "disease_percent": d.get("disease_percent"),
                 "image_path": d.get("image_path"), "detected_at": d.get("detected_at")})
    # ACS가 로봇 카메라 실제 이미지를 image_data(base64)로 실어주면 파일로 저장 → 웹앱/알림이 /detections 로 표시
    ev_disease["image_url"] = d.get("image_url")
    ev_disease["image_data"] = d.get("image_data")
    if d.get("image_data") and d.get("image_path"):
        try:
            lp = os.path.join(DETECTION_IMG_DIR, d["image_path"].lstrip("/"))
            os.makedirs(os.path.dirname(lp), exist_ok=True)
            with open(lp, "wb") as f:
                f.write(base64.b64decode(d["image_data"]))
        except Exception:
            pass
    _schedule_telegram_fallback(ev_disease)   # App 열려있으면 App 발송(스펙), 닫혀있으면 서버 대신 발송
    return jsonify({"success": True})


@app.post("/internal/v1/alerts/task-failed")
def internal_alerts_task_failed():
    """E2-13: ACS가 작업 실패 시 전달 → App 으로 task_failed 푸시(+텔레그램).
       reason  : BLOCKED / BLOCKED_UNRECOVERABLE / DOCK_FAILED / BATTERY_DEPLETED / HARDWARE_ERROR
       recovery_action : RETURN_TO_CHARGER / NONE"""
    d = request.get_json(force=True, silent=True) or {}
    wlog("◀ ACS 콜백: 작업 실패 task_id=%s robot=%s reason=%s → App task_failed 푸시 + 텔레그램" % (
        d.get("task_id"), d.get("robot_id"), d.get("reason")))
    ev = _push_event({"event": "task_failed",
                      "task_id": d.get("task_id"), "robot_id": d.get("robot_id"),
                      "task_type": d.get("task_type"), "reason": d.get("reason"),
                      "recovery_action": d.get("recovery_action"),
                      "message": d.get("message"), "failed_at": d.get("failed_at")})
    _schedule_telegram_fallback(ev)   # App 열려있으면 App 발송(스펙), 닫혀있으면 서버 대신 발송
    return jsonify({"success": True})


# ---- 텔레그램 발송: App-driven(스펙) + 서버 폴백(웹앱 닫힌 경우) ----
# 스펙 "앱이 알림 발송" 과 "웹앱 닫혀도 텔레그램 받기" 의 모순 해결:
#   App 이 열려있으면 App 이 /api/v1/notify/telegram 로 발송(스펙), 그 사이 안 오면 서버가 대신 발송.
#   seq 로 이중발송 방지(누가 먼저 보내든 1회).
_TG_SENT_SEQ = set()

def _tg_claim(seq):
    """seq 발송권 선점. 이미 처리됐으면 False."""
    with LOCK:
        if seq is not None and seq in _TG_SENT_SEQ:
            return False
        if seq is not None:
            _TG_SENT_SEQ.add(seq)
            if len(_TG_SENT_SEQ) > 2000:
                _TG_SENT_SEQ.clear()
    return True

def _send_event_telegram(ev):
    if ev.get("event") == "disease_alert":
        caption = ("🐛 <b>병해충 발견</b>\n로봇: %s · WP%s · 작업 #%s\n병해충 <b>%s%%</b>" % (
            ev.get("robot_id"), ev.get("waypoint_id"), ev.get("task_id"), ev.get("disease_percent")))
        return _send_telegram_photo(caption, image_path=ev.get("image_path"),
                                    image_url=ev.get("image_url"), image_data=ev.get("image_data"))
    if ev.get("event") == "patrol_completed":
        s = ev.get("summary") or {}
        return _send_telegram(
            "✅ <b>순찰 완료</b>\n로봇: %s · 작업 #%s\n익음 %s%% · 안익음 %s%% · 썩음 %s%% · 병해충 %s%%" % (
                ev.get("robot_id"), ev.get("task_id"), s.get("ripe_percent", 0), s.get("unripe_percent", 0),
                s.get("rotten_percent", 0), s.get("disease_percent", 0)))
    if ev.get("event") == "task_failed":
        _RE = {"BLOCKED": "⚠️ 통로 막힘 → 충전소로 복귀합니다",
               "BLOCKED_UNRECOVERABLE": "🚨 로봇이 갇혔습니다 — 직접 옮긴 뒤 '복구 완료'를 눌러 주세요",
               "DOCK_FAILED": "⚠️ 정밀 정차 실패 — 마커를 확인해 주세요",
               "BATTERY_DEPLETED": "🔋 작업 중 배터리 소진",
               "HARDWARE_ERROR": "🛑 로봇 하드웨어 이상"}
        return _send_telegram("🛑 <b>작업 실패</b>\n로봇: %s · 작업 #%s (%s)\n%s\n%s" % (
            ev.get("robot_id"), ev.get("task_id"), ev.get("task_type") or "",
            _RE.get(ev.get("reason"), ev.get("reason") or ""), ev.get("message") or ""))
    return False, "ignored"

def _telegram_fallback(ev):
    if _tg_claim(ev.get("seq")):     # App 이 아직 안 보냈으면 서버가 발송
        _send_event_telegram(ev)

def _schedule_telegram_fallback(ev, delay=6.0):
    try:
        threading.Timer(delay, _telegram_fallback, args=[ev]).start()
    except Exception:
        pass


# ---- 라이브 자체 순찰 시뮬 (외부 ACS 없을 때 Web이 ACS 역할까지 겸함) ----
# wp2=disease 4%(스킵) / wp3=disease 7%(알림) → E2/E3 두 분기 모두 커버.
# ACS 연동(CONTROL_SERVICE_URL) 시엔 이 함수가 아니라 진짜 ACS 콜백이 이벤트를 만든다.
_SIM_PLAN = [
    {"ripe": 50, "unripe": 50, "rotten": 0, "disease": 0},
    {"ripe": 70, "unripe": 28, "rotten": 2, "disease": 0},
    {"ripe": 60, "unripe": 33, "rotten": 3, "disease": 4},   # 4% < 5 → 알림 스킵
    {"ripe": 40, "unripe": 45, "rotten": 8, "disease": 7},   # 7% >= 5 → 병해충 알림
]

def _sim_now():
    return time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime())

def _simulate_local_patrol(task_id, robot_id, step=1.2):
    """ACS 없이 Web이 순찰을 스스로 진행: waypoint별 검출현황 → (조건 시)병해충 → 완료."""
    last = _SIM_PLAN[-1]
    for wp, p in enumerate(_SIM_PLAN):
        time.sleep(step)
        now = _sim_now()
        _push_event({"event": "patrol_progress", "task_id": task_id, "waypoint_id": wp,
                     "robot_id": robot_id, "ripe_percent": p["ripe"], "unripe_percent": p["unripe"],
                     "rotten_percent": p["rotten"], "disease_percent": p["disease"], "detected_at": now})
        if p["disease"] >= 5:                       # E3 트리거 (ACS 판단분을 여기선 Web이 대행)
            ev = _push_event({"event": "disease_alert", "task_id": task_id, "waypoint_id": wp,
                              "robot_id": robot_id, "disease_percent": p["disease"],
                              "image_path": None, "detected_at": now})
            _schedule_telegram_fallback(ev)
    now = _sim_now()
    ev = _push_event({"event": "patrol_completed", "task_id": task_id, "robot_id": robot_id,
                      "status": "COMPLETED", "unvisited_waypoint_ids": [],   # 스펙 E2-9-1/10
                      "completed_at": now,
                      "summary": {"ripe_percent": last["ripe"], "unripe_percent": last["unripe"],
                                  "rotten_percent": last["rotten"], "disease_percent": last["disease"]}})
    _demo_set_status(robot_id, "IDLE")              # 자체 함대: 순찰 끝 → 대기 복귀 (텔레메트리 동기화)
    try:                                            # (구 로컬 모델도 함께 되돌림 — 무해)
        with LOCK:
            pd = load_patrol()
            for r in pd["robots"]:
                if r["status"] == "PATROLLING":
                    r["status"] = "IDLE"
            save_patrol(pd)
    except Exception:
        pass
    _schedule_telegram_fallback(ev)

@app.post("/api/v1/notify/telegram")
def notify_telegram():
    """스펙 E2-10/E3-2: 앱이 disease_alert/patrol_completed 수신 시 텔레그램 발송(릴레이)."""
    d = request.get_json(force=True, silent=True) or {}
    if d.get("event") not in ("disease_alert", "patrol_completed", "task_failed"):
        return jsonify({"sent": False, "reason": "ignored"})
    if not _tg_claim(d.get("seq")):          # 다른 클라이언트/서버폴백이 이미 보냄
        return jsonify({"sent": False, "reason": "dup"})
    ok, info = _send_event_telegram(d)
    return jsonify({"sent": ok, "info": info})


@app.get("/api/v1/patrol/events")
def patrol_events():
    """E2-10/E3-2 대체: Farm Admin App 실시간 피드(폴링). ?since=<seq> 이후 이벤트만 반환."""
    since = request.args.get("since", type=int) or 0
    d = _relay_read()
    evs = [e for e in d["events"] if e.get("seq", 0) > since]
    return jsonify({"events": evs, "last_seq": d["event_seq"]})


@app.get("/detections/<path:p>")
def serve_detection_image(p):
    """병해충 레이블 검출 이미지 서빙(웹앱 배너·알림에서 실제 사진 표시). ACS_IMAGE_BASE_URL 미사용 시 Web 로컬."""
    try:
        return send_from_directory(DETECTION_IMG_DIR, p)
    except Exception:
        return jsonify({"error": "not_found"}), 404


@app.route("/api/v1/telegram/config", methods=["GET", "POST"])
def telegram_config():
    """텔레그램 봇토큰·chat_id 설정/조회. POST {bot_token, chat_id}. (봇은 @BotFather, chat_id는 그룹/개인 ID)"""
    if request.method == "POST":
        d = request.get_json(force=True, silent=True) or {}
        try:
            with open(TELEGRAM_FILE, "w", encoding="utf-8") as f:
                json.dump({"bot_token": d.get("bot_token", ""), "chat_id": d.get("chat_id", "")}, f)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 500
    tok, cid = _telegram_cfg()
    return jsonify({"configured": bool(tok and cid), "chat_id": cid,
                    "bot_token_set": bool(tok)})


@app.post("/api/v1/telegram/test")
def telegram_test():
    """설정 확인용 테스트 발송."""
    ok, info = _send_telegram("🔔 Automato 텔레그램 알림 테스트 — 연결 정상")
    return jsonify({"sent": ok, "info": info})


# ===== 수확 실적 (오늘 수확량·최근7일·정상/폐기 · 순찰 히트맵처럼 서버 통신 · 7일 롤링) =====
import datetime as _dt
HARVEST_FILE = os.path.join(BASE, "harvest.json")


def _kst_today():
    return (_dt.datetime.utcnow() + _dt.timedelta(hours=9)).date()   # 서버는 UTC → 한국시각 날짜


def harvest_default():
    seed = [92, 104, 88, 118, 110, 121, 128.6]                       # 최근 7일 데모 시드
    base = _kst_today()
    days = {(base - _dt.timedelta(days=6 - i)).isoformat(): seed[i] for i in range(7)}
    return {"days": days, "sold_kg": 50.1, "discard_kg": 16.8, "updated_at": None}


def load_harvest_stats():
    try:
        with open(HARVEST_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return harvest_default()


def save_harvest_stats(d):
    try:
        with open(HARVEST_FILE, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False)
    except Exception:
        pass


def _prune_harvest(d):
    """최근 7일(KST)만 유지 → 8일 지난 날 자동 삭제. 빈 날은 0으로 채워 7칸 유지."""
    base = _kst_today()
    valid = {(base - _dt.timedelta(days=i)).isoformat() for i in range(7)}
    d["days"] = {k: v for k, v in d.get("days", {}).items() if k in valid}
    for i in range(7):
        d["days"].setdefault((base - _dt.timedelta(days=6 - i)).isoformat(), 0)
    return d


def _bump_harvest(kg):
    with LOCK:
        d = load_harvest_stats()
        _prune_harvest(d)
        today = _kst_today().isoformat()
        d["days"][today] = round(d["days"].get(today, 0) + kg, 1)
        d["updated_at"] = time.strftime("%m/%d %H:%M")
        save_harvest_stats(d)


def _harvest_payload():
    d = load_harvest_stats()
    _prune_harvest(d)          # 응답용 in-memory prune만(파일 저장은 bump/post 때만 → GET 동시요청 쓰기경쟁 방지)
    base = _kst_today()
    week = [{"date": (base - _dt.timedelta(days=6 - i)).isoformat(),
             "kg": d["days"][(base - _dt.timedelta(days=6 - i)).isoformat()]} for i in range(7)]
    return {"today_kg": d["days"][base.isoformat()], "sold_kg": d.get("sold_kg", 0),
            "discard_kg": d.get("discard_kg", 0), "week": week,
            "week_total": round(sum(x["kg"] for x in week), 1), "updated_at": d.get("updated_at")}


@app.get("/api/harvest/stats")
def get_harvest_stats():
    """웹앱 홈 '오늘 수확 요약'이 폴링. 7일 롤링·8일전 자동삭제는 서버가 처리."""
    return jsonify(_harvest_payload())


@app.post("/api/harvest/stats")
def post_harvest_stats():
    """실기 연동: 수확 로봇/집계가 실제 수확량 업로드.
       body {token, add_kg? | today_kg?, sold_kg?, discard_kg?}"""
    data = request.get_json(force=True, silent=True) or {}
    if data.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    with LOCK:
        d = load_harvest_stats()
        _prune_harvest(d)
        today = _kst_today().isoformat()
        if "add_kg" in data:
            d["days"][today] = round(d["days"].get(today, 0) + float(data["add_kg"]), 1)
        if "today_kg" in data:
            d["days"][today] = float(data["today_kg"])
        if "sold_kg" in data:
            d["sold_kg"] = float(data["sold_kg"])
        if "discard_kg" in data:
            d["discard_kg"] = float(data["discard_kg"])
        d["updated_at"] = time.strftime("%m/%d %H:%M")
        save_harvest_stats(d)
    return jsonify(_harvest_payload())


# ===== 밀집 히트맵 데이터 (순찰 카메라 D435+4분류 모델이 감지한 완숙 밀집도 · 순찰마다 갱신) =====
HEAT_FILE = os.path.join(BASE, "heat.json")
HEAT_DEFAULT = {"pillars": [0.95, 0.90, 0.72],                 # 기둥(재배 베드) 3곳 밀집도 0~1
                "crop": {"ripe": 342, "unripe": 588, "pest": 47, "rot": 23},
                "patrol_count": 0, "updated_at": None}


def load_heat():
    try:
        with open(HEAT_FILE, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return json.loads(json.dumps(HEAT_DEFAULT))


def save_heat(d):
    try:
        with open(HEAT_FILE, "w", encoding="utf-8") as f:
            json.dump(d, f, ensure_ascii=False)
    except Exception:
        pass


def _evolve_heat():
    """순찰 1회 = 새 스캔. 실기 연동 전까지 데모용으로 값을 흔들어 '매 순찰 변화'를 재현.
       실기 연동 시엔 로봇/비전이 POST /api/heatmap 으로 실제값을 덮어씀."""
    import random
    with LOCK:
        d = load_heat()
        d["pillars"] = [round(min(1.0, max(0.45, p + random.uniform(-0.15, 0.18))), 2)
                        for p in d.get("pillars", HEAT_DEFAULT["pillars"])]
        c = dict(d.get("crop", HEAT_DEFAULT["crop"]))
        c["ripe"]   = max(0, c.get("ripe", 342)   + random.randint(-40, 70))
        c["unripe"] = max(0, c.get("unripe", 588) + random.randint(-60, 50))
        c["pest"]   = max(0, c.get("pest", 47)    + random.randint(-12, 16))
        c["rot"]    = max(0, c.get("rot", 23)     + random.randint(-8, 12))
        d["crop"] = c
        d["patrol_count"] = d.get("patrol_count", 0) + 1
        d["updated_at"] = time.strftime("%m/%d %H:%M")
        save_heat(d)


@app.get("/api/heatmap")
def get_heatmap():
    """웹앱이 밀집 히트맵/작물 상태를 그릴 때 폴링."""
    return jsonify(load_heat())


@app.post("/api/heatmap")
def post_heatmap():
    """실기 연동: 순찰 로봇/비전이 새 스캔 결과 업로드.
       body {token, pillars:[..3], crop:{ripe,unripe,pest,rot}}"""
    data = request.get_json(force=True, silent=True) or {}
    if data.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    with LOCK:
        d = load_heat()
        if isinstance(data.get("pillars"), list):
            d["pillars"] = [float(x) for x in data["pillars"]]
        if isinstance(data.get("crop"), dict):
            base = d.get("crop", HEAT_DEFAULT["crop"])
            d["crop"] = {k: int(data["crop"].get(k, base.get(k, 0)))
                         for k in ("ripe", "unripe", "pest", "rot")}
        d["patrol_count"] = d.get("patrol_count", 0) + 1
        d["updated_at"] = time.strftime("%m/%d %H:%M")
        save_heat(d)
    return jsonify({"ok": True, **load_heat()})


# ===== E2 수확 로봇 배정 (로봇팔 필수 · arm+pinky만 · RP-67) =====
#   순찰과 같은 fleet(patrol.json) 공유 — 한 로봇은 순찰/수확을 동시에 못 함.
MIN_BAT_HARVEST = 50          # 스펙: 수확 배터리 50% 이상(기본). mock 과 동일.


def _harvest_reason(r):
    if r.get("compose") != "arm+pinky":          # 로봇팔 없으면 수확 불가
        return False, "NO_ARM"
    if r["status"] != "IDLE":
        return False, "ROBOT_BUSY"
    if r["battery_percent"] < MIN_BAT_HARVEST:
        return False, "BATTERY_TOO_LOW"
    return True, None


@app.get("/api/v1/robots/harvest/available")
def harvest_available():
    """수확 가능한 로봇 목록(dg_01·dg_02). 배터리>=50 & 대기중.
       ※ 순찰과 달리 수확은 여러 대 동시 가능 — '최대 1대' 제한 없음."""
    robots = _demo_available("harvest")
    return jsonify({"requested_at": _sim_now(), "min_battery_percent": MIN_BAT_HARVEST,
                    "robots": robots, "available_count": sum(1 for r in robots if r["available"])})


@app.post("/api/v1/harvest/requests")
def harvest_request():
    """수확 요청. {robot_selection: auto|manual, robot_id?}
       Control 연동 시 /internal/v1/tasks/harvest 로 중계(대시보드=요청 일치). 아니면 로컬 데모."""
    data = request.get_json(force=True, silent=True) or {}
    sel = data.get("robot_selection", "auto")
    if _control_on():
        try:
            wlog("▶ App 요청: 수확 요청(robot_selection=%s robot_id=%s) → ACS 중계 POST /internal/v1/tasks/harvest"
                 % (sel, data.get("robot_id")))
            r = _control_post("/internal/v1/tasks/harvest", data)
            js = r.json()
            wlog("◀ ACS 응답:", js.get("status"), "→ App 반환")
            return (jsonify(js), r.status_code)
        except Exception as e:
            wlog("⚠ ACS 수확 요청 실패, 로컬 폴백:", e)
            app.logger.warning("control tasks/harvest 실패, 로컬 폴백: %s", e)
    # ACS 없음(라이브 단독) → 자체 함대(dg_01·dg_02=수확)로 처리. 수확은 동시 여러 대 가능.
    with LOCK:
        avail = [r for r in _DEMO_FLEET.values() if r["role"] == "harvest" and _demo_reason(r) is None]
        if not avail:
            return jsonify({"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                            "message": "수확 가능한 로봇이 없습니다."}), 409
        if sel == "manual":
            rid = data.get("robot_id")
            chosen = next((r for r in avail if r["robot_id"] == rid), None)
            if not chosen:
                return jsonify({"status": "REJECTED", "reason": "ROBOT_NOT_AVAILABLE",
                                "message": "선택한 로봇을 지금 쓸 수 없습니다."}), 409
        else:
            chosen = max(avail, key=lambda r: (r["battery_percent"], [-ord(c) for c in r["robot_id"]]))
        _DEMO_SEQ[0] += 1
        task_id = _DEMO_SEQ[0]
        chosen["status"] = "HARVESTING"
        rid = chosen["robot_id"]
    import random
    _bump_harvest(round(random.uniform(4, 11), 1))   # 수확 나가면 오늘 수확량 누적
    def _restore_harvest():                          # 5초 후 대기 복귀 (텔레메트리 동기화)
        time.sleep(5)
        _demo_set_status(rid, "IDLE")
    threading.Thread(target=_restore_harvest, daemon=True).start()
    return jsonify({"task_id": task_id, "assigned_robot_id": rid,
                    "status": "ACCEPTED", "message": "수확 요청이 접수되었습니다."})


# ===== 로봇 센서·모터 실시간 텔레메트리 (E0 · servo_health 원본) =====
def _wob(base, amp, ph=0):
    tk = int(time.time())
    return round(base + amp * (((tk + ph) % 10) - 5) / 5.0, 1)


TELE_HIST = os.path.join(BASE, "tele_hist.json")
TELE_KEEP = 2016          # 최근 N개 (5분 간격 ≈ 7일). DB 없이 파일 링버퍼.
TELE_MIN_GAP = 295        # 5분 간격으로만 저장 (장기 아카이브 — 파일 가볍게)


_LAST_SAVE = [0]


def _append_hist(sample):
    # 메모리로 먼저 간격 체크 → 저장할 때만 파일 IO. 1초 폴링이어도 파일은 5분마다만 건드림.
    if sample["ts"] - _LAST_SAVE[0] < TELE_MIN_GAP:
        return
    with LOCK:
        try:
            h = json.load(open(TELE_HIST)) if os.path.exists(TELE_HIST) else []
        except Exception:
            h = []
        if h and sample["ts"] - h[-1].get("ts", 0) < TELE_MIN_GAP:
            _LAST_SAVE[0] = h[-1].get("ts", 0)
            return
        h.append(sample)
        if len(h) > TELE_KEEP:
            h = h[-TELE_KEEP:]
        try:
            json.dump(h, open(TELE_HIST, "w"))
            _LAST_SAVE[0] = sample["ts"]
        except Exception:
            pass


@app.get("/api/telemetry/history")
def telemetry_history():
    """과거 텔레메트리(온도 추이) — DB 없이 서버 파일 링버퍼에서. 최근 8시간."""
    try:
        h = json.load(open(TELE_HIST)) if os.path.exists(TELE_HIST) else []
    except Exception:
        h = []
    return jsonify({"history": h, "count": len(h)})


_FLEET_POS = {}

@app.post("/api/fleet/pos")
def fleet_pos():
    """로봇(노트북)이 실제 주행 위치를 push. nx,nz=맵 정규화 좌표(-1~1).
       실제 push되면 웹 3D 맵의 Pinky가 즉시 그 위치로 이동(45초 유효)."""
    data = request.get_json(force=True, silent=True) or {}
    if data.get("token") != INGEST_TOKEN:
        return jsonify({"error": "unauthorized"}), 401
    rid = data.get("robot_id")
    if rid:
        _FLEET_POS[rid] = {"nx": float(data.get("nx", 0)), "nz": float(data.get("nz", 0)),
                           "yaw": float(data.get("yaw", 0)), "ts": time.time()}
    return jsonify({"ok": True})


@app.get("/api/telemetry")
def telemetry():
    """각 로봇의 센서·모터 실시간 상태. 실서비스에선 fleet_telemetry 구독.
       DG1·DG2=로봇팔+주행 / DG3=주행만. (데모: 값이 실시간으로 요동)"""
    import math as _math
    tk = int(time.time())
    specs = [("DG1", True, 85, 0), ("DG2", True, 74, 3), ("DG3", False, 62, 6)]
    robots = []
    for idx, (rid, has_arm, bat, ph) in enumerate(specs):
        pinky = {
            "battery_pct": round(bat - (tk % 30) / 30.0, 1),
            "battery_v": round(11.6 + (bat - 60) / 40.0, 2),
            "lidar": "정상", "ultrasonic_cm": int(_wob(45, 15, ph)),
            "imu": "정상", "motor_temp": int(_wob(38, 4, ph)), "led": "ON",
        }
        r = {"robot_id": rid, "has_arm": has_arm, "pinky": pinky}
        fp = _FLEET_POS.get(rid)
        if fp and (time.time() - fp["ts"] < 45):
            r["position"] = {"nx": fp["nx"], "nz": fp["nz"], "yaw": fp["yaw"], "live": True}
        else:
            _p = tk * 0.4 + idx * 2.1
            r["position"] = {"nx": round(0.05 + _math.sin(_p) * 0.5, 3),
                             "nz": round(_math.sin(_p * 0.6) * 0.55, 3), "yaw": 0.0, "live": False}
        if has_arm:
            joints = []
            for i, a in enumerate([10, -30, 45, 0, -12, 5]):
                hot = 18 if (rid == "DG1" and i == 2) else 0        # DG1 J3 과열 데모
                ovl = (rid == "DG2" and i == 4)                     # DG2 J5 과부하 데모
                joints.append({"no": i + 1, "angle": _wob(a, 6, ph + i),
                               "temp": int(_wob(40 + hot, 3, ph + i)),
                               "current": round(_wob(0.3, 0.1, i), 2), "overload": ovl})
            r["arm"] = {"joints": joints, "gripper": int(_wob(80, 12, ph))}
        robots.append(r)
    return jsonify({"ts": tk, "robots": robots})


@app.get("/")
def index():
    resp = send_from_directory(app.static_folder, "index.html")
    resp.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return resp


# E0-5 텔레메트리 WS 클라이언트 시작 (모든 정의 후 — WSGI/직접실행 둘 다에서 동작)
_start_telemetry_ws()

if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5000))
    app.run(host="0.0.0.0", port=port, threaded=True)   # threaded: flask-sock WebSocket 동시 처리

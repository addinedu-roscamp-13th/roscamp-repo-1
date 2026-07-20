#!/usr/bin/env python3
"""LIVE 모드 — 실물 ACS(8200)의 교통관제 상태를 폴링해서 SIM 과 같은 모양으로 바꾼다.

왜 이 파일이 필요한가 —
  SIM 모드에서는 RoutingEngine 이 '이 프로세스 안에' 있어서 engine.holder_of() 를
  그냥 부르면 됐다. 실물에서는 예약표가 ACS 프로세스의 메모리 안에 있다. 다른
  프로세스의 메모리는 못 읽으니, ACS 가 HTTP 로 내놓은 것을 가져오는 수밖에 없다.
  (그 창문이 automato_control_service/traffic_debug.py 의 /internal/v1/debug/traffic.)

핵심 설계 — '같은 모양으로 변환한다':
  snapshot() 이 VerifySim.snapshot() 과 **똑같은 키**를 돌려준다. 그래서 서버의
  방송 루프도, 프런트의 렌더 코드도 한 줄도 안 바뀐다. 모드 스위치가 데이터 출처만
  갈아끼우는 구조가 된다(어댑터 패턴).

SIM 과 근본적으로 다른 점 세 가지 — 화면에서도 이 차이가 드러나야 한다:
  1) 위치가 '노드 번호'가 아니라 '좌표'다.
     실물 로봇은 "나 13번 지점이야"라고 말하지 않는다. 오도메트리 x/y/yaw 만 준다.
     그래서 waypoint_id 는 좌표에서 '가장 가까운 노드'로 역추정하고, 얼마나 가까운지
     (near_m)도 같이 준다 — 추정값을 사실처럼 보여주지 않기 위해서다.
  2) 이벤트 로그가 없다.
     SIM 은 디스패처의 logger 를 직접 가로챘지만, 실물 디스패처의 로그는 ACS
     프로세스의 stdout 으로 나간다. HTTP 로는 못 가져온다. 대신 예약표의 '변화'를
     직접 감지해서 이벤트를 만든다(통로 17 ← dg_01 / 통로 17 반납).
  3) 조작이 없다.
     통로 막기·데드락 시나리오는 실물 로봇을 위험하게 만든다. LIVE 는 관측 전용.

폴링 주기(2Hz)를 화면 방송 주기(10Hz)보다 느리게 잡은 이유:
  원천 데이터인 FleetTelemetry 가 1Hz 다. 10Hz 로 물어봐도 같은 값을 5번 받을 뿐
  ACS 에 부하만 준다. 방송 루프는 캐시된 마지막 값을 계속 내보내면 된다.
"""
import json
import math
import threading
import time
import urllib.error
import urllib.request


POLL_SEC = 0.5          # ACS 폴링 주기(2Hz) — 원천이 1Hz 라 이보다 빠를 이유가 없다
HTTP_TIMEOUT = 2.0      # ACS 가 느려도 폴링 스레드가 오래 잡히지 않게
NEAR_NODE_M = 0.08      # 이 거리 안이면 '그 노드에 있다'고 본다(통로 최단 길이보다 짧게)


class LiveSource:
    """ACS 를 백그라운드로 폴링해 최신 스냅샷 1장을 들고 있는 객체.

    방송 루프(비동기)가 snapshot() 을 10Hz 로 부르지만, 실제 HTTP 요청은
    별도 스레드가 2Hz 로만 한다 — 화면 응답성과 ACS 부하를 분리한다.
    """

    def __init__(self, acs_base: str, wp_meta: dict, *, poll_sec: float = POLL_SEC):
        self.acs_base = acs_base.rstrip("/")
        self.url = f"{self.acs_base}/internal/v1/debug/traffic"
        # {waypoint_id: {"x","y",...}} — 좌표→노드 역추정에 쓴다. 짝(pair)도 들어 있다.
        self.wp_meta = wp_meta
        self.poll_sec = poll_sec

        self._lock = threading.Lock()
        self._latest = None          # 마지막으로 성공한 ACS 응답
        self._error = None           # 마지막 실패 사유(화면에 그대로 띄운다)
        self._t0 = time.monotonic()

        # 이벤트: 예약표 '변화'를 감지해 만든다(아래 _diff_events 참고)
        self._events = []            # [{"seq","t","level","msg"}]
        self._seq = 0
        self._prev_res = {}          # 직전 틱의 {corridor_id(str): robot_id}
        self._prev_online = {}       # 직전 틱의 {robot_id: online 여부}

        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ #
    def start(self) -> None:
        if self._thread is not None:
            return
        # ★ stop() 이 세팅한 종료 플래그를 반드시 지운다.
        # 안 지우면 LIVE→SIM→LIVE 로 되돌아왔을 때 새 스레드가 while 문 첫 줄에서
        # 바로 빠져나가고, 마지막 성공 데이터가 그대로 남아 ACS 가 죽었는데도
        # 화면이 '연결됨'을 계속 보여준다(검증 도구가 거짓말을 하는 상태).
        self._stop.clear()
        # 이전 세션의 잔상도 버린다 — 재연결 시 옛날 예약표가 한 틱 보이면 안 된다.
        with self._lock:
            self._latest = None
            self._error = None
        self._prev_res = {}
        self._thread = threading.Thread(
            target=self._poll_loop, name="live-poll", daemon=True)
        self._thread.start()
        self._add_event("INFO", f"[LIVE] ACS 관측 시작 → {self.url}")

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with urllib.request.urlopen(self.url, timeout=HTTP_TIMEOUT) as r:
                    data = json.loads(r.read())
                if "error" in data:
                    raise RuntimeError(data.get("message", data["error"]))
                with self._lock:
                    prev_err = self._error
                    self._latest = data
                    self._error = None
                if prev_err is not None:
                    self._add_event("INFO", "[LIVE] ACS 재연결됨")
                self._diff_events(data)
            except Exception as exc:              # noqa: BLE001
                with self._lock:
                    first = self._error is None
                    self._error = f"{type(exc).__name__}: {exc}"
                    err = self._error
                if first:
                    # 매 틱 같은 에러를 쏟지 않는다 — 상태가 '바뀔 때'만 기록.
                    self._add_event("WARN", f"[LIVE] ACS 연결 실패 — {err}")
            self._stop.wait(self.poll_sec)

    # ------------------------------------------------------------------ #
    def _add_event(self, level: str, msg: str) -> None:
        with self._lock:
            self._seq += 1
            self._events.append({
                "seq": self._seq,
                "t": round(time.monotonic() - self._t0, 2),
                "level": level, "msg": msg,
            })
            if len(self._events) > 500:
                del self._events[:-500]

    def _diff_events(self, data: dict) -> None:
        """예약표/접속상태의 '변화'만 골라 사람이 읽을 문장으로 만든다.

        SIM 은 디스패처가 직접 말해준 문장을 썼지만 여기선 결과만 보인다. 그래서
        '무엇이 달라졌나'를 우리가 계산한다 — 예약 취득/반납/손바뀜 세 가지.
        """
        cur = data.get("reservations", {})
        prev = self._prev_res
        for cid, holder in cur.items():
            was = prev.get(cid)
            if was is None:
                self._add_event("INFO", f"통로 {cid} 예약 ← {holder}")
            elif was != holder:
                self._add_event("INFO", f"통로 {cid} 보유자 변경 {was} → {holder}")
        for cid, was in prev.items():
            if cid not in cur:
                self._add_event("INFO", f"통로 {cid} 반납 ({was})")
        self._prev_res = dict(cur)

        for r in data.get("robots", []):
            rid, online = r["robot_id"], r.get("online", False)
            if self._prev_online.get(rid) != online:
                if self._prev_online:      # 첫 틱은 전부 '변화'라 시끄러우니 건너뛴다
                    self._add_event(
                        "INFO" if online else "WARN",
                        f"{rid} {'접속' if online else '미수신(텔레메트리 끊김)'}")
            self._prev_online[rid] = online

    def last_seq(self) -> int:
        """마지막 이벤트 번호. VerifySim 과 같은 이름/의미(모드 전환을 단순하게)."""
        with self._lock:
            return self._seq

    # ------------------------------------------------------------------ #
    def _nearest_node(self, x, y):
        """좌표에서 가장 가까운 waypoint 를 역추정. (id, 거리m) 또는 (None, None).

        실물 로봇은 자기가 몇 번 지점에 있는지 모른다. 화면의 'wp13' 표시는
        어디까지나 우리 추정이므로, 거리도 함께 돌려줘 화면이 확신 정도를 표현하게 한다.
        """
        if x is None or y is None:
            return None, None
        best, best_d = None, None
        for wid, m in self.wp_meta.items():
            d = math.hypot(m["x"] - x, m["y"] - y)
            if best_d is None or d < best_d:
                best, best_d = wid, d
        return best, best_d

    def snapshot(self, since_seq: int = 0) -> dict:
        """VerifySim.snapshot() 과 같은 키를 돌려준다(프런트 재사용의 핵심)."""
        with self._lock:
            data = self._latest
            error = self._error
            events = [e for e in self._events if e["seq"] > since_seq]
            seq = self._seq

        base = {
            "t": round(time.monotonic() - self._t0, 2),
            "mode": "LIVE",
            "connected": data is not None and error is None,
            # '아직 한 번도 안 물어봤다'와 '물어봤는데 실패했다'는 다른 상태다.
            # LIVE 전환 직후엔 폴링(최대 POLL_SEC)이 끝나기 전이라 둘 다 connected=False
            # 인데, 그 순간을 '미연결'로 표시하면 멀쩡한 ACS 를 죽은 것처럼 보여준다.
            "connecting": data is None and error is None,
            "error": error,
            "events": events,
            "seq": seq,
            # LIVE 에는 '사용자가 만든 물리적 막힘'이 없다 — 조작이 없으니 항상 빈 값.
            "blocked": [],
        }
        if data is None:
            base.update(robots=[], reservations={}, avoiding=[],
                        engine_ready=False)
            return base

        # ACS 와 끊긴 동안에는 '마지막으로 본 값'을 사실처럼 보여주지 않는다.
        #   · 예약표는 통째로 비운다 — 통로 17 을 dg_01 이 쥐고 있다고 표시했는데
        #     실은 1분 전에 반납했을 수 있다. 확인 못 하는 교통관제 정보는 위험하다.
        #   · 로봇은 위치를 남긴다(‘마지막으로 본 자리’는 그 자체로 쓸모 있다).
        #     대신 전부 online=False 로 내려 목록에서 '미수신'으로 드러나게 한다.
        lost = error is not None

        robots = []
        for r in data["robots"]:
            wid, dist = self._nearest_node(r.get("x"), r.get("y"))
            near = dist is not None and dist <= NEAR_NODE_M
            robots.append({
                "robot_id": r["robot_id"],
                "x": r.get("x"), "y": r.get("y"), "yaw": r.get("yaw") or 0.0,
                # 가까울 때만 노드 번호를 말한다. 통로 한복판이면 None(=주행 중).
                "waypoint_id": wid if near else None,
                "capture_wp": None,
                # 실물은 '움직이는 중'을 nav_status 로만 알 수 있다(제자리회전 구분 불가).
                "moving": r.get("nav_status") not in (None, "IDLE"),
                "spinning": False,
                "status": "RUNNING" if r.get("task_id") else "IDLE",
                "task_id": r.get("task_id"),
                "result": None,
                "kind": None,
                # LIVE 전용 정보 — 화면 목록에 덧붙여 보여준다.
                "online": (not lost) and r.get("online", False),
                "age_sec": r.get("age_sec"),
                "battery_percent": r.get("battery_percent"),
                "nav_status": r.get("nav_status"),
                "near_m": round(dist, 3) if dist is not None else None,
            })
        base.update(
            robots=sorted(robots, key=lambda r: r["robot_id"]),
            reservations={} if lost else data.get("reservations", {}),
            avoiding=[] if lost else data.get("avoiding", []),
            engine_ready=(not lost) and data.get("engine_ready", False),
        )
        return base

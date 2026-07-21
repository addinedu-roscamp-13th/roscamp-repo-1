#!/usr/bin/env python3
"""RP-90  E0 텔레메트리 WebSocket 앱 계층 — 가용판정·커넥션 관리·방송을 담당한다.

telemetry_ws_node.py 가 '동작(ROS 구독 + 실행 조립)'을, 이 파일이 'WebSocket 쪽 로직'을
맡는다(patrol 의 patrol_node.py ↔ patrol_api.py 분리와 동일한 관례).

이 파일은 여러 조각으로 나눠 만든다(RP-90 구현 순서):
  ② judge_robot_availability ← 지금 이 조각(가용 여부·사유 판정, 외부의존 없는 순수 함수)
  ③ ConnectionManager    ← 이후(접속/해제/브로드캐스트 + 클라이언트별 예외 격리)
  ④ 메시지 조립 + 1Hz 방송 루프
  ⑤ create_ws_app        ← FastAPI 앱 팩토리(@app.websocket 라우트 + 방송 태스크 기동)

judge_robot_availability 는 ROS/DB/asyncio 를 모른다 → 값만 넣으면 값이 나오는 순수 함수라
단위테스트가 쉽다. 실제 입력(캐시/ DB/시각)은 방송 루프가 모아서 넣어준다.
"""
import asyncio
import json
import time
from datetime import datetime, timezone

from fastapi import FastAPI, WebSocket, WebSocketDisconnect

# unavailable_reason 확정 우선순위(위에서부터 검사, 처음 걸리는 값 채택).
# 여러 조건이 동시에 참일 수 있어 순서를 고정한다(예: 충전 중 + 배터리 낮음 → CHARGING).
# 의미상 '사람이 가야 풀림(IMMOBILIZED) → 데이터를 못 믿음(OFFLINE) →
#         물리 문제(CHARGING/배터리) → 정상 작업(BUSY)' 순.
# IMMOBILIZED 가 맨 앞인 이유: 유일하게 관리자 개입 없이는 저절로 풀리지 않는 상태라,
# 다른 사유에 가려지면 화면을 보는 사람이 현장에 나갈 판단을 못 한다.
# patrol_api.judge_robot(E1 배정 판정)도 같은 순서를 쓴다 — 방송 화면과 배정 결과가
# 어긋나면 "가용이라며 왜 배정이 안 되냐"가 된다.
UNAVAILABLE_PRIORITY = ("IMMOBILIZED", "ROBOT_OFFLINE", "CHARGING",
                        "BATTERY_TOO_LOW", "ROBOT_BUSY")

# 텔레메트리가 이 시간(초)보다 오래되면 미수신으로 본다(로봇 header.stamp 기준).
# patrol_api.STALE_SEC 과 동일한 3초 — 시스템 전체가 같은 기준을 쓴다.
OFFLINE_SEC = 3.0

# DB tasks.task_type 유효값(참고용). None = 이 로봇에 배정된 활성 task 없음.
TASK_TYPES = ("PATROL", "HARVEST", "TRANSFER")

# DB 조회가 첫 성공 전부터 실패할 때 쓸 배터리 임계값 부트스트랩값(이후엔 직전 성공값 재사용).
DEFAULT_BATTERY_THRESHOLD = 30.0

# 방송 주기(초). 1.0 = 1Hz.
BROADCAST_INTERVAL_SEC = 1.0


def judge_robot_availability(entry: dict, now: float, active_task_type,
                             operational_status: str, battery_threshold: float,
                             offline_sec: float = OFFLINE_SEC) -> dict:
    """로봇 1대의 최신 상태로 RP-90 규격의 로봇 항목 1건을 만든다(가용 판정 포함).

    입력:
      entry            : FleetCache.snapshot() 항목 1개
                         (robot_id, nav_status, is_charging, x, y, yaw,
                          battery_percent, stamp 를 담음)
      now              : 현재 시각(time.time(), epoch 초) — stamp 와 비교해 미수신 판정
      active_task_type : 이 로봇의 활성 task 종류('PATROL'|'HARVEST'|'TRANSFER') 또는
                         None(활성 task 없음). 활성 task 가 있거나 nav_status!='IDLE'(이동 중)
                         이면 ROBOT_BUSY 다.
      operational_status: robots.operational_status ('NORMAL'|'IMMOBILIZED'|'MAINTENANCE').
                         NORMAL 이 아니면 IMMOBILIZED — 통로에 갇혔거나(E2 22-2) 점검 중이라
                         관리자 개입 전에는 일을 줄 수 없다. 기본값을 두지 않는다(호출부가
                         빠뜨리면 갇힌 로봇이 조용히 '가용'으로 방송된다).
      battery_threshold: 배터리 임계값(설정값; 미만이면 BATTERY_TOO_LOW)
      offline_sec      : 미수신 판정 기준 초(기본 OFFLINE_SEC)

    반환(웹서비스로 그대로 나가는 로봇 항목):
      {robot_id, nav_status, position{x,y,yaw}, battery_percent,
       available, unavailable_reason, task_type}
    available=True 는 '지금 새 task 를 배차할 수 있는 상태'를 뜻한다(위 조건 모두 통과).
    """
    # stamp 가 얼마나 오래됐나 — 로봇이 멈추면 stamp 가 얼어붙어 age 가 계속 커진다.
    age = now - entry["stamp"]

    # 우선순위대로 위에서부터. elif 라서 '처음 걸리는' 하나만 채택된다.
    if operational_status != "NORMAL":
        reason = "IMMOBILIZED"            # ⓪ 사람이 가야만 풀림 — 무엇에도 가려지면 안 됨
    elif age > offline_sec:
        reason = "ROBOT_OFFLINE"          # ① 데이터 자체를 못 믿음(다른 필드는 마지막 값)
    elif entry["is_charging"]:
        reason = "CHARGING"               # ② 충전 중이면 배터리 낮아도 CHARGING 이 우선
    elif entry["battery_percent"] < battery_threshold:
        reason = "BATTERY_TOO_LOW"        # ③ 충전도 안 하는데 임계값 미만
    elif active_task_type is not None or entry["nav_status"] != "IDLE":
        reason = "ROBOT_BUSY"             # ④ 진행 중 task 또는 이동 중(nav!=IDLE) → 배차 불가
    else:
        reason = None                     # 어느 것에도 안 걸림 → 배차 가능

    return {
        "robot_id": entry["robot_id"],
        "nav_status": entry["nav_status"],           # ROBOT_BUSY 판정에 사용(+표시)
        "position": {
            "x": entry["x"], "y": entry["y"], "yaw": entry["yaw"],
        },
        "battery_percent": entry["battery_percent"],
        "available": reason is None,
        "unavailable_reason": reason,                # 가용이면 None
        "task_type": active_task_type,               # 활성 task 없으면 None
    }


def _iso_ms(ts: float) -> str:
    """epoch 초 → UTC ISO8601(밀리초 3자리 + 'Z'). 예: 2026-07-12T09:12:33.512Z.

    datetime.isoformat() 은 마이크로초(6자리)와 '+00:00' 을 주므로, 규격(ms + 'Z')에
    맞춰 직접 조립한다.
    """
    dt = datetime.fromtimestamp(ts, tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (dt.microsecond // 1000)


def build_message(robots_cache: list, active_types: dict, operational: dict,
                  threshold: float, seq: int, now: float) -> dict:
    """캐시 스냅샷 + DB 사실로 RP-90 방송 메시지 1건(봉투)을 조립한다(순수 함수).

    robots_cache : FleetCache.snapshot() 결과(로봇별 최신 상태 dict 리스트)
    active_types : robot_id -> 활성 task 종류('PATROL'|'HARVEST'|'TRANSFER').
                   여기 없는 로봇은 활성 task 가 없다는 뜻(→ ROBOT_BUSY 아님, task_type None).
    operational  : robot_id -> operational_status. 없는 로봇은 'NORMAL' 로 본다
                   (DB 조회가 아직 성공 못 했거나, 캐시에만 있고 robots 에 없는 로봇).
    threshold    : 배터리 임계값(설정값)
    seq          : 이 메시지의 전역 단조증가 번호(끊김 없이 1씩)
    now          : 이 틱의 기준 시각(epoch 초). timestamp 와 미수신 판정에 '같은 값'을 쓴다.

    반환: {event, seq, timestamp, data:{robots:[...]}}  (JSON 으로 직렬화해 전송)
    외부의존(ROS/DB/네트워크)이 없어 단위테스트가 쉽다.
    """
    robots = [
        judge_robot_availability(
            entry, now, active_types.get(entry["robot_id"]),
            operational.get(entry["robot_id"], "NORMAL"), threshold)
        for entry in robots_cache
    ]
    return {
        "event": "telemetry",
        "seq": seq,
        "timestamp": _iso_ms(now),
        "data": {"robots": robots},
    }


# --------------------------------------------------------------------------- #
# 커넥션 매니저 — 접속한 WebSocket 클라이언트들을 관리하고 한 메시지를 모두에게 방송.
#
# 이 집합(_active)은 오직 asyncio 이벤트 루프(엔드포인트 코루틴 + 방송 루프)에서만
# 건드린다. 서로 다른 OS 스레드가 아니라 '한 이벤트 루프 안 코루틴들'이라, await 를
# 만나기 전엔 아무도 끼어들지 못한다 → threading.Lock 이 필요 없다.
# (FleetCache 는 ROS 스레드 ↔ 이벤트 루프라 락이 필요했다 — 대비되는 지점.)
# --------------------------------------------------------------------------- #
class ConnectionManager:
    def __init__(self):
        self._active = set()    # 접속 중인 WebSocket 들
        self._last = None       # 마지막으로 방송한 메시지(신규 접속자에게 즉시 줄 최신본)

    async def connect(self, ws: WebSocket) -> None:
        """WebSocket handshake 를 수락하고 접속 목록에 넣는다. 최신본이 있으면 즉시 1건 전송.

        await ws.accept(): 클라이언트의 연결 요청(handshake)을 수락해 연결을 확정한다.
          async 함수라 await 로 부른다 — 'I/O 가 끝날 때까지 이벤트 루프에 양보하고 기다림'.
        접속 직후 self._last 를 보내주면, 다음 1Hz 방송을 기다리지 않고 '즉시 최신 스냅샷'을
          받는다. self._last 는 직전 방송 메시지 그대로라 seq 가 끊기지 않는다.
        """
        await ws.accept()
        self._active.add(ws)
        if self._last is not None:
            try:
                await ws.send_text(self._last)
            except Exception:            # noqa: BLE001 (방금 접속인데 실패하면 조용히 제거)
                self._active.discard(ws)

    def disconnect(self, ws: WebSocket) -> None:
        """접속 목록에서 제거(엔드포인트가 연결 종료를 감지하면 호출). 동기 함수라 await 없음."""
        self._active.discard(ws)         # 없어도 예외 안 나는 discard 사용

    async def broadcast(self, text: str) -> None:
        """접속 중인 모든 클라이언트에게 text 를 전송. 한 명 실패가 전체를 멈추지 않게 격리.

        list(self._active) 로 '복사본'을 순회하는 이유: send 는 await 라, 전송 도중
          이벤트 루프가 다른 코루틴(connect/disconnect)으로 잠깐 넘어가 집합이 바뀔 수
          있다. 원본을 직접 순회하면 'set changed during iteration' 으로 터진다.
        try/except: 특정 클라이언트 전송 실패(비정상 종료 등)를 그 클라이언트만 정리하고
          나머지 방송은 계속한다.
        """
        self._last = text                # 다음 접속자에게 즉시 줄 최신본으로 보관
        for ws in list(self._active):
            try:
                await ws.send_text(text)
            except Exception:            # noqa: BLE001
                self._active.discard(ws)

    def count(self) -> int:
        """현재 접속 수. 방송 루프가 '접속 0명이면 방송 skip' 판단에 쓴다."""
        return len(self._active)


# --------------------------------------------------------------------------- #
# 1Hz 방송 루프 — 이벤트 루프에서 asyncio 태스크로 상시 돈다(⑤ create_ws_app 이 기동).
# --------------------------------------------------------------------------- #
async def broadcast_loop(manager: ConnectionManager, cache, read_db_state,
                         interval: float = BROADCAST_INTERVAL_SEC,
                         logger=None) -> None:
    """1초마다 캐시+DB 로 메시지를 조립해 접속 중인 모든 클라이언트에 방송한다.

    manager       : ConnectionManager (broadcast / count)
    cache         : FleetCache (snapshot). writer(ROS 스레드)가 채우고 여기서 읽는다.
    read_db_state : 인자 없는 콜러블 → (active_types: dict, threshold: float,
                    operational: dict). DB/psycopg 세부를 이 콜백 뒤에 숨겨, 이 루프는
                    DB 를 전혀 모른다(가짜 콜백으로 테스트가 쉽다 — ⑥ 조립부가 실제 DB 연결).
    interval      : 방송 주기(초). 1.0 = 1Hz.
    logger        : 로그용(.warning/.info). rclpy 로거·std logging 둘 다 가능(없으면 무시).

    설계 요점:
      · 접속 0명이면 DB·조립·방송을 모두 건너뛴다 → 클라이언트 없을 때 에러/부하 0(티켓).
      · seq 는 '실제 방송할 때만' 1 증가 → 보낸 메시지들은 항상 끊김 없이 1씩 증가.
      · DB 조회가 실패해도 방송을 멈추지 않는다 → active_types/threshold/operational 을
        직전 성공값으로 재사용(첫 성공 전엔 부트스트랩값). 상태가 바뀔 때만 로그해 1Hz
        도배를 막는다.
      · operational 부트스트랩이 빈 dict = 전 로봇 NORMAL 취급이라 낙관적이다. 그래도
        되는 이유: 이 방송은 화면 표시용이고, 실제 배정은 E1 API(patrol_api)가 그때그때
        DB 를 직접 읽어 판단한다(DB 가 죽어 있으면 503 으로 거절). 즉 여기서 잠깐
        낙관적으로 보여도 갇힌 로봇에 일이 배정되지는 않는다.
    """
    seq = 0
    # 첫 DB 성공 전 부트스트랩
    active_types, threshold, operational = {}, DEFAULT_BATTERY_THRESHOLD, {}
    db_healthy = True

    while True:
        # 1초 양보. 그동안 이벤트 루프가 접속/해제/전송 코루틴을 처리한다.
        await asyncio.sleep(interval)

        # 접속자가 없으면 아무것도 하지 않는다(DB 도 안 건드림) → 조용히 다음 틱.
        if manager.count() == 0:
            continue

        # 캐시 스냅샷(복사본) — writer 와 겹치지 않게 ① FleetCache 가 복사해 준다.
        robots_cache = cache.snapshot()

        # DB 사실(활성 task 종류 + 배터리 임계값 + 운영 상태). 실패해도 방송은 계속한다.
        try:
            active_types, threshold, operational = read_db_state()
            if not db_healthy:
                db_healthy = True
                if logger is not None:
                    logger.info("DB 조회 복구 — busy/task_type 판정 재개")
        except Exception as exc:   # noqa: BLE001
            # 직전 성공값(active_types/threshold)을 그대로 재사용(여기서 갱신하지 않음).
            if db_healthy:
                db_healthy = False
                if logger is not None:
                    logger.warning(
                        "DB 조회 실패 — 직전 값으로 방송 지속(busy/task_type 정확도 저하): %s"
                        % exc)

        seq += 1
        msg = build_message(robots_cache, active_types, operational, threshold,
                            seq, time.time())
        # dict → JSON text frame. ensure_ascii=False 로 한글 등도 그대로 싣는다.
        await manager.broadcast(json.dumps(msg, ensure_ascii=False))

        # 흐름 가시화 로그(캐시 읽기 → WS 발행). 첫 방송 + 5틱(≈5초)마다만 찍어 1Hz 도배 방지.
        # rclpy 전용 throttle 대신 seq 수동 스로틀 — 이 루프는 std logging 도 허용하기 때문.
        # '로봇 N대(캐시)'가 캐시 읽은 결과, '클라이언트 M명'이 실제 발행 대상.
        if logger is not None and (seq == 1 or seq % 5 == 0):
            logger.info("방송 seq=%d: 로봇 %d대(캐시) → 클라이언트 %d명"
                        % (seq, len(robots_cache), manager.count()))


# --------------------------------------------------------------------------- #
# FastAPI 앱 팩토리 — WebSocket 라우트 + 1Hz 방송 태스크를 묶는다(patrol create_app 대응).
# --------------------------------------------------------------------------- #
def create_ws_app(node, pool) -> FastAPI:
    """노드(텔레메트리 캐시)와 DB 풀을 주입받아 WebSocket FastAPI 앱을 만든다.

    node: telemetry_ws_node.TelemetryNode  (node.cache=FleetCache, node.get_logger())
    pool: psycopg_pool.ConnectionPool
    이 팩토리만 DB(automato_db)를 안다 — 순수 함수/루프는 DB 를 모른 채로 둔다(테스트 용이).
    """
    from automato_control_service import automato_db

    app = FastAPI(title="Automato Control Service — Telemetry WS")
    manager = ConnectionManager()

    def read_db_state():
        """방송 루프가 매 틱 호출하는 DB 조회 콜백 → (active_types, threshold, operational)."""
        return automato_db.get_telemetry_state(pool)

    @app.websocket("/ws/telemetry")
    async def telemetry_endpoint(ws: WebSocket):
        """웹서비스가 접속하는 엔드포인트. 서버는 방송만 하므로 이 코루틴은
        접속 등록 → 연결 유지 → 끊김 감지 → 해제 정리만 담당한다(전송은 방송 루프가)."""
        await manager.connect(ws)
        node.get_logger().info("텔레메트리 WS 접속 (현재 %d명)" % manager.count())
        try:
            while True:
                # 서버는 받을 게 없지만, 이 await 가 '클라이언트가 끊었는지'를 감지한다.
                # 정상 종료/비정상 종료 모두 WebSocketDisconnect 로 빠져나온다. 받은 값은 무시.
                await ws.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            manager.disconnect(ws)
            node.get_logger().info("텔레메트리 WS 해제 (현재 %d명)" % manager.count())

    @app.on_event("startup")
    async def _start_broadcast():
        # 이벤트 루프가 도는 시점에 1Hz 방송 태스크를 백그라운드로 띄운다.
        # (startup/shutdown 은 lifespan 으로도 쓸 수 있으나, 여기선 읽기 쉬운 on_event 사용.)
        app.state.broadcast_task = asyncio.create_task(
            broadcast_loop(manager, node.cache, read_db_state,
                           logger=node.get_logger()))
        node.get_logger().info("텔레메트리 1Hz 방송 태스크 시작")

    @app.on_event("shutdown")
    async def _stop_broadcast():
        task = getattr(app.state, "broadcast_task", None)
        if task is not None:
            task.cancel()

    @app.get("/health")
    def health():
        return {"ok": True, "service": "Automato Control Service (Telemetry WS)",
                "clients": manager.count()}

    return app

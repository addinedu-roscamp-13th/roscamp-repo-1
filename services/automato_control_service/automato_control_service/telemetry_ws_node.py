#!/usr/bin/env python3
"""RP-90  E0 텔레메트리 WebSocket 서버 — 로봇 텔레메트리를 1Hz로 웹서비스에 방송.

ACS 가 로봇별로 구독한 상태를 WebSocket 클라이언트(Automato Web Service)에게
1Hz 로 브로드캐스트한다. QT 로 가는 원본(④)과 달리 여기는 관리자 화면용 **축약본**이다.

RP-114 로 입력이 /{robot_id}/telemetry(RobotTelemetry, 로봇 수만큼)로 바뀌었다.
옛 /automato/telemetry/fleet 는 팀원의 DG 이전이 끝날 때까지 함께 구독한다.

실행 구조(기존 patrol_node 와 동일한 골격 — 두 세계가 한 프로세스에 공존):
  - [백그라운드 스레드]  rclpy 노드가 spin → fleet 구독 콜백이 FleetCache 에 최신 상태를 씀(writer)
  - [메인 스레드]        uvicorn(FastAPI) 이벤트 루프 → 1Hz 방송 코루틴이 FleetCache 를 읽음(reader)
  - FleetCache 는 두 세계를 잇는 다리. writer(스레드)와 reader(asyncio)가 서로 다른
    OS 스레드에 살기 때문에 threading.Lock 으로 보호한다.

이 파일은 여러 조각으로 나눠 만든다(RP-90 구현 순서):
  ① FleetCache            ← 지금 이 조각(락으로 보호되는 로봇별 최신상태 저장소)
  ⑥ TelemetryNode + main() ← 이후 조각(fleet 구독 노드 + 전체 조립)
가용 판정/방송 루프/커넥션 매니저는 telemetry_ws.py 에 둔다(patrol 의 api/node 분리와 동일).
"""
import threading

from automato_interfaces.msg import FleetTelemetry
import rclpy
from rclpy.node import Node

from automato_control_service.fleet_collector import (
    DEFAULT_ROBOT_IDS,
    LEGACY_FLEET_TOPIC,
    robot_telemetry_topic,
    subscribe_per_robot,
)


# --------------------------------------------------------------------------- #
# 텔레메트리 캐시 — 로봇별 '그 로봇의 최신 상태' 1건을 메모리에 보관(수신마다 덮어씀).
#
# 왜 필요한가: ROS2 구독은 발행자가 밀어보낼 때(1Hz)마다 콜백이 실행되는 push 방식이다.
# 반면 방송 루프는 자기 타이밍(1초마다)에 '지금의 최신값'을 읽고 싶다. 들어오는 타이밍과
# 읽는 타이밍이 달라서, 그 사이에 '최신값을 놔두는 선반'이 필요하다 — 그게 이 캐시다.
# DB 저장은 없다(실시간 현재상태 전용, 프로세스가 죽으면 사라져도 됨).
# --------------------------------------------------------------------------- #
class FleetCache:
    def __init__(self):
        # 콜백 스레드(쓰기) ↔ 방송 코루틴(읽기)이 동시에 이 dict 를 만지므로 락으로 보호.
        # 서로 '다른 OS 스레드'라서 asyncio.Lock 이 아니라 threading.Lock 이다.
        self._lock = threading.Lock()
        self._robots = {}   # robot_id -> 최신 상태 dict

    @staticmethod
    def _entry(robot_id, d) -> dict:
        """DdagoTelemetry 하나를 방송용 상태 dict 로.

        stamp: 로봇이 직접 찍은 header.stamp(초 단위 epoch). '3초 미수신'(ROBOT_OFFLINE)
          판정의 기준이다. 우리가 받은 시각이 아니라 로봇의 stamp 를 쓰는 이유 —
          중간 계층이 죽은 로봇을 어떻게 다루든 stamp 는 로봇이 멈춘 순간 함께 얼어붙어,
          now 와의 차이로 미수신을 정확히 드러내기 때문이다. 예컨대 마지막 값을 계속
          재발행하는 구현에서는 '수신 시각'으로 재면 영영 신선해 보여 틀린다.
          patrol_api 도 동일하게 ddago header.stamp 로 staleness 를 잰다(시스템 일관성).
        """
        return {
            "robot_id": robot_id,
            "nav_status": d.nav_status,
            "is_charging": bool(d.is_charging),
            "x": float(d.x),
            "y": float(d.y),
            "yaw": float(d.yaw),
            "battery_percent": float(d.battery_percent),
            "stamp": d.header.stamp.sec + d.header.stamp.nanosec * 1e-9,
        }

    def update_from_robot(self, robot_id: str, msg) -> None:
        """RP-114 주 경로: 로봇 하나의 RobotTelemetry 로 최신 상태를 덮어쓴다(writer).

        RP-90 은 주행 로봇(ddago)의 위치·배터리·주행상태만 방송하므로 msg.ddagos 만 본다
        (로봇팔 ddagis 는 이 축약본과 무관 — QT 로 가는 원본 ④에는 그대로 실린다).
        어느 로봇인지는 토픽 네임스페이스가 말해주므로 robot_id 를 인자로 받는다.
        """
        with self._lock:                       # 열쇠를 집는다(누가 쥐고 있으면 대기)
            for d in msg.ddagos:
                # 한 로봇의 전체 필드를 새 dict 로 만들어 통째로 교체 → '반쯤 바뀐' 상태가 없다.
                # (그래도 dict 에 키를 더하며 순회 대상을 바꾸므로 락은 필요하다.)
                self._robots[robot_id] = self._entry(robot_id, d)
        # with 블록을 벗어나면 열쇠를 자동 반납(예외가 나도 반드시 반납).

    def update_from_fleet(self, msg) -> None:
        """[삭제 예정] 옛 경로: FleetTelemetry 1건(로봇 3대분)을 로봇별로 나눠 저장한다.

        옛 구조에는 네임스페이스가 없어 로봇 구분이 payload 의 robot_id 뿐이다.
        robot_id 가 빈 항목은 어느 로봇인지 알 수 없어 건너뛴다.
        """
        with self._lock:
            for d in msg.ddagos:
                if not d.robot_id:
                    continue
                self._robots[d.robot_id] = self._entry(d.robot_id, d)

    def snapshot(self) -> list:
        """지금 알고 있는 모든 로봇의 최신 상태 '복사본' 리스트를 반환(reader, 방송 코루틴).

        복사본을 주는 이유: 호출자가 락 밖에서 느긋하게 읽는 동안 writer 가 원본을 바꿔도
        안전하게. 락은 '복사만 하고 즉시 반납' — 그 짧은 순간만 이벤트 루프를 잡는다.
        오프라인 로봇도 여기서 빠지지 않는다(수신이 끊겨도 마지막 값이 남아 있음 →
        ROBOT_OFFLINE 판정은 이후 가용판정 단계가 stamp 로 내린다).
        """
        with self._lock:
            return [dict(entry) for entry in self._robots.values()]


# --------------------------------------------------------------------------- #
# 텔레메트리 구독 노드 — /{robot_id}/telemetry 를 로봇 수만큼 구독해 FleetCache 를 채운다.
# 이게 '실제로 데이터를 끌어오는' 부분. 이 노드가 없으면 캐시는 영영 비어 있다.
# --------------------------------------------------------------------------- #
class TelemetryNode(Node):
    def __init__(self, **kwargs):
        super().__init__("telemetry_ws_node", **kwargs)
        self.cache = FleetCache()
        # 로봇별 첫 수신을 1회만 INFO 로 알리기 위한 표시(이후엔 throttle 로만 로그).
        self._first_rx_logged = set()

        self.declare_parameter("robot_ids", DEFAULT_ROBOT_IDS)
        self.declare_parameter("legacy_input", True)
        robot_ids = list(self.get_parameter("robot_ids").value)
        legacy_input = bool(self.get_parameter("legacy_input").value)

        # 1Hz 상시 구독. DG 발행자와 맞춰 기본 QoS(RELIABLE, depth 10).
        subscribe_per_robot(self, robot_ids, self._on_robot_telemetry)
        # [삭제 예정] 팀원의 DG 이전 전까지 옛 경로도 함께 받는다.
        if legacy_input:
            self.create_subscription(
                FleetTelemetry, LEGACY_FLEET_TOPIC, self._on_fleet, 10)

        self.get_logger().info(
            "텔레메트리 WS 노드 준비: 구독 %s → 캐시 갱신%s"
            % ([robot_telemetry_topic(r) for r in robot_ids],
               " (옛 %s 도 함께)" % LEGACY_FLEET_TOPIC if legacy_input else ""))

    def _on_robot_telemetry(self, robot_id, msg) -> None:
        # 콜백(백그라운드 spin 스레드)에서 캐시에 쓴다(writer). 방송 루프(메인 스레드)가 읽는다.
        # 두 스레드가 겹치지 않게 FleetCache 내부 threading.Lock 이 보호한다.
        self.cache.update_from_robot(robot_id, msg)

        # --- 흐름 가시화 로그 (수신 + 캐시 갱신 확인) ---
        log = self.get_logger()
        if robot_id not in self._first_rx_logged:   # 첫 수신은 로봇마다 1회 확실히 알림
            self._first_rx_logged.add(robot_id)
            log.info("%s 첫 수신: ddago %d → 캐시 갱신" % (robot_id, len(msg.ddagos)))
        else:                                       # 이후엔 5초에 한 번만(1Hz 도배 방지)
            log.info("텔레메트리 수신 중: %s" % robot_id, throttle_duration_sec=5.0)
        # 값이 실제로 바뀌는지 검증용 상세는 DEBUG — 평소 숨김, --log-level debug 로만.
        for d in msg.ddagos:
            log.debug("  %s nav=%s batt=%.0f pos=(%.2f,%.2f)"
                      % (robot_id, d.nav_status, d.battery_percent, d.x, d.y))

    def _on_fleet(self, msg: FleetTelemetry) -> None:
        """[삭제 예정] 옛 /automato/telemetry/fleet 경로."""
        self.cache.update_from_fleet(msg)
        self.get_logger().info(
            "[삭제 예정] 옛 fleet 경로로 수신 중(ddago %d대) — DG 이전 후 "
            "legacy_input 을 끄세요" % len(msg.ddagos),
            throttle_duration_sec=30.0)


# --------------------------------------------------------------------------- #
# 조립 루트 — rclpy 노드(백그라운드 spin) + uvicorn/FastAPI(메인, WebSocket)를 함께 띄운다.
# patrol_node.main() 과 동일한 골격: spin 은 백그라운드, uvicorn(이벤트 루프)은 메인.
# --------------------------------------------------------------------------- #
def main(args=None) -> None:
    import os

    import uvicorn
    from rclpy.executors import MultiThreadedExecutor

    from automato_control_service import automato_db
    from automato_control_service.telemetry_ws import create_ws_app

    rclpy.init(args=args)
    node = TelemetryNode()

    # DB 풀(활성 task 종류·배터리 임계값 조회용). DB 가 아직 안 떠 있어도 서비스는 기동된다.
    pool = automato_db.create_pool()

    # rclpy 는 백그라운드 스레드에서 상시 spin — 구독 콜백(writer)이 여기서 실행된다.
    # uvicorn(이벤트 루프)은 메인 스레드에서 돌려야 하므로 spin 을 분리한다.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, name="rclpy_spin", daemon=True)
    spin_thread.start()

    app = create_ws_app(node, pool)
    port = int(os.environ.get("ACS_WS_PORT", "8000"))
    node.get_logger().info(
        "Automato Control Service (텔레메트리) WebSocket → "
        "ws://0.0.0.0:%d/ws/telemetry" % port)

    try:
        # ws_ping_interval/timeout: 서버가 주기적으로 ping 을 보내 죽은 연결을 감지(keepalive).
        uvicorn.run(app, host="0.0.0.0", port=port, log_level="info",
                    ws_ping_interval=20.0, ws_ping_timeout=20.0)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        try:
            pool.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()

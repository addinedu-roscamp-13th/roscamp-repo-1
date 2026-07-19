#!/usr/bin/env python3
"""RP-78 ② ROS2 노드 — 텔레메트리 캐시 + Navigate 액션 클라이언트 + 순찰 디스패치.

이 파일은 'ROS 표면(로봇과의 실제 통신)'을 담당한다.
  - 구독:  /automato/telemetry/fleet (FleetTelemetry, 1Hz) → 로봇별 최신 상태 캐시
  - 발신:  /{robot_id}/navigate (Navigate 액션) → DG(DG Control Service) 경유로 경로(배열) 하달
  - 종료:  방문 결과에 따라 tasks 를 DONE/PARTIAL/FAILED 로 마감(automato_db)

배경 지식 (초보자용) —
  * ROS2 '토픽 구독'은 '요청하면 받아오는' 방식이 아니라, 발행자가 보낼 때마다
    콜백이 자동 실행되는 '스트리밍(push)'이다. 순찰 요청과 무관하게 캐시가 1Hz로 계속
    최신화되고, 요청 시점엔 '이미 들어있는 최신값'을 읽어 스냅샷으로 저장한다.
  * ROS2 '액션'은 시간이 걸리는 작업 요청(Goal/Feedback/Result)이다. 여기선 연속 예약된
    waypoint 여러 개(세그먼트)를 1개의 Goal(Waypoint[] 배열)로 보내고, 도착(Result)까지 기다린다.

실행 구조 (한 프로세스):
  - rclpy 노드는 MultiThreadedExecutor로 '백그라운드 스레드'에서 상시 spin.
  - FastAPI(uvicorn)는 '메인 스레드'에서 실행.
  - 텔레메트리 캐시는 락으로 보호(콜백 스레드가 쓰고, API 스레드가 읽음).
  - 순찰 디스패치는 '로봇당 1 스레드'로 동시 실행 → 3대가 동시에 움직이며 통로를 놓고 경합.
    공유 통로 예약표는 routing_engine 이 락으로 보호한다.

교통관제 알고리즘(세그먼트 예약·룩어헤드·막힘 우회)은 patrol_dispatcher.PatrolDispatcher
로 분리했다(composition). 이 노드는 엔진/클라이언트를 만들어 넘기고 결과만 tasks 에 마감한다.
"""
import os
import threading
import time

from automato_interfaces.action import Navigate
from automato_interfaces.msg import FleetTelemetry
from automato_interfaces.srv import SaveDetection
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_control_service import automato_db
from automato_control_service.patrol_config import (
    FLEET_TOPIC,
    RESERVATION_TTL_SEC,
    SAVE_DETECTION_SRV,
)
from automato_control_service.patrol_dispatcher import PatrolDispatcher
from automato_control_service.routing_engine import RoutingEngine
from automato_control_service.telemetry_cache import TelemetryCache


# --------------------------------------------------------------------------- #
# 순찰 제어 노드
# --------------------------------------------------------------------------- #
class PatrolControlNode(Node):
    def __init__(self, **kwargs):
        super().__init__("patrol_control_node", **kwargs)
        self.cache = TelemetryCache()
        self._db_pool = None                       # main()에서 주입
        # robot_id -> Navigate ActionClient.
        # ⚠️ 이름 주의: rclpy.Node 는 서비스 클라이언트 목록을 self._clients(리스트)로
        # 보관하고 node.clients 프로퍼티로 노출한다. 여기에 self._clients 를 dict 로
        # 덮으면 executor 가 node.clients 를 순회할 때 dict 의 '키(robot_id 문자열)'가
        # 나와 죽는다("'str' object has no attribute ...", RP-76). → 반드시 다른 이름 사용.
        self._action_clients = {}
        self._action_clients_lock = threading.Lock()

        # 라우팅/예약 엔진(공유 단일 인스턴스). 첫 순찰 때 그래프를 로드해 생성한다.
        self._engine = None
        self._engine_lock = threading.Lock()

        # 교통관제 알고리즘(세그먼트 이동·통로 예약·룩어헤드·막힘 우회)은 별도 클래스로
        # 분리(composition). 노드는 필요한 것(logger·engine·client)을 넘겨주고 위임만 한다.
        # wp_meta·블랙리스트는 디스패처가 소유하며, 그래프 로드 시 노드가 wp_meta 를 채운다.
        self._dispatcher = PatrolDispatcher(self.get_logger())

        # FleetTelemetry 상시 구독(1Hz)
        self.create_subscription(FleetTelemetry, FLEET_TOPIC, self._on_fleet, 10)

        self.get_logger().info(
            f"순찰 제어 노드 준비: 구독 {FLEET_TOPIC}, 하달 /<robot_id>/navigate "
            "(세그먼트 단위 + 통로 예약)")

    # ---------------------------- 주입/구독 ---------------------------- #
    def set_db_pool(self, pool) -> None:
        self._db_pool = pool

    def _on_fleet(self, msg: FleetTelemetry) -> None:
        self.cache.update_from_fleet(msg, time.time())

    # ---------------------------- 엔진/클라이언트 ---------------------------- #
    def _get_engine(self):
        """공유 라우팅 엔진을 얻는다(최초 1회 DB에서 그래프 로드). 실패 시 None."""
        with self._engine_lock:
            if self._engine is None:
                if self._db_pool is None:
                    return None
                try:
                    graph = automato_db.load_graph(self._db_pool)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().error(f"라우팅 그래프 로드 실패: {exc}")
                    return None
                self._engine = RoutingEngine(
                    graph["waypoints"], graph["corridors"],
                    reservation_ttl=RESERVATION_TTL_SEC)
                # wp_meta 는 디스패처가 소유(세그먼트 하달 시 좌표/촬영 여부에 사용) → 여기서 채운다.
                self._dispatcher.wp_meta = {
                    w["waypoint_id"]: {
                        "x": w["x"], "y": w["y"],
                        "yaw": w["yaw"], "capture": w["is_patrol_point"],
                    }
                    for w in graph["waypoints"]
                }
                self.get_logger().info(
                    f"라우팅 그래프 로드: 노드 {len(graph['waypoints'])} / "
                    f"통로 {len(graph['corridors'])}")
                if not graph["corridors"]:
                    self.get_logger().warn(
                        "corridors 가 비어 있음 — 순찰 이동이 모두 skip 될 수 있음"
                        "(DB corridors 시드 확인)")
            return self._engine

    def _client_for(self, robot_id: str) -> ActionClient:
        with self._action_clients_lock:
            client = self._action_clients.get(robot_id)
            if client is None:
                client = ActionClient(self, Navigate, f"/{robot_id}/navigate")
                self._action_clients[robot_id] = client
            return client

    def prewarm_clients(self, robot_ids) -> None:
        """알려진 로봇의 Navigate 액션 클라이언트를 executor spin 시작 전에 미리 만든다.

        기능상 필수는 아니지만(_client_for 가 필요 시 생성) 정리 목적의 이점이 있다:
          - 시작 시점에 ACS 가 어떤 로봇과 통신할지 로그로 드러난다(가시성).
          - 모든 ActionClient 생성이 spin 이전(메인 스레드)에 끝나, 순찰 디스패치
            작업 스레드는 '이미 있는 것'을 꺼내 쓰기만 한다(런타임 엔티티 생성 없음).
        (RP-76 크래시의 실제 원인은 self._clients 이름 충돌이며, 그건 __init__ 에서 해결.)
        """
        for rid in robot_ids:
            self._client_for(rid)
        if robot_ids:
            self.get_logger().info(
                f"Navigate 액션 클라이언트 프리웜 완료: {list(robot_ids)}")

    # ---------------------------- 디스패치 진입점 ---------------------------- #
    def start_patrol(self, task_id: int, robot_id: str, waypoints: list) -> None:
        """API가 호출. 로봇마다 별도 스레드로 순찰을 돌린다(동시 3대 → 통로 경합 발생)."""
        t = threading.Thread(
            target=self._patrol_job, args=(task_id, robot_id, waypoints),
            name=f"patrol-{robot_id}-{task_id}", daemon=True)
        t.start()
        self.get_logger().info(
            f"순찰 디스패치 시작: task={task_id} robot={robot_id} "
            f"지점 {len(waypoints)}개")

    def _patrol_job(self, task_id: int, robot_id: str, waypoints: list) -> None:
        """스레드 본체: 엔진/클라이언트를 준비해 디스패처에 위임하고, 결과를 tasks 에 마감.

        엔진 로드(DB)·액션 클라이언트 생성은 ROS/DB 자원이라 노드가 맡고, 실제 이동
        알고리즘은 self._dispatcher.run_patrol 에 넘긴다.
        """
        try:
            engine = self._get_engine()
            if engine is None:
                status = "FAILED"
            else:
                client = self._client_for(robot_id)
                status = self._dispatcher.run_patrol(
                    task_id, robot_id, waypoints, engine, client)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"디스패치 예외 task={task_id}: {exc}")
            status = "FAILED"
        if self._db_pool is not None:
            try:
                automato_db.set_task_status(self._db_pool, task_id, status)
                self.get_logger().info(f"순찰 종료 task={task_id} → {status}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"tasks 종료 갱신 실패 task={task_id}: {exc}")


# --------------------------------------------------------------------------- #
# 조립 루트 — rclpy 노드(백그라운드 spin) + FastAPI(메인, uvicorn) 를 함께 띄운다.
# --------------------------------------------------------------------------- #
def main(args=None) -> None:
    import uvicorn

    from automato_control_service.detection_service import DetectionHandler
    from automato_control_service.patrol_api import create_app

    rclpy.init(args=args)
    node = PatrolControlNode()

    pool = automato_db.create_pool()
    node.set_db_pool(pool)

    # RP-79: 탐지 저장/중계/알림 서비스(/automato/save_detection) 등록.
    # ReentrantCallbackGroup 로 두어 텔레메트리 구독·다른 탐지 콜백과 병행 실행되게 한다
    # (DB 커넥션 풀이 동시성을 감당; notify/alert 는 핸들러가 백그라운드로 뺀다).
    detection_handler = DetectionHandler(pool, logger=node.get_logger())
    node.create_service(
        SaveDetection, SAVE_DETECTION_SRV, detection_handler.on_request,
        callback_group=ReentrantCallbackGroup())
    node.get_logger().info(f"탐지 저장 서비스 준비: {SAVE_DETECTION_SRV}")

    # 알려진 로봇의 Navigate 액션 클라이언트를 spin 시작 전에 미리 만든다(정리·가시성 목적).
    # RP-76 크래시의 실제 원인이던 self._clients 이름 충돌은 __init__ 에서 해결했다.
    try:
        robot_ids = automato_db.get_availability_snapshot(pool)["robots"]
        node.prewarm_clients(robot_ids)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().warn(
            f"액션 클라이언트 프리웜 실패(런타임 생성으로 폴백): {exc}")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, name="rclpy_spin", daemon=True)
    spin_thread.start()

    app = create_app(node, pool)
    port = int(os.environ.get("ACS_API_PORT", "8200"))
    node.get_logger().info(
        f"Automato Control Service (순찰) HTTP API → http://0.0.0.0:{port}")

    try:
        uvicorn.run(app, host="0.0.0.0", port=port, log_level="info")
    except KeyboardInterrupt:
        pass
    finally:
        detection_handler.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        try:
            pool.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()

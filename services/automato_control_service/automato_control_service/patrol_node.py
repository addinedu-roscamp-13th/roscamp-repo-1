#!/usr/bin/env python3
"""RP-78 ② ROS2 노드 — 텔레메트리 캐시 + Navigate 액션 클라이언트 + 순찰 디스패치.

이 파일은 'ROS 표면(로봇과의 실제 통신)'을 담당한다.
  - 구독:  /{robot_id}/telemetry (RobotTelemetry, 1Hz, 로봇 수만큼) → 로봇별 최신 상태 캐시
  - 발신:  /{robot_id}/navigate (Navigate 액션) → DG(DG Control Service) 경유로 경로(배열) 하달
  - 종료:  방문 결과에 따라 tasks 를 COMPLETED/COMPLETED_PARTIAL/FAILED 로 마감(automato_db)

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
from datetime import datetime, timezone

from automato_interfaces.action import Dock, Navigate
from automato_interfaces.msg import FleetTelemetry
from automato_interfaces.srv import SaveDetection
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_control_service import automato_db
from automato_control_service.fleet_collector import (
    DEFAULT_ROBOT_IDS,
    LEGACY_FLEET_TOPIC,
    robot_telemetry_topic,
    subscribe_per_robot,
)
from automato_control_service.patrol_config import (
    REAP_INTERVAL_SEC,
    RESERVATION_TTL_SEC,
    SAVE_DETECTION_SRV,
)
from automato_control_service import patrol_notify
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
        # 순찰 종료·실패 알림을 보낼 Web Service base URL. 탐지 저장(detection_service)과
        # 같은 env 를 공유해 두 경로가 같은 백엔드를 가리키게 한다.
        self._web_url = os.environ.get(
            "AUTOMATO_WEB_SERVICE_URL", "http://localhost:8100")
        # robot_id -> Navigate ActionClient.
        # ⚠️ 이름 주의: rclpy.Node 는 서비스 클라이언트 목록을 self._clients(리스트)로
        # 보관하고 node.clients 프로퍼티로 노출한다. 여기에 self._clients 를 dict 로
        # 덮으면 executor 가 node.clients 를 순회할 때 dict 의 '키(robot_id 문자열)'가
        # 나와 죽는다("'str' object has no attribute ...", RP-76). → 반드시 다른 이름 사용.
        self._action_clients = {}
        # robot_id -> Dock ActionClient (충전소 정밀 도킹, E4-6/7 · 22-1).
        # Navigate 와 같은 캐시 규칙이며 같은 락으로 보호한다.
        self._dock_clients = {}
        self._action_clients_lock = threading.Lock()

        # 라우팅/예약 엔진(공유 단일 인스턴스). 첫 순찰 때 그래프를 로드해 생성한다.
        self._engine = None
        self._engine_lock = threading.Lock()

        # 교통관제 알고리즘(세그먼트 이동·통로 예약·룩어헤드·막힘 우회)은 별도 클래스로
        # 분리(composition). 노드는 필요한 것(logger·engine·client)을 넘겨주고 위임만 한다.
        # wp_meta·블랙리스트는 디스패처가 소유하며, 그래프 로드 시 노드가 wp_meta 를 채운다.
        self._dispatcher = PatrolDispatcher(self.get_logger())

        # 텔레메트리 상시 구독(1Hz) — 로봇별 /{robot_id}/telemetry
        self.declare_parameter("robot_ids", DEFAULT_ROBOT_IDS)
        self.declare_parameter("legacy_input", True)
        robot_ids = list(self.get_parameter("robot_ids").value)
        legacy_input = bool(self.get_parameter("legacy_input").value)
        subscribe_per_robot(self, robot_ids, self._on_robot_telemetry)
        # [삭제 예정] 팀원의 DG 이전 전까지 옛 취합 경로도 함께 받는다.
        if legacy_input:
            self.create_subscription(
                FleetTelemetry, LEGACY_FLEET_TOPIC, self._on_fleet, 10)

        # 죽은 예약 주기 회수. 엔진이 아직 없으면(순찰 전) 아무 일도 안 한다.
        self.create_timer(REAP_INTERVAL_SEC, self._reap_expired)

        self.get_logger().info(
            f"순찰 제어 노드 준비: 구독 {[robot_telemetry_topic(r) for r in robot_ids]}"
            f"{f' + 옛 {LEGACY_FLEET_TOPIC}' if legacy_input else ''}, "
            "하달 /<robot_id>/navigate (세그먼트 단위 + 통로 예약)")

    # ---------------------------- 주입/구독 ---------------------------- #
    def set_db_pool(self, pool) -> None:
        self._db_pool = pool

    def _on_robot_telemetry(self, robot_id, msg) -> None:
        self.cache.update_from_robot(robot_id, msg, time.time())

    def _on_fleet(self, msg: FleetTelemetry) -> None:
        """[삭제 예정] 옛 /automato/telemetry/fleet 경로."""
        self.cache.update_from_fleet(msg, time.time())
        self.get_logger().info(
            "[삭제 예정] 옛 fleet 경로로 텔레메트리 수신 중 — DG 이전 후 "
            "legacy_input 을 끄세요", throttle_duration_sec=30.0)

    # ---------------------------- 엔진/클라이언트 ---------------------------- #
    def _reap_expired(self) -> None:
        """TTL 이 지난 죽은 예약을 회수한다(REAP_INTERVAL_SEC 마다).

        ⚠️ _get_engine() 을 부르지 않는다 — 그건 없으면 DB 를 읽어 '만들어' 버린다.
        순찰을 한 번도 안 돌린 상태에서 청소 타이머가 엔진을 깨우는 건 부작용이다.
        예약이 생기려면 엔진이 이미 있어야 하므로, 없을 때 할 일도 없다.
        """
        engine = self._engine
        if engine is None:
            return
        reaped = engine.reap_expired()
        if reaped:
            # 정상 동작에서는 나오지 않는다. 찍혔다면 로봇/스레드가 죽었거나 하트비트가
            # 안 돈 것이므로 원인을 봐야 한다 → info 가 아니라 warn.
            self.get_logger().warn(
                f"죽은 예약 회수(하트비트 끊김): {reaped}")

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
                # 라우팅 그래프에는 '짝(pair)'을 넣지 않는다. 짝은 같은 자리에서 방향만
                # 바꾸는 촬영 전용 지점이라 통로(corridor)가 없다. 그래프에 노드로 섞이면
                # Dijkstra 가 '도달할 수 없는 목적지'를 후보로 잡을 수 있다.
                routing_nodes = [
                    w for w in graph["waypoints"] if w["pair_of"] is None]
                self._engine = RoutingEngine(
                    routing_nodes, graph["corridors"],
                    reservation_ttl=RESERVATION_TTL_SEC)
                # wp_meta 는 디스패처가 소유(세그먼트 하달 시 좌표/촬영 여부에 사용) → 여기서 채운다.
                # 이쪽은 짝까지 '전부' 넣는다 — 짝을 하달하려면 그 좌표와 yaw 가 필요하다.
                self._dispatcher.wp_meta = {
                    w["waypoint_id"]: {
                        "x": w["x"], "y": w["y"],
                        "yaw": w["yaw"], "capture": w["is_patrol_point"],
                    }
                    for w in graph["waypoints"]
                }
                # 부모 → 짝 맵. 디스패처가 부모 도착 직후 이 짝을 추가로 하달한다.
                # 짝 관계는 정적이라 기동 시 1회만 만든다(DB 왕복 없음).
                self._dispatcher.pair_of = {
                    w["pair_of"]: w["waypoint_id"]
                    for w in graph["waypoints"] if w["pair_of"] is not None
                }
                self.get_logger().info(
                    f"라우팅 그래프 로드: 노드 {len(routing_nodes)}"
                    f"(짝 {len(self._dispatcher.pair_of)} 제외) / "
                    f"통로 {len(graph['corridors'])}")
                if not graph["corridors"]:
                    self.get_logger().warn(
                        "corridors 가 비어 있음 — 순찰 이동이 모두 skip 될 수 있음"
                        "(DB corridors 시드 확인)")
            return self._engine

    def _start_waypoint_for(self, robot_id: str):
        """이 로봇이 순찰을 시작하는 노드(= 전용 충전소의 진입 노드). 실패 시 None.

        순찰은 로봇이 자기 충전소에 도킹한 상태에서 시작하므로, 출발 노드는 로봇마다 다르다
        (dg_01→22, dg_02→23, dg_03→24). DB 조회가 실패하거나 충전소가 등록돼 있지 않으면
        None 을 돌려주고, 디스패처가 설정 상수(PATROL_START_WAYPOINT_ID)로 폴백한다.
        """
        if self._db_pool is None:
            return None
        try:
            wp = automato_db.get_patrol_start_waypoint(self._db_pool, robot_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"{robot_id} 충전소 시작 노드 조회 실패(설정 상수로 폴백): {exc}")
            return None
        if wp is None:
            self.get_logger().warn(
                f"{robot_id} 에 연결된 충전소(charge_point_id)가 없음 → 설정 상수로 폴백")
        else:
            self.get_logger().info(f"{robot_id} 순찰 시작 노드 = {wp}(전용 충전소)")
        return wp

    def _client_for(self, robot_id: str) -> ActionClient:
        with self._action_clients_lock:
            client = self._action_clients.get(robot_id)
            if client is None:
                client = ActionClient(self, Navigate, f"/{robot_id}/navigate")
                self._action_clients[robot_id] = client
            return client

    def _dock_client_for(self, robot_id: str) -> ActionClient:
        """robot_id 의 Dock 액션 클라이언트(/{robot_id}/dock). Navigate 와 같은 캐시 규칙."""
        with self._action_clients_lock:
            client = self._dock_clients.get(robot_id)
            if client is None:
                client = ActionClient(self, Dock, f"/{robot_id}/dock")
                self._dock_clients[robot_id] = client
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
        unvisited = []
        last_wp = None
        engine = None
        try:
            engine = self._get_engine()
            if engine is None:
                status = "FAILED_ABORTED"
            else:
                client = self._client_for(robot_id)
                status, unvisited, last_wp = self._dispatcher.run_patrol(
                    task_id, robot_id, waypoints, engine, client,
                    start_wp=self._start_waypoint_for(robot_id))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"디스패치 예외 task={task_id}: {exc}")
            status = "FAILED_ABORTED"
        # run_patrol 의 status 는 후속 처리를 가르려 4가지로 세분된다(COMPLETED /
        # COMPLETED_PARTIAL / FAILED_ABORTED / FAILED_BLOCKED). DB tasks.status 는 세 값만
        # 허용하므로 FAILED_* 는 모두 'FAILED' 로 마감한다.
        db_status = status if status in ("COMPLETED", "COMPLETED_PARTIAL") else "FAILED"
        if self._db_pool is not None:
            try:
                automato_db.set_task_status(self._db_pool, task_id, db_status)
                self.get_logger().info(
                    f"순찰 종료 task={task_id} → {status}(DB:{db_status})")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"tasks 종료 갱신 실패 task={task_id}: {exc}")
        # tasks 마감과 별개로 Web Service 에 순찰 결과를 알린다(E2 9-1/12/13).
        # 이 스레드는 순찰이 이미 끝난 자리라, 여기서 동기로 보내도 순찰 루프를 막지 않는다.
        self._report_task_result(task_id, robot_id, status, unvisited)
        # 복귀·도킹: 정상 종료(E4)와 막힘 확정(22-1)은 모두 전용 충전소로 복귀한다.
        # 순찰은 마지막 자리를 '쥔 채' 넘겼다(run_patrol) → 복귀가 그 자리를 이어받아 도킹
        # 후 해제한다. FAILED_ABORTED(로봇 중단/서버·디스패치 이상)는 복귀하지 않고 자리를
        # 유지한다(로봇이 그 자리에 물리적으로 서 있으므로 놓으면 남이 들어와 겹친다).
        if engine is not None and last_wp is not None:
            if status in ("COMPLETED", "COMPLETED_PARTIAL", "FAILED_BLOCKED"):
                self._return_and_dock(task_id, robot_id, engine, last_wp)
            else:  # FAILED_ABORTED
                self.get_logger().warn(
                    f"순찰 FAILED task={task_id} {robot_id} 위치 {last_wp} — 자리 유지")

    def _report_task_result(self, task_id, robot_id, status, unvisited) -> None:
        """순찰 종료를 Web Service 로 알린다.

        COMPLETED/COMPLETED_PARTIAL → patrol_completed(E2 9-1). summary(탐지 평균치)를
                                      DB 에서 집계해 싣는다.
        FAILED_BLOCKED  → 통로 막힘(22-1). event_logs(TRAFFIC_CONTROL) + task_failed
                          (reason=BLOCKED, recovery_action=RETURN_TO_CHARGER). 이후 복귀.
        FAILED_ABORTED  → 로봇 중단/서버·디스패치 이상. event_logs(HARDWARE_ERROR) +
                          task_failed(reason=HARDWARE_ERROR, recovery_action=NONE).
        """
        now = datetime.now(timezone.utc)
        if status in ("COMPLETED", "COMPLETED_PARTIAL"):
            summary = {"ripe_percent": 0, "unripe_percent": 0,
                       "rotten_percent": 0, "disease_percent": 0}
            if self._db_pool is not None:
                try:
                    summary = automato_db.get_detection_summary(
                        self._db_pool, task_id)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f"summary 집계 실패(0으로 발송) task={task_id}: {exc}")
            payload = patrol_notify.build_completed_payload(
                task_id=task_id, robot_id=robot_id, status=status,
                unvisited_waypoint_ids=unvisited, completed_at=now,
                summary=summary)
            patrol_notify.send_patrol_completed(
                self._web_url, payload, log=self.get_logger())
            return
        # 실패 — 막힘(22-1)과 그 외를 사유로 가른다. 영구 기록 먼저, 그다음 알림.
        if status == "FAILED_BLOCKED":
            event_type, reason, recovery = (
                "TRAFFIC_CONTROL", "BLOCKED", "RETURN_TO_CHARGER")
            evt_msg = (f"task {task_id} {robot_id} 통로 막힘으로 순찰 중단 "
                       f"→ 충전소 복귀")
        else:  # FAILED_ABORTED
            event_type, reason, recovery = (
                "HARDWARE_ERROR", "HARDWARE_ERROR", "NONE")
            evt_msg = f"task {task_id} 순찰 실패 (로봇 {robot_id})"
        if self._db_pool is not None:
            try:
                automato_db.save_event_log(
                    self._db_pool, robot_id=robot_id, task_id=task_id,
                    event_type=event_type, severity="CRITICAL",
                    message=evt_msg, created_at=now)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"event_logs 기록 실패 task={task_id}: {exc}")
        payload = patrol_notify.build_task_failed_payload(
            task_id=task_id, robot_id=robot_id, reason=reason,
            recovery_action=recovery, failed_at=now)
        patrol_notify.send_task_failed(
            self._web_url, payload, log=self.get_logger())

    # ---------------------------- E4 복귀·도킹 오케스트레이션 ---------------------------- #
    def _return_and_dock(self, task_id, robot_id, engine, last_wp) -> None:
        """E4: 순찰을 마친 로봇을 전용 충전소로 복귀시키고 도킹한다(같은 task_id 유지).

        순찰이 '쥔 채' 넘긴 마지막 자리(last_wp)를 복귀 주행이 이어받아 충전소 진입 노드
        까지 가고, 도킹 성공 시점에 그 자리까지 한 번에 해제한다(문서 E4 8번). 복귀는 새
        task 를 만들지 않는다 — 끝난 순찰의 task_id 를 그대로 쓴다(그 task 의 뒷정리).

        실패 세 갈래(충전소 미등록/복귀 막힘/도킹 실패)는 지금은 자리·로그만 정리한다.
        task_failed(DOCK_FAILED)·22-2 현장 정지(operational_status=IMMOBILIZED)는
        6·7단계에서 이 자리에 채운다.
        """
        # 1) 전용 충전소 + 도킹 마커 조회.
        charge, marker = None, None
        if self._db_pool is not None:
            try:
                charge = automato_db.get_charge_point(self._db_pool, robot_id)
                if charge is not None:
                    marker = automato_db.get_dock_marker(
                        self._db_pool, charge["task_point_id"])
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"충전소/마커 조회 실패 task={task_id}: {exc}")
        if charge is None or charge["waypoint_id"] not in self._dispatcher.wp_meta:
            self.get_logger().warn(
                f"{robot_id} 전용 충전소 미등록/그래프에 없음 → 복귀 생략, 자리 반납 "
                f"task={task_id}")
            engine.release(engine.node_slot(last_wp), robot_id)
            return

        target = charge["waypoint_id"]
        charge_point_id = charge["task_point_id"]
        nav_client = self._client_for(robot_id)
        dock_client = self._dock_client_for(robot_id)

        # 2) 복귀 주행 — 순찰 마지막 자리를 이어받아 충전소 진입 노드까지(촬영 없음).
        outcome, pos = self._dispatcher.drive_to_point(
            task_id, robot_id, last_wp, target, engine, nav_client)
        if outcome != "arrived":
            # 복귀 경로마저 막힘(skipped) 또는 중단(aborted) → 22-2 현장 정지.
            self.get_logger().warn(
                f"복귀 주행 실패({outcome}) task={task_id} {robot_id} 위치 {pos} "
                f"→ 현장 정지(22-2)")
            self._immobilize(task_id, robot_id, engine, pos)
            return

        # 3) 도킹 — 진입 노드 자리를 쥔 채(하트비트로 TTL 방어), N_dock 재시도.
        entry_slot = engine.node_slot(target)
        success, code, msg = self._dispatcher.dock(
            task_id, robot_id, charge_point_id, marker, dock_client,
            heartbeat=(engine, [entry_slot], robot_id))
        if success:
            # 문서 E4 8번: Dock 성공 → 예약 전부 해제. 복귀 도착 후엔 진입 노드 자리 하나만
            # 남아 있어, 그것을 놓으면 이 로봇의 예약이 완전히 빈다.
            engine.release(entry_slot, robot_id)
            self.get_logger().info(
                f"복귀·도킹 완료 task={task_id} {robot_id} @ {charge_point_id} "
                f"— 예약 전부 해제")
        else:
            # 도킹 N_dock 소진/마커 없음 → 진입 노드에 정지, 관리자 개입 대기.
            # 로봇이 진입 노드에 물리적으로 서 있으므로 자리는 놓지 않는다.
            self.get_logger().warn(
                f"도킹 실패(code={code}) task={task_id} {robot_id}: {msg} "
                f"→ 진입 노드 정지, DOCK_FAILED 알림")
            self._notify_dock_failed(task_id, robot_id, code, msg)

    def _notify_dock_failed(self, task_id, robot_id, code, msg) -> None:
        """도킹 N_dock 소진 시 작업 실패 알림(문서 E4 Dock 실패 · reason=DOCK_FAILED).

        복귀 주행 자체는 성공했고 마지막 도킹만 실패한 상황이라 recovery_action=NONE
        (로봇은 충전소 진입 노드에 서서 관리자 개입을 기다린다). tasks 상태는 이미 순찰
        종료값(COMPLETED/PARTIAL)으로 마감돼 있어 여기서 바꾸지 않는다 — 순찰은 끝났고
        뒷정리(도킹)만 실패한 것이다.
        """
        now = datetime.now(timezone.utc)
        payload = patrol_notify.build_task_failed_payload(
            task_id=task_id, robot_id=robot_id, reason="DOCK_FAILED",
            recovery_action="NONE", failed_at=now)
        patrol_notify.send_task_failed(
            self._web_url, payload, log=self.get_logger())
        self.get_logger().warn(
            f"DOCK_FAILED 알림 발송 task={task_id} {robot_id} (code={code}): {msg}")

    def _immobilize(self, task_id, robot_id, engine, pos) -> None:
        """22-2 현장 정지: 충전소 복귀조차 실패한 로봇을 IMMOBILIZED 로 세운다(문서 22-2).

        이 값은 사람이 로봇을 물리적으로 옮긴 뒤 NORMAL 로 되돌려야 풀린다(E1 가용 조건 1).
        '모든 예약 해제'는 갇힌 로봇의 예약이 다른 로봇들의 길을 영구히 막지 않게 하기
        위함이다 — 이 로봇을 배정에서 빼는 일은 nav_status 가 아니라 IMMOBILIZED 축이
        따로 보장하므로, 예약을 놓아도 다시 배정 후보로 올라오지 않는다.
        """
        now = datetime.now(timezone.utc)
        # 1) 예약 해제 — 복귀 주행 실패 후엔 서 있는 자리 하나만 쥐고 있다(_navigate 가
        #    나머지를 이미 반납). 그것까지 놓아 교통관제 데드락을 막는다.
        engine.release(engine.node_slot(pos), robot_id)
        # 2) operational_status = IMMOBILIZED + 3) event_logs 영구 기록.
        if self._db_pool is not None:
            try:
                automato_db.set_operational_status(
                    self._db_pool, robot_id, "IMMOBILIZED")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"IMMOBILIZED 설정 실패 task={task_id} {robot_id}: {exc}")
            try:
                automato_db.save_event_log(
                    self._db_pool, robot_id=robot_id, task_id=task_id,
                    event_type="TRAFFIC_CONTROL", severity="CRITICAL",
                    message=(f"task {task_id} {robot_id} 충전소 복귀 실패 "
                             f"→ 현장 정지(IMMOBILIZED). 관리자 개입 필요"),
                    created_at=now)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f"event_logs 기록 실패 task={task_id}: {exc}")
        # 4) task_failed 알림 — 복귀조차 불가라 recovery_action=NONE(그 자리 정지).
        payload = patrol_notify.build_task_failed_payload(
            task_id=task_id, robot_id=robot_id, reason="BLOCKED_UNRECOVERABLE",
            recovery_action="NONE", failed_at=now)
        patrol_notify.send_task_failed(
            self._web_url, payload, log=self.get_logger())
        self.get_logger().error(
            f"현장 정지(22-2) task={task_id} {robot_id} 위치 {pos} "
            f"— IMMOBILIZED, 관리자 개입 대기")


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

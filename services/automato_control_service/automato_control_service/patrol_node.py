#!/usr/bin/env python3
"""RP-78 ② ROS2 노드 — 텔레메트리 캐시 + Navigate 액션 클라이언트 + 세그먼트 디스패치.

이 파일은 '동작(로봇과의 실제 통신)'을 담당한다.
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

Phase 2 (교통관제):
  세그먼트(연속으로 예약 가능한 통로 묶음) 단위로 예약→배열 하달→도착→전부 해제를 반복한다.
    - (C) 다른 로봇이 통로 점유 → 예약 대기, 타임아웃 넘으면 순찰(최하위)이 양보(우회/미룸)
    - (B) 진짜 막힘(로봇이 result_code=1 보고) → 그 통로 N초 블랙리스트 → Dijkstra 우회 →
          우회 없으면 그 지점 건너뛰고 다음, 마지막에 1회 재시도
    - (A) 사람·물건 잠깐 막음은 로봇 Nav2가 자체 예산(순찰 2분×3)으로 처리 → ACS는 결과만 기다림
  통로 예약·경로 탐색은 routing_engine(독립 모듈)이 담당하고 여기선 호출만 한다.
"""
import copy
import os
import threading
import time

from automato_interfaces.action import Navigate
from automato_interfaces.msg import FleetTelemetry, Waypoint
from automato_interfaces.srv import SaveDetection
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_control_service import automato_db
from automato_control_service.routing_engine import Route, RoutingEngine

FLEET_TOPIC = "/automato/telemetry/fleet"
# RP-79: DG 가 waypoint 마다 탐지 결과를 넘기는 ROS2 Service (ACS 가 서버).
SAVE_DETECTION_SRV = "/automato/save_detection"


def _envf(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


# 액션 서버 접속 대기 / Goal 수락 대기 / 세그먼트(1 waypoint) 결과 대기(초)
SERVER_WAIT_SEC = _envf("ACS_SERVER_WAIT_SEC", 5.0)
GOAL_ACCEPT_TIMEOUT_SEC = _envf("ACS_GOAL_ACCEPT_TIMEOUT_SEC", 30.0)
# 순찰은 로봇 Nav2가 자체 재시도(2분×3=6분)를 하므로 그보다 넉넉히 기다린다.
SEGMENT_TIMEOUT_SEC = _envf("ACS_SEGMENT_TIMEOUT_SEC", 420.0)
# 통로 예약 대기(양보 전) / 예약 폴링 간격
RESERVE_WAIT_SEC = _envf("ACS_RESERVE_WAIT_SEC", 30.0)
RESERVE_POLL_SEC = _envf("ACS_RESERVE_POLL_SEC", 1.0)
# 막힘/양보 통로를 재계획에서 제외해 둘 시간(N초 블랙리스트)
BLOCK_TTL_SEC = _envf("ACS_BLOCK_TTL_SEC", 30.0)
# 이동 대기 중 예약 유지용 하트비트 간격 / 엔진 예약 TTL(하트비트보다 커야 함)
HEARTBEAT_SEC = _envf("ACS_HEARTBEAT_SEC", 5.0)
RESERVATION_TTL_SEC = _envf("ACS_RESERVATION_TTL_SEC", 15.0)


# --------------------------------------------------------------------------- #
# 텔레메트리 캐시 — 로봇별 '그 로봇의 전체 상태' 1건을 메모리에 보관(수신마다 덮어씀).
# --------------------------------------------------------------------------- #
class TelemetryCache:
    def __init__(self):
        self._lock = threading.Lock()   # 콜백 스레드(쓰기) ↔ API 스레드(읽기) 보호
        self._data = {}                 # robot_id -> entry dict

    def update_from_fleet(self, msg: FleetTelemetry, rx_wall: float) -> None:
        """FleetTelemetry 1건을 받아 로봇별로 병합 저장한다.

        같은 메시지가 ddagos/ddagis 두 배열을 함께 담고 있어 각각 순회하며 robot_id로 매칭.
        이번 메시지에 없는 로봇은 지우지 않고 이전 값을 유지 → ddago_stamp가 늙어
        staleness(미수신)로 자연히 드러난다. dg_03처럼 팔이 없으면 ddagi는 계속 None(정상).
        """
        with self._lock:
            for d in msg.ddagos:
                entry = self._data.setdefault(d.robot_id, {"robot_id": d.robot_id})
                entry["ddago"] = {
                    "nav_status": d.nav_status,
                    "is_charging": bool(d.is_charging),   # 스냅샷엔 담되 판정엔 안 씀
                    "task_id": int(d.task_id),
                    "x": float(d.x), "y": float(d.y), "yaw": float(d.yaw),
                    "battery_percent": float(d.battery_percent),
                    "battery_voltage": float(d.battery_voltage),
                    "us_range_m": float(d.us_range_m),
                }
                entry["ddago_stamp"] = (
                    d.header.stamp.sec + d.header.stamp.nanosec * 1e-9)
                entry["local_rx"] = rx_wall
            for a in msg.ddagis:
                entry = self._data.setdefault(a.robot_id, {"robot_id": a.robot_id})
                entry["ddagi"] = {
                    "is_paused": bool(a.is_paused),
                    "task_id": int(a.task_id),
                    "joint_angles": [float(v) for v in a.joint_angles],
                    "tcp_coords": [float(v) for v in a.tcp_coords],
                    "servo_health": [
                        {
                            "joint_no": int(s.joint_no),
                            "voltage_ok": bool(s.voltage_ok),
                            "temperature": int(s.temperature),
                            "current": float(s.current),
                            "overload": bool(s.overload),
                            "gripper_value": int(s.gripper_value),
                        }
                        for s in a.servo_health
                    ],
                }
                entry["ddagi_stamp"] = (
                    a.header.stamp.sec + a.header.stamp.nanosec * 1e-9)

    def get(self, robot_id: str):
        """가용 판정용 얕은 복사본(없으면 None)."""
        with self._lock:
            entry = self._data.get(robot_id)
            return dict(entry) if entry else None

    def snapshot(self, robot_id: str):
        """DB 스냅샷 저장용 깊은 복사본(JSON 직렬화 대상). 없으면 None."""
        with self._lock:
            entry = self._data.get(robot_id)
            return copy.deepcopy(entry) if entry else None


def _spin_wait(future, timeout: float):
    """executor(백그라운드 spin)가 완료해 줄 future를, 다른 스레드에서 기다린다.

    executor가 이미 spin 중이므로 여기서 또 spin하면 안 된다. done 콜백이 Event를
    set 하게 걸고 Event를 기다린다. 타임아웃/예외 시 None.
    """
    done = threading.Event()
    future.add_done_callback(lambda _f: done.set())
    if not done.wait(timeout):
        return None
    try:
        return future.result()
    except Exception:  # noqa: BLE001
        return None


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
        self._wp_meta = {}                         # waypoint_id -> {"x","y","yaw","capture"}
        self._engine_lock = threading.Lock()

        # 막힘/양보로 잠시 회피할 통로: corridor_id -> 만료 monotonic 시각
        self._blacklist = {}
        self._bl_lock = threading.Lock()

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
                self._wp_meta = {
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

    # ---------------------------- 블랙리스트(시간 기반) ---------------------------- #
    def _blacklist_add(self, corridor_id) -> None:
        with self._bl_lock:
            self._blacklist[corridor_id] = time.monotonic() + BLOCK_TTL_SEC

    def _blacklist_active(self) -> set:
        now = time.monotonic()
        with self._bl_lock:
            for cid in [c for c, exp in self._blacklist.items() if exp <= now]:
                del self._blacklist[cid]
            return set(self._blacklist.keys())

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
        try:
            status = self._run_patrol(task_id, robot_id, waypoints)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"디스패치 예외 task={task_id}: {exc}")
            status = "FAILED"
        if self._db_pool is not None:
            try:
                automato_db.set_task_status(self._db_pool, task_id, status)
                self.get_logger().info(f"순찰 종료 task={task_id} → {status}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"tasks 종료 갱신 실패 task={task_id}: {exc}")

    # ---------------------------- 순찰 본체 ---------------------------- #
    def _run_patrol(self, task_id: int, robot_id: str, waypoints: list) -> str:
        """순찰 지점을 순서대로 방문. 반환: 'DONE' | 'PARTIAL' | 'FAILED'."""
        client = self._client_for(robot_id)
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            self.get_logger().warn(
                f"{robot_id} Navigate 액션 서버 미기동 → task {task_id} FAILED")
            return "FAILED"

        engine = self._get_engine()
        if engine is None:
            return "FAILED"

        targets = [wp["waypoint_id"] for wp in waypoints]
        if not targets:
            return "DONE"                      # 방문할 지점이 없음

        visited = set()
        # 첫 순찰 지점: 시작 노드를 알 수 없어(위치→노드 매핑은 향후 과제) 통로 예약 없이 직접 접근.
        # 세그먼트 하달 경로를 재사용하되, waypoint 1개짜리 배열로 보낸다(순찰점이므로 촬영).
        code, _ = self._dispatch_segment(
            client, task_id, [targets[0]], capture_on_last=True)
        if code == 0:
            visited.add(targets[0])
            current = targets[0]
        else:
            self.get_logger().warn(f"첫 순찰 지점 도달 실패 → task {task_id} FAILED")
            return "FAILED"

        # 나머지 지점: 세그먼트(연속 통로 묶음) 단위로 이동
        skipped = []
        for target in targets[1:]:
            outcome, current = self._navigate(
                engine, client, task_id, robot_id, current, target)
            if outcome == "arrived":
                visited.add(target)
            elif outcome == "skipped":
                skipped.append(target)
            else:  # aborted (중단)
                return "FAILED"

        # 건너뛴 지점 마지막에 1회 재시도
        for target in list(skipped):
            outcome, current = self._navigate(
                engine, client, task_id, robot_id, current, target)
            if outcome == "arrived":
                visited.add(target)
                skipped.remove(target)
            elif outcome == "aborted":
                return "FAILED"

        if len(visited) == len(targets):
            return "DONE"
        if len(visited) <= 1:                  # 사실상 첫 지점만 방문
            return "FAILED"
        return "PARTIAL"

    def _navigate(self, engine, client, task_id, robot_id, current, target):
        """current→target 까지 '세그먼트' 단위로 이동. 반환: (outcome, 도달한 노드).

        한 번의 while 반복 = 연속으로 예약 가능한 만큼을 한 세그먼트로 묶어 Waypoint[]
        배열로 한 번에 하달 → 도착하면 그 통로들을 전부 해제 → 남았으면 다음 세그먼트.
        outcome: 'arrived'(목표 도달) | 'skipped'(우회 불가로 포기) | 'aborted'(중단).
        """
        attempt_block = set()   # 이번 target 시도에서 회피할 통로(예약실패/막힘 누적)
        while current != target:
            route = self._plan_route(engine, current, target, attempt_block)
            if route is None:
                self.get_logger().warn(
                    f"경로 없음 task={task_id} {current}→{target} → 건너뜀")
                return "skipped", current

            hops = route.hops()                 # [(다음 노드, 통로 id), ...] (최소 1개)
            # 1) 첫 통로는 대기하며 예약(코앞이라 이게 없으면 한 발도 못 감).
            #    끝내 못 잡으면 양보: 그 통로 회피하고 재계획.
            first_wp, first_cid = hops[0]
            if not self._reserve_with_wait(engine, first_cid, robot_id):
                self._blacklist_add(first_cid)
                attempt_block.add(first_cid)
                continue

            # 2) 이어지는 통로는 '지금 당장 잡히는 만큼만' 예약해 세그먼트로 묶는다(대기 없음).
            seg_wps = [first_wp]
            seg_cids = [first_cid]
            for next_wp, cid in hops[1:]:
                if not engine.try_reserve(cid, robot_id):
                    break                       # 세그먼트 끊김 → 여기까지만 하달
                seg_wps.append(next_wp)
                seg_cids.append(cid)
            reached = (seg_wps[-1] == target)   # 세그먼트 끝이 목표면 마지막에 촬영

            self.get_logger().info(
                f"세그먼트 하달 task={task_id} {current}→{seg_wps} "
                f"통로={seg_cids} 촬영={reached}")

            # 3) 세그먼트 배열 하달(대기 중 하트비트로 전부 유지) / 4) 무조건 전부 해제.
            try:
                code, last_wp = self._dispatch_segment(
                    client, task_id, seg_wps, capture_on_last=reached,
                    heartbeat=(engine, list(seg_cids), robot_id))
            finally:
                for cid in seg_cids:
                    engine.release(cid, robot_id)   # 도착/실패/중단/예외 모두 해제

            if code == 2:
                self.get_logger().warn(f"중단 보고 task={task_id} → 순찰 실패")
                return "aborted", current
            if code == 0:
                current = seg_wps[-1]            # 세그먼트 끝까지 도착 → 전진
                continue

            # code == 1: 진짜 막힘. 로봇이 실제 도달한 지점(last_wp)까지는 전진하고,
            # 그 다음 '막힌 통로'만 블랙리스트 후 우회 재계획.
            current, blocked_cid = self._segment_progress(
                current, seg_wps, seg_cids, last_wp)
            if blocked_cid is not None:
                self.get_logger().warn(
                    f"세그먼트 막힘 task={task_id} 통로 {blocked_cid} "
                    f"(로봇 위치 {current}) → 블랙리스트 후 우회")
                self._blacklist_add(blocked_cid)
                attempt_block.add(blocked_cid)

        return "arrived", current

    @staticmethod
    def _segment_progress(current, seg_wps, seg_cids, last_wp):
        """막힘(code=1) 시 로봇이 실제 도달한 노드와 '막힌 통로'를 추정한다.

        세그먼트 경로:  current -[seg_cids[0]]-> seg_wps[0] -[seg_cids[1]]-> seg_wps[1] ...
        로봇이 last_wp 까지 갔다면(Result.last_waypoint_id) 그 다음 통로가 막힌 것이다.
        반환: (새 current, 막힌 corridor_id | None). last_wp 를 못 알아보면 진입 통로로 간주.
        """
        path = [current] + list(seg_wps)        # 시작점 포함 노드 나열
        try:
            j = path.index(last_wp)             # 로봇이 도달한 위치(인덱스)
        except ValueError:
            return current, seg_cids[0]         # 알 수 없음 → 진입 통로가 막힌 것으로 간주
        blocked = seg_cids[j] if j < len(seg_cids) else None  # path[j]->path[j+1] 통로
        return path[j], blocked

    def _plan_route(self, engine, current, target, attempt_block):
        """current→target 경로. 정상 순찰은 인접 직행, 막히면 Dijkstra 우회. 없으면 None."""
        blocked = set(attempt_block) | self._blacklist_active()
        direct = engine.corridor_between(current, target)
        if direct is not None and direct not in blocked:
            return Route((current, target), (direct,))   # 인접 지점 직행(세그먼트 1개)
        return engine.find_path(current, target, blocked=blocked)

    def _reserve_with_wait(self, engine, corridor_id, robot_id) -> bool:
        """통로 예약을 대기하며 재시도. RESERVE_WAIT_SEC 넘으면 양보(False)."""
        deadline = time.monotonic() + RESERVE_WAIT_SEC
        while True:
            if engine.try_reserve(corridor_id, robot_id):
                return True
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f"통로 {corridor_id} 예약 대기 타임아웃 → 순찰 양보")
                return False
            time.sleep(RESERVE_POLL_SEC)

    # ---------------------------- 세그먼트 하달 ---------------------------- #
    def _dispatch_segment(self, client, task_id, waypoint_ids,
                          capture_on_last, heartbeat=None):
        """확보된 세그먼트(연속 waypoint 목록)를 Navigate Goal(Waypoint[] 배열)로 한 번에 하달.

        waypoint_ids: [세그먼트 첫 노드 ... 끝 노드] — 예약을 확보한 통로들을 지나는 경로.
        capture_on_last: 세그먼트 끝이 순찰 목표면 True → 마지막 waypoint 만 capture=True
                         (도착·정지 후 촬영). 중간 노드는 통과만 하므로 전부 capture=False.
        heartbeat=(engine, [cid...], robot_id): 결과 대기 중 세그먼트의 모든 통로 예약을 갱신.
        반환: (result_code, last_waypoint_id). result_code 0 성공/1 실패·막힘/2 중단.
        """
        wps = []
        last_idx = len(waypoint_ids) - 1
        for i, wid in enumerate(waypoint_ids):
            m = self._wp_meta.get(wid, {})
            wps.append(Waypoint(
                waypoint_id=int(wid),
                x=float(m.get("x", 0.0)),
                y=float(m.get("y", 0.0)),
                yaw=float(m.get("yaw") or 0.0),          # 비순찰점 yaw=None → 0.0
                capture=bool(capture_on_last if i == last_idx else False),
            ))
        goal = Navigate.Goal()
        goal.task_id = int(task_id)
        goal.waypoints = wps

        goal_handle = _spin_wait(
            client.send_goal_async(goal), GOAL_ACCEPT_TIMEOUT_SEC)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn(
                f"Goal 거부/수락 타임아웃 task={task_id} waypoints={waypoint_ids}")
            return 1, None

        return self._await_result(goal_handle.get_result_async(), heartbeat)

    def _await_result(self, result_future, heartbeat):
        """결과 대기. 대기 중 HEARTBEAT_SEC마다 세그먼트의 모든 통로 예약을 갱신한다.

        반환: (result_code, last_waypoint_id). 실패/타임아웃/파싱실패 시 (1, None).
        """
        done = threading.Event()
        result_future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + SEGMENT_TIMEOUT_SEC
        while not done.wait(HEARTBEAT_SEC):
            if heartbeat is not None:
                engine, cids, robot_id = heartbeat
                for cid in cids:
                    engine.heartbeat(cid, robot_id)
            if time.monotonic() >= deadline:
                self.get_logger().warn("세그먼트 결과 대기 타임아웃 → 실패 취급")
                return 1, None
        try:
            res = result_future.result().result
            return int(res.result_code), int(res.last_waypoint_id)
        except Exception:  # noqa: BLE001
            return 1, None


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

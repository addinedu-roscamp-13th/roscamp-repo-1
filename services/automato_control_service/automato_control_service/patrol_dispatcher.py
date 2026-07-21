#!/usr/bin/env python3
"""RP-78 교통관제 — 순찰 세그먼트 이동·통로 예약·룩어헤드·막힘 우회 (composition 분리).

patrol_node(ROS 표면)에서 '동작 결정' 로직을 떼어낸 클래스. rclpy 노드를 직접
참조하지 않고, 필요한 것(logger, 라우팅 engine, Navigate 액션 client)을 인자로 받아
동작한다 → ROS 를 안 띄우고 fake engine/client 로 단위 테스트할 수 있다.

세그먼트(연속으로 예약 가능한 통로 묶음) 단위로 예약→배열 하달→도착→전부 해제를 반복:
  - (C) 다른 로봇이 통로 점유 → 예약 대기, 타임아웃 넘으면 순찰(최하위)이 양보(우회/미룸)
  - (B) 진짜 막힘(로봇이 result_code=1 보고) → 그 통로 N초 블랙리스트 → Dijkstra 우회 →
        우회 없으면 그 지점 건너뛰고 다음, 마지막에 1회 재시도
  - (A) 사람·물건 잠깐 막음은 로봇 Nav2가 자체 예산(순찰 2분×3)으로 처리 → 결과만 기다림
통로 예약·경로 탐색은 routing_engine(독립 모듈)이 담당하고 여기선 호출만 한다.
"""
import threading
import time

from automato_interfaces.action import Navigate
from automato_interfaces.msg import Waypoint

from automato_control_service.patrol_config import (
    BLOCK_TTL_SEC,
    GOAL_ACCEPT_TIMEOUT_SEC,
    HEARTBEAT_SEC,
    PATROL_START_WAYPOINT_ID,
    RESERVE_POLL_SEC,
    RESERVE_WAIT_SEC,
    SEGMENT_TIMEOUT_SEC,
    SERVER_WAIT_SEC,
)
from automato_control_service.routing_engine import Route


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


class PatrolDispatcher:
    """순찰 1건을 세그먼트 단위로 실행하는 교통관제 로직(로봇당 스레드가 run_patrol 호출).

    노드에서 넘겨받는 것:
      - logger: 생성자에서 1회 (ROS 로거를 그대로 사용).
      - engine/client: run_patrol 인자로 매번 (rclpy 엔티티는 노드가 만든다).
    스스로 소유하는 공유 상태(모든 로봇 스레드가 함께 씀):
      - wp_meta : waypoint_id -> {x,y,yaw,capture}. 그래프 로드 시 노드가 채운다(1회, 읽기전용).
      - pair_of : 부모 waypoint_id -> 짝 waypoint_id. 같이 채워진다(1회, 읽기전용).
      - _blacklist : 막힘/양보로 잠시 회피할 통로(시간 만료). 자체 락으로 보호.
    """

    def __init__(self, logger):
        self._log = logger
        # waypoint_id -> {"x","y","yaw","capture"}; 그래프 로드 시 노드가 채운다.
        self.wp_meta = {}
        # 부모 waypoint_id -> 짝 waypoint_id (같은 자리, 반대 촬영 방향).
        # 부모에 도착해 촬영한 뒤 이 짝을 추가로 하달해 제자리 회전 촬영을 시킨다.
        self.pair_of = {}
        # 막힘/양보로 잠시 회피할 통로: corridor_id -> 만료 monotonic 시각
        self._blacklist = {}
        self._bl_lock = threading.Lock()

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

    def blacklist_view(self, engine) -> dict:
        """회피 중인 목록을 통로/지점으로 갈라서 돌려준다(관측 도구용).

        _blacklist 는 통로 id(양수)와 자리 id(음수)를 한 바구니에 담는다. 화면이 이걸
        그대로 받으면 음수를 통로 번호로 오해해 엉뚱한 선을 칠한다.
        반환: {"corridors": [id...], "nodes": [노드id...]}
        """
        corridors, nodes = self._split_blocked(engine, self._blacklist_active())
        return {"corridors": sorted(corridors), "nodes": sorted(nodes)}

    # ---------------------------- 순찰 본체 ---------------------------- #
    def run_patrol(self, task_id, robot_id, waypoints, engine, client,
                   start_wp=None) -> str:
        """순찰 지점을 순서대로 방문. 반환: 'COMPLETED' | 'COMPLETED_PARTIAL' | 'FAILED'.

        engine/client 는 노드(ROS 표면)가 만들어 넘긴다 — 이 클래스는 rclpy 엔티티를
        생성하지 않고 받은 것만 사용한다.
        start_wp: 이 로봇이 서 있는 출발 노드(전용 충전소의 진입 노드). None 이면
                  설정 상수 PATROL_START_WAYPOINT_ID 로 폴백한다.
        """
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            self._log.warn(
                f"{robot_id} Navigate 액션 서버 미기동 → task {task_id} FAILED")
            return "FAILED"

        targets = [wp["waypoint_id"] for wp in waypoints]
        if not targets:
            return "COMPLETED"                 # 방문할 지점이 없음

        visited = set()
        # 순찰 시작 노드(로봇 전용 충전소의 진입 노드). 그래프(wp_meta)에 있으면 current 로 두고
        # 첫 순찰 지점도 _navigate 로 이동해 '첫 구간까지 통로 예약'으로 보호한다.
        # 미설정/미상이면 옛 동작으로 폴백: 첫 지점만 예약 없이 직행(이 구간은 통로 보호 없음).
        # start_wp(로봇별, DB 유도)가 우선이고, 없을 때만 전역 설정 상수를 쓴다.
        start = start_wp if start_wp is not None else PATROL_START_WAYPOINT_ID
        if start and start in self.wp_meta:
            current = start
            remaining = targets
            self._log.info(f"순찰 시작 노드 {start} 에서 출발 task={task_id}")
        else:
            self._log.warn(
                f"순찰 시작 waypoint({start}) 미설정/그래프에 없음 → 첫 지점 예약 없이 "
                f"직행(폴백) task={task_id}")
            code, _ = self._dispatch_segment(
                client, task_id, [targets[0]], capture_on_last=True)
            if code != 0:
                self._log.warn(f"첫 순찰 지점 도달 실패 → task {task_id} FAILED")
                return "FAILED"
            visited.add(targets[0])
            current = targets[0]
            remaining = targets[1:]

        # 출발선에서 '지금 서 있는 자리'부터 잡는다. 첫 구간의 _navigate 가 잡아주긴
        # 하지만 그 전에 짝 촬영(제자리 회전)이 낀 경로가 있어, 그동안 이 로봇이 예약표에
        # 안 보이면 남이 그 지점으로 들어온다. 이 예약은 구간마다 _navigate 가 이어받아
        # 순찰 내내 유지되고, 맨 끝에서 아래 finally 가 반납한다.
        start_slot = engine.node_slot(current)
        if engine.try_reserve(start_slot, robot_id):
            self._log.info(f"출발 지점 {current} 자리 확보 task={task_id}")
        else:
            self._log.warn(
                f"출발 지점 {current} 자리를 남(로봇 {engine.holder_of(start_slot)})이 "
                f"쥐고 있다 task={task_id} — 예약표와 실제 위치가 어긋남")

        # 순찰 지점: 세그먼트(연속 통로 묶음) 단위로 이동(시작 노드가 있으면 첫 지점부터)
        skipped = []
        # 여기서부터 로봇은 '서 있는 자리'를 계속 쥔 채 구간을 이어간다(_navigate 가
        # 서로 넘겨준다). 마지막 한 장은 순찰 전체를 소유하는 이 함수가 반납해야 하므로
        # 어떤 경로로 빠져나가든 finally 를 지나게 감싼다.
        try:
            # 폴백으로 첫 지점에 직행한 경우, 그 지점의 짝 촬영은 여기서 따로 챙긴다.
            # 통로는 못 쥐었어도(예약 없이 간 구간) 자리는 위에서 잡았으므로 회전하는
            # 동안 그 자리가 하트비트로 유지된다.
            if visited and not self._capture_pair(
                    client, task_id, targets[0], engine, [start_slot], robot_id):
                visited.discard(targets[0])
                skipped.append(targets[0])
            for target in remaining:
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
                return "COMPLETED"
            if len(visited) <= 1:                  # 사실상 첫 지점만 방문
                return "FAILED"
            return "COMPLETED_PARTIAL"
        finally:
            # 순찰이 끝나면 마지막 자리를 반납한다. 로봇은 아직 거기 서 있으므로 이
            # 시점부터 교통관제에 안 보인다 — 충전소 복귀가 붙으면 복귀 경로가 자리를
            # 이어받게 되고, 그때 이 반납은 복귀 도착 지점으로 옮겨가야 한다.
            engine.release(engine.node_slot(current), robot_id)
            self._log.info(
                f"순찰 종료 task={task_id} 지점 {current} 자리 반납 "
                f"(복귀 로직 전까지 이 지점은 교통관제에 비어 보인다)")

    def _navigate(self, engine, client, task_id, robot_id, current, target):
        """current→target 까지 '세그먼트 + 룩어헤드'로 이동. 반환: (outcome, 도달한 노드).

        상태 2개로 움직인다:
          - held: 지금 예약(점유)한 통로들. 하트비트로 유지하며 항상 예약표=현실이 되게 한다.
          - seg : 다음에 하달할 세그먼트. 룩어헤드가 주행 중 미리 채워두면 재확보 없이 이어 달린다.
        선획득 후해제: 도착 시 '다음 구간을 먼저 잡았으면' 지나온 통로만 반납, 못 잡았으면
        '서 있는 통로'만 남기고 반납 → 순간적으로 아무 통로도 예약 안 된 구간이 안 생긴다.
        outcome: 'arrived'(목표 도달) | 'skipped'(우회 불가로 포기) | 'aborted'(중단).
        """
        attempt_block = set()   # 이번 target 시도에서 회피할 통로(예약실패/막힘 누적)
        held = []               # 지금 예약(점유)한 자원들 — dispatch 하트비트에 live 로 넘김
        seg = None              # 다음에 하달할 세그먼트 (룩어헤드가 미리 채웠을 수 있음)
        # 출발 전에 '지금 서 있는 자리'부터 확보한다. 앞 구간에서 넘겨받았으면 내 것이라
        # 즉시 성공(멱등), 순찰 첫 구간이면 여기서 처음 잡는다. 이게 없으면 이동 중이
        # 아닌 로봇이 예약표에 안 보여서 남이 그 지점으로 들어온다(원래 결함).
        start_slot = engine.node_slot(current)
        if engine.try_reserve(start_slot, robot_id):
            held.append(start_slot)
        else:
            self._log.warn(
                f"현재 지점 {current} 자리를 남(로봇 {engine.holder_of(start_slot)})이 "
                f"쥐고 있다 task={task_id} — 예약표와 실제 위치가 어긋남")
        try:
            while current != target:
                # 1) 하달할 세그먼트 확보(룩어헤드가 미리 잡아놨으면 그걸 사용).
                if seg is None:
                    route = self._plan_route(engine, current, target, attempt_block)
                    if route is None:
                        self._log.warn(
                            f"경로 없음 task={task_id} {current}→{target} → 건너뜀")
                        return "skipped", current
                    seg = self._acquire_segment(
                        engine, robot_id, route.hops(), attempt_block, held)
                    if seg is None:
                        continue                    # 첫 홉 못 잡음 → 양보·재계획
                    # 새로 잡은 자원(통로+자리)을 점유 목록에 추가. 서 있는 자리는 이미
                    # held 에 있으므로 중복은 걸러낸다(중복이 있으면 반납이 꼬인다).
                    held.extend(c for c in self._seg_resources(engine, seg)
                                if c not in held)
                seg_wps, seg_cids = seg
                seg = None
                seg_start = current                # 이 세그먼트 진입 노드(피드백 판정 기준)
                reached = (seg_wps[-1] == target)  # 세그먼트 끝이 목표면 마지막에 촬영

                # 2) 주행 중 훅 2개: 조기 반납(피드백) + 룩어헤드(다음 구간 선예약).
                look = {"seg": None}
                fb = {"wp": None}                  # 피드백이 적어두는 '최근 도달 노드'
                fb_lock = threading.Lock()

                def on_feedback(wp_id, fb=fb, fb_lock=fb_lock):
                    """ROS executor 스레드 — 값만 기록(예약은 절대 안 건드림)."""
                    with fb_lock:
                        fb["wp"] = wp_id

                def on_tick(look=look, fb=fb, fb_lock=fb_lock, seg_end=seg_wps[-1],
                            seg_start=seg_start, seg_wps=seg_wps, seg_cids=seg_cids):
                    # (a) 조기 반납: 지나온 게 확실한 통로를 세그먼트 끝까지 안 기다리고 반납.
                    #     held 변경은 이 디스패치 스레드에서만 일어난다(락 불필요).
                    with fb_lock:
                        reached_wp = fb["wp"]
                    if reached_wp is not None:
                        freed = [c for c in self._passed_resources(
                            engine, seg_start, seg_wps, seg_cids, reached_wp)
                            if c in held]
                        for cid in freed:
                            engine.release(cid, robot_id)
                            held.remove(cid)
                        if freed:
                            self._log.info(
                                f"조기 반납 task={task_id} 로봇 위치 {reached_wp} "
                                f"→ 자원 {freed} 해제(음수=지점 자리)")
                    # (b) 룩어헤드: 다음 구간을 대기 없이 미리 예약.
                    if look["seg"] is None:
                        look["seg"] = self._try_reserve_ahead(
                            engine, robot_id, seg_end, target,
                            attempt_block, held)

                self._log.info(
                    f"세그먼트 하달 task={task_id} {current}→{seg_wps} "
                    f"통로={seg_cids} 촬영={reached}")

                # 3) 세그먼트 배열 하달. 하트비트엔 live held 를 넘겨 룩어헤드분도 함께 유지.
                code, last_wp = self._dispatch_segment(
                    client, task_id, seg_wps, capture_on_last=reached,
                    heartbeat=(engine, held, robot_id), on_tick=on_tick,
                    on_feedback=on_feedback)

                # 4) 결과 처리 = 선획득 후해제.
                if code == 2:
                    self._log.warn(f"중단 보고 task={task_id} → 순찰 실패")
                    return "aborted", current
                if code == 1:                       # 진짜 막힘 → 우회
                    current, blocked_cid, standing = self._segment_progress(
                        current, seg_wps, seg_cids, last_wp)
                    if blocked_cid is not None:
                        self._log.warn(
                            f"세그먼트 막힘 task={task_id} 통로 {blocked_cid} "
                            f"(로봇 위치 {current}) → 블랙리스트 후 우회")
                        self._blacklist_add(blocked_cid)
                        attempt_block.add(blocked_cid)
                    self._release_except(engine, robot_id, held, {standing}, current)
                    continue
                # code == 0: 세그먼트 끝 도착.
                current = seg_wps[-1]
                if look["seg"] is not None:         # 룩어헤드 성공 → 끊김 없이 연장
                    seg = look["seg"]
                    self._release_except(
                        engine, robot_id, held,
                        self._seg_resources(engine, seg), current)
                    self._log.info(
                        f"룩어헤드 연장 task={task_id} 위치 {current} 다음 통로={seg[1]}")
                elif reached:                       # 세그먼트 끝 = 목표 → 정상 도착
                    # 뒤처리(서 있는 통로만 남기고 반납)는 아래 '대기'와 같지만 의미가 다르다.
                    # 둘을 같은 문구로 찍으면 정상 도착이 전부 '막혀서 대기'로 보여, 로그로
                    # 막힘을 추적할 때 원인이 어긋난다.
                    self._log.info(
                        f"목표 도달 task={task_id} 위치 {current} "
                        f"(통로 {seg_cids[-1]}·자리 {engine.node_slot(current)} 유지 "
                        f"— 촬영·짝 처리 후 반납)")
                    self._release_except(
                        engine, robot_id, held, {seg_cids[-1]}, current)
                else:                               # 다음 못 잡음 → 세그먼트 끝에서 정지·대기
                    self._log.info(
                        f"세그먼트 끝 대기 task={task_id} 위치 {current} — 다음 홉 "
                        f"미확보, 정지 후 재시도(통로 {seg_cids[-1]}·자리 "
                        f"{engine.node_slot(current)} 유지)")
                    self._release_except(
                        engine, robot_id, held, {seg_cids[-1]}, current)

            # 목표 도달. 짝(같은 자리·반대 촬영 방향)이 있으면 여기서 제자리 회전 촬영까지
            # 마친 뒤에야 이 지점을 '방문 완료'로 본다. 아직 finally 이전이라 지금 쥔 통로
            # (held)를 그대로 들고 회전한다 — 회전 중 다른 로봇이 들어오지 못하게.
            if not self._capture_pair(
                    client, task_id, target, engine, held, robot_id):
                return "skipped", current
        finally:
            # 어떻게 나가든 남은 예약을 반납하되, '지금 서 있는 자리'만은 넘겨준다.
            # 로봇이 물리적으로 거기 있는 한 자리를 놓으면 남이 그 지점으로 들어온다.
            # 이 한 장은 다음 구간의 _navigate 가 이어받고, 순찰이 끝나면 run_patrol 이
            # 반납한다(구간과 구간 사이에 예약이 끊기는 순간을 없앤다).
            self._release_except(engine, robot_id, held, set(), current)
        return "arrived", current

    def _capture_pair(self, client, task_id, parent_wp, engine, held, robot_id) -> bool:
        """부모 지점의 '짝'이 있으면 제자리 회전 촬영을 하달한다. 짝이 없으면 아무 것도 안 함.

        왜 필요한가: 촬영 카메라가 로봇 한쪽에 고정돼 있어 통로를 한 번 지나면 한쪽 베드만
        찍힌다. 그래서 같은 자리에서 방향(yaw)만 180° 돌려 한 번 더 찍는다. 짝은 부모와
        x·y 가 완전히 같아서, 로봇은 '직전 waypoint 와 좌표가 같다' → 주행이 아니라 제자리
        회전(Spin)으로 처리한다. 통로를 지나지 않으므로 corridor 예약도 필요 없다.

        짝을 별도 waypoint_id 로 하달하는 이유는 사진마다 고유 식별자가 남아야 detection_logs
        와 병해충 알림이 '어느 지점의 어느 방향'인지 그대로 특정할 수 있기 때문이다.

        heartbeat 로 지금 쥔 통로(held)를 회전하는 동안 계속 유지한다 — 회전 중에 다른 로봇이
        그 통로로 들어오면 좁은 길에서 마주친다.

        반환: True(짝 없음 또는 촬영 성공) / False(짝 촬영 실패 → 이 지점은 미완).
        """
        pair_wp = self.pair_of.get(parent_wp)
        if pair_wp is None:
            return True                     # 짝이 없는 지점 — 부모 촬영으로 끝
        if pair_wp not in self.wp_meta:
            self._log.warn(
                f"짝 {pair_wp}(부모 {parent_wp}) 좌표를 그래프에서 못 찾음 → 촬영 생략")
            return False
        self._log.info(
            f"짝 촬영 하달 task={task_id} 부모 {parent_wp} → 짝 {pair_wp} "
            f"(같은 자리 제자리 회전, 통로 {list(held)} 유지)")
        code, _ = self._dispatch_segment(
            client, task_id, [pair_wp], capture_on_last=True,
            heartbeat=(engine, held, robot_id))
        if code != 0:
            self._log.warn(
                f"짝 {pair_wp} 촬영 실패(code={code}) task={task_id} → 지점 {parent_wp} 미완 처리")
            return False
        self._log.info(f"짝 촬영 완료 task={task_id} 부모 {parent_wp} / 짝 {pair_wp}")
        return True

    @staticmethod
    def _segment_progress(current, seg_wps, seg_cids, last_wp):
        """막힘(code=1) 시 로봇의 실제 도달 노드·'막힌 통로'·'서 있는 통로'를 추정한다.

        세그먼트 경로:  current -[seg_cids[0]]-> seg_wps[0] -[seg_cids[1]]-> seg_wps[1] ...
        로봇이 last_wp(Result.last_waypoint_id)까지 갔다면 그 다음 통로가 막힌 것이고,
        마지막으로 지나온 통로가 지금 서 있는 통로다.
        반환: (새 current, 막힌 corridor_id | None, 서 있는 corridor_id).
        last_wp 를 못 알아보면 세그먼트에 못 들어온 것으로 보고 진입 지점 기준으로 처리.
        """
        path = [current] + list(seg_wps)        # 시작점 포함 노드 나열
        try:
            j = path.index(last_wp)             # 로봇이 도달한 위치(인덱스)
        except ValueError:
            j = 0                               # 알 수 없음 → 진입 지점으로 간주
        blocked = seg_cids[j] if j < len(seg_cids) else None    # path[j]->path[j+1] 통로
        standing = seg_cids[j - 1] if j >= 1 else seg_cids[0]   # 마지막으로 점유한 통로
        return path[j], blocked, standing

    @staticmethod
    def _passed_resources(engine, seg_start, seg_wps, seg_cids, reached_wp):
        """주행 중 피드백의 '도달 노드' 기준으로 확실히 벗어난 자원(통로+자리)을 돌려준다.

        세그먼트 경로:  seg_start -[seg_cids[0]]-> seg_wps[0] -[seg_cids[1]]-> seg_wps[1] ...
        로봇이 reached_wp(=path[j]) 에 도착했다면 거기까지 오는 데 쓴 자원 — 통로
        seg_cids[0..j-1] 과 자리 path[0..j-1] — 은 전부 벗어난 것이다. 지금 있는 자리
        path[j] 하나만 남기고 반납한다.

        왜 직전 통로까지 놓아도 되는가(예전에는 한 칸 남겼다):
          통로만 예약하던 시절에는 통로를 놓는 순간 남이 그 통로로 들어와 한복판에서
          마주칠 수 있어, 로봇 길이를 감안해 '서 있는 통로'를 여유로 남겼다. 지금은
          홉이 (통로, 도착 자리) 쌍이라 자리를 못 잡으면 통로도 못 잡는다:
            · path[j-1] → path[j] 방향으로 들어오려면 도착 자리 path[j] 가 필요한데
              그 자리는 내가 쥐고 있다.
            · 반대 방향은 path[j] 에서 출발해야 하는데 거기에 내가 서 있다.
          어느 쪽으로도 진입할 수 없으므로 그 여유분은 이미 자리 예약이 대신하고 있다.
          한 칸을 더 붙들고 있으면 같은 보호를 두 번 하면서 남의 길만 막는다.

        _segment_progress 와는 판정이 다르다 — 그쪽은 '막힘' 상황이라 로봇이 통로
        한복판에 멈춰 있을 수 있어 '도착했다'는 전제가 성립하지 않는다(보수적으로 유지).

        모르는 노드/시작 지점이면 빈 목록 → 아무것도 반납하지 않는다(안전한 쪽으로 실패).
        """
        path = [seg_start] + list(seg_wps)
        try:
            j = path.index(reached_wp)
        except ValueError:
            return []                           # 알 수 없는 노드 → 반납 안 함
        # path[j] 에 도착 = path[0..j-1] 과 그 사이 통로는 전부 벗어났다.
        return list(seg_cids[:j]) + [engine.node_slot(n) for n in path[:j]]

    @staticmethod
    def _seg_resources(engine, seg):
        """세그먼트가 점유하는 자원 전체 = 통로들 + 지나갈 자리들."""
        seg_wps, seg_cids = seg
        return set(seg_cids) | {engine.node_slot(w) for w in seg_wps}

    @staticmethod
    def _release_except(engine, robot_id, held, keep, standing_node=None):
        """held(지금 쥔 자원 리스트)에서 keep 에 없는 것을 모두 해제하고 held 를 갱신한다.

        선획득 후해제의 '후해제' — 유지할 자원(keep)만 남기고 나머지를 반납한다.
        standing_node: 지금 로봇이 물리적으로 서 있는 노드. 그 자리는 무조건 유지한다.
        로봇이 거기 있는 한 자리를 놓으면 남이 그 지점으로 들어와 겹친다 — 이 결함이
        '통로만 예약하던' 시절의 원래 버그였다.
        """
        keep = set(keep)
        if standing_node is not None:
            keep.add(engine.node_slot(standing_node))
        for cid in list(held):
            if cid not in keep:
                engine.release(cid, robot_id)
                held.remove(cid)

    @staticmethod
    def _split_blocked(engine, ids):
        """회피 대상 id 집합을 (통로 집합, 노드 집합)으로 가른다.

        블랙리스트·attempt_block 은 통로 id(양수)와 노드 자리 id(음수)를 한 바구니에
        담는다 — 부호로 갈리니 자료구조를 따로 만들 필요가 없다. find_path 는 둘을 다른
        인자로 받으므로(통로를 빼는 것과 지점을 통째로 빼는 것은 효과가 다르다) 여기서 푼다.
        """
        corridors = {i for i in ids if not engine.is_node_slot(i)}
        nodes = {engine.node_of_slot(i) for i in ids if engine.is_node_slot(i)}
        return corridors, nodes

    def _plan_route(self, engine, current, target, attempt_block):
        """current→target 경로. 정상 순찰은 인접 직행, 막히면 Dijkstra 우회. 없으면 None."""
        blocked, blocked_nodes = self._split_blocked(
            engine, set(attempt_block) | self._blacklist_active())
        direct = engine.corridor_between(current, target)
        # 직행도 '도착 지점이 막혔는지'를 같이 본다 — 통로가 비어도 그 자리에 남이 서
        # 있으면 갈 수 없다. 이 검사를 빠뜨리면 우회 등록해 둔 지점으로 곧장 되돌아간다.
        if (direct is not None and direct not in blocked
                and target not in blocked_nodes):
            return Route((current, target), (direct,))   # 인접 지점 직행(세그먼트 1개)
        return engine.find_path(current, target, blocked=blocked,
                                blocked_nodes=blocked_nodes)

    @staticmethod
    def _res_name(engine, cid) -> str:
        """자원 id 를 사람이 읽는 이름으로. 로그에 '통로 -7' 이 찍히면 아무도 못 읽는다."""
        return (f"지점 {engine.node_of_slot(cid)} 자리"
                if engine.is_node_slot(cid) else f"통로 {cid}")

    def _reserve_with_wait(self, engine, corridor_id, robot_id, held=None) -> bool:
        """자원 예약을 '확인+획득+대기검사'(reserve_or_wait)로 시도하며 대기. 성공 True.

        - reserved → True.
        - deadlock(쥔 채 기다리면 대기 사이클) → 즉시 양보 False(호출부가 블랙리스트+우회).
        - waiting  → RESERVE_POLL_SEC 간격 재시도, RESERVE_WAIT_SEC 넘으면 양보 False.
        양보(False)로 나갈 땐 대기 그래프에서 이 로봇의 대기를 지운다(end_wait).

        held: 이 로봇이 이미 쥔 자원들. 대기하는 동안 폴링마다 하트비트를 갱신한다.
        서서 기다리는 중에는 주행 하트비트(_dispatch_segment)가 안 돌기 때문에, 이게
        없으면 RESERVE_WAIT_SEC(30초)를 기다리다 RESERVATION_TTL_SEC(15초)에 걸려
        '내가 지금 서 있는 자리'가 남에게 회수된다.
        """
        deadline = time.monotonic() + RESERVE_WAIT_SEC
        while True:
            outcome = engine.reserve_or_wait(corridor_id, robot_id)
            if outcome == "reserved":
                return True
            if outcome == "deadlock":
                self._log.warn(
                    f"{self._res_name(engine, corridor_id)} 대기 시 데드락 예상 "
                    f"→ 양보(우회)")
                engine.end_wait(robot_id)
                return False
            # outcome == "waiting": 안전하게 대기 중
            if time.monotonic() >= deadline:
                self._log.warn(
                    f"{self._res_name(engine, corridor_id)} 예약 대기 타임아웃 "
                    f"→ 순찰 양보")
                engine.end_wait(robot_id)
                return False
            for cid in list(held or ()):    # 대기 중에도 쥔 자원은 살려둔다(TTL 방어)
                engine.heartbeat(cid, robot_id)
            time.sleep(RESERVE_POLL_SEC)

    def _acquire_segment(self, engine, robot_id, hops, attempt_block, held=None):
        """route.hops() 를 받아 한 세그먼트를 예약한다.

        홉 하나 = (통로, 도착 자리) 한 쌍이다. 통로만 잡고 도착 자리를 못 잡으면 '들어가도
        설 곳이 없는' 상태가 되고, 그렇다고 안 들어가면 통로만 붙잡아 남의 길을 막는다.
        그래서 쌍 단위로 성공/실패를 판정하고, 깨진 쌍은 즉시 되돌린다.
        첫 홉은 대기하며 예약(_reserve_with_wait), 이어지는 홉은 대기 없이(try_reserve)
        잡히는 만큼 묶는다. 반환: (seg_wps, seg_cids) 또는 None.
        None(첫 홉 확보 실패=양보)이면 못 잡은 자원을 블랙리스트+attempt_block 에 넣어
        호출부가 우회 재계획하게 한다.
        자리 id 는 seg_wps 에서 node_slot 으로 언제든 얻으므로 따로 담아 다니지 않는다
        (두 목록을 따로 들면 어긋날 때 조용한 예약 누수가 된다).
        """
        first_wp, first_cid = hops[0]
        first_slot = engine.node_slot(first_wp)
        if not self._reserve_with_wait(engine, first_cid, robot_id, held):
            self._blacklist_add(first_cid)
            attempt_block.add(first_cid)
            return None
        # 통로를 잡은 뒤 자리를 기다리는 동안, 방금 잡은 통로도 하트비트 대상에 넣는다.
        if not self._reserve_with_wait(engine, first_slot, robot_id,
                                       list(held or ()) + [first_cid]):
            # 통로는 잡았는데 도착 자리를 못 얻었다 → 쥔 통로를 반드시 도로 뱉는다.
            # 안 뱉으면 '가지도 못하면서 길만 막는' 로봇이 되어 상대까지 묶인다.
            engine.release(first_cid, robot_id)
            self._log.warn(
                f"지점 {first_wp} 자리 점유 중(로봇 {engine.holder_of(first_slot)}) "
                f"→ 통로 {first_cid} 반납 후 그 지점을 피해 우회")
            self._blacklist_add(first_slot)
            attempt_block.add(first_slot)
            return None
        seg_wps = [first_wp]
        seg_cids = [first_cid]
        for next_wp, cid in hops[1:]:
            if not engine.try_reserve(cid, robot_id):
                break                       # 세그먼트 끊김 → 여기까지
            if not engine.try_reserve(engine.node_slot(next_wp), robot_id):
                engine.release(cid, robot_id)   # 쌍이 깨졌으니 방금 잡은 통로도 반납
                break
            seg_wps.append(next_wp)
            seg_cids.append(cid)
        return seg_wps, seg_cids

    def _try_reserve_ahead(self, engine, robot_id, node, target,
                           attempt_block, held_cids):
        """룩어헤드: node→target 경로의 다음 구간을 '대기 없이'(try_reserve) 미리 예약.

        주행 중(on_tick) 호출된다. 잡은 자원은 held_cids 에 더해 하트비트로 유지되게 한다.
        _acquire_segment 와 같은 규칙 — 홉 하나 = (통로, 도착 자리) 쌍, 쌍이 깨지면 되돌린다.
        반환: (seg_wps, seg_cids) 또는 None(다음 홉을 아직 못 잡음 / 더 갈 곳 없음).
        """
        if node == target:
            return None                     # 이미 목표 → 미리 잡을 것 없음
        route = self._plan_route(engine, node, target, attempt_block)
        if route is None:
            return None
        seg_wps = []
        seg_cids = []
        for next_wp, cid in route.hops():
            slot = engine.node_slot(next_wp)
            if cid in held_cids or not engine.try_reserve(cid, robot_id):
                break                       # 이미 쥠/남이 점유 → 대기 없이 여기서 멈춤
            if slot not in held_cids and not engine.try_reserve(slot, robot_id):
                engine.release(cid, robot_id)   # 쌍이 깨졌으니 방금 잡은 통로도 반납
                break
            seg_wps.append(next_wp)
            seg_cids.append(cid)
            held_cids.append(cid)           # 하트비트 대상에 즉시 포함
            if slot not in held_cids:
                held_cids.append(slot)
        if not seg_cids:
            return None                     # 한 칸도 못 잡음
        return seg_wps, seg_cids

    # ---------------------------- 세그먼트 하달 ---------------------------- #
    def _dispatch_segment(self, client, task_id, waypoint_ids,
                          capture_on_last, heartbeat=None, on_tick=None,
                          on_feedback=None):
        """확보된 세그먼트(연속 waypoint 목록)를 Navigate Goal(Waypoint[] 배열)로 한 번에 하달.

        waypoint_ids: [세그먼트 첫 노드 ... 끝 노드] — 예약을 확보한 통로들을 지나는 경로.
        capture_on_last: 세그먼트 끝이 순찰 목표면 True → 마지막 waypoint 만 capture=True
                         (도착·정지 후 촬영). 중간 노드는 통과만 하므로 전부 capture=False.
        heartbeat=(engine, [cid...], robot_id): 결과 대기 중 세그먼트의 모든 통로 예약을 갱신.
        on_tick: 결과 대기 중 하트비트 틱마다 호출되는 콜백(룩어헤드 = 다음 구간 선예약용).
        on_feedback: Navigate Feedback 의 current_waypoint_id 를 받는 콜백(조기 반납용).
                     ⚠️ ROS executor 스레드에서 실행되므로 '값 전달'만 하고, 예약 반납 같은
                     공유 상태 변경은 디스패치 스레드(on_tick)에서 해야 한다.
        반환: (result_code, last_waypoint_id). result_code 0 성공/1 실패·막힘/2 중단.
        """
        wps = []
        last_idx = len(waypoint_ids) - 1
        for i, wid in enumerate(waypoint_ids):
            m = self.wp_meta.get(wid, {})
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

        def _fb(msg):
            """ROS executor 스레드에서 실행 — 도달 노드만 꺼내 호출부에 넘긴다."""
            try:
                on_feedback(int(msg.feedback.current_waypoint_id))
            except Exception as exc:  # noqa: BLE001
                self._log.warn(f"피드백 처리 예외(무시): {exc}")

        goal_handle = _spin_wait(
            client.send_goal_async(
                goal, feedback_callback=(_fb if on_feedback is not None else None)),
            GOAL_ACCEPT_TIMEOUT_SEC)
        if goal_handle is None or not goal_handle.accepted:
            self._log.warn(
                f"Goal 거부/수락 타임아웃 task={task_id} waypoints={waypoint_ids}")
            return 1, None

        return self._await_result(
            goal_handle.get_result_async(), heartbeat, on_tick)

    def _await_result(self, result_future, heartbeat, on_tick=None):
        """결과 대기. 대기 중 HEARTBEAT_SEC마다 세그먼트의 모든 통로 예약을 갱신하고,
        on_tick(있으면)을 호출해 룩어헤드(다음 구간 선예약)를 시도한다.

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
                # 초당 반복이라 debug: 주행 중 예약을 유지하는 통로 목록(룩어헤드로 늘어남)
                self._log.debug(
                    f"주행 중 {robot_id} 예약유지 통로={list(cids)}")
            if on_tick is not None:
                try:
                    on_tick()               # 주행 중 다음 구간 선예약 시도(룩어헤드)
                except Exception as exc:  # noqa: BLE001
                    self._log.warn(f"룩어헤드 tick 예외(무시): {exc}")
            if time.monotonic() >= deadline:
                self._log.warn("세그먼트 결과 대기 타임아웃 → 실패 취급")
                return 1, None
        try:
            res = result_future.result().result
            return int(res.result_code), int(res.last_waypoint_id)
        except Exception:  # noqa: BLE001
            return 1, None

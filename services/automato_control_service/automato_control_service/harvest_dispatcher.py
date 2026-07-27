#!/usr/bin/env python3
"""RP-123 시나리오2 — 수확 오케스트레이션(E2~E6)을 주관하는 디스패처.

automato_node(ROS 표면)에서 '수확 동작 결정' 로직을 떼어낸 클래스(composition).
순찰 PatrolDispatcher 와 같은 관례를 따른다: rclpy 노드를 직접 참조하지 않고, 노드가
준비해 넘겨준 엔진·액션 클라이언트만 받아 쓴다(그래서 단위테스트가 쉽다).

경로탐색/통로예약은 순찰과 '같은' RoutingEngine 인스턴스를 공유한다 — 순찰 로봇과
수확 로봇이 같은 통로를 놓고 경합하므로, 예약표가 하나여야 교통관제가 성립한다.

수확 task 하나의 전 생애:
  E2   수확지까지 주행 + ChArUco 도킹
  E3~4 Harvest 액션(Ddagi 주관) — 진행 피드백 중계, 최종 집계 수신
  E5   수확 실적 저장 + 예냉실까지 주행 + 도킹
  E6   완료 처리 + (보너스) 바구니 하역

⚠️ 지금은 E2 앞부분(수확지까지 주행)까지만 구현됐다. 도킹부터는 아직 없다.

■ 확정된 설계 노트
  (1) ✅ 주행 재사용 — RouteRunner 로 추출 완료(C-1). E2/E5 주행은 self.runner.drive 를
      쓴다. 촬영·짝이 없는 평범한 주행이라 DriveHooks 를 안 넘기면 된다(기본값).
  (2) ✅ DB 경계 — 이 클래스는 DB 를 직접 만지지 않는다. 미리 조회할 수 있는 것
      (수확지·예냉실 진입노드, ChArUco 마커)은 **노드가 조회해 인자로 넘긴다** — 순찰이
      start_wp 를 노드에서 받는 것과 같은 관례이고, 덕분에 단위테스트에 DB 가 필요 없다.
      반대로 '도중에 생기는' 저장 2건(E5 harvest_batches, E6 unload_logs)은 결과가 나와야
      값이 생기므로 나중에 **콜백**으로 받는다. 반환값으로 미루면 안 되는 이유: E5 실적은
      '수확 종료 직후' 남겨야 하는데 그 뒤 예냉실 주행이 길어, 반환을 기다리면 그 사이
      크래시에 실적이 통째로 날아간다.
"""

import threading
import time

from action_msgs.msg import GoalStatus
from automato_interfaces.action import Harvest, Unload

from automato_control_service import docking
from automato_control_service.patrol_config import (
    GOAL_ACCEPT_TIMEOUT_SEC,
    HARVEST_MAX_CAPACITY,
    HARVEST_RESULT_TIMEOUT_SEC,
    HEARTBEAT_SEC,
    SERVER_WAIT_SEC,
    UNLOAD_RESULT_TIMEOUT_SEC,
    UNLOAD_SHAKE_DELAY_SEC,
)
from automato_control_service.route_runner import RouteRunner, spin_wait

# 최종 상태 — 노드가 tasks 에 마감한다(automato_db.set_task_status 의 유효값과 호환).
STATUS_COMPLETED = "COMPLETED"
STATUS_FAILED = "FAILED"

# 실패 사유 — 노드가 task_failed 알림(문서 13번)에 그대로 싣는다.
# 값은 patrol_notify.FAIL_REASONS 안에 있어야 한다(아니면 payload 생성이 ValueError).
# reason=None 은 '알림 없이 tasks 만 FAILED 로 마감'이다 — 주행 실패(길 막힘·로봇 중단)는
# 아직 시나리오2 문서에 알림 규격이 없어 지금은 로그만 남기고 통지하지 않는다.
REASON_DOCK_FAILED = "DOCK_FAILED"


class HarvestDispatcher:
    """수확 task 하나의 E2~E6 흐름을 주관한다(ROS 표면·DB는 노드가 주입/처리)."""

    def __init__(self, logger, runner=None):
        self._log = logger
        # 예약하며 목적지까지 이동하는 엔진. **순찰과 같은 인스턴스**를 노드가 주입한다
        # — wp_meta(좌표)와 블랙리스트(막힌 통로)를 공유해야 교통관제가 한 벌로 돈다.
        # 생략하면 스스로 하나 만든다(수확만 돌리는 단위테스트용).
        self.runner = runner if runner is not None else RouteRunner(logger)

    @property
    def wp_meta(self):
        """waypoint_id -> {x,y,yaw,capture}. 실체는 runner 소유(주행 골에 좌표가 필요)."""
        return self.runner.wp_meta

    def run_harvest(self, task_id, robot_id, harvest_point, marker, engine,
                    clients, start_wp=None, on_progress=None,
                    precool_point=None, precool_marker=None, save_batch=None,
                    save_unload=None, on_completed=None):
        """수확 task 하나를 E2~E6 순서로 처리하고 최종 상태를 돌려준다.

        harvest_point : 수확지 정보 dict — 노드가 automato_db.get_task_point 로 조회해 넘긴다.
                        {"task_point_id","point_type","waypoint_id","x","y","yaw"}
                        (설계노트 2: 미리 조회 가능한 것은 노드가 맡는다)
        marker        : 수확지 ChArUco 보드 dict — 노드가 automato_db.get_dock_marker 로
                        조회해 넘긴다. **None 이 정상적인 값**이다(실측값은 도킹 튜닝 후
                        시드되므로 아직 비어 있을 수 있다) → 그 경우 도킹은 즉시 실패한다.
                        harvest_point 와 다른 테이블(charuco_boards)에서 오므로 따로 받는다.
        engine        : 순찰과 공유하는 RoutingEngine(경로탐색+통로예약)
        clients       : 노드가 만든 액션 클라이언트 묶음
                        {"nav","dock","harvest","unload"} (robot_id 로 바인딩됨)
        start_wp      : 출발 노드(로봇 전용 충전소 진입노드).
        on_progress   : 수확 진행 상황을 받을 콜백(dict). 노드가 이걸 Web Service 로
                        중계한다 — 이 클래스는 HTTP 를 모른 채로 남아야 ROS·DB·네트워크
                        없이 단위테스트가 된다(설계노트 2 와 같은 이유).
        precool_point : 예냉실 진입노드 dict(get_precool_point). 수확지와 같은 규약이고,
                        **수확을 시작하기 전에** 노드가 조회해 넘긴다 — 갈 곳이 없는데
                        몇 분씩 토마토를 따는 건 낭비다.
        precool_marker: 예냉실 ChArUco 보드 dict. 없으면(None) 예냉실 도킹만 실패한다.
        save_batch    : 수확 실적을 저장하는 콜백. 집계 dict 를 받아 batch_id 를 돌려준다.
                        이 클래스가 DB 를 모르는 채로 남기 위한 통로다(on_progress 와 같은
                        이유). 반환된 batch_id 는 E6 완료 통지에 실린다.
        save_unload   : 하역 입고를 기록하는 콜백(E6). **하역이 성공했을 때만** 불린다 —
                        실패했는데 입고 행이 남으면 재고가 실제보다 늘어난다.
        on_completed  : 수확 task 완료를 알리는 콜백. 집계 + batch_id 를 받는다.
                        노드가 Web Service 로 보낸다.

        반환: (status, reason)
          status: STATUS_COMPLETED | STATUS_FAILED — 노드가 tasks 에 마감한다.
          reason: 실패 사유(REASON_*) 또는 None. 노드가 이걸 보고 task_failed 알림을
                  보낼지 정한다 — 디스패처는 '무슨 일이 있었는지'만 보고하고 HTTP 는
                  모른 채로 남는다(순찰 run_patrol 과 같은 관례).

        ⚠️ 지금은 E2(주행+도킹)까지 구현됐다. 도킹까지 성공해도 수확을 안 했으므로
        FAILED 로 마감한다. 남은 순서:
          outcome = E3~4 Harvest 액션(Ddagi 주관)+피드백 중계  (clients['harvest'])
          ...     E5  수확 실적 저장(콜백) + 예냉실 주행·도킹  (clients['nav'], clients['dock'])
          ...     E6  완료 처리 + (보너스) Unload 하역          (clients['unload'])
        """
        target = (harvest_point or {}).get("waypoint_id")
        label = (harvest_point or {}).get("task_point_id", "?")

        # --- 출발 전 점검 3가지. 여기서 걸러야 하는 이유를 각각 적어 둔다 ---
        if not clients["nav"].wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            # 안 걸러내면 Goal 마다 수락 타임아웃(30초)을 다 기다린 뒤에야 실패한다.
            self._log.warning(
                f"[HARVEST] {robot_id} Navigate 액션 서버 미기동 → task {task_id} FAILED")
            return STATUS_FAILED, None
        if target is None or target not in self.wp_meta:
            self._log.error(
                f"[HARVEST] 수확지 {label}(노드 {target})가 라우팅 그래프에 없다 "
                f"→ task {task_id} FAILED")
            return STATUS_FAILED, None
        # 순찰과 달리 출발점 폴백을 두지 않는다. 순찰은 지점이 여러 개라 하나쯤 예약 없이
        # 가도 나머지가 이어지지만, 수확은 목적지가 하나뿐이라 출발점을 모르면 경로 예약
        # 자체가 성립하지 않고, 도착 직후 ChArUco 도킹이 붙어 위치가 어긋나면 그대로
        # 도킹 실패가 된다. 로봇별 충전소는 DB(robots.charge_point_id)에 있어야 한다.
        if start_wp is None or start_wp not in self.wp_meta:
            self._log.error(
                f"[HARVEST] {robot_id} 출발 노드({start_wp}) 미상 → task {task_id} FAILED "
                f"(robots.charge_point_id 확인 필요)")
            return STATUS_FAILED, None

        current = start_wp
        self._log.info(
            f"[HARVEST] E2 주행 시작 task={task_id} robot={robot_id} "
            f"{current} → {target}({label})")
        try:
            # 훅을 안 넘긴다 = DriveHooks 기본값(촬영·짝·방문 마킹 없는 평범한 주행).
            # 순찰과 같은 예약 규칙으로 움직이므로 두 로봇이 통로를 놓고 경합해도 안전하다.
            outcome, current = self.runner.drive(
                engine, clients["nav"], task_id, robot_id, current, target)
            if outcome != "arrived":
                # skipped = 블랙리스트·점유로 우회로가 없어 포기 / aborted = 로봇이 중단 보고.
                self._log.warning(
                    f"[HARVEST] E2 주행 실패({outcome}) task={task_id} "
                    f"로봇 위치 {current} 목표 {target}({label}) → FAILED")
                return STATUS_FAILED, None

            self._log.info(
                f"[HARVEST] E2 수확지 도착 task={task_id} robot={robot_id} "
                f"위치 {current}({label})")

            # --- E2 ChArUco 도킹 — 순찰(충전소)·예냉실과 같은 절차라 docking 모듈 공용 ---
            # heartbeat 가 핵심이다: 도킹은 마커 탐색~후진까지 수십 초 걸리는데 그동안
            # 주행 하트비트가 멎어, 이걸 안 넘기면 RESERVATION_TTL_SEC(15초)에 걸려
            # '지금 로봇이 서 있는 자리'가 회수되고 남이 그 지점으로 들어온다.
            entry_slot = engine.node_slot(current)
            success, code, msg = docking.dock(
                self._log, task_id, robot_id, label, marker, clients["dock"],
                heartbeat=(engine, [entry_slot], robot_id))
            if not success:
                self._log.warning(
                    f"[HARVEST] E2 도킹 실패(code={code}) task={task_id} "
                    f"{robot_id} @ {label}: {msg} → FAILED")
                return STATUS_FAILED, REASON_DOCK_FAILED

            # 순찰(복귀)은 도킹 성공 시 예약을 전부 해제하지만 수확은 놓지 않는다 —
            # 로봇이 여기 붙어서 팔 작업을 이어가므로, 자리를 놓으면 남이 들어온다.
            # 반납은 이 함수 맨 끝 finally 가 한다(수확 task 전체가 끝나는 시점).
            self._log.info(
                f"[HARVEST] E2 도킹 완료 task={task_id} {robot_id} @ {label} "
                f"(자리 {entry_slot} 유지 — 수확 중 남이 들어오면 안 된다)")

            # --- E3~E4 수확 — Ddagi 가 주관하고 ACS 는 시키고 기다린다 ---
            # 도킹과 같은 이유로 heartbeat 를 넘긴다. 수확은 분 단위라 이게 없으면
            # 팔 작업 중에 자리를 뺏긴다.
            harvested = self._run_harvest_action(
                task_id, robot_id, clients["harvest"], on_progress,
                heartbeat=(engine, [entry_slot], robot_id))
            if harvested is None:
                self._log.warning(
                    f"[HARVEST] E3~4 수확 실패/중단 task={task_id} {robot_id} → FAILED")
                return STATUS_FAILED, None

            self._log.info(
                f"[HARVEST] E3~4 수확 종료 task={task_id} {robot_id} "
                f"정상 {harvested['normal_count']} / 폐기 {harvested['discard_count']} "
                f"/ 실패 {harvested['failed_count']} (사유 {harvested['exit_reason']})")

            # --- E5-1 실적 저장 — 예냉실로 '출발하기 전에' 남긴다 ---
            # 순서가 핵심이다. 예냉실 이송은 분 단위이고 그 사이 로봇·프로세스가 죽을 수
            # 있는데, 그때 아직 안 적었으면 애써 딴 실적이 통째로 사라진다(바구니엔 있는데
            # 시스템은 모르는 상태). 먼저 적어두면 이송이 실패해도 개수는 남는다.
            batch_id = self._save_batch(save_batch, task_id, harvested)

            # --- E5-2 예냉실까지 이송 + 도킹 ---
            precool_wp = (precool_point or {}).get("waypoint_id")
            precool_label = (precool_point or {}).get("task_point_id", "?")
            if precool_wp is None or precool_wp not in self.wp_meta:
                self._log.error(
                    f"[HARVEST] 예냉실({precool_label}/노드 {precool_wp})이 라우팅 "
                    f"그래프에 없다 → task {task_id} 이송 불가 FAILED "
                    f"(수확 실적 batch_id={batch_id} 은 저장됨)")
                return STATUS_FAILED, None

            self._log.info(
                f"[HARVEST] E5 예냉실 이송 시작 task={task_id} {robot_id} "
                f"{current} → {precool_wp}({precool_label})")
            # 수확지 → 예냉실. 여기도 훅 없이 부른다(촬영·짝 없는 평범한 주행).
            # 수확지 자리는 drive 가 출발하며 이어받아 반납한다.
            outcome, current = self.runner.drive(
                engine, clients["nav"], task_id, robot_id, current, precool_wp)
            if outcome != "arrived":
                self._log.warning(
                    f"[HARVEST] E5 예냉실 이송 실패({outcome}) task={task_id} "
                    f"로봇 위치 {current} → FAILED")
                return STATUS_FAILED, None

            precool_slot = engine.node_slot(current)
            success, code, msg = docking.dock(
                self._log, task_id, robot_id, precool_label, precool_marker,
                clients["dock"], heartbeat=(engine, [precool_slot], robot_id))
            if not success:
                self._log.warning(
                    f"[HARVEST] E5 예냉실 도킹 실패(code={code}) task={task_id} "
                    f"{robot_id} @ {precool_label}: {msg} → FAILED")
                return STATUS_FAILED, REASON_DOCK_FAILED

            self._log.info(
                f"[HARVEST] E5 예냉실 도킹 완료 task={task_id} {robot_id} "
                f"@ {precool_label}")

            # --- E6 하역 — '보너스' 라서 실패해도 task 를 되돌리지 않는다 ---
            # 이 task 의 목적은 '따서 예냉실로 옮기기'이고, 도착한 순간 이미 달성됐다.
            # 바구니를 자동으로 비우는 건 편의 기능이라 실패하면 사람이 손으로 비우면
            # 된다. 여기서 FAILED 로 되돌리면 '토마토는 무사히 옮겨졌는데 수확은 실패'
            # 라는 기록이 남아 나중에 통계가 어긋난다(Unload.action 주석의 명시 규칙).
            self._unload(task_id, robot_id, clients["unload"], harvested,
                         save_unload,
                         heartbeat=(engine, [precool_slot], robot_id))

            self._log.info(
                f"[HARVEST] 수확 task 완료 task={task_id} {robot_id} "
                f"batch_id={batch_id} 정상 {harvested['normal_count']} / "
                f"폐기 {harvested['discard_count']}")
            if on_completed is not None:
                # 완료 통지는 노드가 보낸다(디스패처는 HTTP 를 모른다). 실패해도 실적은
                # DB 에 있으므로 여기서 예외를 삼켜 task 마감을 막지 않는다.
                try:
                    on_completed(dict(harvested, batch_id=batch_id))
                except Exception as exc:  # noqa: BLE001
                    self._log.warning(
                        f"[HARVEST] 완료 통지 실패(무시) task={task_id}: {exc}")
            return STATUS_COMPLETED, None
        finally:
            # drive 는 '지금 서 있는 자리'를 일부러 남기고 나온다 — 다음 구간이 이어받아
            # 예약이 끊기는 순간을 없애기 위해서다. 수확은 아직 뒷단계가 없으므로 여기서
            # 반납하지 않으면 그 자리가 영영 점유된 채로 남아 남의 길을 막는다.
            engine.release(engine.node_slot(current), robot_id)
            self._log.info(
                f"[HARVEST] task={task_id} 지점 {current} 자리 반납")

    def _save_batch(self, save_batch, task_id, harvested):
        """수확 실적을 콜백으로 저장하고 batch_id 를 돌려준다. 실패해도 None 만 낸다.

        저장이 실패해도 **예냉실 이송은 멈추지 않는다.** 토마토는 이미 바구니에 있고
        딴 순간부터 상하기 시작하므로, 기록 문제로 냉장을 미루면 실물을 버리게 된다.
        기록은 로그로 남겨 나중에 사람이 복구할 수 있다 — 둘 중 되돌릴 수 없는 쪽은
        실물이다.
        """
        if save_batch is None:
            return None
        try:
            batch_id = save_batch(harvested)
            self._log.info(
                f"[HARVEST] E5 수확 실적 저장 task={task_id} batch_id={batch_id}")
            return batch_id
        except Exception as exc:  # noqa: BLE001
            self._log.error(
                f"[HARVEST] E5 수확 실적 저장 실패 task={task_id}: {exc} "
                f"— 이송은 계속한다(실적: {harvested})")
            return None

    # ---------------------------- E6 하역 ---------------------------- #
    def _unload(self, task_id, robot_id, unload_client, harvested, save_unload,
                heartbeat=None):
        """바구니를 비우고, 성공했을 때만 입고 기록을 남긴다. 반환: 성공 여부.

        ⚠️ 실패해도 호출부는 task 를 FAILED 로 되돌리지 않는다(보너스 기능). 그래서
        이 함수는 예외를 밖으로 내지 않고 전부 로그로 흡수한다 — 하역 문제로 수확
        기록이 실패로 뒤집히는 일이 없어야 한다.

        판정은 Navigate·Dock 과 같은 result_code 방식이다(Harvest 만 goal 상태였다).
        """
        if unload_client is None:
            self._log.warning(f"[HARVEST] E6 하역 클라이언트 없음 task={task_id} → 건너뜀")
            return False
        try:
            code, msg = self._run_unload_action(
                task_id, robot_id, unload_client, heartbeat)
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"[HARVEST] E6 하역 예외(무시) task={task_id}: {exc}")
            return False
        if code != 0:
            # 1=손잡이 파지 실패 / 2=중단. 사람이 손으로 비우면 되므로 task 는 성공이다.
            self._log.warning(
                f"[HARVEST] E6 하역 실패(code={code}) task={task_id} {robot_id}: {msg} "
                f"— task 는 성공으로 유지한다(사람이 바구니를 비우면 된다)")
            return False
        self._log.info(f"[HARVEST] E6 하역 완료 task={task_id} {robot_id}")
        # 입고 기록은 '성공했을 때만' 남긴다 — 실패했는데 입고 행이 있으면 재고가 는다.
        if save_unload is not None:
            try:
                unload_id = save_unload(harvested)
                self._log.info(
                    f"[HARVEST] E6 입고 기록 task={task_id} unload_id={unload_id}")
            except Exception as exc:  # noqa: BLE001
                self._log.error(
                    f"[HARVEST] E6 입고 기록 실패 task={task_id}: {exc} "
                    f"— 하역은 됐다(수확 실적은 harvest_batches 에 남아 있다)")
        return True

    def _run_unload_action(self, task_id, robot_id, unload_client, heartbeat):
        """Unload 액션을 하달하고 결과를 기다린다. 반환: (result_code, message)."""
        if not unload_client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            return 1, "Unload 액션 서버 미기동"

        goal = Unload.Goal()
        goal.task_id = int(task_id)
        goal.shake_delay_sec = float(UNLOAD_SHAKE_DELAY_SEC)

        def _fb(msg):
            """ROS executor 스레드 — 하역 단계(phase)를 로그로만 남긴다."""
            try:
                self._log.info(
                    f"[HARVEST] 하역 진행 task={task_id} phase={msg.feedback.phase}")
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"[HARVEST] 하역 피드백 처리 예외(무시): {exc}")

        self._log.info(
            f"[HARVEST] E6 하역 시작 task={task_id} {robot_id} "
            f"(들고 {UNLOAD_SHAKE_DELAY_SEC}초 대기 후 흔들기)")
        goal_handle = spin_wait(
            unload_client.send_goal_async(goal, feedback_callback=_fb),
            GOAL_ACCEPT_TIMEOUT_SEC)
        if goal_handle is None or not goal_handle.accepted:
            # DG 는 예냉실 도킹에 성공한 task 의 goal 만 accept 한다.
            return 1, "Unload Goal 거부/수락 타임아웃"

        result_future = goal_handle.get_result_async()
        done = threading.Event()
        result_future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + UNLOAD_RESULT_TIMEOUT_SEC
        while not done.wait(HEARTBEAT_SEC):
            if heartbeat is not None:
                engine, cids, rid = heartbeat
                for cid in cids:
                    engine.heartbeat(cid, rid)
            if time.monotonic() >= deadline:
                return 1, f"하역 결과 대기 타임아웃({UNLOAD_RESULT_TIMEOUT_SEC}s)"
        try:
            res = result_future.result().result
            return int(res.result_code), str(res.message)
        except Exception:  # noqa: BLE001
            return 1, "하역 결과 파싱 실패"

    # ---------------------------- E3~E4 수확 ---------------------------- #
    def _run_harvest_action(self, task_id, robot_id, harvest_client,
                            on_progress=None, heartbeat=None):
        """Harvest 액션을 하달하고 끝날 때까지 기다린다. 성공하면 집계 dict, 아니면 None.

        수확 루프(관측·검출요청·제외목록·라운드·파지)는 전부 Ddagi 안에서 돈다. DG 는
        중계만 하고, ACS 는 Goal 하나 던지고 Feedback 을 받아 넘기다가 결과를 받는다.

        ⚠️ 성공 판정이 Navigate·Dock 과 다르다. Harvest.action 에는 result_code 필드가
           **없고**, 성공/취소/중단은 액션 goal 상태(SUCCEEDED/CANCELED/ABORTED)로
           표현한다. 상태를 안 보고 결과 본문만 읽으면, 로봇이 중간에 중단(ABORTED)해도
           집계가 0으로 채워져 있어 '수확 성공'으로 오인한다 → 빈 바구니를 예냉실로 나른다.

        on_progress(dict): Feedback 이 올 때마다 호출(라운드·누적 3개·남은 개수).
            ⚠️ ROS executor 스레드에서 실행된다. 호출부(노드)는 여기서 무거운 일을 하지
            말고 fire-and-forget 통지만 해야 한다 — 막히면 액션 콜백 처리가 밀린다.
            이 클래스가 직접 통지하지 않는 이유: HTTP 를 모르는 순수 오케스트레이터로
            남겨야 ROS·DB·네트워크 없이 단위테스트가 된다.
        heartbeat=(engine, [cid...], robot_id): 결과를 기다리는 동안 자리 예약 갱신.
            수확은 분 단위라 이게 없으면 팔 작업 중에 자리를 뺏긴다.
        반환: {"normal_count","discard_count","failed_count","exit_reason","message"}
              또는 None(서버 미기동 / Goal 거부 / 타임아웃 / 중단·취소).
        """
        if not harvest_client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            self._log.warning(
                f"[HARVEST] {robot_id} Harvest 액션 서버 미기동 task={task_id}")
            return None

        goal = Harvest.Goal()
        goal.task_id = int(task_id)
        goal.max_capacity = int(HARVEST_MAX_CAPACITY)

        def _fb(msg):
            """ROS executor 스레드 — 진행 상황을 호출부에 넘기기만 한다."""
            try:
                fb = msg.feedback
                self._log.info(
                    f"[HARVEST] 진행 task={task_id} 라운드 {fb.round} "
                    f"정상 {fb.normal_count} 폐기 {fb.discard_count} "
                    f"실패 {fb.failed_count} 남은 {fb.remaining_in_round}")
                if on_progress is not None:
                    on_progress({
                        "round": int(fb.round),
                        "normal_count": int(fb.normal_count),
                        "discard_count": int(fb.discard_count),
                        "failed_count": int(fb.failed_count),
                        "remaining_in_round": int(fb.remaining_in_round),
                    })
            except Exception as exc:  # noqa: BLE001
                self._log.warning(f"[HARVEST] 진행 피드백 처리 예외(무시): {exc}")

        self._log.info(
            f"[HARVEST] E3~4 수확 시작 task={task_id} {robot_id} "
            f"(만차 기준 {HARVEST_MAX_CAPACITY})")
        goal_handle = spin_wait(
            harvest_client.send_goal_async(goal, feedback_callback=_fb),
            GOAL_ACCEPT_TIMEOUT_SEC)
        if goal_handle is None or not goal_handle.accepted:
            # DG 는 '도킹 성공한 task' 의 goal 만 accept 한다 → 거부는 도킹 상태 불일치 신호.
            self._log.warning(
                f"[HARVEST] Harvest Goal 거부/수락 타임아웃 task={task_id} "
                f"(DG 가 도킹 상태를 다르게 알고 있을 수 있다)")
            return None

        return self._await_harvest_result(
            goal_handle.get_result_async(), task_id, heartbeat)

    def _await_harvest_result(self, result_future, task_id, heartbeat):
        """수확 결과 대기. 대기 중 HEARTBEAT_SEC 마다 쥔 자리 예약을 갱신한다.

        반환: 집계 dict(성공) 또는 None(타임아웃/중단·취소/파싱 실패).
        """
        done = threading.Event()
        result_future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + HARVEST_RESULT_TIMEOUT_SEC
        while not done.wait(HEARTBEAT_SEC):
            if heartbeat is not None:
                engine, cids, robot_id = heartbeat
                for cid in cids:
                    engine.heartbeat(cid, robot_id)
            if time.monotonic() >= deadline:
                self._log.warning(
                    f"[HARVEST] 수확 결과 대기 타임아웃"
                    f"({HARVEST_RESULT_TIMEOUT_SEC}s) task={task_id} → 실패 취급")
                return None
        try:
            response = result_future.result()
            status = int(response.status)
            res = response.result
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"[HARVEST] 수확 결과 파싱 실패 task={task_id}: {exc}")
            return None
        if status != GoalStatus.STATUS_SUCCEEDED:
            # 여기가 이 함수의 존재 이유다. 집계 필드는 중단됐어도 0 으로 채워져 오므로
            # 본문만 보면 '아무것도 못 땄지만 성공'과 구분되지 않는다.
            self._log.warning(
                f"[HARVEST] 수확이 정상 종료되지 않았다 task={task_id} "
                f"(goal status={status}) → 실패 취급")
            return None
        return {
            "normal_count": int(res.normal_count),
            "discard_count": int(res.discard_count),
            "failed_count": int(res.failed_count),
            "exit_reason": str(res.exit_reason),
            "message": str(res.message),
        }

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

from automato_control_service import docking
from automato_control_service.patrol_config import SERVER_WAIT_SEC
from automato_control_service.route_runner import RouteRunner

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
                    clients, start_wp=None):
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
            # TODO(E3~E6): 여기부터 Harvest 액션·실적 저장·예냉실 이송이 붙는다.
            self._log.warning(
                f"[HARVEST] E3~E6 미구현 → task {task_id} FAILED 로 마감 "
                f"(수확지 도킹까지는 성공)")
            return STATUS_FAILED, None
        finally:
            # drive 는 '지금 서 있는 자리'를 일부러 남기고 나온다 — 다음 구간이 이어받아
            # 예약이 끊기는 순간을 없애기 위해서다. 수확은 아직 뒷단계가 없으므로 여기서
            # 반납하지 않으면 그 자리가 영영 점유된 채로 남아 남의 길을 막는다.
            engine.release(engine.node_slot(current), robot_id)
            self._log.info(
                f"[HARVEST] task={task_id} 지점 {current} 자리 반납")

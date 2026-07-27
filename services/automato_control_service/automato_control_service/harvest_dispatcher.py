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

⚠️ 지금은 뼈대(스텁)다. run_harvest 의 흐름만 잡아 두고, 각 단계 로직은 이후에 채운다.

■ 채울 때 확정할 설계 노트
  (1) ✅ 주행 재사용 — RouteRunner 로 추출 완료(C-1). E2/E5 주행은 self.runner.drive 를
      쓴다. 촬영·짝이 없는 평범한 주행이라 DriveHooks 를 안 넘기면 된다(기본값).
  (2) DB 경계 — E5 실적 저장은 '수확 종료 직후'(예냉실 도착 아님)라 오케스트레이션 중간에
      DB 쓰기가 낀다. 저장을 노드가 할지(반환/콜백), 디스패처가 pool 로 직접 할지 그때 정한다.
"""

from automato_control_service.route_runner import RouteRunner

# 최종 상태 — 노드가 tasks 에 마감한다(automato_db.set_task_status 의 유효값과 호환).
STATUS_COMPLETED = "COMPLETED"
STATUS_FAILED = "FAILED"


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

    def run_harvest(self, task_id, robot_id, harvest_location, engine, clients,
                    start_wp=None):
        """수확 task 하나를 E2~E6 순서로 처리하고 최종 상태를 돌려준다.

        harvest_location : 수확 위치 task_point_id (예: 'HARVEST_01')
        engine           : 순찰과 공유하는 RoutingEngine(경로탐색+통로예약)
        clients          : 노드가 만든 액션 클라이언트 묶음
                           {"nav","dock","harvest","unload"} (robot_id 로 바인딩됨)
        start_wp         : 출발 노드(로봇 전용 충전소 진입노드). None 이면 디스패처가 폴백.

        반환: STATUS_COMPLETED | STATUS_FAILED  (노드가 tasks 에 마감)

        ⚠️ 스텁: 아직 실제 수행 없이 FAILED 를 돌려준다(골격 배선 확인용).
        채울 순서:
          status  = E2  수확지까지 주행 + ChArUco 도킹        (clients['nav'], clients['dock'])
          outcome = E3~4 Harvest 액션(Ddagi 주관)+피드백 중계  (clients['harvest'])
          ...     설계노트(2)에 따라 수확 실적 저장(harvest_batches)...
          status  = E5  예냉실까지 주행 + 도킹                 (clients['nav'], clients['dock'])
          ...     E6 완료 처리 + (보너스) Unload 하역           (clients['unload'])
        """
        self._log.warning(
            f"[HARVEST] run_harvest 스텁 — task={task_id} robot={robot_id} "
            f"목적지={harvest_location} (E2~E6 미구현, 골격만)")
        return STATUS_FAILED

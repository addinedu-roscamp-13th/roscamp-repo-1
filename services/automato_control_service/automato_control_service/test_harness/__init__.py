"""RP-78 테스트 전용 임시 스탠드인 (test harness).

⚠️ 여기 노드들은 '실제 DG Control Service(HQ)'가 아니다. RP-78(ACS)을 실물 로봇으로
   검증하기 위해 아직 없는 상·하류 부품의 '최소 대역'만 흉내내는 테스트 도구다.
   실제 DG Control Service가 준비되면 이 폴더는 통째로 걷어내면 된다.

포함:
  dg_stub          : /ddago(·ddagi)/telemetry 를 구독해 msg.robot_id 로 로봇을 갈라
                     /{robot_id}/telemetry (RobotTelemetry) 로 발행 — DG 중계 대역.
                     (RP-114 이전 이름은 fleet_aggregator. 3대분 취합은 ACS 가 가져갔다.)
  fake_telemetry   : 물리 로봇 없이 /ddago/telemetry 를 발행하는 가짜 로봇 대역.
  patrol_bridge    : ACS가 보내는 /dg_0x/patrol (Patrol 액션) 을 받아
                     sim(가짜 도착) 또는 nav2(실제 navigate_to_pose 주행) 로 처리하는 주행 대역.
"""

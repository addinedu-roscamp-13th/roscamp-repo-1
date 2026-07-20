#!/usr/bin/env python3
"""시나리오1 E1 E2E 시뮬 스탠드인 일괄 기동 — 물리 로봇 없이 순찰 접수~주행을 돌린다.

⚠️ 여기서 뜨는 노드는 전부 '테스트 스탠드인'이다(실제 로봇/DG Control Service 아님).
   ACS 본체(patrol_node)는 일부러 빼 두었다 — 아래 '왜 ACS는 따로 띄우나' 참고.

띄우는 것 (robots 인자에 적은 로봇마다 2개 + 공통 1개):
  로봇마다  fake_telemetry   : /ddago/telemetry 1Hz 발행 (가짜 DdaGo 상태)
            patrol_bridge    : /<robot_id>/navigate 액션 서버 (가짜 DCS+DdaGo 주행)
  공통      fleet_aggregator : 위를 묶어 /automato/telemetry/fleet 발행 (가짜 DCS 취합)

실행 (ROS + 워크스페이스 2개 소싱 후, 리포 안에서):
  # 터미널 1 — 스탠드인 일괄
  ros2 launch automato_control_service patrol_e2e_sim.launch.py
  # 터미널 2 — ACS 본체
  ros2 run automato_control_service patrol_node
  # 터미널 3 — 접수
  curl -X POST localhost:8200/internal/v1/tasks/patrol \\
       -H 'Content-Type: application/json' -d '{"robot_selection":"auto","robot_id":null}'

인자:
  robots            콤마구분 robot_id 목록          (기본 dg_01)
  batteries         콤마구분 배터리%, 위와 같은 순서 (기본 빈값 → 전부 90.0)
                    → 임계값(70) 아래로 주면 BATTERY_TOO_LOW 재현, auto 선정 규칙 확인용
  sim_seconds       waypoint 하나당 가짜 이동 시간(초) (기본 1.0)
  fail_waypoint_ids 막힘으로 응답할 waypoint_id 콤마구분 (기본 빈값)
                    → 막힘→블랙리스트→Dijkstra 우회 경로를 로봇 없이 재현

예:
  ros2 launch automato_control_service patrol_e2e_sim.launch.py \\
      robots:=dg_01,dg_02,dg_03 batteries:=90.0,65.0,80.0 fail_waypoint_ids:=14

⚠️ 룩어헤드·조기반납을 관측하려면 sim_seconds 를 ACS 의 하트비트 주기보다 크게 잡아야 한다.
   둘은 ACS 의 결과 대기 루프(_await_result)에서 맞물린다 — 룩어헤드(on_tick)는 하트비트
   틱마다 돌기 때문에, 세그먼트 주행이 한 틱보다 빨리 끝나면 틱이 0번 돌아 매번 '다음 통로
   미확보'로 보인다(코드 문제가 아니라 시뮬이 너무 빠른 것). 둘 중 하나로 맞춘다:
     sim_seconds:=6.0 으로 느리게        (기본 하트비트 5초보다 길게)
     ACS_HEARTBEAT_SEC=0.3 ros2 run ...  (ACS 쪽 틱을 잘게; 실물 튜닝값은 안 건드림)

왜 ACS(patrol_node)는 이 launch에 없나:
  ① patrol_node 는 DB 접속 문자열을 '실행한 디렉터리에서 위로 올라가며' services/database/.env
     를 찾아 얻는다. 별도 터미널에서 리포 안에 서서 띄우는 편이 사고가 없다.
  ② FastAPI(uvicorn) 로그와 스탠드인 로그가 한 화면에 섞이면 API 응답 확인이 어렵다.
  ③ 코드를 고쳐 ACS만 재기동하는 일이 잦은데, 그때마다 가짜 로봇까지 다시 뜨면 느리다.

왜 OpaqueFunction 을 쓰나 (launch 파일 문법 배경):
  launch 인자(LaunchConfiguration)는 '나중에 문자열로 치환될 자리표시자'라, launch 파일을
  읽는 시점엔 아직 값이 없다. 그래서 "robots 값을 콤마로 쪼개 그 개수만큼 노드를 만든다"
  같은 파이썬 계산은 실행 시점에 호출되는 OpaqueFunction 안에서 해야 한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'automato_control_service'
DEFAULT_BATTERY = 90.0


def _split(csv: str) -> list:
    """콤마구분 문자열 → 공백 제거한 항목 리스트(빈 항목은 버림)."""
    return [s.strip() for s in (csv or '').split(',') if s.strip()]


def _launch_setup(context, *args, **kwargs):
    """인자가 실제 문자열로 정해진 뒤 호출된다 → 여기서 노드 목록을 만든다."""
    robots = _split(LaunchConfiguration('robots').perform(context)) or ['dg_01']
    batteries = _split(LaunchConfiguration('batteries').perform(context))
    sim_seconds = float(LaunchConfiguration('sim_seconds').perform(context))
    fail_ids = LaunchConfiguration('fail_waypoint_ids').perform(context)

    actions = [LogInfo(msg=(
        f'[TEST] 스탠드인 기동: 로봇 {robots} / waypoint당 {sim_seconds}초 '
        f'/ 막힘 waypoint "{fail_ids or "없음"}" — 실제 로봇·DCS 아님'))]

    for i, rid in enumerate(robots):
        # 배터리는 지정한 만큼만 순서대로 매핑하고, 모자라면 기본값.
        battery = float(batteries[i]) if i < len(batteries) else DEFAULT_BATTERY
        actions.append(Node(
            package=PKG, executable='fake_telemetry',
            namespace=rid, name='fake_telemetry', output='screen',
            # 여기 타입(실수/문자열)은 노드의 declare_parameter 선언과 맞아야 한다.
            # 실행 중 값을 바꿀 땐: ros2 param set /<rid>/fake_telemetry battery_percent 65.0
            parameters=[{
                'robot_id': rid,
                'nav_status': 'IDLE',        # 'IDLE' 이어야 가용 후보가 된다
                'battery_percent': battery,
            }],
        ))
        actions.append(Node(
            package=PKG, executable='patrol_bridge',
            # 네임스페이스가 액션 이름을 만든다: 상대명 'navigate' → /<rid>/navigate
            namespace=rid, name='patrol_bridge', output='screen',
            parameters=[{
                'mode': 'sim',               # Nav2 없이 '도달했다'고만 응답
                'robot_id': rid,
                'sim_seconds': sim_seconds,
                'fail_waypoint_ids': fail_ids,
            }],
        ))

    # 취합 노드는 토픽이 절대경로(/automato/telemetry/fleet) 단일이라 1개만, 네임스페이스 없이.
    actions.append(Node(
        package=PKG, executable='fleet_aggregator',
        name='fleet_aggregator', output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots', default_value='dg_01',
            description='콤마구분 robot_id 목록 (예: dg_01,dg_02,dg_03)'),
        DeclareLaunchArgument(
            'batteries', default_value='',
            description=f'콤마구분 배터리%(robots 와 같은 순서). 비면 전부 {DEFAULT_BATTERY}'),
        DeclareLaunchArgument(
            'sim_seconds', default_value='1.0',
            description='sim 모드에서 waypoint 하나당 가짜 이동 시간(초)'),
        DeclareLaunchArgument(
            'fail_waypoint_ids', default_value='',
            description='막힘(result_code=1)으로 응답할 waypoint_id 콤마구분'),
        OpaqueFunction(function=_launch_setup),
    ])

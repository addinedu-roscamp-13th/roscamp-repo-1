#!/usr/bin/env python3
"""RP-EX  Automato Control Service 실전 스택 일괄 기동 launch (bringup).

시나리오1 순찰에서 '실제로 쓰이는' ACS 노드 3개를 한 번에 올린다. 테스트 스탠드인
(fake_telemetry·patrol_bridge·dg_stub)은 여기 없다 — 그건 로봇 없이 흐름만 볼 때 쓰는
patrol_e2e_sim.launch.py 몫이다.

띄우는 것:
  patrol_node                 순찰 오케스트레이터 + HTTP API   http://0.0.0.0:8200
  telemetry_ws_node           텔레메트리 WebSocket 방송        ws://0.0.0.0:8000/ws/telemetry
  fleet_telemetry_aggregator  QT 대시보드용 취합 발행          (기존 launch 재사용)

HTTP·WebSocket 은 따로 켜지 않는다:
  patrol_node·telemetry_ws_node 는 각자 프로세스 안에서 rclpy 노드(백그라운드 spin)와
  uvicorn(FastAPI, 메인 스레드)을 함께 돌린다. 그래서 이 노드를 launch 에 넣기만 하면
  HTTP·WS 서버가 자동으로 함께 뜬다. 포트는 환경변수로 바꿀 수 있다:
    ACS_API_PORT (기본 8200)   ACS_WS_PORT (기본 8000)

⚠️ 반드시 '리포 안에서' 실행한다:
  patrol_node·telemetry_ws_node 는 DB(automato_db)에 붙는데, 접속문자열을
  ① 환경변수 DATABASE_URL → ② 없으면 '현재 작업 디렉터리(CWD)에서 위로 올라가며
  services/database/.env' 순으로 찾는다. ros2 launch 는 CWD 를 바꾸지 않으므로,
  리포 안에서 실행하면 노드가 그 CWD 를 물려받아 .env 를 그대로 찾는다(수동 ros2 run 과 동일).
  리포 밖에서 띄우려면 미리 DATABASE_URL 을 export 한다.

robot_ids (세 노드가 공유):
  세 노드 모두 /{robot_id}/telemetry 를 구독한다. 그래서 '구독할 로봇 목록'을 최상위
  인자 하나로 두고 셋 다에 넘긴다 — 로봇 구성이 바뀌면 여기 한 곳만 고친다.

  타입 주의(launch 문법): launch 인자는 항상 '문자열'로 들어온다. 그런데 robot_ids
  파라미터는 문자열 배열(list)이라야 한다. 그래서 Node 로 직접 넘기는 두 노드는
  PythonExpression 으로 문자열 "['dg_01',...]" 를 실제 파이썬 리스트로 평가해 넘긴다.
  (fleet_telemetry_aggregator.launch.py 는 자기 안에서 같은 변환을 하므로 문자열 그대로 넘긴다.)

실행 예:
  # (ROS + 워크스페이스 2개 소싱 후, 리포 안에서)
  ros2 launch automato_control_service acs_bringup.launch.py
  # 로봇 2대만 구독
  ros2 launch automato_control_service acs_bringup.launch.py robot_ids:="['dg_01','dg_02']"
  # 접수(HTTP) 확인
  curl -X POST localhost:8200/internal/v1/tasks/patrol \\
       -H 'Content-Type: application/json' -d '{"robot_selection":"auto","robot_id":null}'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

PKG = 'automato_control_service'


def generate_launch_description():
    # 설치된 share 트리의 launch 디렉터리 — 재사용할 aggregator launch 가 여기 있다.
    launch_dir = os.path.join(get_package_share_directory(PKG), 'launch')
    robot_ids = LaunchConfiguration('robot_ids')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ids', default_value="['dg_01','dg_02','dg_03']",
            description="구독할 로봇 목록(세 노드 공유). 예: robot_ids:=\"['dg_01','dg_02']\""),

        # ── patrol_node — 순찰 오케스트레이터 + HTTP API(기본 8200) ──
        Node(
            package=PKG, executable='patrol_node', output='screen',
            # 문자열 인자를 실제 리스트로 평가해 넘긴다(위 '타입 주의' 참고).
            parameters=[{'robot_ids': PythonExpression(robot_ids)}],
        ),

        # ── telemetry_ws_node — 텔레메트리 WebSocket 방송(기본 8000) ──
        Node(
            package=PKG, executable='telemetry_ws_node', output='screen',
            parameters=[{'robot_ids': PythonExpression(robot_ids)}],
        ),

        # ── fleet_telemetry_aggregator — QT 대시보드용 취합(기존 launch 재사용) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'fleet_telemetry_aggregator.launch.py')),
            # 이 하위 launch 는 자기 안에서 문자열→리스트 변환을 하므로 문자열 그대로 넘긴다.
            launch_arguments={'robot_ids': robot_ids}.items(),
        ),
    ])

#!/usr/bin/env python3
"""RP-114  E0 ③④: Fleet 텔레메트리 취합 노드 기동 launch.

구독은 로봇별 네임스페이스(/{robot_id}/telemetry)라 robot_ids 로 목록을 주고,
발행은 절대경로 단일 토픽(/automato/dashboard/...)이라 노드 자체엔 네임스페이스를 두지 않는다.

실행 예:
  ros2 launch automato_control_service fleet_telemetry_aggregator.launch.py
  # 로봇 2대만 띄운 상태로 검증할 때
  ros2 launch automato_control_service fleet_telemetry_aggregator.launch.py \\
      robot_ids:="['dg_01','dg_02']"
  # 팀원의 DG 이전이 끝나 옛 /automato/telemetry/fleet 구독이 필요 없어지면
  ros2 launch automato_control_service fleet_telemetry_aggregator.launch.py \\
      legacy_input:=false

(launch 없이 바로 실행해도 된다: ros2 run automato_control_service fleet_telemetry_aggregator)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_ids = LaunchConfiguration('robot_ids')
    output_topic = LaunchConfiguration('output_topic')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    legacy_input = LaunchConfiguration('legacy_input')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ids',
            default_value="['dg_01','dg_02','dg_03']",
            description="구독할 로봇 목록. 예: robot_ids:=\"['dg_01','dg_02']\"",
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/automato/dashboard/fleet_telemetry',
            description='QT 대시보드용 취합 발행 토픽(FleetTelemetry)',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='1.0',
            description='취합 발행 주기(Hz)',
        ),
        DeclareLaunchArgument(
            'legacy_input',
            default_value='true',
            description='[삭제 예정] 옛 /automato/telemetry/fleet 도 함께 구독할지',
        ),
        Node(
            package='automato_control_service',
            executable='fleet_telemetry_aggregator',
            name='fleet_telemetry_aggregator',
            output='screen',
            parameters=[{
                # launch 인자는 문자열로 들어온다. 리스트/불리언 파라미터는 그대로 넘기면
                # 문자열 "['dg_01',...]" 로 선언돼 타입이 어긋나므로 PythonExpression 으로
                # 실제 파이썬 값으로 평가해 넘긴다.
                'robot_ids': PythonExpression(robot_ids),
                'output_topic': output_topic,
                'publish_rate_hz': PythonExpression(publish_rate_hz),
                'legacy_input': PythonExpression(['"', legacy_input, '" == "true"']),
            }],
        ),
    ])

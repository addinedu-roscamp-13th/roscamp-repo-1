#!/usr/bin/env python3
"""dg_sim — DCS 테스트용 상대편 시뮬 4종을 한 번에 기동.

  acs_sim   : Automato Control Service 대역 (Navigate 클라 / SaveDetection 서버 / Fleet 구독)
  ddago_sim : DdaGo 대역 (Navigate 서버(Waypoint[]) / DdagoTelemetry / AnalyzeFrame 클라)
  ddagi_sim : Ddagi 대역 (DdagiTelemetry)
  dg_ai_sim : DG AI Service 대역 (TCP 9100)

사용:
  ros2 launch dg_sim dg_sim.launch.py
  ros2 launch dg_sim dg_sim.launch.py robot_id:=dg_01 auto_start:=false

개별 on/off 는 대시보드(dashboard.sh / dg_web)에서 하고, 이 launch 는 4종 동시 기동용.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    auto_start = LaunchConfiguration('auto_start')
    scenario = LaunchConfiguration('scenario')
    dock_mode = LaunchConfiguration('dock_mode')
    return LaunchDescription([
        # 로봇 식별자는 환경변수 ROBOT_ID(~/.bashrc) 를 따른다. 인자로 덮어쓸 수 있다:
        #   ros2 launch dg_sim dg_sim.launch.py robot_id:=dg_02
        DeclareLaunchArgument(
            'robot_id',
            default_value=EnvironmentVariable('ROBOT_ID', default_value='dg_01')),
        DeclareLaunchArgument('auto_start', default_value='true'),
        # auto_start 시 실행할 시나리오: patrol(S1) | harvest(S2 E2 이동+도킹)
        #   ros2 launch dg_sim dg_sim.launch.py scenario:=harvest
        DeclareLaunchArgument('scenario', default_value='patrol'),
        # DdaGo 도킹 결과 시뮬: success | no_marker | error_exceeded | hang
        #   ros2 launch dg_sim dg_sim.launch.py scenario:=harvest dock_mode:=no_marker
        DeclareLaunchArgument('dock_mode', default_value='success'),
        Node(package='dg_sim', executable='ddagi_sim', name='ddagi_sim',
             parameters=[{'robot_id': robot_id}]),
        Node(package='dg_sim', executable='ddago_sim', name='ddago_sim',
             parameters=[{'robot_id': robot_id, 'dock_mode': dock_mode}]),
        Node(package='dg_sim', executable='acs_sim', name='acs_sim',
             parameters=[{'robot_id': robot_id, 'auto_start': auto_start,
                          'scenario': scenario}]),
        Node(package='dg_sim', executable='dg_ai_sim', name='dg_ai_sim'),
    ])

#!/usr/bin/env python3
"""RP-77  E0 ③④: Fleet 텔레메트리 릴레이 기동 launch.

토픽이 절대경로 단일 인스턴스(/automato/...)라 네임스페이스가 필요 없다.
input_topic·output_topic 인자로 오버라이드할 수 있게만 열어둔 최소 launch.

실행 예:
  ros2 launch automato_control_service fleet_telemetry_relay.launch.py
  ros2 launch automato_control_service fleet_telemetry_relay.launch.py \\
      input_topic:=/automato/telemetry/fleet \\
      output_topic:=/automato/dashboard/fleet_telemetry

(launch 없이 바로 실행해도 된다: ros2 run automato_control_service fleet_telemetry_relay)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/automato/telemetry/fleet',
            description='구독할 HQ 취합 토픽(FleetTelemetry)',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/automato/dashboard/fleet_telemetry',
            description='QT 대시보드용 재발행 토픽(FleetTelemetry)',
        ),
        Node(
            package='automato_control_service',
            executable='fleet_telemetry_relay',
            name='fleet_telemetry_relay',
            output='screen',
            parameters=[{
                'input_topic': input_topic,
                'output_topic': output_topic,
            }],
        ),
    ])

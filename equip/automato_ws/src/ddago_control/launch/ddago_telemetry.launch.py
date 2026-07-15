#!/usr/bin/env python3
"""RP-75  E0: DdaGo 텔레메트리 Publisher 기동 launch.

인자를 두 개로 분리한다:
  namespace  토픽/TF 가 올라갈 위치(기본 dg_01/ddago). 드라이버·Nav2 와 같아야
             소스 토픽(odom, amcl_pose ...)이 상대 이름으로 매칭된다.
  robot_id   보고서(DdagoTelemetry.robot_id)에 적을 로봇 식별자(기본 dg_01).
             네임스페이스와 별개라, 위치를 바꿔도 식별자는 그대로 유지된다.

실행 예:
  ros2 launch ddago_control ddago_telemetry.launch.py
  ros2 launch ddago_control ddago_telemetry.launch.py namespace:=dg_02/ddago robot_id:=dg_02

결과 토픽: /<namespace>/telemetry  (예: /dg_01/ddago/telemetry)

※ 위치/배터리/초음파/Nav2 상태 소스 토픽도 같은 /<namespace> 로 올라와야 상대
   토픽명이 매칭된다(핑키 드라이버·Nav2를 동일 네임스페이스로 기동).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_id = LaunchConfiguration('robot_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='dg_01/ddago',
            description='토픽/TF 가 올라갈 네임스페이스 (드라이버·Nav2 와 동일하게)',
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='dg_01',
            description='보고서에 적을 로봇 식별자 (네임스페이스와 별개)',
        ),
        GroupAction([
            PushRosNamespace(namespace),
            Node(
                package='ddago_control',
                executable='telemetry_publisher',
                name='telemetry_publisher',
                output='screen',
                parameters=[{'robot_id': robot_id}],
                # /tf·/tf_static 을 상대명으로 리맵 → 네임스페이스 tf(/dg_01/ddago/tf)를
                # 구독한다. bringup 쪽 드라이버 tf 도 같은 네임스페이스에 있으므로
                # telemetry 가 이 로봇의 map→base_footprint 변환을 읽을 수 있다.
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            ),
        ]),
    ])

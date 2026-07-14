#!/usr/bin/env python3
"""RP-75  E0: DdaGo 텔레메트리 Publisher 기동 launch.

robot_id 인자 하나로 (1) 네임스페이스 접두어와 (2) 노드 robot_id 파라미터를
동시에 지정한다 → 한 번만 입력하면 되고 서로 어긋날 일이 없다.

실행 예:
  ros2 launch ddago_control ddago_telemetry.launch.py robot_id:=dg_01
  ros2 launch ddago_control ddago_telemetry.launch.py robot_id:=dg_02   # 동시 실행 가능

결과 토픽: /<robot_id>/ddago/telemetry  (예: /dg_01/ddago/telemetry)

※ 위치/배터리/초음파/Nav2 상태 소스 토픽도 같은 /<robot_id> 네임스페이스로
   올라와야 상대 토픽명이 매칭된다(핑키 드라이버·Nav2를 동일 네임스페이스로 기동).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='dg_01',
            description='로봇 식별자. 네임스페이스 접두어 + telemetry robot_id 로 함께 쓰임',
        ),
        GroupAction([
            PushRosNamespace(robot_id),
            Node(
                package='ddago_control',
                executable='telemetry_publisher',
                name='telemetry_publisher',
                output='screen',
                parameters=[{'robot_id': robot_id}],
                # /tf·/tf_static 을 상대명으로 리맵 → 네임스페이스 tf(/dg_01/tf)를
                # 구독한다. bringup 쪽 드라이버 tf 도 같은 네임스페이스에 있으므로
                # telemetry 가 이 로봇의 map→base_footprint 변환을 읽을 수 있다.
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            ),
        ]),
    ])

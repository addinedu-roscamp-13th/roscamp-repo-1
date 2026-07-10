#!/usr/bin/env python3
"""RP-76  E2: DdaGo Patrol Action 서버 기동 launch.

robot_id 인자 하나로 (1) 네임스페이스 접두어와 (2) 노드 robot_id 파라미터를
동시에 지정한다 → 한 번만 입력하면 되고 서로 어긋날 일이 없다.
(telemetry launch 와 동일한 방식)

카메라 토픽은 실기 드라이버 bringup(RP-85) 후 확정되므로 camera_topic 인자로
노출해 하드코딩 없이 주입한다. 기본값은 상대이름 'image_raw'(v4l2_camera 관례).

네임스페이스가 붙는 이름 / 안 붙는 이름:
  ddago/patrol       → /<robot_id>/ddago/patrol      (Patrol 서버, 로봇별)
  navigate_to_pose   → /<robot_id>/navigate_to_pose  (Nav2, 로봇별)
  <camera_topic>     → /<robot_id>/image_raw         (카메라, 로봇별)
  /dg/analyze_frame  → /dg/analyze_frame             (절대이름 → HQ 공용, 안 붙음)

실행 예:
  ros2 launch ddago_control ddago_patrol.launch.py robot_id:=dg_01
  ros2 launch ddago_control ddago_patrol.launch.py robot_id:=dg_01 camera_topic:=camera/image_raw
  ros2 launch ddago_control ddago_patrol.launch.py robot_id:=dg_02   # 동시 실행 가능

※ Nav2(navigate_to_pose)와 카메라 드라이버도 같은 /<robot_id> 네임스페이스로
   올라와야 상대 토픽명이 매칭된다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    camera_topic = LaunchConfiguration('camera_topic')
    analyze_service = LaunchConfiguration('analyze_service')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='dg_01',
            description='로봇 식별자. 네임스페이스 접두어 + patrol robot_id 로 함께 쓰임',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='image_raw',
            description='구독할 카메라 이미지 토픽(상대). RP-85 확정 후 실제 토픽으로 주입',
        ),
        DeclareLaunchArgument(
            'analyze_service',
            default_value='/dg/analyze_frame',
            description='HQ 분석요청 서비스(절대이름 — 로봇 공용이라 네임스페이스 안 붙음)',
        ),
        GroupAction([
            PushRosNamespace(robot_id),
            Node(
                package='ddago_control',
                executable='patrol_server',
                name='patrol_server',
                output='screen',
                parameters=[{
                    'robot_id': robot_id,
                    'camera_topic': camera_topic,
                    'analyze_service': analyze_service,
                }],
            ),
        ]),
    ])

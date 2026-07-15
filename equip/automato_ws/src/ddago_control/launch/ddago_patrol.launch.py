#!/usr/bin/env python3
"""RP-76  E2: DdaGo Patrol Action 서버 기동 launch.

인자를 두 개로 분리한다(telemetry launch 와 동일한 방식):
  namespace  토픽/액션이 올라갈 위치(기본 dg_01/ddago). Nav2·카메라 드라이버와
             같아야 상대 이름(navigate_to_pose, image_raw)이 매칭된다.
  robot_id   로그/식별용 로봇 id(기본 dg_01). 네임스페이스와 별개.

카메라 토픽은 실기 드라이버 bringup(RP-85) 후 확정되므로 camera_topic 인자로
노출해 하드코딩 없이 주입한다. 기본값은 상대이름 'image_raw'(v4l2_camera 관례).

네임스페이스가 붙는 이름 / 안 붙는 이름:
  patrol             → /<namespace>/patrol            (Patrol 서버, 로봇별)
  navigate_to_pose   → /<namespace>/navigate_to_pose  (Nav2, 로봇별)
  <camera_topic>     → /<namespace>/image_raw         (카메라, 로봇별)
  /dg/analyze_frame  → /dg/analyze_frame              (절대이름 → HQ 공용, 안 붙음)

실행 예:
  ros2 launch ddago_control ddago_patrol.launch.py
  ros2 launch ddago_control ddago_patrol.launch.py camera_topic:=camera/image_raw
  ros2 launch ddago_control ddago_patrol.launch.py namespace:=dg_02/ddago robot_id:=dg_02

※ Nav2(navigate_to_pose)와 카메라 드라이버도 같은 /<namespace> 로 올라와야
   상대 토픽명이 매칭된다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_id = LaunchConfiguration('robot_id')
    camera_topic = LaunchConfiguration('camera_topic')
    analyze_service = LaunchConfiguration('analyze_service')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='dg_01/ddago',
            description='토픽/액션이 올라갈 네임스페이스 (Nav2·카메라와 동일하게)',
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='dg_01',
            description='로그/식별용 로봇 id (네임스페이스와 별개)',
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
            PushRosNamespace(namespace),
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

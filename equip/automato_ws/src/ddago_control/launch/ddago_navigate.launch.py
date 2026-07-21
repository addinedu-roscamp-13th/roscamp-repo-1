#!/usr/bin/env python3
"""RP-113  E1/E2: DdaGo Navigate Action 서버 기동 launch.

로봇별 물리망 분리 이후, 이 로봇 내부의 이름(navigate_to_pose, spin, image_raw ...)은
드라이버·Nav2 와 함께 네임스페이스 없이 bare 로 뜬다. 그래서 이 launch 는 노드를
GroupAction/PushRosNamespace 로 감싸지 않고 root(bare) 로 띄운다 → 노드의 상대 이름이
bare 소스와 그대로 매칭된다. ddago_telemetry.launch.py 와 같은 규칙이다.

이름이 어떻게 뜨는지:
  /ddago/navigate      Navigate 서버      — 코드에 절대이름으로 박아둠(DCS 가 이 이름으로 호출)
  navigate_to_pose     Nav2 주행          → /navigate_to_pose
  spin                 Nav2 제자리회전     → /spin
  <camera_topic>       카메라             → /image_raw (기본값 기준)
  /dg/analyze_frame    DCS 분석요청       — 절대이름(로봇 공용)

robot_id 는 로그 표기용일 뿐이다. ddago 는 자기 정체를 모르며, 어느 로봇인지는
DCS 가 채운다(텔레메트리와 같은 원칙). 그래서 기본값을 그대로 써도 동작에는 영향이 없다.

실행 예:
  ros2 launch ddago_control ddago_navigate.launch.py
  ros2 launch ddago_control ddago_navigate.launch.py camera_topic:=camera/image_raw
  ros2 launch ddago_control ddago_navigate.launch.py arrival_settle_sec:=0.5

※ Nav2(navigate_to_pose·spin)와 카메라 드라이버도 같은 bare 이름으로 떠 있어야 한다.
   spin 은 Nav2 behavior server 가 제공하므로 behavior server 가 살아 있어야 짝 노드
   (양방향 촬영 지점)의 제자리 회전이 동작한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    camera_topic = LaunchConfiguration('camera_topic')
    analyze_service = LaunchConfiguration('analyze_service')
    arrival_settle_sec = LaunchConfiguration('arrival_settle_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='dg_01',
            description='로그 표기용 로봇 id (동작에는 영향 없음)',
        ),
        DeclareLaunchArgument(
            'camera_topic',
            default_value='image_raw',
            description='구독할 카메라 이미지 토픽(상대). RP-85 확정 후 실제 토픽으로 주입',
        ),
        DeclareLaunchArgument(
            'analyze_service',
            default_value='/dg/analyze_frame',
            description='DCS 분석요청 서비스(절대이름 — 로봇 공용이라 네임스페이스 안 붙음)',
        ),
        DeclareLaunchArgument(
            'arrival_settle_sec',
            default_value='0.3',
            description='도착 후 촬영까지 정지 대기(초). 잔상이 보이면 늘린다',
        ),
        Node(
            package='ddago_control',
            executable='navigate_server',
            name='navigate_server',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'camera_topic': camera_topic,
                'analyze_service': analyze_service,
                'arrival_settle_sec': arrival_settle_sec,
            }],
        ),
    ])

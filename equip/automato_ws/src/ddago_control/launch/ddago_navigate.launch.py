#!/usr/bin/env python3
"""RP-76  E2: DdaGo 로봇측 E2 스택 기동 launch — 카메라 노드 + Navigate 서버.

로봇별 물리망 분리 이후, 로봇 내부의 이름(navigate_to_pose, spin, /ddago/... )은
드라이버·Nav2 와 함께 네임스페이스 없이 bare 로 뜬다. 그래서 이 launch 는 노드를
GroupAction/PushRosNamespace 로 감싸지 않고 root(bare)로 띄운다 → 노드의 상대 이름이
bare 소스와 그대로 매칭된다. ddago_telemetry.launch.py 와 같은 규칙이다.

무엇이 뜨나:
  camera_node      CaptureFrame 서버   /ddago/capture_frame  — 웹캠 프레임 1장 제공
  navigate_server  Navigate 액션 서버   /ddago/navigate        — 경로 주행 + 촬영 요청

촬영 흐름: navigate_server 가 capture 지점 도착 시 /ddago/capture_frame 를 호출 →
camera_node 가 프레임 반환 → navigate_server 가 /dg/analyze_frame(DCS)로 전달.

카메라 소스(source):
  device : 실물 USB 웹캠(cv2.VideoCapture, /dev/video{device_index})
  file   : 웹캠 없이 테스트 — image_path 의 JPEG 1장을 매 요청마다 반환

robot_id 는 로그 표기용일 뿐이다. ddago 는 자기 정체를 모르며, 어느 로봇인지는
DCS 가 채운다(텔레메트리와 같은 원칙). 기본값을 그대로 써도 동작에 영향이 없다.

실행 예:
  # 실물(웹캠 연결)
  ros2 launch ddago_control ddago_navigate.launch.py
  ros2 launch ddago_control ddago_navigate.launch.py device_index:=2
  # 웹캠 없이(정지 이미지로 촬영 사슬 확인)
  ros2 launch ddago_control ddago_navigate.launch.py \
      source:=file image_path:=/home/ane/dummy_tomato.jpg

※ Nav2(navigate_to_pose·spin)도 같은 bare 이름으로 떠 있어야 주행/짝회전이 된다.
   spin 은 Nav2 behavior server 가 제공하므로 그것도 살아 있어야 한다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    capture_service = LaunchConfiguration('capture_service')
    analyze_service = LaunchConfiguration('analyze_service')
    arrival_settle_sec = LaunchConfiguration('arrival_settle_sec')
    source = LaunchConfiguration('source')
    device_index = LaunchConfiguration('device_index')
    image_path = LaunchConfiguration('image_path')
    frame_width = LaunchConfiguration('frame_width')
    frame_height = LaunchConfiguration('frame_height')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id', default_value='dg_01',
            description='로그 표기용 로봇 id (동작에는 영향 없음)'),
        DeclareLaunchArgument(
            'capture_service', default_value='/ddago/capture_frame',
            description='카메라 촬영 서비스 이름(절대). 두 노드가 같은 값을 써야 연결된다'),
        DeclareLaunchArgument(
            'analyze_service', default_value='/dg/analyze_frame',
            description='DCS 분석요청 서비스(절대이름 — 로봇 공용이라 네임스페이스 안 붙음)'),
        DeclareLaunchArgument(
            'arrival_settle_sec', default_value='0.3',
            description='도착 후 촬영까지 정지 대기(초). 잔상이 보이면 늘린다'),
        DeclareLaunchArgument(
            'source', default_value='device',
            description="카메라 프레임 소스: 'device'(실물 웹캠) 또는 'file'(정지 이미지)"),
        DeclareLaunchArgument(
            'device_index', default_value='0',
            description='device 모드에서 열 /dev/videoN 의 N'),
        DeclareLaunchArgument(
            'image_path', default_value='',
            description='file 모드에서 반환할 JPEG 경로'),
        DeclareLaunchArgument(
            'frame_width', default_value='640',
            description='device 모드 요청 해상도(가로)'),
        DeclareLaunchArgument(
            'frame_height', default_value='480',
            description='device 모드 요청 해상도(세로)'),

        # 카메라 노드 — CaptureFrame 서버(웹캠 프레임 제공)
        Node(
            package='ddago_control',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'capture_service': capture_service,
                'source': source,
                'device_index': device_index,
                'image_path': image_path,
                'frame_width': frame_width,
                'frame_height': frame_height,
            }],
        ),

        # Navigate 서버 — 경로 주행 + capture 지점에서 촬영 요청
        Node(
            package='ddago_control',
            executable='navigate_server',
            name='navigate_server',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'capture_service': capture_service,
                'analyze_service': analyze_service,
                'arrival_settle_sec': arrival_settle_sec,
            }],
        ),
    ])

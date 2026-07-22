#!/usr/bin/env python3
"""RP-EX  DdaGo 로봇측 스택 일괄 기동 launch (bringup).

로봇측 노드(주행+촬영+텔레메트리)를 한 번에 올린다. 지금까지는 두 개로 갈려 있었다:
  ddago_navigate.launch.py   → camera_node + navigate_server (주행·촬영)
  ddago_telemetry.launch.py  → telemetry_publisher          (상태 발행)
이 bringup 은 그 둘을 그대로 **불러와(include)** 합치기만 한다 — 노드 정의·인자·주석은
각 원본 launch 에 그대로 두고 여기선 중복하지 않는다. 그래서 필요하면 원본을 따로도
(주행만, 텔레메트리만) 쓸 수 있고, 평소엔 이 파일 하나로 전부 띄운다.

  ('bringup' 은 "한 컴포넌트의 스택 전체를 올린다"는 ROS2 관례 이름이다. nav2_bringup,
   turtlebot3_bringup 처럼.)

왜 include 인가:
  IncludeLaunchDescription 은 '다른 launch 파일을 이 자리에서 실행'해 준다. 노드를 여기
  다시 나열하면 인자 선언(9개)까지 복사돼 두 곳이 어긋날 위험이 생긴다. include 는 원본
  한 곳만 진실로 두므로 그 위험이 없다.

어디서 원본을 찾나:
  get_package_share_directory('ddago_control') 는 colcon 이 설치한 share 트리(install/…)의
  경로를 준다. 소스 트리(src/…)가 아니라 설치본을 읽으므로, 어느 워크스페이스 디렉터리에서
  실행해도 같은 파일을 찾는다. (그래서 새 launch 파일은 colcon build 후에야 보인다.)

노출 인자: 카메라 테스트 때 자주 바꾸는 것만 최상위로 끌어올려 ddago_navigate 로 넘긴다.
  나머지(해상도·서비스 이름 등)는 ddago_navigate 의 기본값을 그대로 쓴다.

실행 예:
  # 실물(웹캠 연결) — 전부 기본값
  ros2 launch ddago_control ddago_bringup.launch.py
  ros2 launch ddago_control ddago_bringup.launch.py device_index:=2
  # 웹캠 없이(정지 이미지로 촬영 사슬 확인)
  ros2 launch ddago_control ddago_bringup.launch.py \\
      source:=file image_path:=/home/ane/dummy_tomato.jpg
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 설치된 share 트리의 launch 디렉터리 — 원본 두 launch 가 여기에 복사돼 있다.
    launch_dir = os.path.join(
        get_package_share_directory('ddago_control'), 'launch')

    return LaunchDescription([
        # ── 최상위로 노출하는 인자(값은 아래 ddago_navigate 로 그대로 전달) ──
        DeclareLaunchArgument(
            'robot_id', default_value='dg_01',
            description='로그 표기용 로봇 id (동작에는 영향 없음)'),
        DeclareLaunchArgument(
            'source', default_value='device',
            description="카메라 프레임 소스: 'device'(실물 웹캠) 또는 'file'(정지 이미지)"),
        DeclareLaunchArgument(
            'device_index', default_value='-1',
            description='device 모드에서 열 /dev/videoN 의 N. -1=자동탐색(by-id, USB 웹캠)'),
        DeclareLaunchArgument(
            'image_path', default_value='',
            description='file 모드에서 반환할 JPEG 경로'),

        # ── 주행 + 촬영 스택 (camera_node + navigate_server) ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'ddago_navigate.launch.py')),
            # 최상위 인자를 그대로 하위 launch 의 같은 이름 인자로 넘긴다.
            # .items() 인 이유: launch_arguments 는 (이름, 값) 튜플의 나열을 받는다.
            launch_arguments={
                'robot_id': LaunchConfiguration('robot_id'),
                'source': LaunchConfiguration('source'),
                'device_index': LaunchConfiguration('device_index'),
                'image_path': LaunchConfiguration('image_path'),
            }.items(),
        ),

        # ── 텔레메트리 발행 (telemetry_publisher) — 넘길 인자 없음 ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'ddago_telemetry.launch.py'))),
    ])

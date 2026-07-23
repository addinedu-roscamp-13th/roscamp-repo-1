#!/usr/bin/env python3
"""RP-102  E4-6 / E2 22-1: DdaGo 정밀 후진 도킹 서버 기동 launch.

충전소 진입 노드까지 Navigate 로 도착한 뒤, ACS 가 Dock goal 을 내리면 로봇이
앞面 카메라로 ChArUco 보드를 보면서 뒷面을 스테이션에 붙인다.

로봇별 물리망 분리 이후 로봇 내부 이름은 드라이버·Nav2 와 함께 네임스페이스 없이
bare 로 뜬다. 그래서 이 launch 도 GroupAction/PushRosNamespace 로 감싸지 않고
root(bare)로 띄운다 — ddago_navigate/ddago_telemetry launch 와 같은 규칙이다.

무엇이 뜨나:
  dock_server   Dock 액션 서버   /ddago/dock   — 마커 탐색 → 중심선 정렬 → 접근
                                                → 180도 회전 → 후진 접붙임

카메라: **정면 picamera(CSI)를 직접 연다.** camera_node 가 쓰는 측면 순찰 웹캠과는
다른 장치라 서로 간섭하지 않는다. 카메라는 goal 실행 중에만 점유하고 끝나면 놓는다.

⚠️ camera_calib_file 은 **로봇마다 다르다.** 캘리브 해상도와 camera_width/height 가
어긋나면 초점거리 축척이 맞지 않아 거리(d)가 통째로 틀어진다(노드가 기동 시 검사해
에러 로그를 남긴다).

⚠️ 안전: bringup 에 cmd_vel 워치독이 없다. 처음 현장 투입 시에는 dry_run:=true 로
명령만 확인한 뒤 실주행할 것. 실주행 중에는 Ctrl+C 를 누를 수 있게 대기한다.

실행 예:
  # 안전 확인(모터 미발행) — 검출·중심선 계획만 로그로 본다
  ros2 launch ddago_control ddago_dock.launch.py dry_run:=true
  # 실주행
  ros2 launch ddago_control ddago_dock.launch.py
  # 후진 갭 튜닝 (odom 실거리라 1cm = 0.01)
  ros2 launch ddago_control ddago_dock.launch.py reverse_distance:=0.14

goal 예 (ACS 가 DB 의 마커 정보로 채워 보낸다. 아래는 mid24 스테이션 A):
  ros2 action send_goal /ddago/dock automato_interfaces/action/Dock \\
    "{task_id: 1, task_point_id: CHARGE_01, marker_id: '500',
      dictionary: DICT_5X5_1000, squares_x: 6, squares_y: 5,
      square_size_m: 0.024, marker_size_m: 0.018}" --feedback

※ odom(/odom)이 있어야 중심선 기동·180도 회전·후진이 동작한다(bringup 필요).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    camera_calib_file = LaunchConfiguration('camera_calib_file')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    rotate_180 = LaunchConfiguration('rotate_180')
    dry_run = LaunchConfiguration('dry_run')
    staging_distance = LaunchConfiguration('staging_distance')
    reverse_distance = LaunchConfiguration('reverse_distance')

    return LaunchDescription([
        # 로그 표기용. ddago 는 자기 정체를 모르며 어느 로봇인지는 DCS 가 안다.
        DeclareLaunchArgument('robot_id', default_value='dg_01'),
        # ★ 로봇마다 다르다. 캡처 해상도와 반드시 같은 조건에서 뽑은 것이어야 한다.
        DeclareLaunchArgument(
            'camera_calib_file',
            default_value='/home/pinky/charuco_dock_ws/camera_calib.npz'),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        # 카메라가 180도 뒤집혀 장착됨 → ISP(Transform)로 회전(CPU 절약).
        DeclareLaunchArgument('rotate_180', default_value='true'),
        # 참이면 cmd_vel 을 발행하지 않는다(검출·계획만 확인).
        DeclareLaunchArgument('dry_run', default_value='false'),
        # 스테이징 거리(카메라-보드중심). 근거리 검출 한계를 감안한 값.
        DeclareLaunchArgument('staging_distance', default_value='0.24'),
        # 후진 거리. odom 실이동거리 기준이라 명령값 1cm = 실제 갭 1cm.
        DeclareLaunchArgument('reverse_distance', default_value='0.15'),

        Node(
            package='ddago_control',
            executable='dock_server',
            name='ddago_dock_server',
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'camera_calib_file': camera_calib_file,
                'camera_width': camera_width,
                'camera_height': camera_height,
                'rotate_180': rotate_180,
                'dry_run': dry_run,
                'staging_distance': staging_distance,
                'reverse_distance': reverse_distance,
            }],
        ),
    ])

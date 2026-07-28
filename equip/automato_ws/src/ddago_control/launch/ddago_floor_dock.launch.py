#!/usr/bin/env python3
"""RP-126: DdaGo 바닥 H 마커(마커리스) 정밀 후진 도킹 서버 기동 launch.

충전소 진입 노드까지 Navigate 로 도착한 뒤, ACS 가 FloorDock goal 을 내리면 로봇이
전면 카메라로 바닥 청색 H 테이프를 보며 후면을 벽에 붙인다(후진 도킹).

ChArUco 도킹(ddago_dock.launch.py)과 별개 노드다. 로봇 내부 이름은 네임스페이스
없이 bare 로 뜬다(ddago_dock/navigate/telemetry 와 같은 규칙).

무엇이 뜨나:
  floor_dock_server   FloorDock 액션 서버   /ddago/floor_dock

⚠️ floor_calib_file 은 **로봇마다 다르다**(바닥 평면 캘리브, mtx/dist 내장).
   floor_calib.py 로 로봇에서 1회 생성한 것. 캘리브/캡처 해상도가 어긋나면 거리(d)가
   통째로 틀어진다.
⚠️ 안전: bringup 에 cmd_vel 워치독이 없다. 첫 투입은 dry_run:=true 로 명령만 확인.

실행 예:
  # 안전 확인(모터 미발행)
  ros2 launch ddago_control ddago_floor_dock.launch.py dry_run:=true
  # 실주행
  ros2 launch ddago_control ddago_floor_dock.launch.py
  # 벽갭 튜닝(후면~벽 목표 간격) + 디버그 로그 + 웹 뷰
  ros2 launch ddago_control ddago_floor_dock.launch.py wall_gap_target:=0.030 debug:=true stream:=true

goal 예 (마커리스라 마커 파라미터 없음; wall_gap/lateral 0 이면 노드 기본):
  ros2 action send_goal /ddago/floor_dock automato_interfaces/action/FloorDock \\
    "{task_id: 1, task_point_id: CHARGE_01, wall_gap_m: 0.0, lateral_offset_m: 0.0}" --feedback

※ odom(/odom)이 있어야 중심선 기동·180도 회전·후진이 동작한다(bringup 필요).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lc = LaunchConfiguration
    names = ('robot_id', 'floor_calib_file', 'camera_width', 'camera_height',
             'rotate_180', 'dry_run', 'wall_gap_target', 'lateral_offset',
             'd_stage', 'reverse_k', 'crossbar_to_wall', 'dynamic_reverse',
             'stage_settle_sec', 'cl_verify_d', 'cl_max_replans', 'post_advance_m',
             'post_dock_hold_sec', 'debug', 'stream', 'stream_port')

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='dg_01'),
        # ★ 로봇마다 다르다(바닥 평면 캘리브, mtx/dist 내장). floor_calib.py 로 생성.
        DeclareLaunchArgument(
            'floor_calib_file',
            default_value='/home/pinky/floor_dock_ws/floor_calib.npz'),
        DeclareLaunchArgument('camera_width', default_value='1280'),
        DeclareLaunchArgument('camera_height', default_value='720'),
        # 카메라 180도 뒤집혀 장착 → ISP(Transform)로 회전. 캘리브와 같은 조건이어야 함.
        DeclareLaunchArgument('rotate_180', default_value='true'),
        # 참이면 cmd_vel 미발행(검출·계획만 확인).
        DeclareLaunchArgument('dry_run', default_value='false'),
        # 목표 후면~벽 간격 [m]. goal 이 wall_gap_m>0 을 주면 그 값이 우선.
        DeclareLaunchArgument('wall_gap_target', default_value='0.025'),
        # 정렬 목표 횡 오프셋 [m] (+왼쪽). goal.lateral_offset_m 로 덮을 수 있다.
        DeclareLaunchArgument('lateral_offset', default_value='0.005'),
        # 스테이징 거리(가로바 중심-base) [m].
        DeclareLaunchArgument('d_stage', default_value='0.20'),
        # 실측 상수(rear_to_wall = d + reverse_k − rev). 가로바-벽 거리 바꾸면 갱신.
        DeclareLaunchArgument('reverse_k', default_value='-0.001'),
        # 가로바 중심→벽 [m] (표시 d 를 벽거리로 환산).
        DeclareLaunchArgument('crossbar_to_wall', default_value='0.0425'),
        DeclareLaunchArgument('dynamic_reverse', default_value='true'),
        # STAGED 정착 대기[s] — 정지한 채 d 여러 프레임 모아 동적후진 편차↓(ddago01 검증).
        DeclareLaunchArgument('stage_settle_sec', default_value='0.6'),
        # 중심선 기동 후 재정렬 검증 거리[m] / 재계획 한도.
        DeclareLaunchArgument('cl_verify_d', default_value='0.25'),
        DeclareLaunchArgument('cl_max_replans', default_value='2'),
        # 도킹 후 후퇴[m]. 0=벽에 유지(실배포). >0=반복 테스트용(다음 마커 보려 후퇴).
        DeclareLaunchArgument('post_advance_m', default_value='0.0'),
        # 반복 시 도킹 완료 후 정지 유지[s] (post_advance_m>0 경로에서만 적용).
        DeclareLaunchArgument('post_dock_hold_sec', default_value='2.5'),
        DeclareLaunchArgument('debug', default_value='false'),
        # 웹 스트리밍(뷰 전용 MJPEG). http://<로봇ip>:<stream_port>/
        DeclareLaunchArgument('stream', default_value='false'),
        DeclareLaunchArgument('stream_port', default_value='8001'),

        Node(
            package='ddago_control',
            executable='floor_dock_server',
            name='ddago_floor_dock_server',
            output='screen',
            parameters=[{n: lc(n) for n in names}],
        ),
    ])

"""반사테이프 라이다 후진 도킹 — 검출 노드 + ReflectiveDock 액션 서버 기동.

두 노드를 함께 띄운다(charuco dock_server, 바닥 H floor_dock_server 와 별개):
  reflective_detector       /scan → /docking_marker_pose (+ /docking_markers RViz)
  reflective_dock_server    ReflectiveDock 액션 서버  /ddago/reflective_dock
                            (pose + /odom 구독 → /cmd_vel 후진 도킹)

ACS 가 goal 을 내리면 서버가 도킹을 수행한다. 로봇별 rear_offset 등 물리값은
config/reflective_dock/<robot>.yaml 에서 읽는다(파일 키 /** = 모든 노드 적용).

⚠️ 안전: bringup 에 cmd_vel 워치독이 없다. 첫 투입은 dry_run:=true 로 명령만 확인.
전제: 라이다가 Standard 모드로 /scan 을, 베이스가 /odom 을 발행 중이어야 한다.

실행 예:
  # 안전 확인(모터 미발행) — ddago01 기본
  ros2 launch ddago_control ddago_reflective_dock.launch.py dry_run:=true
  # 실주행(다른 로봇 config 지정)
  ros2 launch ddago_control ddago_reflective_dock.launch.py \\
    config_file:=$(ros2 pkg prefix ddago_control)/share/ddago_control/config/reflective_dock/ddago02.yaml

goal 예 (마커리스라 마커 파라미터 없음; stop_gap_m 0 이면 config 기본):
  ros2 action send_goal /ddago/reflective_dock automato_interfaces/action/ReflectiveDock \\
    "{task_id: 1, task_point_id: CHARGE_01, stop_gap_m: 0.0}" --feedback

디버그(액션 없이 1회성 도킹): ros2 run ddago_control reflective_dock --ros-args \\
    --params-file <robot>.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchConfiguration
    default_cfg = PathJoinSubstitution([
        FindPackageShare('ddago_control'),
        'config', 'reflective_dock', 'ddago01.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='dg_01'),
        # 로봇별 물리값(rear_offset_m, stop_gap_m). 파일 키는 /** (모든 노드).
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        # 참이면 cmd_vel 미발행(검출·계획만 확인). 첫 투입 필수.
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),

        Node(
            package='ddago_control',
            executable='reflective_detector',
            name='marker_detector',
            output='screen',
        ),
        Node(
            package='ddago_control',
            executable='reflective_dock_server',
            name='ddago_reflective_dock_server',
            output='screen',
            # config yaml(/** 로 rear_offset 등) + launch 인자(robot_id/dry_run/debug).
            # 뒤 dict 가 앞 파일보다 우선한다.
            parameters=[lc('config_file'),
                        {'robot_id': lc('robot_id'),
                         'dry_run': lc('dry_run'),
                         'debug': lc('debug')}],
        ),
    ])

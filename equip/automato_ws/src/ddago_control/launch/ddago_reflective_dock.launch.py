"""반사테이프 라이다 도킹 — 검출 노드 기동 (charuco dock_server 와 별개).

이 launch 는 상시 켜두는 '검출' 노드(reflective_detector)만 띄운다:
  /scan → /docking_marker_pose (+ /docking_markers RViz).

실제 도킹 동작(사전정렬+후진 FSM)은 1회성이라 launch 대신 ros2 run 으로 실행.
로봇별 rear_offset 은 config/reflective_dock/<robot>.yaml 로 관리(ddago01/02/03):
  ros2 run ddago_control reflective_dock --ros-args --params-file \
    $(ros2 pkg prefix ddago_control)/share/ddago_control/config/reflective_dock/ddago02.yaml

전제: 라이다가 Standard 모드로 /scan 을, 베이스가 /odom 을 발행 중이어야 한다.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ddago_control',
            executable='reflective_detector',
            name='marker_detector',
            output='screen',
        ),
    ])

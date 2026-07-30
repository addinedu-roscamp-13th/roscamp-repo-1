#!/usr/bin/env python3
"""로봇팔 PC 한 대에서 AI 서비스 + Ddagi 수확 노드를 함께 띄운다.

    ros2 launch ddagi_harvest harvest_with_ai.launch.py

로봇팔별로 그 팔을 제어하는 PC 에서 각자 AI 를 함께 돌리는 구성이다(2026-07-30 확정).
카메라는 그 PC 에 USB 직결되고, Ddagi ↔ AI 는 같은 기기 안의 ROS2 Service 호출이라
네트워크를 타지 않는다. 통신 규격은 별도 기기로 테스트했을 때와 동일하다.

    [ 로봇팔 PC ]
      detect_tomatoes_server  ──/ai/detect_tomatoes──▶  (서버)
      harvest_node            ──────────────────────▶   (클라이언트)  ──TCP 9010──▶ Pi 팔

■ 왜 launch 로 묶는가
둘 다 venv 파이썬으로 띄워야 하고(ultralytics·pyrealsense2 가 시스템 python 에 없다),
AI 는 모델 경로를 인자로 받아야 한다. 매번 두 터미널에 긴 명령을 치면 한쪽을 빠뜨리기
쉽다 — 특히 AI 를 안 띄운 채 Goal 을 보내면 'wait_for_service 실패'로 나와 원인이
팔·도메인·AI 중 어디인지 헷갈린다.

■ 카메라는 한 프로세스만 열 수 있다
AI 서비스가 카메라를 쓰므로 수확 노드는 반드시 detector:=ros 여야 한다. yolo/mock 으로
띄우면 둘이 D435 를 두고 다퉈 `Device or resource busy` 가 난다.

인자:
    python      두 노드를 실행할 파이썬 (ultralytics 가 있는 것)
    model       YOLO 가중치 .pt 경로
    arm         network | fake (fake 는 팔 없이 launch 배선만 확인할 때)
    arm_ip      Pi 의 arm_server.py 주소 (9010)
    dry_run     true 면 파지 없이 검출·마커만
    conf        YOLO 신뢰도 임계
    with_ai     false 면 AI 는 띄우지 않는다(이미 딴 데서 돌고 있을 때)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_VENV_PYTHON = "/home/cornerstone/venv/automato/bin/python3"
DEFAULT_MODEL = "/home/cornerstone/Downloads/tomato_4cls_v8.pt"


def generate_launch_description():
    py = LaunchConfiguration("python")
    args = [
        DeclareLaunchArgument("python", default_value=DEFAULT_VENV_PYTHON,
                              description="ultralytics·pyrealsense2 가 있는 파이썬"),
        DeclareLaunchArgument("model", default_value=DEFAULT_MODEL,
                              description="YOLO 가중치 .pt (저장소에 없으므로 경로 필요)"),
        DeclareLaunchArgument("arm_ip", default_value="192.168.3.12",
                              description="Pi 의 arm_server.py 주소 (9010)"),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="파지 없이 검출·마커만"),
        DeclareLaunchArgument("max_rounds", default_value="5"),
        DeclareLaunchArgument("conf", default_value="0.4"),
        DeclareLaunchArgument("with_ai", default_value="true",
                              description="false 면 AI 서비스는 띄우지 않는다"),
        DeclareLaunchArgument("arm", default_value="network",
                              description="network | fake (fake = 팔 없이 배선만 확인)"),
    ]

    ai = Node(
        package="dg_ai_service",
        executable="detect_tomatoes_server",
        name="detect_tomatoes_server",
        output="screen",
        emulate_tty=True,
        prefix=[py],                      # colcon 이 박은 셔뱅(시스템 python)을 우회
        condition=IfCondition(LaunchConfiguration("with_ai")),
        parameters=[{"model_path": LaunchConfiguration("model")}],
    )

    harvest = Node(
        package="ddagi_harvest",
        executable="harvest_node",
        name="ddagi_harvest_node",
        output="screen",
        emulate_tty=True,
        prefix=[py],
        parameters=[{
            "arm": LaunchConfiguration("arm"),
            "arm_ip": LaunchConfiguration("arm_ip"),
            "detector": "ros",            # 카메라는 AI 가 잡는다 — yolo/mock 은 충돌
            "max_rounds": LaunchConfiguration("max_rounds"),
            "dry_run": LaunchConfiguration("dry_run"),
            "conf": ParameterValue(LaunchConfiguration("conf"), value_type=float),
        }],
    )
    return LaunchDescription(args + [ai, harvest])

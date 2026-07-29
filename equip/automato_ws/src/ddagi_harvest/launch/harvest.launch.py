#!/usr/bin/env python3
"""Ddagi 수확 액션 서버 기동.

    ros2 launch ddagi_harvest harvest.launch.py
    ros2 launch ddagi_harvest harvest.launch.py detector:=yolo weights:=/경로/best.pt
    ros2 launch ddagi_harvest harvest.launch.py arm:=fake detector:=list   # 하드웨어 없이

■ venv 를 쓰는 이유
YOLO(ultralytics)가 시스템 python 에 없고 venv 에만 있다. 그런데 colcon 이 만든
실행 스크립트의 셔뱅은 **colcon 자신의 인터프리터**(/usr/bin/python3)로 박혀서,
venv 를 activate 해도 `ros2 run` 은 시스템 python 으로 뜬다(= ultralytics 없음).
그래서 Node(prefix=...) 로 venv python 을 앞에 붙여 셔뱅을 우회한다. 스크립트를
인자로 넘겨 실행하는 형태라 셔뱅이 무시되고, Node 액션이라 파라미터·리맵·
네임스페이스는 평소대로 동작한다.

venv 경로가 다르면 python:= 로 넘긴다. 시스템 python 에 ultralytics 가 있으면
python:=python3 로 두면 된다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_VENV_PYTHON = "/home/cornerstone/venv/automato/bin/python3"


def generate_launch_description():
    args = [
        DeclareLaunchArgument("python", default_value=DEFAULT_VENV_PYTHON,
                              description="노드를 실행할 파이썬 (ultralytics 가 있는 것)"),
        DeclareLaunchArgument("arm", default_value="network",
                              description="network | fake"),
        DeclareLaunchArgument("arm_ip", default_value="192.168.3.12",
                              description="Pi 의 arm_server.py 주소 (9010)"),
        DeclareLaunchArgument("detector", default_value="ros",
                              description="ros | yolo | mock | list"),
        DeclareLaunchArgument("weights", default_value="",
                              description="detector:=yolo 일 때 .pt 경로"),
        DeclareLaunchArgument("max_rounds", default_value="5",
                              description="촬영-수확 라운드 상한"),
    ]

    node = Node(
        package="ddagi_harvest",
        executable="harvest_node",
        name="ddagi_harvest_node",
        output="screen",
        emulate_tty=True,          # 로그를 줄 단위로 즉시 흘려보낸다
        prefix=[LaunchConfiguration("python")],
        parameters=[{
            "arm": LaunchConfiguration("arm"),
            "arm_ip": LaunchConfiguration("arm_ip"),
            "detector": LaunchConfiguration("detector"),
            "weights": LaunchConfiguration("weights"),
            "max_rounds": LaunchConfiguration("max_rounds"),
        }],
    )
    return LaunchDescription(args + [node])

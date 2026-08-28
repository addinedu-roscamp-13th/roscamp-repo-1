#!/usr/bin/env python3
"""RP-75  E0: DdaGo 텔레메트리 Publisher 기동 launch.

로봇별 물리망 분리 이후, 이 로봇 내부 토픽(odom, amcl_pose, /tf ...)은 드라이버·Nav2 와
함께 네임스페이스 없이 bare 로 뜬다. 그래서 이 launch 는 telemetry_publisher 를
GroupAction/PushRosNamespace 로 감싸지 않고 root(bare) 로 띄운다 → 노드의 상대 구독명
(odom, amcl_pose ...)이 bare 소스 토픽과 그대로 매칭된다.

telemetry 발행 토픽은 /ddago/telemetry 로 나간다. ddago 는 자기 정체(robot_id)를 모르며,
어느 로봇인지는 dcs(dg control service)가 수신 후 채운다. 그래서 이 launch 는 넘길
인자가 없다.

실행 예:
  ros2 launch ddago_control ddago_telemetry.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ddago_control',
            executable='telemetry_publisher',
            name='telemetry_publisher',
            output='screen',
        ),
    ])

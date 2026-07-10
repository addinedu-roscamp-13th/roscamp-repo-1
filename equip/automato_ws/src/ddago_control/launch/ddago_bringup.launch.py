#!/usr/bin/env python3
"""RP-75 부속: 핑키(pinky_pro) 드라이버를 로봇별 네임스페이스로 기동하는 wrapper.

우리는 모든 로봇을 같은 ROS_DOMAIN_ID·같은 네트워크에서 운용하므로, 드라이버가
전역 토픽(`/odom`, `/batt_state` ...)으로 발행하면 dg_01/dg_02 데이터가 섞인다.
이를 막기 위해 핑키 순정 드라이버 launch 를 **수정하지 않고** IncludeLaunchDescription
으로 그대로 불러오되, GroupAction + PushRosNamespace 로 `/<robot_id>` 접두어를 씌운다.
→ 드라이버 토픽이 `/dg_01/odom`, `/dg_01/batt_state` 처럼 로봇별로 갈라진다.

(전제) 핑키 토픽은 모두 상대 이름이라 네임스페이스 접두어가 자동 적용됨을
       pinky_pro 소스에서 확인함:
         bringup.py            odom, joint_states
         sensor_adc/main_node  us_sensor/range, ir_sensor/range, batt_state
         battery_publisher.py  battery/percent, battery/voltage (Float32)

기동 대상:
  pinky_bringup/bringup_robot.launch.xml
      - bringup           → odom, joint_states
      - battery_publisher → battery/percent, battery/voltage
      - (+ sllidar, robot_state_publisher 도 같은 네임스페이스로 딸려 옴)
  pinky_sensor_adc/main_node → us_sensor/range, batt_state
      (bringup_robot.launch.xml 에는 포함돼 있지 않아 여기서 따로 기동)

nav2(amcl_pose, navigate_to_pose 액션 상태)는 이 파일에 넣지 않는다.
pinky_navigation 의 localization/navigation launch 가 이미 namespace 인자를 지원하므로
같은 네임스페이스를 주어 별도 기동한다:
  ros2 launch pinky_navigation localization_launch.xml namespace:=/dg_01 ...
  ros2 launch pinky_navigation navigation_launch.xml   namespace:=/dg_01 ...

이 wrapper 는 핑키 드라이버(pinky_bringup, pinky_sensor_adc)가 설치된
실제 로봇(또는 로봇 PC)에서 실행한다. 드라이버가 없는 개발 PC 에서는 뜨지 않는다.

실행 예:
  ros2 launch ddago_control ddago_bringup.launch.py robot_id:=dg_01
  ros2 launch ddago_control ddago_bringup.launch.py robot_id:=dg_02   # 다른 로봇
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')

    # 핑키 순정 bringup(XML) 경로 — 설치된 pinky_bringup 패키지에서 찾는다.
    bringup_xml = PathJoinSubstitution([
        FindPackageShare('pinky_bringup'),
        'launch',
        'bringup_robot.launch.xml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='dg_01',
            description='로봇 식별자. 드라이버 토픽 앞에 붙는 네임스페이스 접두어',
        ),
        GroupAction([
            # 이 그룹 안의 모든 노드/포함 launch 에 /<robot_id> 네임스페이스 주입
            PushRosNamespace(robot_id),

            # 핑키 순정 드라이버 launch (수정 없이 그대로 포함)
            IncludeLaunchDescription(AnyLaunchDescriptionSource(bringup_xml)),

            # ADC 센서 노드: us_sensor/range·batt_state 발행.
            # bringup_robot.launch.xml 에 없으므로 따로 기동한다.
            # interface(/dev/i2c-1)·rate(20Hz)는 노드 기본값 사용.
            Node(
                package='pinky_sensor_adc',
                executable='main_node',
                name='pinky_sensor_adc',
                output='screen',
            ),
        ]),
    ])

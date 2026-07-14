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
  pinky_sensor_adc/main_node → (미기동) us_sensor/range, batt_state
      원래 여기서 따로 기동했으나, 이 ADC 노드를 함께 띄우면 배터리값이
      75→25 로 튀는 문제가 있어(같은 I2C/ADC 를 battery_publisher 와 동시에
      읽는 충돌로 추정) 배터리 텔레메트리 안정화를 위해 기동에서 제외했다.
      그 결과 us_sensor/range(초음파) 발행자가 없어지므로 telemetry_publisher
      는 us_range_m 을 항상 0.0 으로 발행한다. ADC 노드를 되살리려면 Node
      import 를 복원하고 아래 GroupAction 안에 Node(pinky_sensor_adc) 블록을
      다시 추가하면 된다.

nav2(amcl_pose, navigate_to_pose 액션 상태)는 이 파일에 넣지 않는다.
pinky_navigation 의 localization/navigation launch 가 이미 namespace 인자를 지원하므로
같은 네임스페이스를 주어 별도 기동한다:
  ros2 launch pinky_navigation localization_launch.xml namespace:=/dg_01 ...
  ros2 launch pinky_navigation navigation_launch.xml   namespace:=/dg_01 ...

이 wrapper 는 핑키 드라이버(pinky_bringup)가 설치된
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
from launch_ros.actions import PushRosNamespace, SetRemap
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
            # /tf·/tf_static 은 원래 전역 토픽이라 네임스페이스를 안 탄다. 상대명으로
            # 리맵해 /<robot_id>/tf 로 가둬야 로봇별(dg_01/dg_02) tf 가 안 섞인다.
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),

            # 핑키 순정 드라이버 launch (수정 없이 그대로 포함)
            IncludeLaunchDescription(AnyLaunchDescriptionSource(bringup_xml)),
        ]),
    ])

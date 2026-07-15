#!/usr/bin/env python3
"""로봇별 네임스페이스(/<namespace>) 데이터를 보는 RViz 뷰어.

문제:
  드라이버(ddago_bringup)와 nav2(pinky_navigation)는 모든 토픽·TF 를 /<namespace>
  아래로 발행한다(예: /dg_01/ddago/map, /dg_01/ddago/scan, /dg_01/ddago/tf). 그런데
  pinky_navigation 의 nav2_view.launch.xml 이 띄우는 RViz 는 네임스페이스가 없어
  루트(/map, /scan, /tf)만 구독한다 → 데이터가 있는 곳과 어긋나서 맵·로봇이 안 뜬다.
  (게다가 그 launch 는 namespace 인자 자체가 없어 namespace:= 를 줘도 무시된다.)

해결:
  RViz 를 같은 /<namespace> 네임스페이스 안에서 띄운다.
  - PushRosNamespace(namespace)
        RViz 노드를 /<namespace> 에서 돌린다. 그러면 rviz 설정 안의 '상대' 토픽
        (map, scan, robot_description...)이 /<namespace>/map, /<namespace>/scan 으로
        자동 해석된다.  ※ 절대경로(/map)로 박힌 토픽은 네임스페이스를 안 타므로
        ddago_view.rviz 는 토픽을 상대경로로 적어 두었다.
  - SetRemap('/tf' -> 'tf'), SetRemap('/tf_static' -> 'tf_static')
        TF 는 원래 전역 토픽(/tf)이라 네임스페이스를 안 탄다. 상대명으로 리맵해야
        RViz 의 TF 리스너가 /<namespace>/tf 를 구독한다. (ddago_bringup 과 동일한 트릭)

실행 예:
  ros2 launch ddago_control ddago_view.launch.py                          # 기본 dg_01/ddago
  ros2 launch ddago_control ddago_view.launch.py namespace:=dg_02/ddago   # 다른 로봇
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    # 이 패키지(ddago_control) 안에 함께 설치되는 rviz 설정을 찾는다.
    rviz_config = PathJoinSubstitution([
        FindPackageShare('ddago_control'),
        'rviz',
        'ddago_view.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='dg_01/ddago',
            description='RViz 가 구독할 /<namespace> (드라이버·nav2 와 동일하게 맞춘다)',
        ),
        GroupAction([
            # 이 그룹 안의 RViz 를 /<namespace> 로 밀어 넣는다.
            PushRosNamespace(namespace),
            # /tf·/tf_static 은 전역 토픽이라 네임스페이스를 안 타므로 상대명으로 리맵.
            SetRemap(src='/tf', dst='tf'),
            SetRemap(src='/tf_static', dst='tf_static'),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            ),
        ]),
    ])

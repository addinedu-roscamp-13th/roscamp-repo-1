"""
YOLO+D435 검출 -> coord_to_goal_node -> MoveIt2(Octomap 충돌회피) 전체
파이프라인을 한 번에 띄우는 launch 파일.

mycobot_280_moveit2의 demo_octomap.launch.py(MoveIt2 + Octomap + D435 +
핸드-아이 캘리브레이션 결과 발행, docs 13장)에 이 패키지의
yolo_d435_detector_node와 coord_to_goal_node를 추가로 얹음. 카메라 노드는
demo_octomap.launch.py가 하나만 띄우고, YOLO 검출은 그 노드가 발행하는
ROS2 토픽(image_raw/aligned_depth_to_color)을 그대로 구독하므로 장치
충돌 없이 Octomap 충돌회피와 검출이 동시에 동작함.

사용법:
  ros2 launch mycobot_280_pick pick_pipeline.launch.py
  ros2 launch mycobot_280_pick pick_pipeline.launch.py \
    model_path:=/path/to/model.pt
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# yolo_d435_detector_node.py의 DEFAULT_MODEL_PATH와 동일 (빈 문자열로
# 넘기면 노드의 기본값을 덮어써버려서 여기서도 같은 기본값을 씀).
DEFAULT_MODEL_PATH = os.path.expanduser('~/Projects/Eval_Yolo/tomato_4cls_model.pt')


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument('model_path', default_value=DEFAULT_MODEL_PATH)
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('mycobot_280_moveit2'), 'launch', 'demo_octomap.launch.py']
                )
            )
        )
    )

    yolo_parameters = []
    model_path = LaunchConfiguration('model_path')
    yolo_parameters.append({'model_path': model_path})

    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='yolo_d435_detector_node',
            name='yolo_d435_detector_node',
            parameters=yolo_parameters,
            # cv_bridge/ultralytics 임포트 우선순위 문제는 노드 코드 안에서
            # sys.path를 재정렬해서 해결함 (PYTHONNOUSERSITE는 ultralytics까지
            # 못 찾게 만들어서 여기선 안 씀 — 노드 파일 상단 주석 참고).
        )
    )

    ld.add_action(
        Node(
            package='mycobot_280_pick',
            executable='coord_to_goal_node',
            name='coord_to_goal_node',
        )
    )

    return ld

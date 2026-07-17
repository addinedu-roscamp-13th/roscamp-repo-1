#!/usr/bin/env python3
"""
좌표 입력(PointStamped) -> TF2 변환(g_base 기준) -> MoveIt2 목표 플래닝 노드.

로드맵 1단계: tomato_scene_test.py(scripts/)에서 검증한 pymoveit2 좌표->플래닝
로직을 TF2 변환을 포함한 정식 ROS2 노드로 정리한 것. JOINT_NAMES/BASE_LINK_NAME/
END_EFFECTOR_NAME/GROUP_NAME 값은 tomato_scene_test.py에서 이미 확인된 값을
그대로 사용함.

입력: /target_point (geometry_msgs/msg/PointStamped)
  - header.frame_id: 좌표 기준 프레임. 핸드-아이 캘리브레이션 완료 전에는
    카메라 프레임이 TF 트리에 없으므로, frame_id를 BASE_LINK_NAME("g_base")로
    맞춰 발행하면 TF2 변환 없이(identity) 바로 테스트 가능. 캘리브레이션 이후
    카메라 프레임 이름으로 그대로 바꿔 발행하면 코드 수정 없이 실제 연동됨.
  - point: 목표 좌표 (m 단위)

동작: 그리퍼가 아직 없어서, 목표 지점보다 APPROACH_OFFSET_X 만큼 로봇 쪽(x축
음의 방향)으로 당긴 위치로 팔 끝(flange)이 접근하는 것으로 대체
(tomato_scene_test.py와 동일한 접근 방식).

Octomap 클리어 로직: 목표(토마토) 자체가 실제 물체라서 D435 point cloud를
통해 Octomap에 장애물로 찍혀 있음 — 그대로 두면 목표 근처로 접근하는 자세가
항상 "충돌"로 판정되어 플래닝이 매번 실패함 (실제 end-to-end 테스트에서
"Unable to sample any valid states for goal tree"로 확인됨). 그래서 목표
좌표에 작은 구(SPHERE) CollisionObject를 등록해서, MoveIt의
PlanningSceneMonitor가 이 구와 겹치는 Octomap voxel을 자동으로 걸러내도록 함
(로봇 자기 몸 필터와 같은 메커니즘 — world 안의 collision object도 제외
대상임). 실제 Octomap이 새 point cloud로 이 필터를 반영하기까지 한 주기
걸리므로(sensors_3d.yaml의 max_update_rate=1.0Hz), 구를 등록한 뒤
OCTOMAP_CLEAR_DELAY_SEC만큼 기다렸다가 플래닝을 시작함.

주의: `MoveIt2.wait_until_executed()`와 (use_move_group_action=False일 때의)
`plan()`은 내부적으로 `rclpy.spin_once(self._node, ...)`를 호출하는데, 이 노드는
이미 자체 MultiThreadedExecutor로 이 노드를 spin 중이라 두 스핀이 충돌해서 첫
성공 실행 이후 후속 콜백이 더 이상 디스패치되지 않는 문제가 있었음. 그래서
(1) MoveIt2를 `use_move_group_action=True`로 생성해 완전히 콜백 기반인
MoveGroup 액션 경로를 쓰고(=spin_once 호출 코드 경로 자체를 안 탐),
(2) 완료 대기는 blocking 호출 대신 타이머로 `query_state()`를 폴링하는 방식으로
처리함 (실행 중인 executor 하나만 스핀 담당).
"""

from geometry_msgs.msg import Point, PointStamped, Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from pymoveit2 import MoveIt2, MoveIt2State
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_geometry_msgs import do_transform_point
import tf2_ros

# 실행 완료 여부를 폴링하는 주기 (초)
COMPLETION_POLL_PERIOD = 0.2

# ---- 로봇 설정 (tomato_scene_test.py에서 확인된 값과 동일) ----
JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]
BASE_LINK_NAME = 'g_base'
END_EFFECTOR_NAME = 'joint6_flange'
GROUP_NAME = 'arm_group'

# 목표 지점보다 이만큼 로봇 쪽(x축 음의 방향)으로 당겨서 접근 (그리퍼 없음).
# TARGET_OBJECT_RADIUS보다 커야 접근 위치가 우리가 등록한 구 자체와
# 충돌하지 않음 (아래 참고).
APPROACH_OFFSET_X = 0.08

# 목표 지점에 등록할 CollisionObject(구)의 반지름 (m). 실측 토마토 지름은
# 4cm(반지름 2cm)지만, 깊이 카메라가 측정하는 지점은 물체의 "카메라 쪽
# 표면"이라 노이즈 섞인 실제 point cloud가 그보다 카메라 쪽으로 튀어나온
# 경우가 있어(육안으로 확인함 — 구 앞쪽에 안 지워진 voxel이 남음), 여유를
# 넉넉히 둠. APPROACH_OFFSET_X보다는 작아야 접근 위치가 이 구와 안 겹침.
TARGET_OBJECT_RADIUS = 0.05
TARGET_OBJECT_ID = 'target_object'

# 구를 planning scene에 등록한 뒤, Octomap이 다음 point cloud로 자기 필터를
# 반영할 때까지 기다리는 시간 (sensors_3d.yaml의 max_update_rate=1.0Hz보다
# 살짝 길게 잡음).
OCTOMAP_CLEAR_DELAY_SEC = 1.2

# ---- 그리퍼 임시 충돌 형상 (URDF에 아직 그리퍼가 없어서, 실제 그리퍼가
# 카메라 바로 앞(~13~21cm)에서 자기 몸 필터 없이 그대로 장애물로 잡히는 문제
# 우회용) ----
# joint6_flange의 회전축(=그리퍼가 붙는 정면 방향)이 flange 로컬 Z축이라서
# (joint6output_to_joint6의 axis="0 0 1"), 그리퍼는 flange 원점에서 로컬
# Z 방향으로 뻗어있다고 가정. 원점에 대칭으로 박스를 두면 절반만 덮여서
# (실측 안 하고 대략치) 로컬 Z 방향으로 오프셋을 줘서 원점~10cm 구간을 덮음.
# 실제와 다르면(반대 방향이면) GRIPPER_BOX_Z_OFFSET 부호를 뒤집으면 됨.
GRIPPER_BOX_DIMENSIONS = [0.15, 0.15, 0.25]  # x, y(단면) / z(길이, flange 로컬 축)
GRIPPER_BOX_Z_OFFSET = GRIPPER_BOX_DIMENSIONS[2] / 2.0
GRIPPER_OBJECT_ID = 'gripper'
# 시작 직후엔 move_group의 planning_scene 구독이 아직 안 됐을 수 있어서
# (디스커버리 지연), 몇 번 반복 발행함.
GRIPPER_PUBLISH_RETRY_COUNT = 5
GRIPPER_PUBLISH_RETRY_PERIOD_SEC = 2.0


class CoordToGoalNode(Node):

    def __init__(self):
        super().__init__('coord_to_goal_node')

        callback_group = ReentrantCallbackGroup()

        self._moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK_NAME,
            end_effector_name=END_EFFECTOR_NAME,
            group_name=GROUP_NAME,
            callback_group=callback_group,
            # MoveGroup 액션(콜백 기반) 경로를 쓰기 위함. False(기본값)면
            # plan()/wait_until_executed()가 내부에서 rclpy.spin_once()를 호출해서
            # 이 노드를 이미 spin 중인 MultiThreadedExecutor와 충돌함.
            use_move_group_action=True,
        )

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # move_to_pose() 실행 중 새 목표가 들어오면 무시하기 위한 플래그.
        self._busy = False
        self._completion_timer = None
        self._clear_timer = None
        self._pending_approach_position = None

        # 목표 지점에 CollisionObject(구)를 등록해 Octomap이 그 자리 실제
        # 물체(토마토)를 자기 필터로 걸러내도록 함 (위 모듈 docstring 참고).
        self._planning_scene_publisher = self.create_publisher(
            PlanningScene, 'planning_scene', 10
        )

        # URDF에 없는 실제 그리퍼를 자기 몸 필터에 포함시키기 위한 임시
        # attached collision object 등록 (위 모듈 상수 설명 참고). 디스커버리
        # 지연으로 첫 발행이 유실될 수 있어 몇 번 반복함.
        self._gripper_publish_count = 0
        self._gripper_publish_timer = self.create_timer(
            GRIPPER_PUBLISH_RETRY_PERIOD_SEC, self._publish_gripper_collision_object
        )

        self._subscription = self.create_subscription(
            PointStamped,
            'target_point',
            self._on_target_point,
            10,
            callback_group=callback_group,
        )

        self.get_logger().info(
            'coord_to_goal_node 준비 완료. /target_point 구독 대기 중...'
        )

    def _on_target_point(self, msg: PointStamped) -> None:
        if self._busy:
            self.get_logger().warn('이전 목표 실행 중이라 새 좌표는 무시함')
            return

        try:
            if msg.header.frame_id and msg.header.frame_id != BASE_LINK_NAME:
                transform = self._tf_buffer.lookup_transform(
                    BASE_LINK_NAME,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=1.0),
                )
                point_in_base = do_transform_point(msg, transform)
            else:
                point_in_base = msg
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().error(
                f'TF2 변환 실패 ({msg.header.frame_id} -> {BASE_LINK_NAME}): {exc}'
            )
            return

        target = point_in_base.point
        approach_position = [target.x - APPROACH_OFFSET_X, target.y, target.z]

        self._busy = True
        self._publish_target_collision_object(target)
        self._pending_approach_position = approach_position
        self.get_logger().info(
            f'목표 지점({target.x:.3f}, {target.y:.3f}, {target.z:.3f})에 '
            f'Octomap 클리어용 구 등록, {OCTOMAP_CLEAR_DELAY_SEC}초 대기 후 플래닝 시작'
        )
        # Octomap이 다음 point cloud로 자기 필터 갱신을 반영할 때까지 기다린
        # 뒤에 플래닝을 시작함 (바로 플래닝하면 아직 예전 voxel이 남아있어서
        # 여전히 충돌로 판정될 수 있음).
        self._clear_timer = self.create_timer(
            OCTOMAP_CLEAR_DELAY_SEC, self._start_planning
        )

    def _publish_gripper_collision_object(self) -> None:
        self._gripper_publish_count += 1
        if self._gripper_publish_count >= GRIPPER_PUBLISH_RETRY_COUNT:
            self._gripper_publish_timer.cancel()
            self._gripper_publish_timer = None

        collision_object = CollisionObject()
        collision_object.header.frame_id = END_EFFECTOR_NAME
        collision_object.id = GRIPPER_OBJECT_ID
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = GRIPPER_BOX_DIMENSIONS
        collision_object.primitives = [primitive]

        pose = Pose()
        pose.position.z = GRIPPER_BOX_Z_OFFSET
        pose.orientation.w = 1.0
        collision_object.primitive_poses = [pose]

        attached_object = AttachedCollisionObject()
        attached_object.link_name = END_EFFECTOR_NAME
        attached_object.object = collision_object
        # 팔 자신이 이 형상에 닿아도 충돌로 안 잡히게 함.
        attached_object.touch_links = [END_EFFECTOR_NAME, 'joint6']

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached_object]
        self._planning_scene_publisher.publish(scene)
        self.get_logger().info(
            f'그리퍼 임시 충돌 형상 등록 ({self._gripper_publish_count}/'
            f'{GRIPPER_PUBLISH_RETRY_COUNT})'
        )

    def _publish_target_collision_object(self, target: Point) -> None:
        collision_object = CollisionObject()
        collision_object.header.frame_id = BASE_LINK_NAME
        collision_object.id = TARGET_OBJECT_ID
        collision_object.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [TARGET_OBJECT_RADIUS]
        collision_object.primitives = [primitive]

        pose = Pose()
        pose.position = target
        pose.orientation.w = 1.0
        collision_object.primitive_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [collision_object]
        self._planning_scene_publisher.publish(scene)

    def _start_planning(self) -> None:
        self._clear_timer.cancel()
        self._clear_timer = None

        approach_position = self._pending_approach_position
        approach_quat = [0.0, 0.0, 0.0, 1.0]  # TODO: 접근 방향에 맞는 orientation 필요

        self.get_logger().info(f'목표 위치로 플래닝: {approach_position}')
        self._moveit2.move_to_pose(
            position=approach_position,
            quat_xyzw=approach_quat,
            cartesian=False,
        )
        # wait_until_executed()는 내부적으로 rclpy.spin_once()를 호출해서 이미
        # 돌고 있는 MultiThreadedExecutor와 충돌하므로 쓰지 않음. 대신 같은
        # executor가 처리하는 타이머로 완료 여부만 폴링함.
        self._completion_timer = self.create_timer(
            COMPLETION_POLL_PERIOD, self._check_motion_complete
        )

    def _check_motion_complete(self) -> None:
        if self._moveit2.query_state() != MoveIt2State.IDLE:
            return

        self._completion_timer.cancel()
        self._completion_timer = None

        if self._moveit2.motion_suceeded:
            self.get_logger().info('플래닝/실행 완료.')
        else:
            self.get_logger().warn('플래닝/실행 실패.')

        self._busy = False


def main():
    rclpy.init()

    node = CoordToGoalNode()

    # 구독 콜백, MoveIt2 액션 응답, 완료 폴링 타이머가 동시에 처리될 수 있어야
    # 하므로 스레드 여러 개 필요.
    executor = MultiThreadedExecutor(4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

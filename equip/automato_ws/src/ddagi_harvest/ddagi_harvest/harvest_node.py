#!/usr/bin/env python3
"""시나리오2 : Ddagi 수확 액션 서버 (/ddagi/harvest).

DG Control Service 가 Harvest Goal(task_id, max_capacity)을 보내면, 관측·검출요청·
제외목록·라운드·파지를 **이 노드가 전부 주관**하고 진행 상황을 Feedback 으로 올린다.
목표 좌표는 Goal 에 없다 — 2026-07-23 구조 변경으로 수확 루프가 DG 에서 Ddagi 로
이관됐기 때문이다.

    Goal ──> [관측 → AI 검출요청 → 제외필터 → 거리순 파지 → 바구니] × 라운드 ──> Result

수확 로직 자체는 harvest.harvest() 그대로다. 이 노드는 ROS 껍데기만 담당한다:
Goal 파싱 · Feedback 발행 · 취소 처리 · Result 매핑.

실행:
    ros2 run ddagi_harvest harvest_node
    ros2 run ddagi_harvest harvest_node --ros-args -p detector:=yolo -p weights:=/경로/best.pt

파라미터:
    arm         network | fake      (기본 network. fake = 팔 없이 명령 로깅만)
    arm_ip      로봇팔 브리지 IP (기본 192.168.3.12, arm_server.py 가 9010 대기)
    detector    ros | yolo | mock | list  (기본 ros = /ai/detect_tomatoes 호출)
    weights     detector=yolo 일 때 .pt 경로
    max_rounds  촬영-수확 라운드 상한 (기본 5)

하드웨어 없이 전 경로(Goal→Feedback→Result) 검증:
    ros2 run ddagi_harvest harvest_node --ros-args -p arm:=fake -p detector:=list
    ros2 action send_goal /ddagi/harvest automato_interfaces/action/Harvest \\
        "{task_id: 1, max_capacity: 7}" --feedback
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_interfaces.action import Harvest
from ddagi_harvest import harvest as hv
from ddagi_harvest import pick as pk
from ddagi_harvest.arm_backend import FakeArm, NetworkArm
from ddagi_harvest.detector import (ListDetector, MockColorDetector, RosDetector,
                                    YoloDetector)

ACTION_NAME = "/ddagi/harvest"      # 로봇 세트 내부라 {robot_id} 네임스페이스 없음

# detector=list 용 더미 배치 — 실측 로그에서 뽑은 작업공간 안쪽 좌표.
# 팔·카메라 없이 액션 껍데기(Goal·Feedback·취소·Result)를 검증하는 용도다.
_FAKE_BATCH = [
    {"base": [206.0, 34.0, 300.0], "grade": "NORMAL", "color": "ripe"},
    {"base": [219.0, 96.0, 288.0], "grade": "NORMAL", "color": "ripe"},
    {"base": [231.0, -12.0, 312.0], "grade": "DISCARD", "color": "rotten"},
]


class HarvestActionServer(Node):
    def __init__(self):
        super().__init__("ddagi_harvest_node")
        self.declare_parameter("arm", "network")
        self.declare_parameter("arm_ip", "192.168.3.12")
        self.declare_parameter("detector", "ros")
        self.declare_parameter("weights", "")
        self.declare_parameter("max_rounds", hv.MAX_ROUNDS)

        self._arm = None
        self._detector = None
        self._busy = threading.Lock()   # 팔은 1대 — Goal 동시 실행을 막는다

        # 액션 실행 콜백 안에서 AI 서비스 응답을 동기 대기하므로 둘 다 Reentrant 여야
        # 한다. 단일 스레드 실행기면 실행 콜백이 실행기를 점유해 응답이 영영 안 온다.
        self._server = ActionServer(
            self, Harvest, ACTION_NAME,
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(f"Ddagi 수확 액션 서버 시작: {ACTION_NAME}")

    # ---- Goal 수락/취소 ------------------------------------------------- #

    def on_goal(self, goal_request):
        if self._busy.locked():
            self.get_logger().warning("수확이 이미 진행 중 — Goal 거절")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        # 실제 중단은 파지 1건이 끝난 경계에서 일어난다(harvest.should_cancel).
        self.get_logger().info("취소 요청 접수 — 현재 파지를 마치고 중단")
        return CancelResponse.ACCEPT

    # ---- 하드웨어 준비 --------------------------------------------------- #

    def _ensure_arm(self):
        if self._arm is None:
            if self.get_parameter("arm").value == "fake":
                self.get_logger().warning("팔 = FakeArm (명령 로깅만, 실제 이동 없음)")
                # 18 = 열매가 걸려 완전히 안 닫힌 값(파지 성공 시뮬). 임계 6 초과.
                self._arm = FakeArm(start_angles=pk.OBSERVE_ANGLES,
                                    grip_result=18, verbose=False)
            else:
                ip = self.get_parameter("arm_ip").value
                self.get_logger().info(f"로봇팔 연결 {ip}:9010")
                self._arm = NetworkArm(ip)
        return self._arm

    def _make_detector(self, task_id: int):
        """Goal 마다 새로 만든다 — task_id·라운드 카운터가 Goal 에 매인다."""
        kind = self.get_parameter("detector").value
        arm = self._ensure_arm()
        if kind == "list":
            # 매 라운드 같은 목록을 준다 → 파지 성공분이 안 사라져 만차(FULL)로 끝난다.
            # 라운드·Feedback·만차 판정 경로를 보는 용도.
            return ListDetector(_FAKE_BATCH)
        if kind == "ros":
            return RosDetector(self, task_id=task_id,
                               angles_provider=arm.get_angles)
        if kind == "yolo":
            weights = self.get_parameter("weights").value
            if not weights:
                raise RuntimeError("detector=yolo 인데 weights 파라미터가 비었다")
            return YoloDetector(weights, angles_provider=arm.get_angles)
        return MockColorDetector(angles_provider=arm.get_angles)

    # ---- 실행 ------------------------------------------------------------ #

    def execute(self, goal_handle):
        req = goal_handle.request
        max_capacity = req.max_capacity or hv.MAX_CAPACITY
        result = Harvest.Result()

        if not self._busy.acquire(blocking=False):
            goal_handle.abort()
            result.exit_reason = "BUSY"
            result.message = "다른 수확이 진행 중"
            return result

        self.get_logger().info(
            f"수확 시작 task_id={req.task_id} max_capacity={max_capacity}")
        detector = None
        try:
            arm = self._ensure_arm()
            detector = self._make_detector(req.task_id)

            def on_progress(round_no, normal, discard, failed, remaining):
                fb = Harvest.Feedback()
                fb.round = int(round_no)
                fb.normal_count = int(normal)
                fb.discard_count = int(discard)
                fb.failed_count = int(failed)
                fb.remaining_in_round = int(remaining)
                goal_handle.publish_feedback(fb)

            summary = hv.harvest(
                arm, detector,
                max_capacity=max_capacity,
                max_rounds=self.get_parameter("max_rounds").value,
                on_progress=on_progress,
                should_cancel=lambda: goal_handle.is_cancel_requested,
            )
        except Exception as exc:                       # 팔·검출 예외 → abort
            self.get_logger().error(f"수확 실패: {exc}")
            goal_handle.abort()
            result.exit_reason = "ERROR"
            result.message = str(exc)
            return result
        finally:
            if detector is not None:
                detector.close()
            self._busy.release()

        result.normal_count = int(summary["normal_count"])
        result.discard_count = int(summary["discard_count"])
        result.failed_count = int(summary["failed_count"])
        result.exit_reason = summary["exit_reason"]
        result.message = (
            f"수확품 {result.normal_count} / 폐기품 {result.discard_count} / "
            f"실패 {result.failed_count} ({summary['rounds']}라운드)")

        if summary["exit_reason"] == "CANCELED":
            goal_handle.canceled()
        else:
            goal_handle.succeed()
        self.get_logger().info(f"수확 종료 [{result.exit_reason}] {result.message}")
        return result

    def destroy_node(self):
        if self._arm is not None:
            self._arm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HarvestActionServer()
    # 실행 콜백이 길게(수 분) 블로킹하고 그 안에서 서비스를 동기 호출하므로 필수.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

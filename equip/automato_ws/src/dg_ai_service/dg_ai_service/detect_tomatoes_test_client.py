#!/usr/bin/env python3
"""RP-127  수동 검증용 — Ddagi 흉내를 내는 DetectTomatoes 테스트 클라이언트.

Ddagi 쪽 실제 소비자(harvest 루프)가 아직 없는 동안, AI Service의
/ai/detect_tomatoes 서버만 독립적으로 검증하기 위한 용도. detect_tomatoes_server가
먼저 떠 있어야 한다(별도 터미널: `ros2 run dg_ai_service detect_tomatoes_server`).

Ddagi가 실제로 라운드를 돌리는 것과 같은 패턴 — 같은 task_id로 round를
1부터 증가시키며 반복 호출 — 으로 요청을 보내고 응답을 그대로 출력한다.

사용법:
  ros2 run dg_ai_service detect_tomatoes_test_client --task-id 1 --rounds 3
"""
import argparse

import rclpy
from automato_interfaces.srv import DetectTomatoes
from rclpy.node import Node

DEFAULT_SERVICE_NAME = '/ai/detect_tomatoes'


def parse_args():
    parser = argparse.ArgumentParser(description='DetectTomatoes 수동 테스트 클라이언트')
    parser.add_argument('--task-id', type=int, default=1)
    parser.add_argument('--rounds', type=int, default=3, help='호출 횟수(Ddagi 라운드 루프 흉내)')
    parser.add_argument('--service', default=DEFAULT_SERVICE_NAME)
    parser.add_argument('--timeout', type=float, default=15.0, help='호출 1건당 응답 대기 시간(초)')
    return parser.parse_args()


def call_once(node: Node, client, task_id: int, round_: int, timeout: float):
    req = DetectTomatoes.Request()
    req.task_id = task_id
    req.round = round_
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        node.get_logger().error(f'[round {round_}] 응답 타임아웃({timeout}s)')
        return None
    return future.result()


def main(args=None):
    rclpy.init(args=args)
    opts = parse_args()
    node = Node('detect_tomatoes_test_client')
    client = node.create_client(DetectTomatoes, opts.service)

    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error(
            f'{opts.service} 서비스를 못 찾음 — detect_tomatoes_server가 떠 있는지 확인'
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    for round_ in range(1, opts.rounds + 1):
        node.get_logger().info(f'--- round {round_} 요청 (task_id={opts.task_id}) ---')
        result = call_once(node, client, opts.task_id, round_, opts.timeout)
        if result is None:
            continue
        if not result.success:
            node.get_logger().warn(
                f'[round {round_}] 실패: error_code={result.error_code} message={result.message}'
            )
            continue
        node.get_logger().info(
            f'[round {round_}] 성공: frame_id={result.frame_id} {len(result.tomatoes)}개 검출'
        )
        for t in result.tomatoes:
            node.get_logger().info(
                f'    id={t.tomato_id} grade={t.grade} '
                f'x={t.x:.3f} y={t.y:.3f} z={t.z:.3f}'
            )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

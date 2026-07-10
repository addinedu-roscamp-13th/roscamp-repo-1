#!/usr/bin/env python3
"""시나리오2 : Ddagi(로봇팔) 수확 액션 서버.

dg_control이 초기 스캔 결과(HarvestTarget[])를 goal로 보내면, order 순으로
순회하며 arm_controller.pick_tomato()를 호출하고 진행 상황을 feedback으로
발행한다.

토픽: /{robot_id}/ddagi/harvest (액션)
메시지: automato_interfaces/action/DdagiHarvest
"""
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from automato_interfaces.action import DdagiHarvest
from ddagi_control.arm_controller import pick_tomato


class HarvestServer(Node):
    def __init__(self):
        super().__init__('ddagi_harvest_server')
        self.declare_parameter('robot_id', 'dg_01')
        robot_id = self.get_parameter('robot_id').value

        self._action_server = ActionServer(
            self,
            DdagiHarvest,
            f'/{robot_id}/ddagi/harvest',
            execute_callback=self.execute_callback,
        )
        self.get_logger().info(f'Ddagi 수확 액션 서버 시작: /{robot_id}/ddagi/harvest')

    def execute_callback(self, goal_handle):
        targets = sorted(goal_handle.request.targets, key=lambda t: t.order)
        total = len(targets)
        harvested_count = 0

        for target in targets:
            try:
                ok = pick_tomato((target.x, target.y, target.z), target.use_moveit)
            except NotImplementedError as exc:
                self.get_logger().error(f'target order={target.order}: {exc}')
                ok = False

            if ok:
                harvested_count += 1
            else:
                self.get_logger().warning(f'target order={target.order} 수확 실패')

            feedback = DdagiHarvest.Feedback()
            feedback.current_order = target.order
            feedback.harvested_count = harvested_count
            feedback.total_count = total
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result = DdagiHarvest.Result()
        result.result_code = 0 if harvested_count == total else 1
        result.message = f'{harvested_count}/{total} 수확 완료'
        result.harvested_count = harvested_count
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HarvestServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

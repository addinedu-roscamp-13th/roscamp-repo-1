"""개발/시연용 모의 ACS (Automato Control Service).

제어탭에서 보내는 RobotMaintenanceCommand 서비스를 받아 로그만 남기고 수락 응답한다.
실제 QT -> ACS -> HQ 경로 중 ACS 역할을 흉내내어, 로봇 없이 제어탭 동작을 시연한다.
"""
from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node

try:
    from automato_interfaces.srv import RobotMaintenanceCommand
except (ImportError, ModuleNotFoundError):
    sys.stderr.write(
        "RobotMaintenanceCommand 인터페이스가 없습니다.\n"
        "제어탭 시연을 하려면 proposals/RobotMaintenanceCommand.srv를 automato_interfaces에\n"
        "추가·빌드해야 합니다 (README '제어탭 제안 인터페이스' 참고).\n"
    )
    raise SystemExit(1)

from .. import config


class MockACS(Node):
    def __init__(self):
        super().__init__("automato_mock_acs")
        self.srv = self.create_service(
            RobotMaintenanceCommand,
            config.SERVICE_MAINTENANCE,
            self._handle,
        )
        self.get_logger().info(
            f"모의 ACS 서비스 대기: {config.SERVICE_MAINTENANCE}"
        )

    def _handle(self, req, resp):
        detail = ""
        if req.command == "TELEOP":
            detail = f" (linear_x={req.linear_x:.2f}, angular_z={req.angular_z:.2f})"
        self.get_logger().info(
            f"[명령 수신] robot={req.robot_id} command={req.command}{detail}"
        )
        resp.accepted = True
        resp.status = "ACCEPTED"
        resp.message = f"{req.robot_id} '{req.command}' 명령을 HQ로 전달(모의)"
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = MockACS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

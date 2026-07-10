"""ROS2 실행 스레드 ↔ Qt GUI 스레드 브리지.

핵심 규칙: rclpy spin은 이 QThread에서 돌고, 위젯은 절대 직접 건드리지 않는다.
데이터는 pyqtSignal(큐 연결)로만 GUI 스레드에 전달한다.
"""
from __future__ import annotations

import queue

import rclpy
from rclpy.executors import SingleThreadedExecutor
from PyQt6.QtCore import QThread, pyqtSignal

from ..model.state import FleetSnapshot
from .telemetry_node import TelemetryNode


class RosWorker(QThread):
    # ROS 스레드 → GUI 스레드
    fleet_snapshot = pyqtSignal(object)                 # FleetSnapshot
    maintenance_result = pyqtSignal(str, str, bool, str)  # robot_id, command, ok, message

    def __init__(self, parent=None):
        super().__init__(parent)
        self._stop = False
        self._node: TelemetryNode | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._cmd_queue: "queue.Queue[tuple]" = queue.Queue()

    # ---- GUI 스레드에서 호출 ----
    def request_maintenance(self, robot_id: str, command: str,
                            linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        """제어탭 버튼에서 호출. 실제 서비스 호출은 ROS 스레드에서 처리한다."""
        self._cmd_queue.put((robot_id, command, linear_x, angular_z))

    def stop(self) -> None:
        self._stop = True

    # ---- ROS 스레드 ----
    def run(self) -> None:
        rclpy.init()
        self._node = TelemetryNode(on_snapshot=self._emit_snapshot)
        # 명령 큐를 주기적으로 비우는 타이머 (ROS 스레드 컨텍스트에서 안전하게 서비스 호출)
        self._node.create_timer(0.05, self._drain_commands)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        try:
            while not self._stop and rclpy.ok():
                self._executor.spin_once(timeout_sec=0.1)
        finally:
            self._executor.remove_node(self._node)
            self._node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    # ---- 내부 ----
    def _emit_snapshot(self, snap: FleetSnapshot) -> None:
        self.fleet_snapshot.emit(snap)

    def _drain_commands(self) -> None:
        while True:
            try:
                robot_id, command, lin, ang = self._cmd_queue.get_nowait()
            except queue.Empty:
                return
            future = self._node.send_maintenance(robot_id, command, lin, ang)
            if future is None:
                self.maintenance_result.emit(
                    robot_id, command, False,
                    "ACS 유지보수 서비스에 연결 안 됨 (mock_acs 미실행?)",
                )
                continue

            def _done(fut, rid=robot_id, cmd=command):
                try:
                    resp = fut.result()
                    self.maintenance_result.emit(
                        rid, cmd, bool(resp.accepted),
                        f"[{resp.status}] {resp.message}",
                    )
                except Exception as exc:
                    self.maintenance_result.emit(rid, cmd, False, f"오류: {exc}")

            future.add_done_callback(_done)

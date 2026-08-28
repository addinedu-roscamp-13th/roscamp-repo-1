"""로봇팔 서보 7개 상세 표 (진단용 원본 데이터)."""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import QTableWidget, QTableWidgetItem, QHeaderView

from .. import config
from ..model.state import DdagiState

HEADERS = ["관절", "전압", "온도(℃)", "전류(A)", "과부하", "그리퍼(%)"]


class ServoTable(QTableWidget):
    def __init__(self, parent=None):
        super().__init__(config.SERVO_COUNT, len(HEADERS), parent)
        self.setHorizontalHeaderLabels(HEADERS)
        self.verticalHeader().setVisible(False)
        self.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch
        )
        self.setFixedHeight(24 * (config.SERVO_COUNT + 1) + 8)

    def update_servos(self, arm: DdagiState | None) -> None:
        servos = arm.servos if arm else []
        for row in range(config.SERVO_COUNT):
            s = servos[row] if row < len(servos) else None
            is_gripper = s is not None and s.joint_no == config.GRIPPER_JOINT_NO
            joint_label = "그리퍼" if is_gripper else (
                f"J{s.joint_no}" if s else f"J{row + 1}"
            )
            values = [
                joint_label,
                ("정상" if s.voltage_ok else "이상") if s else "-",
                str(s.temperature) if s else "-",
                f"{s.current:.2f}" if s else "-",
                ("있음" if s.overload else "없음") if s else "-",
                (str(s.gripper_value) if is_gripper else "-") if s else "-",
            ]
            for col, val in enumerate(values):
                item = QTableWidgetItem(val)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                if s is not None:
                    if col == 2 and s.temperature >= config.SERVO_TEMP_CRIT_C:
                        item.setForeground(QColor("#c62828"))
                    if col == 4 and s.overload:
                        item.setForeground(QColor("#c62828"))
                    if col == 1 and not s.voltage_ok:
                        item.setForeground(QColor("#c62828"))
                self.setItem(row, col, item)

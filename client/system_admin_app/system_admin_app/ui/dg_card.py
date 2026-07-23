"""DG 단위 카드. 주행(Ddago) 섹션 + 로봇팔(Ddagi) 섹션을 한 카드에 분리 표시."""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QFrame, QLabel, QVBoxLayout, QHBoxLayout, QGridLayout, QProgressBar,
    QPushButton, QWidget,
)

from .. import config
from ..model.state import DGUnit
from ..model import thresholds
from .servo_table import ServoTable
from . import style


def _tile(title: str, unsupported: bool = False) -> tuple[QFrame, QLabel]:
    """작은 상태 타일. (프레임, 값라벨) 반환."""
    f = QFrame()
    f.setProperty("class", "TileUnsupported" if unsupported else "Tile")
    lay = QVBoxLayout(f)
    lay.setContentsMargins(8, 6, 8, 6)
    lay.setSpacing(1)
    value = QLabel("미지원" if unsupported else "-")
    value.setAlignment(Qt.AlignmentFlag.AlignCenter)
    value.setStyleSheet(
        "font-size:15px; font-weight:700;" + ("color:#9e9e9e;" if unsupported else "")
    )
    cap = QLabel(title)
    cap.setAlignment(Qt.AlignmentFlag.AlignCenter)
    cap.setStyleSheet("color:#7a887a; font-size:11px;")
    lay.addWidget(value)
    lay.addWidget(cap)
    return f, value


class DGCard(QFrame):
    def __init__(self, robot_id: str, parent=None):
        super().__init__(parent)
        self.robot_id = robot_id
        self.setObjectName("DGCard")
        self.setMinimumWidth(260)

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)

        # --- 헤더: 제목 + 종합 상태 뱃지 ---
        head = QHBoxLayout()
        title = QLabel(robot_id.upper())
        title.setObjectName("CardTitle")
        self.badge = QLabel("정상")
        self.badge.setStyleSheet(style.badge_qss("idle"))
        head.addWidget(title)
        head.addStretch(1)
        head.addWidget(self.badge)
        root.addLayout(head)

        # ===== 주행(Ddago) 섹션 =====
        root.addWidget(self._section_label("주행 (Ddago)"))
        self.nav_badge = QLabel("-")
        self.nav_badge.setStyleSheet(style.badge_qss("idle"))
        navrow = QHBoxLayout()
        navrow.addWidget(QLabel("주행 상태"))
        navrow.addStretch(1)
        navrow.addWidget(self.nav_badge)
        root.addLayout(navrow)

        # 배터리 바
        self.batt_bar = QProgressBar()
        self.batt_bar.setRange(0, 100)
        self.batt_bar.setTextVisible(True)
        self.batt_bar.setFormat("배터리 %p%")
        root.addWidget(self.batt_bar)
        self.batt_detail = QLabel("- V")
        self.batt_detail.setStyleSheet("color:#7a887a; font-size:11px;")
        root.addWidget(self.batt_detail)

        # 주행 타일: 위치 + (미지원) 라이다/IMU/모터온도
        # 초음파(us_range)는 실제로 데이터가 오지 않아 모니터링에서 제외했다.
        drive_grid = QGridLayout()
        drive_grid.setSpacing(6)
        self.tile_pos, self.val_pos = _tile("위치 x,y")
        self.tile_yaw, self.val_yaw = _tile("방향 yaw")
        self.tile_lidar, _ = _tile("라이다", unsupported=True)
        self.tile_imu, _ = _tile("IMU", unsupported=True)
        for col, w in enumerate(
            [self.tile_pos, self.tile_yaw]
        ):
            drive_grid.addWidget(w, 0, col)
        # 미지원(추후 협의) 타일 — 주행모터온도는 제거됨
        for col, w in enumerate([self.tile_lidar, self.tile_imu]):
            drive_grid.addWidget(w, 1, col)
        root.addLayout(drive_grid)

        # ===== 로봇팔(Ddagi) 섹션 =====
        root.addWidget(self._section_label("로봇팔 (Ddagi)"))
        self.arm_body = QWidget()
        arm_lay = QVBoxLayout(self.arm_body)
        arm_lay.setContentsMargins(0, 0, 0, 0)
        arm_lay.setSpacing(6)

        arm_grid = QGridLayout()
        arm_grid.setSpacing(6)
        self.tile_pause, self.val_pause = _tile("동작")
        self.tile_temp, self.val_temp = _tile("서보 최고온도")
        self.tile_over, self.val_over = _tile("과부하")
        self.tile_grip, self.val_grip = _tile("그리퍼")
        for col, w in enumerate(
            [self.tile_pause, self.tile_temp, self.tile_over, self.tile_grip]
        ):
            arm_grid.addWidget(w, 0, col)
        arm_lay.addLayout(arm_grid)

        # 관절 상세 펼치기
        self.toggle_btn = QPushButton("관절 상세 ▼")
        self.toggle_btn.setCheckable(True)
        self.toggle_btn.toggled.connect(self._on_toggle)
        arm_lay.addWidget(self.toggle_btn)
        self.servo_table = ServoTable()
        self.servo_table.setVisible(False)
        arm_lay.addWidget(self.servo_table)

        root.addWidget(self.arm_body)

        # 주행 전용 안내 (로봇팔 없음)
        self.arm_none = QLabel("로봇팔 없음 (주행 전용)")
        self.arm_none.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.arm_none.setStyleSheet("color:#9e9e9e; padding:8px;")
        self.arm_none.setVisible(False)
        root.addWidget(self.arm_none)

        # ===== 경고 패널 =====
        self.warn = QLabel("")
        self.warn.setObjectName("WarnPanel")
        self.warn.setWordWrap(True)
        self.warn.setVisible(False)
        root.addWidget(self.warn)

        root.addStretch(1)

    # ---- helpers ----
    def _section_label(self, text: str) -> QLabel:
        lb = QLabel(text)
        lb.setObjectName("SectionLabel")
        return lb

    def _on_toggle(self, checked: bool) -> None:
        self.servo_table.setVisible(checked)
        self.toggle_btn.setText("관절 상세 ▲" if checked else "관절 상세 ▼")

    # ---- 갱신 ----
    def update_unit(self, unit: DGUnit | None, online: bool) -> None:
        if unit is None or not online:
            self.badge.setText("미수신")
            self.badge.setStyleSheet(style.badge_qss("idle"))

        # 종합 상태
        if unit is not None and online:
            level, reasons = thresholds.unit_status(unit)
            self.badge.setText(thresholds.LEVEL_LABEL[level])
            self.badge.setStyleSheet(style.badge_qss(level))
            self.warn.setVisible(bool(reasons))
            if reasons:
                self.warn.setText("⚠ " + " · ".join(reasons))
        else:
            self.warn.setVisible(False)

        self._update_drive(unit.ddago if unit else None, online)
        self._update_arm(unit)

    def _update_drive(self, d, online: bool) -> None:
        if d is None or not online:
            self.nav_badge.setText("미수신")
            self.nav_badge.setStyleSheet(style.badge_qss("idle"))
            self.batt_bar.setValue(0)
            self.batt_detail.setText("- V")
            for lbl in (self.val_pos, self.val_yaw):
                lbl.setText("-")
            return
        meta = config.nav_status_meta(d.nav_status)
        self.nav_badge.setText(meta["label"])
        self.nav_badge.setStyleSheet(style.badge_qss(
            {"ok": "ok", "busy": "ok", "warn": "warn", "crit": "crit"}[meta["level"]]
        ))
        self.batt_bar.setValue(int(d.battery_percent))
        self.batt_detail.setText(f"{d.battery_voltage:.2f} V")
        self.val_pos.setText(f"{d.x:.2f}, {d.y:.2f}")
        self.val_yaw.setText(f"{d.yaw:.2f}")

    def _update_arm(self, unit) -> None:
        drive_only = unit.is_drive_only if unit else (
            self.robot_id in config.DRIVE_ONLY_ROBOTS
        )
        if drive_only:
            self.arm_body.setVisible(False)
            self.arm_none.setVisible(True)
            return
        self.arm_body.setVisible(True)
        self.arm_none.setVisible(False)

        a = unit.ddagi if unit else None
        if a is None:
            for lbl in (self.val_pause, self.val_temp, self.val_over, self.val_grip):
                lbl.setText("-")
            self.servo_table.update_servos(None)
            return
        self.val_pause.setText("일시정지" if a.is_paused else "동작 중")
        mt = a.max_temperature
        self.val_temp.setText(f"{mt}℃" if mt is not None else "-")
        self.val_temp.setStyleSheet(
            "font-size:15px; font-weight:700;"
            + ("color:#c62828;" if mt is not None and mt >= config.SERVO_TEMP_CRIT_C else "")
        )
        self.val_over.setText("있음" if a.any_overload else "없음")
        self.val_over.setStyleSheet(
            "font-size:15px; font-weight:700;"
            + ("color:#c62828;" if a.any_overload else "")
        )
        gp = a.gripper_percent
        self.val_grip.setText(f"{gp}%" if gp is not None else "-")
        self.servo_table.update_servos(a)

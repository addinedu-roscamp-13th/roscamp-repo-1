"""탭2: 로봇 제어(정비). [예시/mock] QT -> ACS -> HQ 유지보수 명령.

아직 팀 합의 전 초안이라, 실제 동작은 mock_acs가 수신해 로그로 확인한다.
주행 로봇 모니터링·유지보수에 초점(로봇팔은 안전상 조작 제외, 모니터링 탭에서만).
"""
from __future__ import annotations

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox, QLabel,
    QPushButton, QComboBox, QDoubleSpinBox, QPlainTextEdit, QFrame,
)

from .. import config
from ..ros.ros_worker import RosWorker
from ..model.state import FleetSnapshot
from .fleet_map import FleetMap


class TeleopPad(QFrame):
    """키보드 teleop 캡처 영역. 클릭(포커스)하면 키 입력을 받는다.

    W/↑ 전진 · S/↓ 후진 · A/← 좌회전 · D/→ 우회전 · Space 정지.
    누르면 이동, 떼면 정지(키다운→이동, 키업→정지). 스핀박스/버튼과 포커스가
    분리되도록 전용 위젯으로 둔다.
    """

    KEYMAP = {
        Qt.Key.Key_W: "fwd", Qt.Key.Key_Up: "fwd",
        Qt.Key.Key_S: "back", Qt.Key.Key_Down: "back",
        Qt.Key.Key_A: "left", Qt.Key.Key_Left: "left",
        Qt.Key.Key_D: "right", Qt.Key.Key_Right: "right",
    }

    def __init__(self, on_move, on_stop, parent=None):
        super().__init__(parent)
        self._on_move = on_move   # callable(kind: str)
        self._on_stop = on_stop   # callable()
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)
        self.setMinimumHeight(84)
        lay = QVBoxLayout(self)
        self._label = QLabel()
        self._label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._label.setWordWrap(True)
        lay.addWidget(self._label)
        self._render(False)

    def _render(self, focused: bool) -> None:
        if focused:
            self.setStyleSheet(
                "QFrame{background:#e8f5e9;border:2px solid #2e7d32;border-radius:8px;}"
            )
            self._label.setText(
                "⌨ 키보드 조작 활성 — W/S 전·후, A/D 좌·우회전, Space 정지 (뗄 때 정지)"
            )
        else:
            self.setStyleSheet(
                "QFrame{background:#f2f2f2;border:2px dashed #bdbdbd;border-radius:8px;}"
            )
            self._label.setText(
                "여기를 클릭하면 키보드 조작 활성화 — W/A/S/D 또는 방향키, Space 정지"
            )

    def focusInEvent(self, e):
        self._render(True)
        super().focusInEvent(e)

    def focusOutEvent(self, e):
        self._render(False)
        self._on_stop()  # 포커스 잃으면 안전하게 정지
        super().focusOutEvent(e)

    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        if e.key() == Qt.Key.Key_Space:
            self._on_stop()
            return
        kind = self.KEYMAP.get(e.key())
        if kind:
            self._on_move(kind)
        else:
            super().keyPressEvent(e)

    def keyReleaseEvent(self, e):
        if e.isAutoRepeat():
            return
        if e.key() in self.KEYMAP:
            self._on_stop()
        else:
            super().keyReleaseEvent(e)


class ControlTab(QWidget):
    def __init__(self, worker: RosWorker, parent=None):
        super().__init__(parent)
        self.worker = worker

        # 좌: 컨트롤 컬럼 / 우: 선택 로봇 위치 맵
        outer = QHBoxLayout(self)
        outer.setContentsMargins(14, 12, 14, 12)
        outer.setSpacing(12)
        root = QVBoxLayout()
        root.setSpacing(12)
        outer.addLayout(root, stretch=3)

        banner = QLabel(
            "⚙  로봇 제어 · 정비 (예시 초안) — 경로: QT → ACS → HQ.  "
            "명령 집합은 팀 협의 후 확정됩니다."
        )
        banner.setStyleSheet(
            "background:#fff8e1; border:1px solid #ffe0a3; border-radius:6px; padding:8px;"
        )
        banner.setWordWrap(True)
        root.addWidget(banner)

        # 대상 로봇 선택
        sel = QHBoxLayout()
        sel.addWidget(QLabel("대상 로봇"))
        self.robot_box = QComboBox()
        self.robot_box.addItems([r.upper() for r in config.ROBOT_IDS])
        sel.addWidget(self.robot_box)
        sel.addStretch(1)
        root.addLayout(sel)

        # 안전/운영 명령
        ops = QGroupBox("안전 · 운영")
        ops_lay = QHBoxLayout(ops)
        self.btn_estop = QPushButton("■ 비상 정지 (E-STOP)")
        self.btn_estop.setObjectName("Estop")
        self.btn_resume = QPushButton("재개 (RESUME)")
        self.btn_dock = QPushButton("충전 복귀 (DOCK)")
        self.btn_restart = QPushButton("재시작 (RESTART)")
        self.btn_disable = QPushButton("운영 비활성 (DISABLE)")
        self.btn_enable = QPushButton("운영 활성 (ENABLE)")
        for b, cmd in [
            (self.btn_estop, "ESTOP"), (self.btn_resume, "RESUME"),
            (self.btn_dock, "DOCK"), (self.btn_restart, "RESTART"),
            (self.btn_disable, "DISABLE"), (self.btn_enable, "ENABLE"),
        ]:
            b.clicked.connect(lambda _=False, c=cmd: self._send(c))
            ops_lay.addWidget(b)
        root.addWidget(ops)

        # 수동 teleop (정비 위치 이동)
        tele = QGroupBox("수동 이동 (TELEOP) — 정비 위치 미세 조정")
        tv = QVBoxLayout(tele)
        tv.setSpacing(8)

        # 속도 입력 — 라벨과 입력을 붙여 왼쪽 정렬
        speed = QHBoxLayout()
        speed.setSpacing(6)
        self.lin = QDoubleSpinBox()
        self.lin.setRange(0.0, 0.5)
        self.lin.setSingleStep(0.05)
        self.lin.setValue(0.10)
        self.lin.setFixedWidth(80)
        self.ang = QDoubleSpinBox()
        self.ang.setRange(0.0, 1.0)
        self.ang.setSingleStep(0.1)
        self.ang.setValue(0.30)
        self.ang.setFixedWidth(80)
        speed.addWidget(QLabel("직진 속도(m/s)"))
        speed.addWidget(self.lin)
        speed.addSpacing(18)
        speed.addWidget(QLabel("회전 속도(rad/s)"))
        speed.addWidget(self.ang)
        speed.addStretch(1)
        tv.addLayout(speed)

        # 방향 D-패드 — 고정폭 버튼을 가운데 정렬
        self.btn_fwd = QPushButton("▲ 전진")
        self.btn_back = QPushButton("▼ 후진")
        self.btn_left = QPushButton("◀ 좌회전")
        self.btn_right = QPushButton("▶ 우회전")
        self.btn_stop = QPushButton("● 정지")
        self.btn_fwd.clicked.connect(lambda: self._teleop_kind("fwd"))
        self.btn_back.clicked.connect(lambda: self._teleop_kind("back"))
        self.btn_left.clicked.connect(lambda: self._teleop_kind("left"))
        self.btn_right.clicked.connect(lambda: self._teleop_kind("right"))
        self.btn_stop.clicked.connect(lambda: self._teleop(0.0, 0.0))
        for b in (self.btn_fwd, self.btn_back, self.btn_left,
                  self.btn_right, self.btn_stop):
            b.setFixedWidth(96)
        pad_grid = QGridLayout()
        pad_grid.setSpacing(6)
        pad_grid.addWidget(self.btn_fwd, 0, 1)
        pad_grid.addWidget(self.btn_left, 1, 0)
        pad_grid.addWidget(self.btn_stop, 1, 1)
        pad_grid.addWidget(self.btn_right, 1, 2)
        pad_grid.addWidget(self.btn_back, 2, 1)
        pad_wrap = QHBoxLayout()
        pad_wrap.addStretch(1)
        pad_wrap.addLayout(pad_grid)
        pad_wrap.addStretch(1)
        tv.addLayout(pad_wrap)

        # 키보드 조작 패드
        self.pad = TeleopPad(on_move=self._teleop_kind,
                             on_stop=lambda: self._teleop(0.0, 0.0))
        tv.addWidget(self.pad)
        root.addWidget(tele)

        # 명령 로그
        root.addWidget(QLabel("명령 로그"))
        self.log = QPlainTextEdit()
        self.log.setObjectName("Log")
        self.log.setReadOnly(True)
        self.log.setFixedHeight(140)
        root.addWidget(self.log)

        # 우측: 선택 로봇 중심 맵
        self.focus_map = FleetMap(focus_mode=True)
        self.focus_map.set_selected(self._current_robot())
        outer.addWidget(self.focus_map, stretch=2)

        self.robot_box.currentIndexChanged.connect(self._on_robot_changed)
        self.worker.maintenance_result.connect(self._on_result)

    # ---- 스냅샷 (GUI 스레드) ----
    def on_snapshot(self, snap: FleetSnapshot) -> None:
        self._latest = snap
        self.focus_map.update_positions(snap)

    def _on_robot_changed(self, _index: int) -> None:
        self.focus_map.set_selected(self._current_robot())
        self.focus_map.update_positions(getattr(self, "_latest", None))

    # ---- 명령 전송 ----
    def _current_robot(self) -> str:
        return config.ROBOT_IDS[self.robot_box.currentIndex()]

    def _send(self, command: str) -> None:
        rid = self._current_robot()
        self._append(f"→ {rid} {command} 전송…")
        self.worker.request_maintenance(rid, command)

    def _teleop_kind(self, kind: str) -> None:
        """방향(fwd/back/left/right)을 현재 속도 설정으로 변환해 전송."""
        lin, ang = {
            "fwd": (+self.lin.value(), 0.0),
            "back": (-self.lin.value(), 0.0),
            "left": (0.0, +self.ang.value()),
            "right": (0.0, -self.ang.value()),
        }.get(kind, (0.0, 0.0))
        self._teleop(lin, ang)

    def _teleop(self, lin: float, ang: float) -> None:
        rid = self._current_robot()
        self._append(f"→ {rid} TELEOP linear_x={lin:.2f} angular_z={ang:.2f} 전송…")
        self.worker.request_maintenance(rid, "TELEOP", lin, ang)

    def _on_result(self, rid: str, command: str, ok: bool, message: str) -> None:
        mark = "✔" if ok else "✗"
        self._append(f"← {rid} {command} {mark} {message}")

    def _append(self, text: str) -> None:
        self.log.appendPlainText(text)

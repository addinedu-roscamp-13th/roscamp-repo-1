"""메인 윈도우: 상단 바 + 탭(모니터링 / 제어)."""
from __future__ import annotations

from PyQt6.QtCore import Qt, QTimer, QTime
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTabWidget, QFrame,
)

from ..ros.ros_worker import RosWorker
from .monitor_tab import MonitorTab
from .control_tab import ControlTab
from . import style


class MainWindow(QMainWindow):
    def __init__(self, worker: RosWorker):
        super().__init__()
        self.worker = worker
        self.setWindowTitle("Automato · System Admin (로봇 상태·제어)")
        self.resize(1360, 860)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # 상단 바
        bar = QFrame()
        bar.setObjectName("TopBar")
        bar.setFixedHeight(46)
        blay = QHBoxLayout(bar)
        blay.setContentsMargins(16, 0, 16, 0)
        title = QLabel("Automato  ·  System Admin APP")
        title.setStyleSheet("font-size:15px; font-weight:700;")
        blay.addWidget(title)
        blay.addStretch(1)
        self.clock = QLabel("--:--:--")
        self.clock.setStyleSheet("font-size:14px; font-weight:600;")
        blay.addWidget(self.clock)
        root.addWidget(bar)

        # 탭
        tabs = QTabWidget()
        self.monitor = MonitorTab()
        self.control = ControlTab(worker)
        tabs.addTab(self.monitor, "실시간 상태")
        tabs.addTab(self.control, "로봇 제어")
        root.addWidget(tabs, stretch=1)

        # ROS 스냅샷 → 모니터 탭 + 제어 탭(위치 맵)
        self.worker.fleet_snapshot.connect(self.monitor.on_snapshot)
        self.worker.fleet_snapshot.connect(self.control.on_snapshot)

        # 시계
        self._clock_timer = QTimer(self)
        self._clock_timer.timeout.connect(self._tick_clock)
        self._clock_timer.start(1000)
        self._tick_clock()

        self.setStyleSheet(style.QSS)

    def _tick_clock(self) -> None:
        self.clock.setText(QTime.currentTime().toString("HH:mm:ss"))

    def closeEvent(self, event) -> None:
        # ROS 스레드 정리 후 종료
        self.worker.stop()
        self.worker.wait(2000)
        super().closeEvent(event)

"""실시간 1분 그래프. 로봇팔 온도 / 주행 배터리를 토글로 전환.

pyqtgraph 사용. 데이터는 FleetHistory(메모리 링버퍼)에서 읽는다(저장 없음).
"""
from __future__ import annotations

import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel

from .. import config
from ..model.history import FleetHistory
from . import style

pg.setConfigOptions(antialias=True, background="w", foreground="#333")

METRIC_META = {
    "temp": {
        "label": "로봇팔 서보 온도",
        "unit": "℃",
        "yrange": (25, 80),
        "threshold": config.SERVO_TEMP_CRIT_C,
    },
    "battery": {
        "label": "주행 배터리",
        "unit": "%",
        "yrange": (0, 100),
        "threshold": None,
    },
}


class GraphPanel(QWidget):
    def __init__(self, history: FleetHistory, parent=None):
        super().__init__(parent)
        self.history = history
        self.metric = "temp"

        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)

        # 상단: 토글 버튼 + 제목
        top = QHBoxLayout()
        self.title = QLabel()
        self.title.setStyleSheet("font-weight:700;")
        top.addWidget(self.title)
        top.addStretch(1)
        self.btn_temp = QPushButton("로봇팔 온도")
        self.btn_batt = QPushButton("주행 배터리")
        self.btn_temp.setCheckable(True)
        self.btn_batt.setCheckable(True)
        self.btn_temp.setChecked(True)
        self.btn_temp.clicked.connect(lambda: self.set_metric("temp"))
        self.btn_batt.clicked.connect(lambda: self.set_metric("battery"))
        top.addWidget(self.btn_temp)
        top.addWidget(self.btn_batt)
        root.addLayout(top)

        # 플롯
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "경과 시간(초, 현재=0)")
        self.plot.addLegend(offset=(10, 10))
        root.addWidget(self.plot)

        # 로봇별 곡선
        self.curves: dict[str, pg.PlotDataItem] = {}
        for rid in config.ROBOT_IDS:
            pen = pg.mkPen(style.ROBOT_COLOR.get(rid, "#555"), width=2)
            self.curves[rid] = self.plot.plot([], [], name=rid.upper(), pen=pen)

        # 과열 기준선
        self.threshold_line = pg.InfiniteLine(
            angle=0, movable=False,
            pen=pg.mkPen("#c62828", width=1, style=pg.QtCore.Qt.PenStyle.DashLine),
            label="과열 65℃", labelOpts={"color": "#c62828", "position": 0.05},
        )
        self.plot.addItem(self.threshold_line)

        self._apply_metric_chrome()

    def set_metric(self, metric: str) -> None:
        self.metric = metric
        self.btn_temp.setChecked(metric == "temp")
        self.btn_batt.setChecked(metric == "battery")
        self._apply_metric_chrome()
        self.refresh()

    def _apply_metric_chrome(self) -> None:
        meta = METRIC_META[self.metric]
        self.title.setText(f"{meta['label']} 추이 (최근 {config.GRAPH_WINDOW_SEC}초)")
        self.plot.setLabel("left", f"{meta['label']} ({meta['unit']})")
        self.plot.setYRange(*meta["yrange"])
        self.plot.setXRange(-config.GRAPH_WINDOW_SEC, 0)
        self.threshold_line.setVisible(meta["threshold"] is not None)
        if meta["threshold"] is not None:
            self.threshold_line.setValue(meta["threshold"])

    def refresh(self) -> None:
        import time
        t0 = time.monotonic()
        series = self.history.metric(self.metric)
        for rid, curve in self.curves.items():
            xs, ys = series[rid].xy(t0)
            curve.setData(xs, ys)

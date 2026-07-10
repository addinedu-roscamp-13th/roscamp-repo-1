"""로봇 실시간 위치 맵 (2D 좌표 평면).

E0 텔레메트리에는 위치 x/y/yaw만 있으므로, 위에서 내려다본 평면에 로봇을 마커 +
방향 화살표로 표시한다. (배경 SLAM 점유격자맵을 깔려면 nav_msgs/OccupancyGrid 등
별도 토픽 필요 → 팀 협의 사항.)

focus_mode=True 면 선택 로봇을 강조(크게+범위 확대)하고, 나머지는 옅게 표시한다.
"""
from __future__ import annotations

import math
import os

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import QRectF
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel

from .. import config
from ..model.state import FleetSnapshot
from . import style

# 이미지 배열을 [row, col] = [y, x]로 다루기 위해 row-major 지정
pg.setConfigOptions(imageAxisOrder="row-major")

HEADING_LEN = 0.4  # 방향 화살표 길이(m)


class FleetMap(QWidget):
    def __init__(self, focus_mode: bool = False, parent=None):
        super().__init__(parent)
        self.focus_mode = focus_mode
        self.selected: str | None = None

        root = QVBoxLayout(self)
        root.setContentsMargins(4, 4, 4, 4)
        title = "선택 로봇 위치" if focus_mode else "편대 실시간 위치"
        lab = QLabel(title)
        lab.setStyleSheet("font-weight:700;")
        root.addWidget(lab)

        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)          # 실제 공간 비율 유지
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "X (m)")
        self.plot.setLabel("left", "Y (m)")
        # 맵을 180° 회전 표시 (우리가 보는 방향에 맞춤).
        # X·Y 축을 모두 뒤집으면 = 중심 기준 180° 회전 → 배경 이미지·로봇 마커·
        # 방향 화살표가 데이터 변경 없이 한꺼번에 회전한다.
        vb = self.plot.getViewBox()
        vb.invertX(True)
        vb.invertY(True)
        root.addWidget(self.plot)

        # 배경 SLAM 맵 (정적 파일)
        self.map_bg: pg.ImageItem | None = None
        self._load_background()

        self.scatter = pg.ScatterPlotItem()
        self.plot.addItem(self.scatter)

        # 로봇별 방향선 + 라벨
        self.head: dict[str, pg.PlotDataItem] = {}
        self.label: dict[str, pg.TextItem] = {}
        for rid in config.ROBOT_IDS:
            color = style.ROBOT_COLOR.get(rid, "#555")
            line = self.plot.plot([], [], pen=pg.mkPen(color, width=2))
            self.head[rid] = line
            txt = pg.TextItem(rid.upper(), color=color, anchor=(0.5, 1.4))
            self.plot.addItem(txt)
            self.label[rid] = txt

    def _load_background(self) -> None:
        """정적 .pgm/.yaml 맵을 배경으로 로드해 월드 좌표에 정렬한다."""
        if config.MAP_SOURCE != "file":
            return
        if not os.path.exists(config.MAP_YAML_PATH):
            return
        try:
            from ..mapdata import load_map
            m = load_map(config.MAP_YAML_PATH)
        except Exception as exc:  # 맵 로드 실패가 앱을 막지 않도록
            print(f"[fleet_map] 배경 맵 로드 실패: {exc}")
            return

        # PGM은 상단이 row 0. ImageItem은 아래→위로 그리므로 상하 반전해
        # row 0(=맵 좌하단)이 origin_y에 오도록 맞춘다.
        img = np.flipud(m.image)
        self.map_bg = pg.ImageItem(img)
        self.map_bg.setLevels((0, 255))
        self.map_bg.setZValue(-100)  # 로봇 마커보다 뒤
        # 월드 사각형에 배치: 좌하단 (origin_x, origin_y), 크기 (W*res, H*res)
        self.map_bg.setRect(
            QRectF(m.origin_x, m.origin_y, m.width_m, m.height_m)
        )
        self.plot.addItem(self.map_bg)
        # 편대 모드 기본 표시 범위를 맵 전체로
        if not self.focus_mode:
            pad = 0.1
            self.plot.setXRange(m.origin_x - pad, m.origin_x + m.width_m + pad)
            self.plot.setYRange(m.origin_y - pad, m.origin_y + m.height_m + pad)

    def set_selected(self, robot_id: str | None) -> None:
        self.selected = robot_id

    def update_positions(self, snap: FleetSnapshot | None) -> None:
        spots = []
        xs = []
        for rid in config.ROBOT_IDS:
            unit = snap.unit(rid) if snap else None
            d = unit.ddago if unit else None
            if d is None:
                self.head[rid].setData([], [])
                self.label[rid].setText("")
                continue
            x, y, yaw = d.x, d.y, d.yaw
            xs.append(x)
            is_sel = (rid == self.selected)
            base = style.ROBOT_COLOR.get(rid, "#555")
            # focus 모드에서 비선택 로봇은 옅게
            faded = self.focus_mode and self.selected is not None and not is_sel
            size = 22 if is_sel else 14
            spots.append({
                "pos": (x, y), "size": size,
                "brush": pg.mkBrush("#bbbbbb" if faded else base),
                "pen": pg.mkPen("#333", width=2 if is_sel else 1),
                "symbol": "o",
            })
            self.head[rid].setData(
                [x, x + HEADING_LEN * math.cos(yaw)],
                [y, y + HEADING_LEN * math.sin(yaw)],
            )
            self.head[rid].setPen(pg.mkPen("#cccccc" if faded else base, width=2))
            self.label[rid].setPos(x, y)
            self.label[rid].setText(rid.upper())
        self.scatter.setData(spots)

        # focus 모드: 선택 로봇 중심으로 범위 확대
        if self.focus_mode and self.selected is not None and xs:
            unit = snap.unit(self.selected) if snap else None
            if unit and unit.ddago:
                cx, cy = unit.ddago.x, unit.ddago.y
                r = 1.5
                self.plot.setXRange(cx - r, cx + r)
                self.plot.setYRange(cy - r, cy + r)

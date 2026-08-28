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
from PyQt6.QtCore import QRectF, QPointF, Qt
from PyQt6.QtGui import QPolygonF, QBrush, QColor, QPen
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QGraphicsPolygonItem

from .. import config
from ..model.state import FleetSnapshot
from . import style

# 이미지 배열을 [row, col] = [y, x]로 다루기 위해 row-major 지정
pg.setConfigOptions(imageAxisOrder="row-major")

# 방향 화살표(삼각형) 치수 — 원 바로 앞에 짧게 (단위 m)
ARROW_TIP = 0.10    # 중심~팁 거리
ARROW_BASE = 0.03   # 중심~밑변 거리 (원 바로 앞에서 시작)
ARROW_HW = 0.035    # 밑변 반폭

# 마커(원) 지름 — 데이터(m) 단위. 화살표와 같은 좌표계라 줌에 함께 스케일된다.
# (픽셀 단위로 두면 화살표만 커지고 원은 그대로여서 확대 시 따로 논다.)
DOT_SIZE = 0.07
DOT_SIZE_SEL = 0.11

# 화면 표시 방향. invertX+invertY = 중심 기준 180° 회전(배경·마커·화살표가 함께 돈다).
# 실물 맵을 물리적으로 180° 돌려놓으면 소프트웨어 회전은 꺼야 화면이 실물과 맞는다.
# 실물 배치가 다시 바뀌면 이 값만 토글한다.
MAP_ROTATE_180 = False


def _arrow_polygon(x: float, y: float, yaw: float) -> QPolygonF:
    """(x,y)에서 yaw 방향으로 향하는 짧은 화살촉 삼각형(데이터 좌표)."""
    c, s = math.cos(yaw), math.sin(yaw)
    px, py = -s, c  # 진행 방향에 수직
    tip = QPointF(x + ARROW_TIP * c, y + ARROW_TIP * s)
    bl = QPointF(x + ARROW_BASE * c + ARROW_HW * px, y + ARROW_BASE * s + ARROW_HW * py)
    br = QPointF(x + ARROW_BASE * c - ARROW_HW * px, y + ARROW_BASE * s - ARROW_HW * py)
    return QPolygonF([tip, bl, br])


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
        # 화면 회전 (MAP_ROTATE_180). 실물 맵을 물리적으로 180° 돌려놔서 현재는 끔.
        vb = self.plot.getViewBox()
        vb.invertX(MAP_ROTATE_180)
        vb.invertY(MAP_ROTATE_180)
        root.addWidget(self.plot)

        # 배경 SLAM 맵 (정적 파일)
        self.map_bg: pg.ImageItem | None = None
        self._load_background()

        # pxMode=False: 마커 크기를 데이터(m) 단위로 → 방향 화살표와 함께 스케일.
        self.scatter = pg.ScatterPlotItem(pxMode=False)
        self.plot.addItem(self.scatter)

        # focus 모드에서 뷰 자동맞춤은 '로봇 선택이 바뀔 때 1회'만. 이후 사용자
        # 줌/팬을 보존한다(매 프레임 setRange 하면 확대가 즉시 풀린다).
        self._pending_fit = True

        # 로봇별 방향 화살표(삼각형) + 라벨
        self.head: dict[str, QGraphicsPolygonItem] = {}
        self.label: dict[str, pg.TextItem] = {}
        for rid in config.ROBOT_IDS:
            color = style.ROBOT_COLOR.get(rid, "#555")
            arrow = QGraphicsPolygonItem()
            arrow.setPen(QPen(Qt.PenStyle.NoPen))
            arrow.setBrush(QBrush(QColor(color)))
            arrow.setZValue(1)                       # 배경 위, 마커와 같은 층
            self.plot.getViewBox().addItem(arrow)
            self.head[rid] = arrow
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
        if robot_id != self.selected:
            self._pending_fit = True   # 선택이 바뀌면 새 로봇에 1회 맞춘다
        self.selected = robot_id

    def update_positions(self, snap: FleetSnapshot | None) -> None:
        spots = []
        xs = []
        for rid in config.ROBOT_IDS:
            unit = snap.unit(rid) if snap else None
            d = unit.ddago if unit else None
            if d is None:
                self.head[rid].setPolygon(QPolygonF())   # 빈 폴리곤 = 숨김
                self.label[rid].setText("")
                continue
            x, y, yaw = d.x, d.y, d.yaw
            xs.append(x)
            is_sel = (rid == self.selected)
            base = style.ROBOT_COLOR.get(rid, "#555")
            # focus 모드에서 비선택 로봇은 옅게
            faded = self.focus_mode and self.selected is not None and not is_sel
            size = DOT_SIZE_SEL if is_sel else DOT_SIZE
            spots.append({
                "pos": (x, y), "size": size,
                "brush": pg.mkBrush("#bbbbbb" if faded else base),
                "pen": pg.mkPen("#333", width=2 if is_sel else 1),
                "symbol": "o",
            })
            self.head[rid].setPolygon(_arrow_polygon(x, y, yaw))
            self.head[rid].setBrush(QBrush(QColor("#cccccc" if faded else base)))
            self.label[rid].setPos(x, y)
            self.label[rid].setText(rid.upper())
        self.scatter.setData(spots)

        # focus 모드: 선택 로봇 중심으로 1회만 맞춘다. 이후에는 사용자가 확대/이동한
        # 뷰를 유지한다(매 프레임 setRange 하면 확대가 즉시 풀려버린다).
        if self.focus_mode and self._pending_fit and self.selected is not None:
            unit = snap.unit(self.selected) if snap else None
            if unit and unit.ddago:
                cx, cy = unit.ddago.x, unit.ddago.y
                r = 1.5
                self.plot.setXRange(cx - r, cx + r)
                self.plot.setYRange(cy - r, cy + r)
                self._pending_fit = False

"""탭1: 실시간 상태 모니터링. 요약 바 + DG 카드 3개 + 1분 그래프."""
from __future__ import annotations

import time

from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel

from .. import config
from ..model.state import FleetSnapshot
from ..model.history import FleetHistory
from ..model import thresholds
from .dg_card import DGCard
from .graph_panel import GraphPanel
from .fleet_map import FleetMap


class MonitorTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.history = FleetHistory()
        self.latest: FleetSnapshot | None = None
        self.last_seen: dict[str, float] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 12)
        root.setSpacing(10)

        # 요약 바
        summary = QHBoxLayout()
        self.summary_label = QLabel("연결 대기 중…")
        self.summary_label.setStyleSheet("font-weight:600;")
        summary.addWidget(self.summary_label)
        summary.addStretch(1)
        self.rx_label = QLabel("수신: -")
        summary.addWidget(self.rx_label)
        root.addLayout(summary)

        # 본문: 좌(카드 + 그래프) / 우(세로 맵)
        body = QHBoxLayout()
        body.setSpacing(10)

        left = QVBoxLayout()
        left.setSpacing(10)
        cards = QHBoxLayout()
        cards.setSpacing(10)
        self.cards: dict[str, DGCard] = {}
        for rid in config.ROBOT_IDS:
            card = DGCard(rid)
            self.cards[rid] = card
            cards.addWidget(card)
        left.addLayout(cards)
        self.graph = GraphPanel(self.history)
        left.addWidget(self.graph, stretch=1)
        body.addLayout(left, stretch=3)

        # 우측: 편대 실시간 위치 맵 (창 높이만큼 세로로)
        self.fleet_map = FleetMap(focus_mode=False)
        body.addWidget(self.fleet_map, stretch=2)

        root.addLayout(body, stretch=1)

        # 주기 갱신(수신 감시 + 그래프 스크롤). 데이터가 안 와도 화면이 산다.
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_tick)
        self.timer.start(500)

    # 시그널 슬롯 (GUI 스레드)
    def on_snapshot(self, snap: FleetSnapshot) -> None:
        self.latest = snap
        now = time.monotonic()
        for rid in snap.units:
            self.last_seen[rid] = now
        self.history.ingest(snap)
        self._render()

    def _on_tick(self) -> None:
        self._render()
        self.graph.refresh()

    def _render(self) -> None:
        now = time.monotonic()
        online_count = 0
        crit = warn = 0
        newest = 0.0
        for rid in config.ROBOT_IDS:
            seen = self.last_seen.get(rid, 0.0)
            online = (now - seen) <= config.STALE_SEC if seen else False
            unit = self.latest.unit(rid) if self.latest else None
            self.cards[rid].update_unit(unit, online)
            if online:
                online_count += 1
                newest = max(newest, seen)
                if unit is not None:
                    level, _ = thresholds.unit_status(unit)
                    if level == "crit":
                        crit += 1
                    elif level == "warn":
                        warn += 1

        self.fleet_map.update_positions(self.latest)

        total = len(config.ROBOT_IDS)
        self.summary_label.setText(
            f"온라인 {online_count}/{total}   ·   점검필요 {crit}   ·   주의 {warn}"
        )
        if newest:
            age = now - newest
            self.rx_label.setText(f"최근 수신: {age:.1f}초 전")
            self.rx_label.setStyleSheet(
                "color:#c62828; font-weight:700;" if age > config.STALE_SEC else "color:#2e7d32;"
            )
        else:
            self.rx_label.setText("수신: 없음 (텔레메트리 미도착)")
            self.rx_label.setStyleSheet("color:#c62828; font-weight:700;")

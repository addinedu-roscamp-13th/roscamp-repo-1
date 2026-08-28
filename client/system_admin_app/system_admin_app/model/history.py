"""1분 실시간 그래프용 메모리 링버퍼.

E0 원칙(저장 안 함)에 맞춰 DB/파일 없이 메모리에만 최근 GRAPH_MAXLEN 포인트를 유지한다.
metric 종류:
  - 'temp'    : 로봇팔 서보 최고온도(℃)
  - 'battery' : 주행 배터리 잔량(%)
"""
from __future__ import annotations

import time
from collections import deque
from typing import Optional

from .. import config
from .state import FleetSnapshot


class RobotSeries:
    """로봇 1대의 (시간, 값) 시계열 한 종류."""

    def __init__(self, maxlen: int = config.GRAPH_MAXLEN):
        self.t: deque[float] = deque(maxlen=maxlen)
        self.v: deque[float] = deque(maxlen=maxlen)

    def push(self, t: float, value: Optional[float]) -> None:
        # 값이 없으면(로봇팔 없음/미수신) 포인트를 넣지 않아 선이 끊긴다.
        if value is None:
            return
        self.t.append(t)
        self.v.append(value)

    def xy(self, t0: float) -> tuple[list[float], list[float]]:
        """t0 기준 상대초(음수) x와 값 y. 오래된 그래프 스크롤 표시에 사용."""
        return [ti - t0 for ti in self.t], list(self.v)


class FleetHistory:
    """metric별 × 로봇별 시계열 모음."""

    METRICS = ("temp", "battery")

    def __init__(self):
        self.series: dict[str, dict[str, RobotSeries]] = {
            m: {rid: RobotSeries() for rid in config.ROBOT_IDS}
            for m in self.METRICS
        }

    def ingest(self, snap: FleetSnapshot) -> None:
        now = time.monotonic()
        for rid in config.ROBOT_IDS:
            unit = snap.unit(rid)
            temp_val = None
            batt_val = None
            if unit is not None:
                # 주행 전용 로봇은 로봇팔이 없으므로 온도선을 그리지 않는다.
                # (ACS가 ddagi를 보내와도 '설정상 팔 없음'인 카드 표시와 맞춘다.)
                if unit.ddagi is not None and not unit.is_drive_only:
                    mt = unit.ddagi.max_temperature
                    temp_val = float(mt) if mt is not None else None
                if unit.ddago is not None:
                    batt_val = float(unit.ddago.battery_percent)
            self.series["temp"][rid].push(now, temp_val)
            self.series["battery"][rid].push(now, batt_val)

    def metric(self, name: str) -> dict[str, RobotSeries]:
        return self.series[name]

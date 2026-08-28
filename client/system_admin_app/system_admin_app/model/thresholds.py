"""임계값 기반 상태 판정. '정상/점검 필요' 뱃지와 경고 사유 문자열을 만든다.

레벨: 'ok'(정상) < 'warn'(주의) < 'crit'(위험/점검필요)
"""
from __future__ import annotations

from typing import Optional

from .. import config
from .state import DdagoState, DdagiState, DGUnit

LEVEL_ORDER = {"ok": 0, "warn": 1, "crit": 2}


def _worst(*levels: str) -> str:
    return max(levels, key=lambda lv: LEVEL_ORDER.get(lv, 0)) if levels else "ok"


def ddago_status(d: Optional[DdagoState]) -> tuple[str, list[str]]:
    """주행 로봇 상태 레벨과 사유 목록."""
    if d is None:
        return "warn", ["주행 텔레메트리 없음"]
    level = "ok"
    reasons: list[str] = []

    if d.battery_percent <= config.BATTERY_CRIT_PERCENT:
        level = _worst(level, "crit")
        reasons.append(f"배터리 위험 ({d.battery_percent:.0f}%)")
    elif d.battery_percent <= config.BATTERY_WARN_PERCENT:
        level = _worst(level, "warn")
        reasons.append(f"배터리 부족 ({d.battery_percent:.0f}%)")

    if config.nav_status_meta(d.nav_status)["level"] == "crit":
        level = _worst(level, "crit")
        reasons.append(f"주행 상태 {config.nav_status_meta(d.nav_status)['label']}")

    return level, reasons


def ddagi_status(a: Optional[DdagiState]) -> tuple[str, list[str]]:
    """로봇팔 상태 레벨과 사유 목록."""
    if a is None:
        return "ok", []  # 주행 전용 로봇은 로봇팔 없음이 정상
    level = "ok"
    reasons: list[str] = []

    if a.any_overload:
        level = _worst(level, "crit")
        reasons.append("서보 과부하")
    if a.any_undervoltage:
        level = _worst(level, "crit")
        reasons.append("서보 전압 이상")

    mt = a.max_temperature
    if mt is not None:
        if mt >= config.SERVO_TEMP_CRIT_C:
            level = _worst(level, "crit")
            reasons.append(f"서보 과열 ({mt}℃)")
        elif mt >= config.SERVO_TEMP_WARN_C:
            level = _worst(level, "warn")
            reasons.append(f"서보 고온 ({mt}℃)")

    return level, reasons


def unit_status(unit: DGUnit) -> tuple[str, list[str]]:
    """DG 단위 종합 상태(주행+로봇팔 중 나쁜 쪽)와 전체 사유.

    통신 두절이면 다른 판정을 덮어쓴다. ACS가 끊긴 로봇의 마지막 값을 계속 보내므로,
    두절을 표시하지 않으면 굳어버린 옛 수치가 정상 상태처럼 보인다.
    """
    if unit.is_offline:
        age = f"{unit.age_sec:.0f}초" if unit.age_sec is not None else "?"
        return "crit", [f"통신 두절 ({age} 미수신)"]

    go_level, go_reasons = ddago_status(unit.ddago)
    if unit.is_drive_only:
        return go_level, go_reasons
    gi_level, gi_reasons = ddagi_status(unit.ddagi)
    return _worst(go_level, gi_level), go_reasons + gi_reasons


LEVEL_LABEL = {"ok": "정상", "warn": "주의", "crit": "점검 필요"}

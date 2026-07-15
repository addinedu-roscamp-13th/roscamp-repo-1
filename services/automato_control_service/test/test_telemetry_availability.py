#!/usr/bin/env python3
"""RP-90 가용판정·메시지조립 순수 로직 단위테스트.

judge_robot_availability / build_message / _iso_ms 는 ROS/DB/네트워크 의존이 없는 순수
함수라 여기서 바로 검증한다. 특히 unavailable_reason 우선순위
(ROBOT_OFFLINE > CHARGING > BATTERY_TOO_LOW > ROBOT_BUSY)를 티켓 완료조건 시나리오로 확인.
(telemetry_ws 임포트가 fastapi 를 끌어오므로 미설치 환경에선 skip)

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_telemetry_availability.py -v
"""
import os
import sys

import pytest

pytest.importorskip("fastapi")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service.telemetry_ws import (  # noqa: E402
    OFFLINE_SEC, _iso_ms, build_message, judge_robot_availability,
)

NOW = 1000.0
THRESHOLD = 70.0


def _entry(**kw):
    """FleetCache.snapshot() 항목 하나(기본: 방금 수신·IDLE·미충전·배터리 85)."""
    e = {
        "robot_id": "dg_01", "nav_status": "IDLE", "is_charging": False,
        "x": 1.0, "y": 2.0, "yaw": 0.5, "battery_percent": 85.0,
        "stamp": NOW,                      # age 0 (방금 수신)
    }
    e.update(kw)
    return e


# --------------------------- 개별 사유 --------------------------- #
def test_available_when_nothing_wrong():
    out = judge_robot_availability(_entry(), NOW, None, THRESHOLD)
    assert out["available"] is True
    assert out["unavailable_reason"] is None
    assert out["task_type"] is None


def test_robot_offline_when_stale():
    out = judge_robot_availability(_entry(stamp=NOW - OFFLINE_SEC - 1), NOW, None, THRESHOLD)
    assert out["unavailable_reason"] == "ROBOT_OFFLINE"
    assert out["available"] is False


def test_charging():
    out = judge_robot_availability(_entry(is_charging=True), NOW, None, THRESHOLD)
    assert out["unavailable_reason"] == "CHARGING"


def test_battery_too_low():
    out = judge_robot_availability(_entry(battery_percent=THRESHOLD - 1), NOW, None, THRESHOLD)
    assert out["unavailable_reason"] == "BATTERY_TOO_LOW"


def test_battery_exactly_threshold_is_ok():
    # '< 임계값' 이므로 정확히 임계값이면 가용
    out = judge_robot_availability(_entry(battery_percent=THRESHOLD), NOW, None, THRESHOLD)
    assert out["available"] is True


def test_robot_busy_from_active_task():
    out = judge_robot_availability(_entry(), NOW, "PATROL", THRESHOLD)
    assert out["unavailable_reason"] == "ROBOT_BUSY"


def test_robot_busy_when_navigating_without_task():
    # nav!=IDLE(이동 중)이면 활성 task 가 없어도 ROBOT_BUSY (배차 불가)
    out = judge_robot_availability(_entry(nav_status="NAVIGATING"), NOW, None, THRESHOLD)
    assert out["unavailable_reason"] == "ROBOT_BUSY"
    assert out["task_type"] is None      # DB 활성 task 는 없으므로 task_type 은 None
    assert out["available"] is False


# --------------------------- 우선순위(핵심) --------------------------- #
def test_priority_offline_beats_all():
    # 오프라인 + 충전 + 배터리낮음 + 활성 task 동시 → ROBOT_OFFLINE
    out = judge_robot_availability(
        _entry(stamp=NOW - 10, is_charging=True, battery_percent=5.0),
        NOW, "PATROL", THRESHOLD)
    assert out["unavailable_reason"] == "ROBOT_OFFLINE"


def test_priority_charging_beats_battery_and_busy():
    # 티켓 명시 예: 충전 중이면서 배터리도 낮으면 → CHARGING
    out = judge_robot_availability(
        _entry(is_charging=True, battery_percent=5.0), NOW, "PATROL", THRESHOLD)
    assert out["unavailable_reason"] == "CHARGING"


def test_priority_battery_beats_busy():
    out = judge_robot_availability(
        _entry(battery_percent=5.0), NOW, "PATROL", THRESHOLD)
    assert out["unavailable_reason"] == "BATTERY_TOO_LOW"


# --------------------------- 필드 통과/보존 --------------------------- #
def test_task_type_passthrough():
    out = judge_robot_availability(_entry(), NOW, "HARVEST", THRESHOLD)
    assert out["task_type"] == "HARVEST"
    assert out["unavailable_reason"] == "ROBOT_BUSY"


def test_offline_keeps_last_known_values():
    # 오프라인이어도 위치·배터리는 마지막 값 유지(null 로 지우지 않음)
    out = judge_robot_availability(
        _entry(stamp=NOW - 10, x=3.2, y=1.1, battery_percent=42.0),
        NOW, None, THRESHOLD)
    assert out["unavailable_reason"] == "ROBOT_OFFLINE"
    assert out["position"] == {"x": 3.2, "y": 1.1, "yaw": 0.5}
    assert out["battery_percent"] == 42.0


# --------------------------- build_message 봉투 --------------------------- #
def test_build_message_envelope_and_per_robot():
    cache = [
        _entry(robot_id="dg_01"),
        _entry(robot_id="dg_02", battery_percent=5.0),   # active 에 없음
    ]
    active = {"dg_01": "PATROL"}                          # dg_01 만 활성 task
    msg = build_message(cache, active, THRESHOLD, seq=42, now=NOW)

    assert msg["event"] == "telemetry"
    assert msg["seq"] == 42
    assert msg["timestamp"].endswith("Z")

    robots = {r["robot_id"]: r for r in msg["data"]["robots"]}
    # dg_01: active_types 에 있음 → ROBOT_BUSY + task_type PATROL
    assert robots["dg_01"]["unavailable_reason"] == "ROBOT_BUSY"
    assert robots["dg_01"]["task_type"] == "PATROL"
    # dg_02: active 에 없음 → busy 아님, 배터리 낮아 BATTERY_TOO_LOW, task_type None
    assert robots["dg_02"]["unavailable_reason"] == "BATTERY_TOO_LOW"
    assert robots["dg_02"]["task_type"] is None


def test_iso_ms_millisecond_and_z():
    s = _iso_ms(1752566400.512)
    assert s.endswith("Z")
    frac = s.split(".")[1]      # 소수부
    assert frac == "512Z"       # 밀리초 3자리 + Z

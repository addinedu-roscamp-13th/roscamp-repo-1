#!/usr/bin/env python3
"""RP-78 ① 가용 판정(4조건)·auto 선정 순수 로직 단위테스트.

judge_robot / select_auto 는 ROS/DB 의존이 없는 순수 함수라 여기서 바로 검증한다.
(patrol_api 임포트가 fastapi/psycopg를 끌어오므로 미설치 환경에선 skip)

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_patrol_availability.py -v
"""
import os
import sys

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("psycopg")
pytest.importorskip("psycopg_pool")

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service.patrol_api import judge_robot, select_auto  # noqa: E402

NOW = 1000.0
THRESHOLD = 70


def _entry(nav_status="IDLE", battery=85.0, x=1.0, y=2.0,
           stamp=NOW, is_charging=False):
    """캐시 항목(TelemetryCache.get 형태) 한 개를 만든다."""
    return {
        "robot_id": "dg_x",
        "ddago": {
            "nav_status": nav_status,
            "is_charging": is_charging,
            "task_id": 0,
            "x": x, "y": y, "yaw": 0.0,
            "battery_percent": battery,
            "battery_voltage": 12.0,
            "us_range_m": 1.0,
        },
        "ddago_stamp": stamp,
    }


# --------------------------- 가용(available=True) --------------------------- #
def test_available_when_all_conditions_pass():
    j = judge_robot("dg_01", _entry(), has_active_task=False,
                    threshold=THRESHOLD, now=NOW)
    assert j["available"] is True
    assert "unavailable_reason" not in j
    assert j["status"] == "IDLE"
    assert j["battery_percent"] == 85.0
    assert j["current_position"] == {"x": 1.0, "y": 2.0}


def test_is_charging_does_not_affect_availability():
    # is_charging=True 여도(현재 항상 false 고정이지만) 판정엔 영향 없어야 한다.
    j = judge_robot("dg_01", _entry(is_charging=True), has_active_task=False,
                    threshold=THRESHOLD, now=NOW)
    assert j["available"] is True


# --------------------------- 불가 사유별 --------------------------- #
def test_robot_busy_by_active_task():
    j = judge_robot("dg_01", _entry(), has_active_task=True,
                    threshold=THRESHOLD, now=NOW)
    assert j["available"] is False
    assert j["unavailable_reason"] == "ROBOT_BUSY"


def test_robot_busy_by_nav_status():
    j = judge_robot("dg_01", _entry(nav_status="NAVIGATING"),
                    has_active_task=False, threshold=THRESHOLD, now=NOW)
    assert j["unavailable_reason"] == "ROBOT_BUSY"


def test_battery_too_low():
    j = judge_robot("dg_01", _entry(battery=62.0), has_active_task=False,
                    threshold=THRESHOLD, now=NOW)
    assert j["unavailable_reason"] == "BATTERY_TOO_LOW"


def test_telemetry_stale_by_old_stamp():
    j = judge_robot("dg_01", _entry(stamp=NOW - 5.0), has_active_task=False,
                    threshold=THRESHOLD, now=NOW)
    assert j["unavailable_reason"] == "ROBOT_OFFLINE"


def test_telemetry_stale_when_never_received():
    j = judge_robot("dg_01", None, has_active_task=False,
                    threshold=THRESHOLD, now=NOW)
    assert j["unavailable_reason"] == "ROBOT_OFFLINE"
    assert j["status"] is None
    assert j["current_position"] is None


# --------------------------- 사유 우선순위 --------------------------- #
def test_active_task_takes_precedence_over_battery():
    # 활성 task(DB 사실) > 배터리 부족: ROBOT_BUSY 가 우선
    j = judge_robot("dg_01", _entry(battery=10.0), has_active_task=True,
                    threshold=THRESHOLD, now=NOW)
    assert j["unavailable_reason"] == "ROBOT_BUSY"


def test_stale_takes_precedence_over_nav_and_battery():
    # 미수신이면 캐시값(nav/battery)을 못 믿으므로 ROBOT_OFFLINE 이 우선
    j = judge_robot("dg_01", _entry(nav_status="NAVIGATING", battery=10.0,
                                    stamp=NOW - 9.0),
                    has_active_task=False, threshold=THRESHOLD, now=NOW)
    assert j["unavailable_reason"] == "ROBOT_OFFLINE"


# --------------------------- auto 선정 --------------------------- #
def _judged(robot_id, available, battery):
    return {"robot_id": robot_id, "available": available,
            "battery_percent": battery}


def test_select_auto_picks_highest_battery():
    judged = [
        _judged("dg_01", True, 80.0),
        _judged("dg_02", True, 92.0),
        _judged("dg_03", False, 99.0),   # 불가 후보는 제외
    ]
    assert select_auto(judged) == "dg_02"


def test_select_auto_tie_breaks_by_robot_id_ascending():
    judged = [
        _judged("dg_02", True, 90.0),
        _judged("dg_01", True, 90.0),    # 동점 → robot_id 오름차순 첫 번째
    ]
    assert select_auto(judged) == "dg_01"


def test_select_auto_none_when_no_candidate():
    judged = [_judged("dg_01", False, 80.0), _judged("dg_02", False, 90.0)]
    assert select_auto(judged) is None

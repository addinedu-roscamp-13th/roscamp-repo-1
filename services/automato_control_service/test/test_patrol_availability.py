#!/usr/bin/env python3
"""RP-78 ① 가용 판정(5조건)·auto 선정 순수 로직 단위테스트.

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
ROBOT = "dg_01"


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


def _judge(entry, has_active_task=False, operational="NORMAL",
           threshold=THRESHOLD, now=NOW, robot_id=ROBOT):
    """judge_robot 호출 래퍼 — 기본은 '가용한 로봇'(운영 정상 + 활성 task 없음).

    테스트마다 관심 있는 조건 하나만 인자로 뒤집으면 되고, judge_robot 의 인자 순서가
    바뀌어도 여기 한 곳만 고치면 된다.
    """
    return judge_robot(robot_id, entry, has_active_task, operational, threshold, now)


# --------------------------- 가용(available=True) --------------------------- #
def test_available_when_all_conditions_pass():
    j = _judge(_entry())
    assert j["available"] is True
    assert "unavailable_reason" not in j
    assert j["status"] == "IDLE"
    assert j["battery_percent"] == 85.0
    assert j["current_position"] == {"x": 1.0, "y": 2.0}


def test_is_charging_does_not_affect_availability():
    # is_charging=True 여도(현재 항상 false 고정이지만) 판정엔 영향 없어야 한다.
    j = _judge(_entry(is_charging=True))
    assert j["available"] is True


# --------------------------- 불가 사유별 --------------------------- #
def test_robot_busy_by_active_task():
    j = _judge(_entry(), has_active_task=True)
    assert j["available"] is False
    assert j["unavailable_reason"] == "ROBOT_BUSY"


def test_robot_busy_by_nav_status():
    j = _judge(_entry(nav_status="NAVIGATING"))
    assert j["unavailable_reason"] == "ROBOT_BUSY"


def test_battery_too_low():
    j = _judge(_entry(battery=62.0))
    assert j["unavailable_reason"] == "BATTERY_TOO_LOW"


def test_telemetry_stale_by_old_stamp():
    j = _judge(_entry(stamp=NOW - 5.0))
    assert j["unavailable_reason"] == "ROBOT_OFFLINE"


def test_telemetry_stale_when_never_received():
    j = _judge(None)
    assert j["unavailable_reason"] == "ROBOT_OFFLINE"
    assert j["status"] is None
    assert j["current_position"] is None


# --------------------------- 운영 상태(E1 가용 조건 1) --------------------------- #
def test_immobilized_blocks_availability():
    # 통로에 갇혀 현장 정지된 로봇(E2 22-2). 나머지 조건은 전부 정상인데도 배정 불가.
    j = _judge(_entry(), operational="IMMOBILIZED")
    assert j["available"] is False
    assert j["unavailable_reason"] == "IMMOBILIZED"


def test_maintenance_reported_as_immobilized():
    # 관리자 수동 점검도 사유명은 IMMOBILIZED 하나로 낸다(E0 5) 표 정의).
    j = _judge(_entry(), operational="MAINTENANCE")
    assert j["available"] is False
    assert j["unavailable_reason"] == "IMMOBILIZED"


def test_immobilized_robot_is_idle_and_healthy():
    # 이 테스트가 조건 1)의 존재 이유다: 갇힌 로봇은 nav_status=IDLE 이고 배터리도
    # 멀쩡하고 텔레메트리도 신선해서, operational_status 축이 없으면 나머지 4조건을
    # 전부 통과해 다시 배정된다.
    entry = _entry(nav_status="IDLE", battery=99.0, stamp=NOW)
    assert _judge(entry)["available"] is True                      # 축이 없다면 가용
    assert _judge(entry, operational="IMMOBILIZED")["available"] is False


# --------------------------- 사유 우선순위 --------------------------- #
def test_immobilized_takes_precedence_over_active_task():
    # 둘 다 DB 사실이지만, 사람이 가야만 풀리는 쪽을 먼저 보여준다.
    j = _judge(_entry(), has_active_task=True, operational="IMMOBILIZED")
    assert j["unavailable_reason"] == "IMMOBILIZED"


def test_immobilized_takes_precedence_over_offline():
    # 오프라인이면서 갇힌 로봇도 관리자에겐 IMMOBILIZED 로 보여야 현장에 나갈 판단이 선다.
    j = _judge(None, operational="IMMOBILIZED")
    assert j["unavailable_reason"] == "IMMOBILIZED"


def test_active_task_takes_precedence_over_battery():
    # 활성 task(DB 사실) > 배터리 부족: ROBOT_BUSY 가 우선
    j = _judge(_entry(battery=10.0), has_active_task=True)
    assert j["unavailable_reason"] == "ROBOT_BUSY"


def test_stale_takes_precedence_over_nav_and_battery():
    # 미수신이면 캐시값(nav/battery)을 못 믿으므로 ROBOT_OFFLINE 이 우선
    j = _judge(_entry(nav_status="NAVIGATING", battery=10.0, stamp=NOW - 9.0))
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


def test_select_auto_skips_immobilized_even_with_highest_battery():
    # judge_robot 이 available=False 로 내려주므로 배터리가 가장 높아도 후보에서 빠진다.
    # (판정과 선정을 이어붙인 통합 확인 — 배터리만 보고 고르면 갇힌 로봇이 뽑힌다.)
    immobilized = _judge(_entry(battery=99.0), operational="IMMOBILIZED",
                         robot_id="dg_02")
    normal = _judge(_entry(battery=71.0), robot_id="dg_01")
    assert select_auto([immobilized, normal]) == "dg_01"

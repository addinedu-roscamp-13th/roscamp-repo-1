#!/usr/bin/env python3
"""RP-90 가용판정·메시지조립 순수 로직 단위테스트.

judge_robot_availability / build_message / _iso_ms 는 ROS/DB/네트워크 의존이 없는 순수
함수라 여기서 바로 검증한다. 특히 unavailable_reason 우선순위
(IMMOBILIZED > ROBOT_OFFLINE > CHARGING > BATTERY_TOO_LOW > ROBOT_BUSY)를
티켓 완료조건 시나리오로 확인.
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


def _judge(entry, active_task_type=None, operational="NORMAL",
           now=NOW, threshold=THRESHOLD):
    """judge_robot_availability 호출 래퍼 — 기본은 '운영 정상·활성 task 없음'.

    테스트마다 관심 있는 조건 하나만 뒤집으면 되고, 함수 인자 순서가 바뀌어도
    여기 한 곳만 고치면 된다.
    """
    return judge_robot_availability(
        entry, now, active_task_type, operational, threshold)


# --------------------------- 개별 사유 --------------------------- #
def test_available_when_nothing_wrong():
    out = _judge(_entry())
    assert out["available"] is True
    assert out["unavailable_reason"] is None
    assert out["task_type"] is None


def test_robot_offline_when_stale():
    out = _judge(_entry(stamp=NOW - OFFLINE_SEC - 1))
    assert out["unavailable_reason"] == "ROBOT_OFFLINE"
    assert out["available"] is False


def test_charging():
    out = _judge(_entry(is_charging=True))
    assert out["unavailable_reason"] == "CHARGING"


def test_battery_too_low():
    out = _judge(_entry(battery_percent=THRESHOLD - 1))
    assert out["unavailable_reason"] == "BATTERY_TOO_LOW"


def test_battery_exactly_threshold_is_ok():
    # '< 임계값' 이므로 정확히 임계값이면 가용
    out = _judge(_entry(battery_percent=THRESHOLD))
    assert out["available"] is True


def test_robot_busy_from_active_task():
    out = _judge(_entry(), "PATROL")
    assert out["unavailable_reason"] == "ROBOT_BUSY"


def test_robot_busy_when_navigating_without_task():
    # nav!=IDLE(이동 중)이면 활성 task 가 없어도 ROBOT_BUSY (배차 불가)
    out = _judge(_entry(nav_status="NAVIGATING"))
    assert out["unavailable_reason"] == "ROBOT_BUSY"
    assert out["task_type"] is None      # DB 활성 task 는 없으므로 task_type 은 None
    assert out["available"] is False


# --------------------------- 운영 상태(IMMOBILIZED) --------------------------- #
def test_immobilized_blocks_availability():
    # 통로에 갇혀 현장 정지된 로봇(E2 22-2). 나머지가 전부 정상이어도 가용이 아니다.
    out = _judge(_entry(), operational="IMMOBILIZED")
    assert out["available"] is False
    assert out["unavailable_reason"] == "IMMOBILIZED"


def test_maintenance_reported_as_immobilized():
    # 관리자 수동 점검도 사유명은 IMMOBILIZED 하나로 낸다(E0 5) 표 정의).
    out = _judge(_entry(), operational="MAINTENANCE")
    assert out["unavailable_reason"] == "IMMOBILIZED"


def test_immobilized_keeps_position_and_battery():
    # 갇힌 로봇의 위치는 관리자가 찾아가야 하므로 반드시 살아 있어야 한다.
    out = _judge(_entry(x=3.2, y=1.1, battery_percent=42.0),
                 operational="IMMOBILIZED")
    assert out["position"] == {"x": 3.2, "y": 1.1, "yaw": 0.5}
    assert out["battery_percent"] == 42.0


# --------------------------- 우선순위(핵심) --------------------------- #
def test_priority_immobilized_beats_everything():
    # 갇힘 + 오프라인 + 충전 + 배터리낮음 + 활성 task 동시 → IMMOBILIZED
    # 다른 사유에 가려지면 화면을 보는 사람이 '현장에 나가야 함'을 알 수 없다.
    out = _judge(_entry(stamp=NOW - 10, is_charging=True, battery_percent=5.0),
                 "PATROL", operational="IMMOBILIZED")
    assert out["unavailable_reason"] == "IMMOBILIZED"


def test_priority_offline_beats_all_but_immobilized():
    # 오프라인 + 충전 + 배터리낮음 + 활성 task 동시 → ROBOT_OFFLINE (운영상태는 정상)
    out = _judge(_entry(stamp=NOW - 10, is_charging=True, battery_percent=5.0),
                 "PATROL")
    assert out["unavailable_reason"] == "ROBOT_OFFLINE"


def test_priority_charging_beats_battery_and_busy():
    # 티켓 명시 예: 충전 중이면서 배터리도 낮으면 → CHARGING
    out = _judge(_entry(is_charging=True, battery_percent=5.0), "PATROL")
    assert out["unavailable_reason"] == "CHARGING"


def test_priority_battery_beats_busy():
    out = _judge(_entry(battery_percent=5.0), "PATROL")
    assert out["unavailable_reason"] == "BATTERY_TOO_LOW"


# --------------------------- 필드 통과/보존 --------------------------- #
def test_task_type_passthrough():
    out = _judge(_entry(), "HARVEST")
    assert out["task_type"] == "HARVEST"
    assert out["unavailable_reason"] == "ROBOT_BUSY"


def test_offline_keeps_last_known_values():
    # 오프라인이어도 위치·배터리는 마지막 값 유지(null 로 지우지 않음)
    out = _judge(_entry(stamp=NOW - 10, x=3.2, y=1.1, battery_percent=42.0))
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
    msg = build_message(cache, active, {}, THRESHOLD, seq=42, now=NOW)

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


def test_build_message_applies_operational_per_robot():
    # operational 은 로봇별로 갈라져 적용된다(한 대만 갇힌 상황).
    cache = [_entry(robot_id="dg_01"), _entry(robot_id="dg_02")]
    msg = build_message(cache, {}, {"dg_02": "IMMOBILIZED"}, THRESHOLD,
                        seq=1, now=NOW)

    robots = {r["robot_id"]: r for r in msg["data"]["robots"]}
    assert robots["dg_01"]["available"] is True
    assert robots["dg_02"]["unavailable_reason"] == "IMMOBILIZED"


def test_build_message_missing_operational_defaults_to_normal():
    # DB 조회가 아직 성공 못 해 operational 이 비어 있어도 방송은 계속돼야 한다
    # (KeyError 로 1Hz 루프를 죽이지 않는다). 배정은 어차피 E1 API 가 DB 를 직접 본다.
    msg = build_message([_entry(robot_id="dg_09")], {}, {}, THRESHOLD,
                        seq=1, now=NOW)
    assert msg["data"]["robots"][0]["available"] is True


def test_iso_ms_millisecond_and_z():
    s = _iso_ms(1752566400.512)
    assert s.endswith("Z")
    frac = s.split(".")[1]      # 소수부
    assert frac == "512Z"       # 밀리초 3자리 + Z

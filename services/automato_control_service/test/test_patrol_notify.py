#!/usr/bin/env python3
"""순찰 종료·작업 실패 발송(patrol_notify) 단위테스트 — 로봇/DB/네트워크 없이.

검증하는 것:
  ① payload 필드가 문서 스펙(E2 9-1 / 13)과 일치하는가
  ② patrol_completed 는 실패해도 재시도하지 않는가(fire-and-forget)
  ③ task_failed 는 최대 retries 회 재시도하는가, 중간에 성공하면 멈추는가
  ④ enum 밖의 reason/recovery_action 을 넣으면 ValueError 인가

post_json 은 monkeypatch 로 갈아끼워 실제 HTTP 를 내지 않는다.
detection_service 쪽 테스트와 같은 패턴이다.

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_patrol_notify.py -v
"""
import os
import sys
from datetime import datetime, timezone

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service import patrol_notify as pn  # noqa: E402

DT = datetime(2026, 7, 22, 9, 35, 12, tzinfo=timezone.utc)
SUMMARY = {"ripe_percent": 50, "unripe_percent": 50,
           "rotten_percent": 0, "disease_percent": 0}


# =========================================================================== #
# ① payload 구성 (순수 함수)
# =========================================================================== #
def test_completed_payload_필드가_스펙과_일치한다():
    p = pn.build_completed_payload(
        task_id=1024, robot_id="dg_01", status="COMPLETED",
        unvisited_waypoint_ids=[], completed_at=DT, summary=SUMMARY)
    assert p == {
        "task_id": 1024,
        "robot_id": "dg_01",
        "status": "COMPLETED",
        "unvisited_waypoint_ids": [],
        "completed_at": "2026-07-22T09:35:12+00:00",
        "summary": SUMMARY,
    }


def test_partial_payload_는_미방문목록을_싣는다():
    p = pn.build_completed_payload(
        task_id=1024, robot_id="dg_01", status="COMPLETED_PARTIAL",
        unvisited_waypoint_ids=[7, 9], completed_at=DT, summary=SUMMARY)
    assert p["status"] == "COMPLETED_PARTIAL"
    assert p["unvisited_waypoint_ids"] == [7, 9]


def test_task_failed_payload_필드가_스펙과_일치한다():
    p = pn.build_task_failed_payload(
        task_id=1024, robot_id="dg_01", reason="HARDWARE_ERROR",
        recovery_action="NONE", failed_at=DT)
    assert p["task_id"] == 1024
    assert p["robot_id"] == "dg_01"
    assert p["task_type"] == "PATROL"          # 기본값
    assert p["reason"] == "HARDWARE_ERROR"
    assert p["recovery_action"] == "NONE"
    assert p["failed_at"] == "2026-07-22T09:35:12+00:00"
    # 막힘 관련 필드는 값이 없어도 키는 유지된다(수신측 스키마 단일화).
    for k in ("blocked_corridor", "blocked_by_robot_id",
              "robot_position", "waited_sec"):
        assert k in p and p[k] is None


def test_task_failed_는_막힘_상세를_실을_수_있다():
    p = pn.build_task_failed_payload(
        task_id=1024, robot_id="dg_01", reason="BLOCKED",
        recovery_action="RETURN_TO_CHARGER", failed_at=DT,
        blocked_corridor={"waypoint_a_id": 3, "waypoint_b_id": 4},
        blocked_by_robot_id="dg_02", waited_sec=60)
    assert p["reason"] == "BLOCKED"
    assert p["blocked_corridor"] == {"waypoint_a_id": 3, "waypoint_b_id": 4}
    assert p["blocked_by_robot_id"] == "dg_02"
    assert p["waited_sec"] == 60


# =========================================================================== #
# ④ enum 검증
# =========================================================================== #
def test_잘못된_reason_은_ValueError():
    with pytest.raises(ValueError):
        pn.build_task_failed_payload(
            task_id=1, robot_id="dg_01", reason="OOPS", failed_at=DT)


def test_잘못된_recovery_action_은_ValueError():
    with pytest.raises(ValueError):
        pn.build_task_failed_payload(
            task_id=1, robot_id="dg_01", reason="BLOCKED",
            recovery_action="OOPS", failed_at=DT)


# =========================================================================== #
# ② patrol_completed — fire-and-forget (재시도 없음)
# =========================================================================== #
def test_patrol_completed_는_url_과_payload_를_그대로_보낸다(monkeypatch):
    calls = []
    monkeypatch.setattr(pn, "post_json",
                        lambda url, payload, timeout: calls.append((url, payload)) or 200)
    payload = pn.build_completed_payload(
        task_id=1024, robot_id="dg_01", status="COMPLETED",
        unvisited_waypoint_ids=[], completed_at=DT, summary=SUMMARY)

    ok = pn.send_patrol_completed("http://web:8100/", payload)

    assert ok is True
    assert len(calls) == 1
    url, sent = calls[0]
    assert url == "http://web:8100" + pn.PATROL_COMPLETED_PATH
    assert sent is payload


def test_patrol_completed_는_실패해도_재시도하지_않는다(monkeypatch):
    calls = []

    def boom(url, payload, timeout):
        calls.append(url)
        raise ConnectionError("web down")

    monkeypatch.setattr(pn, "post_json", boom)
    ok = pn.send_patrol_completed("http://web:8100", {"task_id": 1})

    assert ok is False
    assert len(calls) == 1, "재시도 없이 딱 1회만 시도해야 한다"


# =========================================================================== #
# ③ task_failed — 최대 retries 회 재시도
# =========================================================================== #
def test_task_failed_는_실패하면_retries_회_시도한다(monkeypatch):
    calls = []

    def always_fail(url, payload, timeout):
        calls.append(url)
        raise ConnectionError("web down")

    monkeypatch.setattr(pn, "post_json", always_fail)
    # sleep 은 실제로 안 재우고 삼킨다(테스트 즉시 진행).
    ok = pn.send_task_failed(
        "http://web:8100", {"task_id": 1, "reason": "HARDWARE_ERROR"},
        retries=3, sleep=lambda _s: None)

    assert ok is False
    assert len(calls) == 3, "최대 3회 시도해야 한다"


def test_task_failed_는_중간에_성공하면_멈춘다(monkeypatch):
    calls = []

    def flaky(url, payload, timeout):
        calls.append(url)
        if len(calls) < 2:
            raise ConnectionError("일시 실패")
        return 200

    monkeypatch.setattr(pn, "post_json", flaky)
    ok = pn.send_task_failed(
        "http://web:8100", {"task_id": 1, "reason": "HARDWARE_ERROR"},
        retries=3, sleep=lambda _s: None)

    assert ok is True
    assert len(calls) == 2, "성공한 순간 멈춰야 한다(3회를 다 쓰지 않는다)"

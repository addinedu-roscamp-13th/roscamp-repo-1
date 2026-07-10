#!/usr/bin/env python3
"""RP-79 탐지 저장/중계/알림 오케스트레이션 단위테스트.

detection_service 는 detection_db(psycopg)를 '지연 임포트'하므로, DB 드라이버 없이도
이 파일을 실행할 수 있다(협력자를 fake 로 주입해 순서·게이트·실패 정책만 검증).

실행:
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_detection_service.py -v
"""
import os
import sys
from datetime import datetime, timezone

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from automato_control_service import detection_service as ds  # noqa: E402

FIXED = datetime(2026, 7, 9, 12, 34, 56, tzinfo=timezone.utc)


# =========================================================================== #
# 순수 함수: 경로 규칙 / payload 구성
# =========================================================================== #
def test_relative_image_path_format():
    rel = ds.relative_image_path(3, "dg_01", FIXED)
    assert rel == "2026-07-09/wp3_dg_01_123456.jpg"


def test_notify_payload_has_detection_id_and_no_zone_cumulative():
    p = ds.build_notify_payload(
        task_id=7, waypoint_id=3, robot_id="dg_01", detection_id=42,
        ripe_percent=80, unripe_percent=10, rotten_percent=5,
        disease_percent=5, detected_at=FIXED)
    assert p["detection_id"] == 42
    assert p["detected_at"] == "2026-07-09T12:34:56+00:00"
    assert "zone_cumulative" not in p
    # 필수 필드 존재
    for k in ("task_id", "waypoint_id", "robot_id", "ripe_percent",
              "unripe_percent", "rotten_percent", "disease_percent"):
        assert k in p


def test_notify_payload_detection_id_null_when_db_failed():
    p = ds.build_notify_payload(
        task_id=7, waypoint_id=3, robot_id="dg_01", detection_id=None,
        ripe_percent=0, unripe_percent=0, rotten_percent=0,
        disease_percent=0, detected_at=FIXED)
    assert p["detection_id"] is None


def test_alert_payload_empty_image_path_when_none():
    p = ds.build_alert_payload(
        task_id=7, waypoint_id=3, robot_id="dg_01", disease_percent=9,
        image_path=None, detected_at=FIXED)
    assert p["image_path"] == ""      # None → "" 로 발송


def test_alert_payload_keeps_image_path():
    p = ds.build_alert_payload(
        task_id=7, waypoint_id=3, robot_id="dg_01", disease_percent=9,
        image_path="2026-07-09/wp3_dg_01_123456.jpg", detected_at=FIXED)
    assert p["image_path"] == "2026-07-09/wp3_dg_01_123456.jpg"


# =========================================================================== #
# 이미지 저장 (파일시스템)
# =========================================================================== #
def test_store_disease_image_writes_and_returns_relative(tmp_path):
    rel = ds.store_disease_image(
        str(tmp_path), 3, "dg_01", FIXED, b"\xff\xd8jpeg", log=None)
    assert rel == "2026-07-09/wp3_dg_01_123456.jpg"
    written = tmp_path / rel
    assert written.read_bytes() == b"\xff\xd8jpeg"   # 절대경로에 실제 저장됨


def test_store_disease_image_failure_returns_none(tmp_path):
    # 파일을 루트로 지정하면 그 아래 디렉터리를 못 만들어 저장 실패 → None
    blocker = tmp_path / "iam_a_file"
    blocker.write_text("x")
    rel = ds.store_disease_image(
        str(blocker), 3, "dg_01", FIXED, b"data", log=None)
    assert rel is None


# =========================================================================== #
# HTTP: notify(재시도 없음) / alert(재시도)
# =========================================================================== #
def test_send_notify_success(monkeypatch):
    calls = []
    monkeypatch.setattr(ds, "post_json",
                        lambda url, payload, timeout: calls.append(url) or 200)
    ok = ds.send_notify("http://x", {"waypoint_id": 1})
    assert ok is True and len(calls) == 1
    assert calls[0].endswith(ds.NOTIFY_PATH)


def test_send_notify_failure_no_retry(monkeypatch):
    calls = []

    def boom(url, payload, timeout):
        calls.append(url)
        raise RuntimeError("net down")

    monkeypatch.setattr(ds, "post_json", boom)
    ok = ds.send_notify("http://x", {"waypoint_id": 1})
    assert ok is False and len(calls) == 1     # 딱 1회, 재시도 없음


def test_send_disease_alert_retries_until_success(monkeypatch):
    attempts = {"n": 0}

    def flaky(url, payload, timeout):
        attempts["n"] += 1
        if attempts["n"] < 2:
            raise RuntimeError("first fails")
        return 200

    monkeypatch.setattr(ds, "post_json", flaky)
    ok = ds.send_disease_alert("http://x", {"waypoint_id": 1},
                               retries=3, sleep=lambda s: None)
    assert ok is True and attempts["n"] == 2


def test_send_disease_alert_final_failure(monkeypatch):
    attempts = {"n": 0}

    def always_fail(url, payload, timeout):
        attempts["n"] += 1
        raise RuntimeError("down")

    slept = []
    monkeypatch.setattr(ds, "post_json", always_fail)
    ok = ds.send_disease_alert("http://x", {"waypoint_id": 1},
                               retries=3, sleep=lambda s: slept.append(s))
    assert ok is False
    assert attempts["n"] == 3                  # 최대 3회 시도
    assert len(slept) == 2                     # 시도 사이 2번만 대기


# =========================================================================== #
# 오케스트레이션: 순서 / 게이트 / 실패 정책 (fake 협력자 주입)
# =========================================================================== #
def _make_handler(*, store_return="REL", db_return=42, db_raises=False,
                  threshold=5):
    """fake 협력자를 주입한 DetectionHandler 와 호출 기록(calls)을 만든다.

    dispatch 는 동기 실행(lambda fn: fn())이라 notify/alert 호출이 즉시 기록된다.
    now_fn 은 고정값이라 detected_at 일관성 검증이 가능하다.
    """
    calls = {"store": [], "db": [], "notify": [], "alert": []}

    def store_fn(root, wp, rid, dt, img, log=None):
        calls["store"].append({"root": root, "wp": wp, "rid": rid,
                               "dt": dt, "img": img})
        return store_return

    def db_fn(pool, **kw):
        calls["db"].append(kw)
        if db_raises:
            raise RuntimeError("db boom")
        return db_return

    def notify_fn(base, payload, **kw):
        calls["notify"].append(payload)

    def alert_fn(base, payload, **kw):
        calls["alert"].append(payload)

    h = ds.DetectionHandler(
        pool=None, logger=None, image_root="/tmp/root",
        web_service_url="http://x", threshold=threshold,
        dispatch=lambda fn: fn(), now_fn=lambda: FIXED,
        store_fn=store_fn, db_fn=db_fn, notify_fn=notify_fn, alert_fn=alert_fn)
    return h, calls


def _args(disease_percent, image_bytes=b""):
    return {"task_id": 7, "waypoint_id": 3, "robot_id": "dg_01",
            "ripe_percent": 80, "unripe_percent": 10, "rotten_percent": 5,
            "disease_percent": disease_percent, "image_bytes": image_bytes}


def test_below_threshold_skips_image_and_alert():
    h, calls = _make_handler()
    success, msg = h._process(**_args(4, image_bytes=b"jpeg"))
    assert success is True
    assert calls["store"] == []                # 이미지 저장 안 함
    assert calls["alert"] == []                # 알림 안 함
    assert len(calls["notify"]) == 1           # 중계는 항상
    assert calls["db"][0]["image_path"] is None
    assert calls["notify"][0]["detection_id"] == 42


def test_at_threshold_stores_and_alerts():
    h, calls = _make_handler(store_return="2026-07-09/wp3_dg_01_123456.jpg")
    success, msg = h._process(**_args(5, image_bytes=b"jpeg"))
    assert success is True
    assert len(calls["store"]) == 1
    assert calls["db"][0]["image_path"] == "2026-07-09/wp3_dg_01_123456.jpg"
    assert len(calls["alert"]) == 1
    assert calls["alert"][0]["image_path"] == "2026-07-09/wp3_dg_01_123456.jpg"
    assert len(calls["notify"]) == 1


def test_db_failure_still_notifies_and_alerts_and_success_false():
    h, calls = _make_handler(db_raises=True)
    success, msg = h._process(**_args(10, image_bytes=b"jpeg"))
    assert success is False                     # DB 실패 → success=false
    assert "DB 저장 실패" in msg
    assert len(calls["notify"]) == 1            # 그래도 notify 발송
    assert calls["notify"][0]["detection_id"] is None   # 실패라 null
    assert len(calls["alert"]) == 1            # 그래도 alert 발송(안전)


def test_image_write_failure_alert_gets_empty_path():
    # 게이트는 통과하지만 이미지 쓰기 실패(store_return=None) → alert image_path=""
    h, calls = _make_handler(store_return=None)
    success, msg = h._process(**_args(10, image_bytes=b"jpeg"))
    assert calls["db"][0]["image_path"] is None
    assert calls["alert"][0]["image_path"] == ""


def test_gate_but_no_image_bytes_skips_store_but_still_alerts():
    # disease>=5 인데 이미지 바이트가 안 옴 → 저장 시도 안 함, alert 는 발송(image_path="")
    h, calls = _make_handler()
    success, msg = h._process(**_args(10, image_bytes=b""))
    assert calls["store"] == []
    assert calls["db"][0]["image_path"] is None
    assert len(calls["alert"]) == 1
    assert calls["alert"][0]["image_path"] == ""


def test_detected_at_shared_across_db_notify_alert():
    h, calls = _make_handler()
    h._process(**_args(10, image_bytes=b"jpeg"))
    db_dt = calls["db"][0]["detected_at"]
    notify_dt = calls["notify"][0]["detected_at"]
    alert_dt = calls["alert"][0]["detected_at"]
    assert db_dt == FIXED                       # DB 엔 datetime 그대로
    assert notify_dt == FIXED.isoformat()       # payload 는 ISO 문자열
    assert alert_dt == FIXED.isoformat()
    # store 로 넘어간 시각도 동일(경로 시각 일관성)
    assert calls["store"][0]["dt"] == FIXED

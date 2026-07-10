#!/usr/bin/env python3
"""모의 Automato Control Service (ACS) — 보연님 실제 ACS 대역.

Sequence Diagram(2026-07 개정판) E1/E2/E3 '내부 API' 계약 구현.
상태ful(로봇 상태 추적) + 모든 요청 로깅 → App→Web→ACS 도달을 로그로 증명하고
시나리오1의 모든 경우의 수(가능/배터리부족/순찰중 · 자동/직접/거절 · disease<5 스킵/≥5 알림 · 완료)를 커버.

노출(Web→Control):  GET /internal/v1/robots/patrol/available   POST /internal/v1/tasks/patrol
콜백(Control→Web):  POST {WEB}/internal/v1/detections/notify · /alerts/disease · /patrol/completed

실행: WEB_SERVICE_URL=http://127.0.0.1:8899 PORT=7001 python3 mock_control_service.py
"""
import os
import time
import datetime
import threading

import requests
from flask import Flask, request, jsonify

app = Flask(__name__)
WEB_SERVICE_URL = os.environ.get("WEB_SERVICE_URL", "http://127.0.0.1:8899").rstrip("/")
MIN_BAT_PATROL = 70
_SEQ = [1024]
_LOCK = threading.Lock()

# 상태ful 로봇: dg_01 대기(가능) / dg_02 배터리부족 / dg_03 순찰중
_ROBOTS = {
    "dg_01": {"robot_id": "dg_01", "status": "IDLE", "battery_percent": 85.2, "current_position": {"x": 3.21, "y": 1.05}},
    "dg_02": {"robot_id": "dg_02", "status": "IDLE", "battery_percent": 62.0, "current_position": {"x": 5.10, "y": 2.30}},
    "dg_03": {"robot_id": "dg_03", "status": "PATROLLING", "battery_percent": 78.0, "current_position": {"x": 1.50, "y": 4.00}},
}


def log(*a):
    print("[가상-ACS]", *a, flush=True)


def _now():
    return datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.") + "512Z"


def _avail(r):
    if r["status"] != "IDLE":
        return False, "ROBOT_BUSY"
    if r["battery_percent"] < MIN_BAT_PATROL:
        return False, "BATTERY_TOO_LOW"
    return True, None


@app.get("/internal/v1/robots/patrol/available")
def available():
    with _LOCK:
        out = []
        for r in _ROBOTS.values():
            av, reason = _avail(r)
            rr = dict(r)
            rr["available"] = av
            if reason:
                rr["unavailable_reason"] = reason
            out.append(rr)
    log("◀ Web: GET available →", [(o["robot_id"], "가능" if o["available"] else o.get("unavailable_reason")) for o in out])
    return jsonify({"requested_at": _now(), "min_battery_percent": MIN_BAT_PATROL, "robots": out})


@app.post("/internal/v1/tasks/patrol")
def tasks_patrol():
    data = request.get_json(force=True, silent=True) or {}
    sel = data.get("robot_selection", "auto")
    rid_req = data.get("robot_id")
    log("◀ Web: POST tasks/patrol  robot_selection=%s robot_id=%s" % (sel, rid_req))
    with _LOCK:
        avail = [r for r in _ROBOTS.values() if _avail(r)[0]]
        if not avail:
            log("  → 거절: NO_AVAILABLE_ROBOT (가능 로봇 없음)")
            return jsonify({"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                            "message": "요청 가능한 로봇이 없습니다."}), 409
        if sel == "auto":
            chosen = max(avail, key=lambda r: r["battery_percent"])
        else:
            chosen = next((r for r in avail if r["robot_id"] == rid_req), None)
            if not chosen:
                log("  → 거절: ROBOT_NOT_AVAILABLE (%s)" % rid_req)
                return jsonify({"status": "REJECTED", "reason": "ROBOT_NOT_AVAILABLE",
                                "message": "선택한 로봇을 지금 쓸 수 없습니다."}), 409
        _SEQ[0] += 1
        task_id = _SEQ[0]
        chosen["status"] = "PATROLLING"            # 상태 변경 → 이제 available 에서 순찰중으로 보임
    log("  → 접수: task_id=%s robot=%s (상태 PATROLLING)" % (task_id, chosen["robot_id"]))
    threading.Thread(target=_simulate_patrol, args=(task_id, chosen["robot_id"]), daemon=True).start()
    return jsonify({"task_id": task_id, "assigned_robot_id": chosen["robot_id"],
                    "status": "ACCEPTED", "message": "순찰 요청이 접수되었습니다."})


# waypoint별 AI 분석(퍼센트). wp2=disease 4%(스킵), wp3=disease 7%(알림) → 두 분기 모두 커버.
_PLAN = [
    {"ripe": 50, "unripe": 50, "rotten": 0, "disease": 0},
    {"ripe": 70, "unripe": 28, "rotten": 2, "disease": 0},
    {"ripe": 60, "unripe": 33, "rotten": 3, "disease": 4},   # 4% < 5 → 알림 없음(스킵)
    {"ripe": 40, "unripe": 45, "rotten": 8, "disease": 7},   # 7% >= 5 → disease_alert
]


def _simulate_patrol(task_id, robot_id):
    last = _PLAN[-1]
    for wp, p in enumerate(_PLAN):
        time.sleep(0.8)
        now = _now()
        _post("/internal/v1/detections/notify", {
            "task_id": task_id, "waypoint_id": wp, "robot_id": robot_id, "detection_id": 50000 + wp,
            "ripe_percent": p["ripe"], "unripe_percent": p["unripe"],
            "rotten_percent": p["rotten"], "disease_percent": p["disease"], "detected_at": now})
        log("  ▶ Web: notify wp%s disease=%s%%" % (wp, p["disease"]) + ("  → 병해충 알림 발동" if p["disease"] >= 5 else "  (5%미만 스킵)"))
        if p["disease"] >= 5:                            # E3 트리거는 ACS가 판단
            _post("/internal/v1/alerts/disease", {
                "task_id": task_id, "waypoint_id": wp, "robot_id": robot_id,
                "disease_percent": p["disease"],
                "image_path": "%s/wp%s_%s.jpg" % (now[:10], wp, robot_id),
                "detected_at": now})
    _post("/internal/v1/patrol/completed", {
        "task_id": task_id, "robot_id": robot_id, "completed_at": _now(),
        "summary": {"ripe_percent": last["ripe"], "unripe_percent": last["unripe"],
                    "rotten_percent": last["rotten"], "disease_percent": last["disease"]}})
    with _LOCK:
        _ROBOTS[robot_id]["status"] = "IDLE"             # 순찰 끝 → 대기 복귀
    log("  ▶ Web: patrol_completed task_id=%s (%s 대기복귀)" % (task_id, robot_id))


def _post(path, body):
    try:
        requests.post(WEB_SERVICE_URL + path, json=body, timeout=4)
    except Exception as e:   # noqa: BLE001
        log("콜백 실패 %s: %s" % (path, e))


@app.post("/reset")
def reset():
    with _LOCK:
        _ROBOTS["dg_01"]["status"] = "IDLE"
        _ROBOTS["dg_03"]["status"] = "PATROLLING"
    log("상태 초기화")
    return jsonify({"ok": True})


@app.get("/healthz")
def healthz():
    return jsonify({"ok": True, "role": "mock-automato-control-service", "web": WEB_SERVICE_URL})


if __name__ == "__main__":
    log("기동 · Web=%s · 로봇 dg_01(대기)/dg_02(배터리부족)/dg_03(순찰중)" % WEB_SERVICE_URL)
    app.run(host="127.0.0.1", port=int(os.environ.get("PORT", "7001")))

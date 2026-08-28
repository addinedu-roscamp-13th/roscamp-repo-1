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
import json
import time
import datetime
import threading

import requests
from flask import Flask, request, jsonify

app = Flask(__name__)
WEB_SERVICE_URL = os.environ.get("WEB_SERVICE_URL", "http://127.0.0.1:8899").rstrip("/")
MIN_BAT_PATROL = 70
MIN_BAT_HARVEST = 50
_SEQ = [1024]
_LOCK = threading.Lock()

# 상태ful 로봇 (역할 고정: dg_03=순찰·운반 전용, dg_01·dg_02=수확 전용)
#   operational_status: NORMAL 이 아니면(현장정지·점검) E0-5 unavailable_reason=IMMOBILIZED
_ROBOTS = {
    "dg_01": {"robot_id": "dg_01", "status": "IDLE", "battery_percent": 85.2, "current_position": {"x": 3.21, "y": 1.05}, "operational_status": "NORMAL", "role": "harvest"},
    "dg_02": {"robot_id": "dg_02", "status": "IDLE", "battery_percent": 62.0, "current_position": {"x": 5.10, "y": 2.30}, "operational_status": "NORMAL", "role": "harvest"},
    "dg_03": {"robot_id": "dg_03", "status": "IDLE", "battery_percent": 78.0, "current_position": {"x": 1.50, "y": 4.00}, "operational_status": "NORMAL", "role": "patrol"},
}

# 순찰/수확/이송 → E0-5 task_type. IDLE(작업 없음)이면 null.
_TASK_TYPE = {"PATROLLING": "PATROL", "HARVESTING": "HARVEST", "TRANSFERRING": "TRANSFER", "IDLE": None}


def log(*a):
    print("[가상-ACS]", *a, flush=True)


def _now():
    return datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.") + "512Z"


def _avail(r):
    # 단일 기준: _unavailable_reason 이 None 이면 배정 가능 (운영정지·충전·배터리·작업중 모두 반영)
    reason = _unavailable_reason(r)
    return (reason is None, reason)


# ============================================================================
#  E0-5) ACS → Web Service 텔레메트리 (WebSocket, ACS가 서버)
#    Endpoint: ws://<acs-host>:8000/ws/telemetry · 1Hz · 관리자용 축약본
#    각 로봇: task_type(PATROL/HARVEST/TRANSFER/null) + unavailable_reason(우선순위 enum)
#    ※ 실서비스에선 보연님 실제 ACS가 발행. 여기선 로컬 테스트용 대역.
# ============================================================================
def _unavailable_reason(r):
    """우선순위 순서대로 처음 걸리는 하나만.
       확정 순서(2026-07-29 팀 확인): IMMOBILIZED > ROBOT_BUSY > ROBOT_OFFLINE > CHARGING > BATTERY_TOO_LOW.
       ROBOT_OFFLINE(3순위)은 텔레메트리 미수신 판정이라 Web 이 자체판정 → 여기선 제외."""
    if r.get("operational_status", "NORMAL") != "NORMAL":
        return "IMMOBILIZED"                        # 1순위
    if r["status"] != "IDLE":
        return "ROBOT_BUSY"                         # 2순위
    if r.get("is_charging"):
        return "CHARGING"                           # 4순위 (현 단계에선 발생하지 않음)
    thr = MIN_BAT_HARVEST if r.get("role") == "harvest" else MIN_BAT_PATROL   # 역할별 배터리 기준
    if r["battery_percent"] < thr:
        return "BATTERY_TOO_LOW"                    # 5순위
    return None


def _telemetry_robots():
    with _LOCK:
        out = []
        for r in _ROBOTS.values():
            st = r["status"]
            reason = _unavailable_reason(r)
            out.append({
                "robot_id": r["robot_id"],
                "task_type": _TASK_TYPE.get(st, None),
                "nav_status": "IDLE" if st == "IDLE" else "NAVIGATING",
                "position": {"x": r["current_position"]["x"], "y": r["current_position"]["y"], "yaw": 0.0},
                "battery_percent": r["battery_percent"],
                "available": reason is None,
                "unavailable_reason": reason,
            })
        return out


_tel_app = Flask("acs_telemetry")
try:
    from flask_sock import Sock
    _tel_sock = Sock(_tel_app)

    @_tel_sock.route("/ws/telemetry")
    def ws_telemetry(ws):                       # noqa: ANN001
        log("🟢 Web 텔레메트리 WS 접속됨 → 1Hz 발행 시작")
        seq = 1000
        try:
            while True:
                seq += 1
                ws.send(json.dumps({"event": "telemetry", "seq": seq, "timestamp": _now(),
                                    "data": {"robots": _telemetry_robots()}}))
                time.sleep(1.0)
        except Exception as e:                  # noqa: BLE001 (연결 끊김)
            log("텔레메트리 WS 종료:", e)
except Exception as e:                          # noqa: BLE001
    log("⚠ flask-sock 없음 — 텔레메트리 WS 비활성:", e)


def _start_telemetry_server():
    threading.Thread(
        target=lambda: _tel_app.run(host="0.0.0.0", port=8000, debug=False, use_reloader=False, threaded=True),
        daemon=True).start()
    log("E0-5 텔레메트리 WS 서버 기동 → ws://0.0.0.0:8000/ws/telemetry (LAN 접근 가능)")


@app.get("/internal/v1/robots/patrol/available")
def available():
    with _LOCK:
        out = []
        for r in _ROBOTS.values():
            if r.get("role") != "patrol":   # 순찰은 dg_03(순찰·운반 로봇)만 배정
                continue
            av, reason = _avail(r)
            rr = dict(r)
            rr["available"] = av
            if reason:
                rr["unavailable_reason"] = reason
            out.append(rr)
    log("◀ Web: GET available →", [(o["robot_id"], "가능" if o["available"] else o.get("unavailable_reason")) for o in out])
    return jsonify({"requested_at": _now(), "min_battery_percent": MIN_BAT_PATROL, "robots": out})


# 실 ACS 는 robot_selection 을 pydantic Literal["auto","manual"] 로 받아 그 밖의 값이면 422 다
# (patrol_api.PatrolRequest / harvest_api.HarvestRequest). 모의도 똑같이 거절해야
# 'specific' 같은 스펙 밖 값이 여기서만 통과하고 실연동에서 터지는 일을 막는다.
def _reject_bad_selection(sel):
    if sel not in ("auto", "manual"):
        log("  → 거절: robot_selection=%r 은 스펙 밖(auto|manual)" % sel)
        return jsonify({"detail": [{"loc": ["body", "robot_selection"],
                                    "msg": "Input should be 'auto' or 'manual'",
                                    "type": "literal_error"}]}), 422
    return None


# 실 ACS 는 병해충 알림에 사진 실물을 base64 로 동봉한다(detection_service.build_alert_payload, PR #84).
# 모의도 같이 보내야 '사진이 실제로 뜨는지' 를 여기서 검증할 수 있다.
# 로봇 카메라가 없으므로 검출 프레임을 흉내낸 작은 JPEG 를 매번 생성한다.
def _fake_detection_jpeg(wp, robot_id, pct):
    try:
        from PIL import Image, ImageDraw
        import io
        im = Image.new("RGB", (320, 240), (34, 72, 40))
        d = ImageDraw.Draw(im)
        d.ellipse([70, 60, 250, 175], fill=(178, 46, 32))          # 병든 열매
        d.text((14, 205), "MOCK detection  WP%s  %s  disease %s%%" % (wp, robot_id, pct),
               fill=(255, 255, 255))
        buf = io.BytesIO(); im.save(buf, "JPEG", quality=85)
        return buf.getvalue()
    except Exception:                       # Pillow 없으면 최소 JPEG 헤더만
        return b"\xff\xd8\xff\xe0" + b"\x00" * 256 + b"\xff\xd9"


@app.post("/internal/v1/tasks/patrol")
def tasks_patrol():
    data = request.get_json(force=True, silent=True) or {}
    sel = data.get("robot_selection", "auto")
    rid_req = data.get("robot_id")
    log("◀ Web: POST tasks/patrol  robot_selection=%s robot_id=%s" % (sel, rid_req))
    _bad = _reject_bad_selection(sel)
    if _bad: return _bad
    with _LOCK:
        avail = [r for r in _ROBOTS.values() if r.get("role") == "patrol" and _avail(r)[0]]
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


# ── 수확: 수확차(dg_01·dg_02) 배정. 동시 여러 대 가능(1대 제한 없음). ──
def _harvest_avail(r):
    return r.get("role") == "harvest" and _unavailable_reason(r) is None


# ===== [시나리오2] 수확 상수 (Confluence 33784289) =====
MAX_CAPACITY = 7          # 수확품 바구니 만차 기준
MAX_ROUNDS = 5            # 촬영-수확 라운드 상한
_BATCH = [0]              # harvest_batches 대용 시퀀스

# 라운드별 검출 배치. (grade, 파지성공여부) — 3회 실패는 failed 로 집계되고 제외목록에 들어감.
_HARVEST_PLAN = [
    [("NORMAL", True), ("NORMAL", True), ("DISCARD", True), ("NORMAL", False)],   # 1라운드
    [("NORMAL", True), ("NORMAL", True), ("DISCARD", True)],                      # 2라운드
    [("NORMAL", True), ("NORMAL", True)],                                         # 3라운드
    [],                                                                            # 4라운드 → DEPLETED
]


def _simulate_harvest(task_id, robot_id, location="HARVEST_01"):
    """E2 이동·도킹 → E3 인식 → E4 파지루프 → E5 종료판정·예냉실 이송 → E6 하역·완료.
       실제 ACS가 보낼 콜백(harvest/progress · harvest/completed)을 그대로 재현한다."""
    normal = discard = failed = 0
    exit_reason = "DEPLETED"
    log("  ▶ [E2] %s 로 이동·ChArUco 도킹 (capture 전 구간 false)" % location)
    time.sleep(1.0)

    for rnd, batch in enumerate(_HARVEST_PLAN, start=1):
        if rnd > MAX_ROUNDS:                                  # E3-2
            exit_reason = "MAX_ROUNDS_EXCEEDED"
            break
        log("  ▶ [E3] 라운드%d 관측자세 복귀 → DetectTomatoes → 대상 %d개" % (rnd, len(batch)))
        if not batch:                                         # E3-8 : 필터 후 0개
            exit_reason = "DEPLETED"
            break
        for i, (grade, picked) in enumerate(batch):           # E4 루프 (토마토 1개 = 1사이클)
            time.sleep(0.35)
            if not picked:                                    # MAX_PICK_RETRY(3) 소진 → 제외목록
                failed += 1
            elif grade == "NORMAL":
                normal += 1
            else:
                discard += 1
            _post("/internal/v1/harvest/progress", {          # E4-3
                "task_id": task_id, "robot_id": robot_id, "round": rnd,
                "normal_count": normal, "discard_count": discard, "failed_count": failed,
                "remaining_in_round": len(batch) - i - 1, "reported_at": _now()})
            log("    · 토마토 %s %s → 수확품%d/폐기%d/실패%d" % (
                grade, "성공" if picked else "3회실패", normal, discard, failed))
            if normal >= MAX_CAPACITY:                        # E4-8 : 만차
                exit_reason = "FULL"
                break
        if exit_reason == "FULL":
            break
    else:
        exit_reason = "MAX_ROUNDS_EXCEEDED"                   # 계획을 다 돌았는데 종료조건 없음

    log("  ▶ [E5] 수확 종료 exit_reason=%s → harvest_batches 저장 → 예냉실 이송" % exit_reason)
    time.sleep(1.0)                                           # E5 예냉실 주행 + E6 도킹
    _BATCH[0] += 1
    _post("/internal/v1/harvest/completed", {                 # E6-3
        "task_id": task_id, "robot_id": robot_id, "batch_id": 330 + _BATCH[0],
        "normal_count": normal, "discard_count": discard, "failed_count": failed,
        "exit_reason": exit_reason, "completed_at": _now()})
    with _LOCK:
        if robot_id in _ROBOTS:
            _ROBOTS[robot_id]["status"] = "IDLE"              # 예냉실 도착 → 대기 복귀
    log("  ▶ [E6] 수확 완료 task=%s 수확품%d/폐기%d/실패%d exit=%s (%s 대기복귀)" % (
        task_id, normal, discard, failed, exit_reason, robot_id))


@app.post("/internal/v1/tasks/harvest")
def tasks_harvest():
    data = request.get_json(force=True, silent=True) or {}
    sel = data.get("robot_selection", "auto")
    rid_req = data.get("robot_id")
    loc = data.get("harvest_location") or "HARVEST_01"
    log("◀ Web: POST tasks/harvest  robot_selection=%s robot_id=%s location=%s" % (sel, rid_req, loc))
    _bad = _reject_bad_selection(sel)
    if _bad: return _bad
    with _LOCK:
        # E1-3-1 : 로봇팔 1대 = 동시 수확 1건 → 진행 중이면 409 HARVEST_IN_PROGRESS
        if any(r["status"] == "HARVESTING" for r in _ROBOTS.values()):
            log("  → 거절: HARVEST_IN_PROGRESS (이미 수확 진행 중)")
            return jsonify({"status": "REJECTED", "reason": "HARVEST_IN_PROGRESS",
                            "message": "이미 수확 작업이 진행 중입니다."}), 409
        if sel == "auto":
            avail = [r for r in _ROBOTS.values() if _harvest_avail(r)]
            if not avail:
                log("  → 거절: NO_AVAILABLE_ROBOT (수확 가능 로봇 없음)")
                return jsonify({"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                                "message": "요청 가능한 로봇이 없습니다."}), 409
            chosen = max(avail, key=lambda r: (r["battery_percent"], [-ord(c) for c in r["robot_id"]]))
        else:                                   # manual : 지정 로봇의 사유를 스펙 우선순위대로 반환
            chosen = next((r for r in _ROBOTS.values() if r["robot_id"] == rid_req), None)
            if chosen is None:
                log("  → 거절: NO_AVAILABLE_ROBOT (%s 없음)" % rid_req)
                return jsonify({"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                                "message": "요청 가능한 로봇이 없습니다."}), 409
            if chosen.get("role") != "harvest":  # dg_03(순찰)은 로봇팔이 없다 — 가용해도 수확 불가
                log("  → 거절: NO_ARM (%s 는 순찰 로봇)" % rid_req)
                return jsonify({"status": "REJECTED", "reason": "NO_ARM",
                                "message": "지정한 로봇은 로봇팔이 없어 수확할 수 없습니다."}), 409
            why = _unavailable_reason(chosen)   # IMMOBILIZED / ROBOT_BUSY / ROBOT_OFFLINE / BATTERY_TOO_LOW
            if why:
                log("  → 거절: %s (%s)" % (why, rid_req))
                return jsonify({"status": "REJECTED", "reason": why,
                                "message": "지정한 로봇을 지금 사용할 수 없습니다."}), 409
        _SEQ[0] += 1
        task_id = _SEQ[0]
        chosen["status"] = "HARVESTING"
    log("  → 수확 접수: task_id=%s robot=%s location=%s (상태 HARVESTING)" % (task_id, chosen["robot_id"], loc))
    threading.Thread(target=_simulate_harvest, args=(task_id, chosen["robot_id"], loc), daemon=True).start()
    return jsonify({"task_id": task_id, "assigned_robot_id": chosen["robot_id"],
                    "status": "ACCEPTED", "harvest_location": loc,
                    "message": "수확 요청이 접수되었습니다."})


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
            import base64 as _b64
            _img = _fake_detection_jpeg(wp, robot_id, p["disease"])
            _post("/internal/v1/alerts/disease", {
                "task_id": task_id, "waypoint_id": wp, "robot_id": robot_id,
                "disease_percent": p["disease"],
                "image_path": "%s/wp%s_%s.jpg" % (now[:10], wp, robot_id),
                "image_data": _b64.b64encode(_img).decode("ascii"),   # 실 ACS 와 동일(PR #84)
                "detected_at": now})
            log("  → 병해충 알림 WP%s %s%% · 사진 %d bytes 동봉" % (wp, p["disease"], len(_img)))
    _post("/internal/v1/patrol/completed", {
        "task_id": task_id, "robot_id": robot_id,
        "status": "COMPLETED", "unvisited_waypoint_ids": [],    # 스펙 E2-9-1
        "completed_at": _now(),
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
        # 전체를 되돌린다. dg_02 가 빠져 있어서 수확 테스트를 두 번 돌리면 두 번째가
        # HARVEST_IN_PROGRESS 로 막혔고, 원인을 찾기 어려웠다(2026-07-30).
        for r in _ROBOTS.values():
            r["status"] = "IDLE"
    log("상태 초기화(전체 IDLE)")
    return jsonify({"ok": True})


@app.get("/healthz")
def healthz():
    return jsonify({"ok": True, "role": "mock-automato-control-service", "web": WEB_SERVICE_URL})


if __name__ == "__main__":
    log("기동 · Web=%s · 로봇 dg_01(대기)/dg_02(배터리부족)/dg_03(순찰중)" % WEB_SERVICE_URL)
    _start_telemetry_server()                   # E0-5 텔레메트리 WS(:8000)
    app.run(host="0.0.0.0", port=int(os.environ.get("PORT", "7001")))   # 0.0.0.0: LAN에서 접근 가능

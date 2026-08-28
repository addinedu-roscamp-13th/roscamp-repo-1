#!/usr/bin/env python3
"""원격 농장 에이전트 (Automato Control Service, edge 측) — 아웃바운드 폴링 방식.

농장은 공개 주소가 없어도(NAT 뒤) 이 프로그램이 공개 Web으로 '먼저 연결(폴링)'만 하면
① 브라우저가 낸 순찰 명령을 가져와 로봇에 실행하고 ② 검출/병해충/완료를 Web으로 되쏜다.
→ 전국·해외 어디 농장이든, 인바운드 포트개방/공개IP 없이 라이브 웹앱이 지휘 가능.

실행(로컬 Web 대상):   WEB_SERVICE_URL=http://127.0.0.1:8899 python3 farm_agent.py
실행(라이브 대상):     WEB_SERVICE_URL=https://geonsulee.pythonanywhere.com python3 farm_agent.py
내일 실기: 보연님 실제 ACS가 이 폴링 규약(POST /internal/v1/farm/poll)만 따르면 그대로 붙음.
"""
import os
import time
import random
import threading

import requests

WEB = os.environ.get("WEB_SERVICE_URL", "https://geonsulee.pythonanywhere.com").rstrip("/")
FARM_ID = os.environ.get("FARM_ID", "farm_gangneung_01")
POLL_SEC = float(os.environ.get("POLL_SEC", "0.5"))   # 0.5s 폴링 → 원거리 명령도 누른 순간 전달
MIN_BAT = 70

# 이 농장의 로봇들(실제로는 ROS2로 로컬 연결). dg_01 대기 / dg_02 대기 / dg_03 배터리부족
_ROBOTS = {
    "dg_01": {"robot_id": "dg_01", "status": "IDLE", "battery_percent": 85.2},
    "dg_02": {"robot_id": "dg_02", "status": "IDLE", "battery_percent": 74.0},
    "dg_03": {"robot_id": "dg_03", "status": "IDLE", "battery_percent": 62.0},
}
_LOCK = threading.Lock()

# waypoint별 AI 분석(퍼센트). wp2=disease 4%(스킵) / wp3=disease 7%(알림)
N_WAYPOINTS = 4          # 순찰 경로 지점 수(농장 배치 고정). 각 지점 검출값은 매번 랜덤.
DISEASE_CHANCE = 0.25    # 각 지점에서 병해충(>=5%) 나올 확률 — 실제처럼 '어디서 나올지 모름'

def _gen_waypoint():
    """한 지점의 AI 분석 결과를 랜덤 생성. 병해충은 대개 없음, 가끔 발견."""
    if random.random() < DISEASE_CHANCE:
        disease = random.randint(5, 15)      # 병해충 발견(알림 발동)
    else:
        disease = random.randint(0, 4)       # 정상(5%미만, 알림 없음)
    rotten = random.randint(0, 8)
    ripe = random.randint(30, 70)
    unripe = max(0, 100 - disease - rotten - ripe)
    return {"ripe": ripe, "unripe": unripe, "rotten": rotten, "disease": disease}


def log(*a):
    print("[농장-ACS]", *a, flush=True)


def _now():
    return time.strftime("%Y-%m-%dT%H:%M:%S.000Z", time.gmtime())


def _robots_report():
    with _LOCK:
        out = []
        for r in _ROBOTS.values():
            if r["status"] != "IDLE":
                av, reason = False, "ROBOT_BUSY"
            elif r["battery_percent"] < MIN_BAT:
                av, reason = False, "BATTERY_TOO_LOW"
            else:
                av, reason = True, None
            rr = dict(r); rr["robot_type"] = "pinky"; rr["available"] = av
            if reason:
                rr["unavailable_reason"] = reason
            out.append(rr)
    return out


def _post(path, body):
    try:
        requests.post(WEB + path, json=body, timeout=8)
    except Exception as e:   # noqa: BLE001
        log("콜백 실패 %s: %s" % (path, e))


def _run_patrol(task_id, robot_selection, robot_id):
    with _LOCK:
        avail = [r for r in _ROBOTS.values()
                 if r["status"] == "IDLE" and r["battery_percent"] >= MIN_BAT]
        if robot_selection == "auto":
            chosen = max(avail, key=lambda r: r["battery_percent"]) if avail else None
        else:
            chosen = next((r for r in avail if r["robot_id"] == robot_id), None)
        if not chosen:
            log("  → 순찰 불가(가능 로봇 없음/지정 불가) task_id=%s" % task_id)
            return
        chosen["status"] = "PATROLLING"
        rid = chosen["robot_id"]
    log("  ▶ 순찰 시작 task_id=%s robot=%s (선택=%s)" % (task_id, rid, robot_selection))
    plan = [_gen_waypoint() for _ in range(N_WAYPOINTS)]   # 매 순찰 새로 — 병해충 위치·유무 랜덤
    last = plan[-1]
    for wp, p in enumerate(plan):
        time.sleep(1.2)
        now = _now()
        _post("/internal/v1/detections/notify", {
            "task_id": task_id, "waypoint_id": wp, "robot_id": rid, "detection_id": 50000 + wp,
            "ripe_percent": p["ripe"], "unripe_percent": p["unripe"],
            "rotten_percent": p["rotten"], "disease_percent": p["disease"], "detected_at": now})
        log("    ▶ Web: 검출 wp%s disease=%s%%" % (wp, p["disease"])
            + ("  → 병해충 알림" if p["disease"] >= 5 else "  (5%미만 스킵)"))
        if p["disease"] >= 5:
            _post("/internal/v1/alerts/disease", {
                "task_id": task_id, "waypoint_id": wp, "robot_id": rid,
                "disease_percent": p["disease"],
                "image_path": "%s/wp%s_%s.jpg" % (now[:10], wp, rid), "detected_at": now})
    _post("/internal/v1/patrol/completed", {
        "task_id": task_id, "robot_id": rid, "completed_at": _now(),
        "summary": {"ripe_percent": last["ripe"], "unripe_percent": last["unripe"],
                    "rotten_percent": last["rotten"], "disease_percent": last["disease"]}})
    with _LOCK:
        _ROBOTS[rid]["status"] = "IDLE"
    log("  ▶ Web: 순찰 완료 task_id=%s (%s 대기복귀)" % (task_id, rid))


def _state_label(x):
    if x.get("available"):
        return "%s 배정가능(🔋%s%%)" % (x["robot_id"], int(x["battery_percent"]))
    reason = {"BATTERY_TOO_LOW": "배터리부족", "ROBOT_BUSY": "순찰중"}.get(
        x.get("unavailable_reason"), x.get("unavailable_reason"))
    return "%s %s(🔋%s%%)" % (x["robot_id"], reason, int(x["battery_percent"]))


def main():
    log("기동 · Web=%s · farm_id=%s · %ss 주기 아웃바운드 폴링" % (WEB, FARM_ID, POLL_SEC))
    log("→ 이 농장은 공개주소가 없어도 Web으로 먼저 연결합니다(인바운드 개방 불필요).")
    last_sig = None
    while True:
        rep = _robots_report()
        try:
            r = requests.post(WEB + "/internal/v1/farm/poll",
                              json={"farm_id": FARM_ID, "robots": rep}, timeout=10)
            resp = r.json() or {}
            cmd = resp.get("command")
            report_now = resp.get("report_now")     # 웹앱이 '순찰 보내기' 눌러 상태를 요청함
        except Exception as e:   # noqa: BLE001
            log("폴링 실패(재시도): %s" % e)
            time.sleep(POLL_SEC)
            continue
        # 상태 보고 로그 — 웹앱이 '순찰 보내기' 눌렀을 때(report_now) 또는 로봇 상태가 바뀌었을 때만 (평소엔 조용히)
        sig = ";".join("%s:%s:%.0f" % (x["robot_id"], x.get("available"), x["battery_percent"]) for x in rep)
        if report_now or sig != last_sig:
            tag = "(웹앱 '순찰 보내기' 요청)" if report_now else "(상태 변화)"
            log("▶ Web에 로봇상태 보고 %s: %s" % (tag, " · ".join(_state_label(x) for x in rep)))
            last_sig = sig
        if cmd and cmd.get("type") == "patrol":
            log("◀ Web에서 명령 수신: 순찰 task_id=%s" % cmd.get("task_id"))
            threading.Thread(target=_run_patrol,
                             args=(cmd.get("task_id"), cmd.get("robot_selection", "auto"),
                                   cmd.get("robot_id")), daemon=True).start()
        time.sleep(POLL_SEC)


if __name__ == "__main__":
    main()

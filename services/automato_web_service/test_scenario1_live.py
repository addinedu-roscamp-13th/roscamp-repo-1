#!/usr/bin/env python3
"""시나리오1 전 케이스 — 라이브(공개 URL) + 컨트롤(farm_agent) 도달 전수 검증.

브라우저 대신 API로 시나리오1의 모든 경우를 순서대로 쏘고,
각 명령이 '컨트롤(farm_agent)까지 실제 도달'했는지 farm_agent 로그로 확인한다.
"""
import os
import re
import sys
import time
import json
import subprocess
import urllib.request
import urllib.error

L = "https://geonsulee.pythonanywhere.com"
HERE = os.path.dirname(os.path.abspath(__file__))
results = []


def api(method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(L + path, data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read().decode())


def farm_log():
    try:
        return open("/tmp/scn1_farm.log", encoding="utf-8").read()
    except OSError:
        return ""


def check(name, ok, detail=""):
    results.append((name, ok, detail))
    print(("  ✅ " if ok else "  ❌ ") + name + ((" — " + detail) if detail else ""), flush=True)


def wait_farm_online(timeout=20):
    for _ in range(timeout * 2):
        s, j = api("GET", "/api/v1/farm/status")
        if s == 200 and j.get("farm_online"):
            return True
        time.sleep(0.5)
    return False


def wait_event(pred, timeout=15):
    for _ in range(timeout * 2):
        s, j = api("GET", "/api/v1/patrol/events?since=0")
        evs = j.get("events", []) if s == 200 else []
        if pred(evs):
            return evs
        time.sleep(0.5)
    return None


def main():
    print("=== farm_agent(컨트롤) 라이브 폴링 기동 ===", flush=True)
    env = dict(os.environ, WEB_SERVICE_URL=L, FARM_ID="scn1_test", POLL_SEC="0.5")
    farm = subprocess.Popen([sys.executable, os.path.join(HERE, "farm_agent.py")],
                            stdout=open("/tmp/scn1_farm.log", "w"), stderr=subprocess.STDOUT, env=env)
    try:
        check("농장(컨트롤) 라이브 접속", wait_farm_online(), "farm_online=true")
        api("POST", "/api/v1/patrol/reset")
        time.sleep(1)

        print("\n[E1-available] 순찰 가능 로봇 조회 (3상태)", flush=True)
        s, j = api("GET", "/api/v1/robots/patrol/available")
        rr = {r["robot_id"]: r for r in j.get("robots", [])}
        check("available 200 + 농장상태 반환", s == 200 and j.get("farm_online"),
              str([(k, "가능" if v.get("available") else v.get("unavailable_reason")) for k, v in rr.items()]))
        check("dg_01 가능", rr.get("dg_01", {}).get("available") is True)
        check("dg_03 배터리부족", rr.get("dg_03", {}).get("unavailable_reason") == "BATTERY_TOO_LOW")

        print("\n[E1-auto] 자동 배정 → ACCEPTED → 컨트롤 도달", flush=True)
        s, j = api("POST", "/api/v1/patrol/requests", {"robot_selection": "auto"})
        tid = j.get("task_id")
        check("auto ACCEPTED", s == 200 and j.get("status") == "ACCEPTED", "task_id=%s" % tid)
        time.sleep(2)
        check("★ 컨트롤(농장)이 명령 수신", ("task_id=%s" % tid) in farm_log(),
              "farm_agent 로그에 '명령 수신 task_id=%s'" % tid)

        print("\n[E2/E3] 검출 진행 → 병해충 → 완료 (컨트롤 콜백)", flush=True)
        evs = wait_event(lambda e: any(x["event"] == "patrol_completed" for x in evs_of(e, tid)), 20) or []
        mine = evs_of(evs, tid)
        wps = sorted({x.get("waypoint_id") for x in mine if x["event"] == "patrol_progress"})
        check("E2 진행 wp0~3", wps == [0, 1, 2, 3], "wps=%s" % wps)
        skip = [x for x in mine if x["event"] == "disease_alert" and x.get("waypoint_id") == 2]
        check("E2 disease<5 스킵(wp2 알림 없음)", len(skip) == 0)
        d3 = [x for x in mine if x["event"] == "disease_alert" and x.get("disease_percent", 0) >= 5]
        check("E3 병해충 알림(wp3 7%)", len(d3) >= 1)
        check("완료 patrol_completed", any(x["event"] == "patrol_completed" for x in mine))
        time.sleep(6)  # 로봇 대기복귀 반영

        print("\n[E1-specific] 직접 선택 dg_01 → ACCEPTED → 컨트롤 도달", flush=True)
        s, j = api("POST", "/api/v1/patrol/requests", {"robot_selection": "specific", "robot_id": "dg_01"})
        tid2 = j.get("task_id")
        check("specific ACCEPTED", s == 200 and j.get("status") == "ACCEPTED", "task_id=%s" % tid2)
        time.sleep(2)
        check("★ 컨트롤(농장)이 명령 수신", ("task_id=%s" % tid2) in farm_log())

        print("\n[E1-거절] 순찰 중 재요청 → 409 ALREADY_PATROLLING", flush=True)
        s, j = api("POST", "/api/v1/patrol/requests", {"robot_selection": "auto"})
        check("순찰 중 재요청 409", s == 409 and j.get("reason") == "ALREADY_PATROLLING", "reason=%s" % j.get("reason"))
        # 완료 대기
        wait_event(lambda e: any(x["event"] == "patrol_completed" for x in evs_of(e, tid2)), 20)
        time.sleep(6)

        print("\n[E1-거절] 배터리부족 로봇 지정 dg_03 → 409 ROBOT_NOT_AVAILABLE", flush=True)
        s, j = api("POST", "/api/v1/patrol/requests", {"robot_selection": "specific", "robot_id": "dg_03"})
        check("dg_03 지정 409", s == 409 and j.get("reason") == "ROBOT_NOT_AVAILABLE", "reason=%s" % j.get("reason"))

        api("POST", "/api/v1/patrol/reset")
    finally:
        farm.terminate()

    print("\n================ 결과 요약 ================", flush=True)
    passed = sum(1 for _, ok, _ in results if ok)
    for name, ok, _ in results:
        print(("✅" if ok else "❌") + " " + name)
    print("\n%d/%d 통과" % (passed, len(results)))
    print("=" * 44)


def evs_of(evs, tid):
    return [x for x in evs if x.get("task_id") == tid]


if __name__ == "__main__":
    main()

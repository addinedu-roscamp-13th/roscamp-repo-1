#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Automato Web Service 시나리오1+2 통합 전수검증.
  시나리오1(순찰) Confluence 23691289 · 시나리오2(수확) Confluence 33784289.
  라이브 app(8899) + mock ACS(7001) 대상. 반복 실행용: python3 validate_all.py <반복횟수>"""
import json, time, urllib.request, urllib.error, sys

import os
WEB = os.environ.get("AUTOMATO_URL", "http://127.0.0.1:8899")
FAILS, OKS = [], []
def ok(m): OKS.append(m)
def bad(m): FAILS.append(m); print("   ❌", m)

def GET(p):
    with urllib.request.urlopen(WEB+p, timeout=6) as r: return r.status, json.loads(r.read())
def _guard_telegram():
    """검증은 이벤트를 수백 건 만든다 → 실제 텔레그램 봇으로 알림이 쏟아진다
       (2026-07-29 실제 발생: 약 142건). 서버가 안전모드가 아니면 실행을 거부한다."""
    try:
        _, cfg = GET("/api/v1/_config")
    except Exception as e:
        print("⚠ 서버 설정 확인 실패(%s) — 서버가 떠 있는지 확인하세요." % e); sys.exit(2)
    if not cfg.get("telegram_disabled"):
        print("=" * 66)
        print("🛑 실행 거부 — 서버가 텔레그램 안전모드가 아닙니다.")
        print("   이대로 돌리면 실제 봇으로 알림이 수백 건 발송됩니다.")
        print("   서버를 이렇게 다시 띄우세요:")
        print("     TELEGRAM_DISABLED=1 PORT=8899 \\")
        print("     CONTROL_SERVICE_URL=http://127.0.0.1:7001 python3 -u app.py")
        print("=" * 66)
        sys.exit(2)
    print("✅ 텔레그램 안전모드 확인 — 실제 발송 없이 검증합니다.\n")

def POST(p, b):
    req=urllib.request.Request(WEB+p, data=json.dumps(b).encode(), headers={"Content-Type":"application/json"}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=6) as r: return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e: return e.code, json.loads(e.read())
def need(o, fs, ctx):
    for f in fs:
        if f in o: ok("%s.%s"%(ctx,f))
        else: bad("%s: '%s' 누락"%(ctx,f))
def enum_ok(v, allowed, ctx):
    if v in allowed: ok(ctx)
    else: bad("%s: 값 '%s' 스펙 enum 밖 %s"%(ctx, v, allowed))

def drain(base):
    evs, seen = [], {}
    t0=time.time()
    while time.time()-t0<14:
        _, ev = GET("/api/v1/patrol/events?since=%d"%base)
        for e in ev.get("events", []):
            if e.get("seq",0)>base: base=e["seq"]
            evs.append(e); seen.setdefault(e.get("event"), e)
        if "patrol_completed" in seen: break
        time.sleep(0.5)
    return seen

def run_once(it):
    print("\n===== 반복 %d ====="%it)
    # E0-5 텔레메트리
    _, tj = GET("/api/v1/telemetry")
    need(tj, ["event","connected","seq","robots"], "telemetry")
    for r in tj.get("robots", []):
        need(r, ["robot_id","task_type","nav_status","position","battery_percent","available","unavailable_reason"], "telemetry.robot")
        enum_ok(r.get("task_type"), [None,"PATROL","HARVEST","TRANSFER"], "telemetry.task_type[%s]"%r.get("robot_id"))
        enum_ok(r.get("unavailable_reason"), [None,"IMMOBILIZED","ROBOT_BUSY","ROBOT_OFFLINE","CHARGING","BATTERY_TOO_LOW"], "telemetry.unavailable_reason[%s]"%r.get("robot_id"))
    # E1-0 available
    _, aj = GET("/api/v1/robots/patrol/available")
    need(aj, ["requested_at","min_battery_percent","robots"], "available")
    if aj.get("min_battery_percent")==70: ok("min_bat=70")
    else: bad("available.min_battery_percent != 70 (%s)"%aj.get("min_battery_percent"))
    for r in aj.get("robots", []):
        need(r, ["robot_id","status","battery_percent","current_position","available"], "available.robot")

    # E1-2 auto 순찰요청 → E2/E3 이벤트
    _, ev0 = GET("/api/v1/patrol/events?since=0"); base=ev0.get("last_seq",0)
    st, pj = POST("/api/v1/patrol/requests", {"robot_selection":"auto","robot_id":None})
    if st==200:
        need(pj, ["task_id","assigned_robot_id","status","message"], "patrol200")
        if pj.get("status")=="ACCEPTED": ok("patrol ACCEPTED")
        else: bad("patrol status != ACCEPTED")
        seen = drain(base)
        if "patrol_progress" in seen:
            need(seen["patrol_progress"], ["event","task_id","waypoint_id","ripe_percent","unripe_percent","rotten_percent","disease_percent","detected_at"], "patrol_progress")
        else: bad("patrol_progress 미수신")
        if "disease_alert" in seen:
            need(seen["disease_alert"], ["event","task_id","waypoint_id","robot_id","disease_percent","image_path","detected_at"], "disease_alert")
            if seen["disease_alert"].get("disease_percent",0)>=5: ok("disease>=5 트리거정상")
            else: bad("disease_alert 인데 disease<5")
        else: bad("disease_alert 미수신(계획상 wp3 disease=7)")
        if "patrol_completed" in seen:
            need(seen["patrol_completed"], ["event","task_id","robot_id","status","unvisited_waypoint_ids","completed_at","summary"], "patrol_completed")
            enum_ok(seen["patrol_completed"].get("status"), ["COMPLETED","COMPLETED_PARTIAL"], "patrol_completed.status")
        else: bad("patrol_completed 미수신")
    else:
        # 이미 순찰 중이면 409 (순찰 1대 제한) — 정상
        if st==409 and pj.get("status")=="REJECTED": ok("순찰중 409 REJECTED 정상")
        else: bad("patrol auto 응답 이상 %s %s"%(st,pj))
        time.sleep(6)  # 진행중 순찰 끝나길 대기

    # 순찰 끝나고 로봇 IDLE 복귀 대기
    time.sleep(1)
    # 거절 케이스: manual 로 없는 로봇 지정 → 409
    st, rj = POST("/api/v1/patrol/requests", {"robot_selection":"manual","robot_id":"dg_99"})
    if st==409 and rj.get("status")=="REJECTED": ok("manual 잘못된로봇 409")
    else: bad("manual dg_99 거절 안됨 %s %s"%(st,rj))

    # COMPLETED_PARTIAL 통과 검증
    _, ev0 = GET("/api/v1/patrol/events?since=0"); b=ev0.get("last_seq",0)
    POST("/internal/v1/patrol/completed", {"task_id":9001,"robot_id":"dg_03","status":"COMPLETED_PARTIAL","unvisited_waypoint_ids":[7,9],"completed_at":"2026-07-21T10:00:00Z","summary":{"ripe_percent":48,"unripe_percent":52,"rotten_percent":0,"disease_percent":0}})
    time.sleep(0.8)
    _, ev = GET("/api/v1/patrol/events?since=%d"%b)
    pc = next((e for e in ev["events"] if e.get("event")=="patrol_completed"), None)
    if pc and pc.get("status")=="COMPLETED_PARTIAL" and pc.get("unvisited_waypoint_ids")==[7,9]: ok("COMPLETED_PARTIAL 통과")
    else: bad("COMPLETED_PARTIAL 통과 실패: %s"%pc)

    # task_failed 5종 reason 전부 통과
    for reason in ["BLOCKED","BLOCKED_UNRECOVERABLE","DOCK_FAILED","BATTERY_DEPLETED","HARDWARE_ERROR"]:
        _, ev0 = GET("/api/v1/patrol/events?since=0"); b=ev0.get("last_seq",0)
        POST("/internal/v1/alerts/task-failed", {"task_id":9100,"robot_id":"dg_03","task_type":"PATROL","reason":reason,"recovery_action":"RETURN_TO_CHARGER" if reason=="BLOCKED" else "NONE","message":"테스트 "+reason,"failed_at":"2026-07-21T10:00:00Z"})
        time.sleep(0.5)
        _, ev = GET("/api/v1/patrol/events?since=%d"%b)
        tf = next((e for e in ev["events"] if e.get("event")=="task_failed" and e.get("reason")==reason), None)
        if tf: need(tf, ["event","task_id","robot_id","task_type","reason","recovery_action","message","failed_at"], "task_failed[%s]"%reason)
        else: bad("task_failed reason=%s 미수신"%reason)



def run_scenario2(it):
    """[시나리오2 수확] Confluence 33784289 — E1 요청/거절 · E4 progress · E6 completed."""
    print("  --- 시나리오2 (수확) ---")
    # E1: 잘못된 harvest_location → 400 INVALID_HARVEST_LOCATION
    st, rj = POST("/api/v1/harvest/requests", {"robot_selection":"auto","harvest_location":"HARVEST_99"})
    if st == 400 and rj.get("reason") == "INVALID_HARVEST_LOCATION": ok("s2 잘못된 위치 400")
    else: bad("s2 잘못된 harvest_location 은 400/INVALID_HARVEST_LOCATION 이어야 함 (got %s/%s)" % (st, rj.get("reason")))

    # E1: 정상 수확 요청 → 200 ACCEPTED
    _, ev0 = GET("/api/v1/patrol/events?since=0"); base = ev0.get("last_seq", 0)
    st, hj = POST("/api/v1/harvest/requests", {"robot_selection":"auto","harvest_location":"HARVEST_01"})
    if st == 200 and hj.get("status") == "ACCEPTED":
        need(hj, ["task_id","assigned_robot_id","status","message"], "s2 harvest ACCEPTED")
        enum_ok(hj.get("harvest_location"), ["HARVEST_01","HARVEST_02",None], "s2 harvest_location 반환")
    else:
        bad("s2 수확 요청 실패 (got %s/%s)" % (st, hj.get("reason")))

    # E1-3-1: 수확 진행 중 재요청 → 409 HARVEST_IN_PROGRESS (로봇팔 1대 = 동시 1건)
    st2, rj2 = POST("/api/v1/harvest/requests", {"robot_selection":"auto","harvest_location":"HARVEST_02"})
    if st2 == 409 and rj2.get("reason") == "HARVEST_IN_PROGRESS": ok("s2 동시수확 409")
    else: bad("s2 수확 중 재요청은 409/HARVEST_IN_PROGRESS 여야 함 (got %s/%s)" % (st2, rj2.get("reason")))

    # E4 / E6: ACS 콜백이 App 이벤트로 흘러오는지
    prog, comp, seen = [], None, set()        # seen: seq 중복 제거(폴링마다 같은 이벤트가 다시 온다)
    for _ in range(40):                       # 최대 20초 대기
        time.sleep(0.5)
        _, ev = GET("/api/v1/patrol/events?since=%d" % base)
        for e in ev.get("events", []):
            sq = e.get("seq")
            if sq in seen: continue
            seen.add(sq)
            if e.get("event") == "harvest_progress": prog.append(e)
            if e.get("event") == "harvest_completed": comp = e
        if comp: break
    if prog:
        ok("s2 harvest_progress %d건" % len(prog))
        need(prog[-1], ["event","task_id","robot_id","round","normal_count",
                        "discard_count","failed_count","remaining_in_round","reported_at"], "s2 harvest_progress")
        n = [p.get("normal_count") for p in prog]
        if n == sorted(n): ok("s2 normal_count 단조증가")
        else: bad("s2 normal_count 가 감소함: %s" % n)
    else:
        bad("s2 harvest_progress 미수신 (ACS 콜백 → WS 푸시 경로 확인)")
    if comp:
        need(comp, ["event","task_id","robot_id","normal_count","discard_count",
                    "failed_count","exit_reason","completed_at"], "s2 harvest_completed")
        enum_ok(comp.get("exit_reason"), ["DEPLETED","FULL","MAX_ROUNDS_EXCEEDED"], "s2 exit_reason")
        if prog and comp.get("normal_count") == prog[-1].get("normal_count"): ok("s2 최종 카운트 일치")
        elif prog: bad("s2 completed 와 마지막 progress 의 normal_count 불일치")
    else:
        bad("s2 harvest_completed 미수신")

    # 내부 콜백 엔드포인트 직접 검증 (내일 보연님 ACS 가 쏠 바로 그 경로)
    _, ev0 = GET("/api/v1/patrol/events?since=0"); b2 = ev0.get("last_seq", 0)
    st3, _ = POST("/internal/v1/harvest/progress", {"task_id":9200,"robot_id":"dg_01","round":1,
                  "normal_count":3,"discard_count":1,"failed_count":0,"remaining_in_round":2,
                  "reported_at":"2026-07-29T10:00:00Z"})
    if st3 == 200: ok("s2 internal harvest/progress 200")
    else: bad("s2 /internal/v1/harvest/progress 가 200 이 아님 (got %s)" % st3)
    st4, _ = POST("/internal/v1/harvest/completed", {"task_id":9200,"robot_id":"dg_01","batch_id":999,
                  "normal_count":7,"discard_count":1,"failed_count":2,"exit_reason":"FULL",
                  "completed_at":"2026-07-29T10:05:00Z"})
    if st4 == 200: ok("s2 internal harvest/completed 200")
    else: bad("s2 /internal/v1/harvest/completed 가 200 이 아님 (got %s)" % st4)
    time.sleep(0.6)
    _, ev = GET("/api/v1/patrol/events?since=%d" % b2)
    got = [e.get("event") for e in ev.get("events", [])]
    for want in ("harvest_progress", "harvest_completed"):
        if want in got: ok("s2 %s 푸시" % want)
        else: bad("s2 %s 가 App 이벤트로 안 나감" % want)


N = int(sys.argv[1]) if len(sys.argv) > 1 else 3
_guard_telegram()        # 실봇 폭탄 방지 — 안전모드 아니면 여기서 중단
for i in range(1, N + 1):
    run_once(i)          # 시나리오1 (순찰)
    run_scenario2(i)     # 시나리오2 (수확)
    time.sleep(1)
print("\n" + "=" * 60)
print("총 검사 %d건 · 통과 %d · 실패 %d" % (len(OKS) + len(FAILS), len(OKS), len(FAILS)))
print("=" * 60)
if FAILS:
    print("실패:")
    for f in sorted(set(FAILS)): print("  -", f)
    sys.exit(1)
else:
    print("✅ 시나리오1+2 전 항목 통과 (반복 %d회)" % N)

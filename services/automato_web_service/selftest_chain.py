#!/usr/bin/env python3
"""이건수 담당분 자체 통신 테스트 (Sequence Diagram 시나리오1, 2026-07 개정판).

로컬에서 Web Service(app.py) + 모의 Control Service(mock_control_service.py)를 띄우고
아래 3개 링크를 자동 검증한다:
  1) Farm Admin App  ↔ Automato Web Service            (App-facing HTTP)
  2) Automato Web Service ↔ Automato Control Service   (내부 API 중계 + 콜백)
  3) App ↔ Web ↔ Control  전체 체인 (순찰요청→검출현황→병해충알림→완료)

실행: python3 selftest_chain.py
실제 보연님 ACS와 붙일 때는 이 스크립트 대신 app.py 에 CONTROL_SERVICE_URL=<보연ACS주소> 만 주면 됨.
"""
import os
import sys
import time
import subprocess
import signal

import requests

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = "http://127.0.0.1:7000"
CTRL = "http://127.0.0.1:7001"
PASS, FAIL = [], []


def check(name, cond, detail=""):
    (PASS if cond else FAIL).append(name)
    print(("  ✅ " if cond else "  ❌ ") + name + (("  — " + detail) if detail else ""), flush=True)
    return cond


def wait_up(url, tries=40):
    for _ in range(tries):
        try:
            requests.get(url, timeout=1)
            return True
        except Exception:
            time.sleep(0.25)
    return False


def main():
    env_web = dict(os.environ, PORT="7000", CONTROL_SERVICE_URL=CTRL, INGEST_TOKEN="automato-live-2026")
    env_ctrl = dict(os.environ, PORT="7001", WEB_SERVICE_URL=WEB)
    logs = open(os.path.join(HERE, "selftest_servers.log"), "w")
    web = subprocess.Popen([sys.executable, "app.py"], cwd=HERE, env=env_web, stdout=logs, stderr=logs)
    ctrl = subprocess.Popen([sys.executable, "mock_control_service.py"], cwd=HERE, env=env_ctrl, stdout=logs, stderr=logs)
    try:
        assert wait_up(WEB + "/api/v1/patrol/events"), "Web Service 안 뜸"
        assert wait_up(CTRL + "/healthz"), "Control(mock) 안 뜸"
        print("서버 기동 완료 (Web:7000, Control:7001)\n", flush=True)

        # ---------- 링크 1: Farm Admin App ↔ Automato Web Service ----------
        print("[링크 1] Farm Admin App ↔ Automato Web Service", flush=True)
        r = requests.get(WEB + "/api/v1/robots/patrol/available", timeout=5)
        j = r.json()
        check("GET /api/v1/robots/patrol/available 200", r.status_code == 200)
        check("robots 배열 반환", isinstance(j.get("robots"), list) and len(j["robots"]) >= 1,
              "robots=%d" % len(j.get("robots", [])))
        requests.post(WEB + "/api/v1/patrol/reset", timeout=5)
        r2 = requests.post(WEB + "/api/v1/patrol/requests", json={"robot_selection": "auto", "robot_id": None}, timeout=5)
        j2 = r2.json()
        check("POST /api/v1/patrol/requests → ACCEPTED", r2.status_code == 200 and j2.get("status") == "ACCEPTED",
              "task_id=%s robot=%s" % (j2.get("task_id"), j2.get("assigned_robot_id")))

        # ---------- E1 순찰요청 전 케이스 (시나리오1 순서) — 전부 Control(ACS)까지 도달 ----------
        print("\n[E1 순찰요청 전 케이스]", flush=True)
        def ctrl_reset():
            try: requests.post(CTRL + "/reset", timeout=3)
            except Exception: pass
            time.sleep(0.3)
        ctrl_reset()
        # 순찰은 역할상 dg_03 전용이다(mock 의 role 고정 + Web 의 PATROL_ROBOT_IDS).
        # 예전 이 블록은 dg_01·dg_02 가 순찰 목록에 나온다고 가정했는데, mock 에 역할이
        # 들어간 뒤(PR #25)로는 성립하지 않아 계속 실패하고 있었다. 역할 기준으로 고쳤다.
        avr = requests.get(WEB + "/api/v1/robots/patrol/available", timeout=5).json().get("robots", [])
        def rb(rid): return next((x for x in avr if x.get("robot_id") == rid), {})
        ids0 = [x.get("robot_id") for x in avr]
        check("E1 available: 순찰 후보는 dg_03 만 · 대기중이면 가능",
              ids0 == ["dg_03"] and rb("dg_03").get("available") is True, "ids=%s" % ids0)
        ctrl_reset()
        rs = requests.post(WEB + "/api/v1/patrol/requests", json={"robot_selection": "manual", "robot_id": "dg_03"}, timeout=5).json()
        check("E1 직접선택(manual dg_03) → ACCEPTED", rs.get("status") == "ACCEPTED" and rs.get("assigned_robot_id") == "dg_03",
              "robot=%s" % rs.get("assigned_robot_id"))
        # 순찰 중인 로봇을 다시 지정 → ACS 판단(ROBOT_BUSY)이 그대로 내려와야 한다
        busy = requests.get(WEB + "/api/v1/robots/patrol/available", timeout=5).json().get("robots", [])
        bz = next((x for x in busy if x.get("robot_id") == "dg_03"), {})
        check("ACS 판단(순찰중=ROBOT_BUSY) 그대로 전달",
              bz.get("available") is False and bz.get("unavailable_reason") == "ROBOT_BUSY",
              "dg_03.reason=%s" % bz.get("unavailable_reason"))
        ctrl_reset()
        rna = requests.post(WEB + "/api/v1/patrol/requests", json={"robot_selection": "manual", "robot_id": "dg_01"}, timeout=5)
        jna = rna.json()
        check("E1 거절: 순찰 자격 없는 로봇(dg_01=수확전용) 지정 → 409",
              rna.status_code == 409, "%s %s" % (rna.status_code, jna.get("reason")))
        ctrl_reset()
        requests.post(WEB + "/api/v1/patrol/requests", json={"robot_selection": "auto"}, timeout=5)      # dg_01 busy
        rno = requests.post(WEB + "/api/v1/patrol/requests", json={"robot_selection": "auto"}, timeout=5)  # 즉시 재요청 → 거절
        jno = rno.json()
        check("E1 거절: 가능 로봇 없음 → 409 NO_AVAILABLE_ROBOT",
              rno.status_code == 409 and jno.get("reason") == "NO_AVAILABLE_ROBOT", "%s %s" % (rno.status_code, jno.get("reason")))
        ctrl_reset()

        # ---------- 링크 2: Web Service ↔ Control Service (중계) ----------
        print("\n[링크 2] Automato Web Service ↔ Automato Control Service", flush=True)
        rc = requests.get(CTRL + "/internal/v1/robots/patrol/available", timeout=5)
        check("Control GET /internal/.../available 200", rc.status_code == 200)
        # Web 의 available 응답이 Control 것과 동일해야(중계 성공): dg_01 + current_position 은 Control만의 형식
        rw = requests.get(WEB + "/api/v1/robots/patrol/available", timeout=5).json()
        ids = [x.get("robot_id") for x in rw.get("robots", [])]
        # current_position 은 Control 응답에만 있는 형식 → 이게 보이면 중계된 것이다
        has_ctrl_shape = any(x.get("robot_id") == "dg_03" and "current_position" in x for x in rw.get("robots", []))
        check("Web available 가 Control 응답을 중계함(형식 보존)", has_ctrl_shape, "robot_ids=%s" % ids)
        # 수확 목록도 ACS 텔레메트리에서 나와야 한다(자체 데모 함대가 아니라)
        rh = requests.get(WEB + "/api/v1/robots/harvest/available", timeout=5).json()
        hids = sorted(x.get("robot_id") for x in rh.get("robots", []))
        check("수확 목록이 ACS 텔레메트리 기반(데모 폴백 아님)",
              rh.get("source", "").startswith("telemetry") and hids == ["dg_01", "dg_02"],
              "source=%s ids=%s" % (rh.get("source"), hids))

        # ---------- 링크 3: 전체 체인 (App→Web→Control→콜백→App) ----------
        print("\n[링크 3] App ↔ Web ↔ Control 전체 체인 (순찰 시나리오 E1→E2→E3)", flush=True)
        requests.post(WEB + "/api/v1/patrol/reset", timeout=5)   # 이벤트 큐 비움
        rr = requests.post(WEB + "/api/v1/patrol/requests", json={"robot_selection": "auto"}, timeout=5).json()
        task = rr.get("task_id")
        check("순찰요청 접수(중계)", rr.get("status") == "ACCEPTED", "task_id=%s" % task)
        # Control 시뮬이 콜백 보낼 시간 동안 이벤트 폴링(App 처럼). 이 순찰(task)의 이벤트만 집계.
        seen = []
        since = 0
        deadline = time.time() + 8
        while time.time() < deadline:
            ev = requests.get(WEB + "/api/v1/patrol/events", params={"since": since}, timeout=5).json()
            for e in ev["events"]:
                since = e["seq"]
                if e.get("task_id") == task:
                    seen.append(e)
            if any(e["event"] == "patrol_completed" for e in seen):
                break
            time.sleep(0.4)

        progress = [e for e in seen if e["event"] == "patrol_progress"]
        alerts = [e for e in seen if e["event"] == "disease_alert"]
        completed = [e for e in seen if e["event"] == "patrol_completed"]
        check("patrol_progress 수신(waypoint별)", len(progress) == 4, "%d건" % len(progress))
        check("검출 데이터가 '퍼센트' 필드(개정 스펙)",
              bool(progress) and all("ripe_percent" in e and "disease_percent" in e and "total_count" not in e for e in progress))
        check("disease_alert 발동 = disease_percent>=5 만(1건, 4%는 스킵)",
              len(alerts) == 1 and alerts[0].get("disease_percent", 0) >= 5,
              "alerts=%d disease=%s" % (len(alerts), alerts[0].get("disease_percent") if alerts else None))
        check("disease_alert 에 사진경로(image_path) 포함", bool(alerts) and bool(alerts[0].get("image_path")),
              alerts[0].get("image_path") if alerts else "")
        check("patrol_completed 수신 + summary 퍼센트",
              len(completed) == 1 and "ripe_percent" in (completed[0].get("summary") or {}),
              "완료 task_id=%s" % (completed[0].get("task_id") if completed else None))
        # 완료 후 로봇 IDLE 복귀(다음 순찰 가능)
        av = requests.get(WEB + "/api/v1/robots/patrol/available", timeout=5).json()
        check("완료 후 순찰 재요청 가능(로봇 대기복귀)", any(x.get("available") for x in av.get("robots", [])))

        # ---------- 링크 3+: WebSocket 실시간 채널 (스펙 /ws/farm-admin) ----------
        print("\n[링크 3+] WebSocket 실시간 채널 (/ws/farm-admin)", flush=True)
        try:
            import simple_websocket
            ws = simple_websocket.Client("ws://127.0.0.1:7000/ws/farm-admin")
            requests.post(WEB + "/internal/v1/detections/notify", json={
                "task_id": 55555, "waypoint_id": 1, "robot_id": "dg_02",
                "ripe_percent": 33, "unripe_percent": 60, "rotten_percent": 7, "disease_percent": 0,
                "detected_at": "2026-07-09T14:00:00Z"}, timeout=5)
            got = None
            end = time.time() + 4
            while time.time() < end:
                try:
                    msg = ws.receive(timeout=2)
                except Exception:
                    break
                if not msg:
                    continue
                import json as _j
                e = _j.loads(msg)
                if e.get("event") == "patrol_progress" and e.get("task_id") == 55555:
                    got = e
                    break
            ws.close()
            check("WebSocket 로 patrol_progress 실시간 수신", got is not None and got.get("ripe_percent") == 33,
                  "task_id=%s" % (got.get("task_id") if got else None))
        except Exception as e:
            check("WebSocket 실시간 수신", False, "예외: %s" % e)

        # ---------- 링크 3+: App-driven 텔레그램 릴레이 (스펙: 앱이 발송) ----------
        print("\n[링크 3+] App-driven 텔레그램 릴레이 (/api/v1/notify/telegram)", flush=True)
        requests.post(WEB + "/api/v1/telegram/config",
                      json={"bot_token": "TESTTOKEN", "chat_id": "123"}, timeout=5)
        tc = requests.get(WEB + "/api/v1/telegram/config", timeout=5).json()
        check("텔레그램 설정 저장·조회", tc.get("configured") is True)
        r1 = requests.post(WEB + "/api/v1/notify/telegram",
                           json={"event": "patrol_completed", "seq": 999001, "robot_id": "dg_01",
                                 "task_id": 1, "summary": {"ripe_percent": 50}}, timeout=8).json()
        check("App이 텔레그램 발송 요청(더미토큰이라 sent=false지만 호출됨)",
              "info" in r1 and r1.get("sent") is False, "info=%s" % r1.get("info"))
        r2 = requests.post(WEB + "/api/v1/notify/telegram",
                           json={"event": "patrol_completed", "seq": 999001}, timeout=5).json()
        check("같은 seq 재요청은 중복차단(다중 클라이언트 안전)", r2.get("reason") == "dup", str(r2))

        # ---------- [역할 분담] 순찰=dg_03 / 수확=dg_01(H01)·dg_02(H02) ----------
        # 위 링크1·2 는 중계 검사를 위해 PATROL_ROBOT_IDS 를 전체로 열어 뒀다.
        # 여기서는 운영 기본값(dg_03)으로 별도 프로세스를 띄워 역할이 실제로 강제되는지 본다.
        print("\n[역할 분담] 순찰=dg_03 · 수확=dg_01(HARVEST_01)/dg_02(HARVEST_02)", flush=True)
        env_role = dict(os.environ, PORT="7010", CONTROL_SERVICE_URL=CTRL,
                        INGEST_TOKEN="automato-live-2026")     # PATROL_ROBOT_IDS 기본값(dg_03) 사용
        role = subprocess.Popen([sys.executable, "app.py"], cwd=HERE, env=env_role,
                                stdout=logs, stderr=logs)
        W2 = "http://127.0.0.1:7010"
        try:
            assert wait_up(W2 + "/api/v1/_config"), "역할검증용 Web 안 뜸"
            cfg = requests.get(W2 + "/api/v1/_config", timeout=5).json()
            check("설정에 역할이 실려 있음",
                  cfg.get("patrol_robot_ids") == ["dg_03"]
                  and cfg.get("harvest_robot_ids") == ["dg_01", "dg_02"]
                  and cfg.get("harvest_bay_map") == {"dg_01": "HARVEST_01", "dg_02": "HARVEST_02"},
                  "patrol=%s harvest=%s" % (cfg.get("patrol_robot_ids"), cfg.get("harvest_robot_ids")))
            try: requests.post(CTRL + "/reset", timeout=3)
            except Exception: pass
            pv = requests.get(W2 + "/api/v1/robots/patrol/available", timeout=5).json()
            pids = [x.get("robot_id") for x in pv.get("robots", [])]
            check("순찰 목록은 dg_03 만", pids == ["dg_03"], "ids=%s" % pids)
            hv = requests.get(W2 + "/api/v1/robots/harvest/available", timeout=5).json()
            hids = sorted(x.get("robot_id") for x in hv.get("robots", []))
            check("수확 목록은 dg_01·dg_02 만", hids == ["dg_01", "dg_02"], "ids=%s" % hids)

            def hreq(body):
                r = requests.post(W2 + "/api/v1/harvest/requests", json=body, timeout=8)
                return r.status_code, r.json()
            for bay, want in (("HARVEST_01", "dg_01"), ("HARVEST_02", "dg_02")):
                try: requests.post(CTRL + "/reset", timeout=3)
                except Exception: pass
                sc, js = hreq({"robot_selection": "auto", "harvest_location": bay})
                check("%s 고르면 %s 자동 배정" % (bay, want),
                      js.get("status") == "ACCEPTED" and js.get("assigned_robot_id") == want
                      and js.get("harvest_location") == bay,
                      "%s robot=%s loc=%s" % (sc, js.get("assigned_robot_id"), js.get("harvest_location")))
            try: requests.post(CTRL + "/reset", timeout=3)
            except Exception: pass
            sc, js = hreq({"robot_selection": "manual", "robot_id": "dg_02"})
            check("dg_02 만 지정하면 HARVEST_02 자동", js.get("harvest_location") == "HARVEST_02",
                  "loc=%s" % js.get("harvest_location"))
            try: requests.post(CTRL + "/reset", timeout=3)
            except Exception: pass
            sc, js = hreq({"robot_selection": "manual", "robot_id": "dg_03"})
            check("dg_03 으로 수확 요청은 거절(로봇팔 없음)",
                  sc == 409 and js.get("status") == "REJECTED", "%s %s" % (sc, js.get("reason")))
            sc, js = hreq({"harvest_location": "HARVEST_09"})
            check("없는 수확대는 거절", sc == 400 and js.get("status") == "REJECTED",
                  "%s %s" % (sc, js.get("reason")))
        finally:
            try:
                role.send_signal(signal.SIGTERM); role.wait(timeout=5)
            except Exception:
                try: role.kill()
                except Exception: pass

    finally:
        try:
            os.remove(os.path.join(HERE, "telegram.json"))   # 더미 텔레그램 설정 정리
        except Exception:
            pass
        for p in (web, ctrl):
            try:
                p.send_signal(signal.SIGTERM)
                p.wait(timeout=5)
            except Exception:
                p.kill()
        logs.close()

    print("\n" + "=" * 56, flush=True)
    print("결과:  통과 %d  /  실패 %d" % (len(PASS), len(FAIL)), flush=True)
    if FAIL:
        print("실패 항목: " + ", ".join(FAIL), flush=True)
        sys.exit(1)
    print("🎉 이건수 담당 3개 링크 전부 통과 (개정 스펙 기준)", flush=True)


if __name__ == "__main__":
    main()

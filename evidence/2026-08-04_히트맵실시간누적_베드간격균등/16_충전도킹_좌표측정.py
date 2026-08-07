#!/usr/bin/env python3
"""dg_03 이 충전 도킹을 마친 **실제 좌표**를 잡는다.

왜 필요한가:
  GUI 의 DG 주차 칸을 어디에 그릴지 정하려면 '로봇이 도킹을 마치고 실제로 서는 자리'
  를 알아야 한다. 노드 22·23·24 는 충전소 '진입' 지점이라 도킹 완료 자리와 다를 수 있고,
  지금까지는 추정으로 그려서 로봇이 칸 앞에 서 보이는 문제가 있었다.

무엇을 판별하나:
  ① 배터리가 **오르기 시작**하면 충전 시작 → 그 시점의 좌표가 도킹 자리
  ② status/unavailable_reason 이 CHARGING 으로 바뀌어도 같은 판정
  ③ 좌표가 계속 (0,0) 이면 **위치추정이 안 붙은 것** — 측정 불가로 보고한다
     (로봇이 진짜 원점에 있는 게 아니라 AMCL/localization 이 안 도는 상태)

결과는 이 폴더에 JSON + 사람이 읽는 요약으로 남긴다. 화면 요약도 같이 출력한다.
"""
import json
import os
import time
import urllib.request

BASE = "http://127.0.0.1:7000"
OUT_DIR = os.path.dirname(os.path.abspath(__file__))
STAMP = time.strftime("%m%d_%H%M%S")
RAW = os.path.join(OUT_DIR, "17_도킹측정_원시_%s.jsonl" % STAMP)
SUM = os.path.join(OUT_DIR, "18_도킹측정_결과_%s.md" % STAMP)

# 씬 좌표 변환 — app.py 의 _map_to_norm 과 같은 식. GUI 박스를 놓을 좌표를 바로 얻는다.
HX, HZ = 6.5, 13.0


def get(path):
    with urllib.request.urlopen(BASE + path, timeout=5) as r:
        return json.load(r)


def map_extent():
    try:
        return get("/api/telemetry").get("map") or {}
    except Exception:
        return {}


def to_scene(x, y, m):
    """ROS 맵 좌표 → 3D 씬 좌표. FARM.stations 에 그대로 넣을 수 있는 값."""
    try:
        ox, oy = float(m["origin_x"]), float(m["origin_y"])
        w, h = float(m["width_m"]), float(m["height_m"])
    except (KeyError, TypeError, ValueError):
        return None
    nx = -(x - (ox + w / 2.0)) / (w / 2.0)
    nz = (y - (oy + h / 2.0)) / (h / 2.0)
    return round(max(-1, min(1, nx)) * HX, 2), round(max(-1, min(1, nz)) * HZ, 2)


def sample():
    """배터리·상태·좌표를 한 번 읽는다. patrol/available 이 셋을 다 준다."""
    d = get("/api/v1/robots/patrol/available")
    for r in d.get("robots", []):
        if r.get("robot_id") == "dg_03":
            p = r.get("current_position") or {}
            return {"t": time.strftime("%H:%M:%S"),
                    "batt": r.get("battery_percent"),
                    "status": r.get("status"),
                    "reason": r.get("unavailable_reason"),
                    "x": p.get("x"), "y": p.get("y")}
    return None


def main():
    m = map_extent()
    print("맵 범위: %s" % (m or "읽기 실패"))
    print("dg_03 충전 도킹 감시 시작 — 배터리가 오르기 시작하는 순간을 잡는다\n")
    print(" 시각      배터리   상태        좌표(ROS)            씬좌표")

    prev_batt = None
    rising = 0
    zero_run = 0
    hits = []
    f = open(RAW, "a", encoding="utf-8")

    for _ in range(3600):                       # 최대 1시간 (2초 간격)
        try:
            s = sample()
        except Exception as e:
            print("  조회 실패: %s" % e)
            time.sleep(2)
            continue
        if not s:
            time.sleep(2)
            continue

        sc = to_scene(s["x"], s["y"], m) if isinstance(s["x"], (int, float)) else None
        s["scene"] = sc
        f.write(json.dumps(s, ensure_ascii=False) + "\n")
        f.flush()

        at_origin = isinstance(s["x"], (int, float)) and abs(s["x"]) < 0.01 and abs(s["y"]) < 0.01
        zero_run = zero_run + 1 if at_origin else 0

        if prev_batt is not None and isinstance(s["batt"], (int, float)):
            if s["batt"] > prev_batt + 0.05:
                rising += 1
            elif s["batt"] < prev_batt - 0.05:
                rising = 0
        prev_batt = s["batt"] if isinstance(s["batt"], (int, float)) else prev_batt

        charging = (s["status"] == "CHARGING" or s["reason"] == "CHARGING" or rising >= 2)
        mark = "  ← 충전 중" if charging else ("  ← 원점(위치추정 없음?)" if at_origin else "")
        print(" %s  %6.2f%%  %-10s (%+.3f,%+.3f)   %s%s"
              % (s["t"], s["batt"] or 0, s["status"] or "-", s["x"] or 0, s["y"] or 0,
                 sc if sc else "-", mark))

        if charging and not at_origin and sc:
            hits.append(s)
            if len(hits) >= 5:                  # 5회 연속 안정되면 확정
                break
        time.sleep(2)

    f.close()
    write_summary(hits, zero_run, m)


def write_summary(hits, zero_run, m):
    lines = ["# dg_03 충전 도킹 좌표 측정 (%s)\n" % time.strftime("%Y-%m-%d %H:%M")]
    if hits:
        xs = [h["scene"][0] for h in hits]
        zs = [h["scene"][1] for h in hits]
        cx, cz = round(sum(xs) / len(xs), 2), round(sum(zs) / len(zs), 2)
        lines += ["## 결과 — 측정 성공\n",
                  "충전이 시작된 시점의 좌표 %d회 평균:\n" % len(hits),
                  "```",
                  "ROS 맵  x=%+.3f  y=%+.3f" % (hits[-1]["x"], hits[-1]["y"]),
                  "씬 좌표 x=%+.2f  z=%+.2f   ← FARM.stations 에 넣을 값" % (cx, cz),
                  "```\n",
                  "이 자리에 실물 크기(1.82 x 1.95) 박스를 놓으면 로봇이 칸 안에 들어온다.\n"]
        print("\n✅ 측정 성공 → 씬 좌표 x=%+.2f z=%+.2f" % (cx, cz))
    elif zero_run > 5:
        lines += ["## 결과 — 측정 불가 (좌표가 계속 원점)\n",
                  "dg_03 이 `(0,0)` 만 보고했다. 로봇이 진짜 원점에 있는 게 아니라",
                  "**위치추정(AMCL/localization)이 안 붙은 상태**로 보인다.\n",
                  "이 상태에서는 도킹 좌표를 잴 수 없고, 3D 지도의 로봇 위치도 원점에 고정된다.",
                  "로봇 쪽에서 위치추정을 띄운 뒤 다시 측정해야 한다.\n"]
        print("\n⚠ 좌표가 계속 원점 — 위치추정이 안 붙은 상태로 보인다")
    else:
        lines += ["## 결과 — 충전 시작을 관측하지 못함\n",
                  "감시 시간 안에 배터리가 오르거나 CHARGING 상태가 되지 않았다.\n"]
        print("\n⚠ 충전 시작을 관측하지 못함")
    lines += ["\n## 원시 기록\n", "`%s`\n" % os.path.basename(RAW),
              "\n## 왜 쟀나\n",
              "GUI 의 DG 주차 칸을 실제 도킹 자리에 그리기 위해서다. 노드 22·23·24 는",
              "충전소 '진입' 지점이라 도킹 완료 자리와 다를 수 있어, 추정으로 그린 탓에",
              "로봇이 칸 앞에 서 보이는 문제가 있었다.\n"]
    with open(SUM, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print("기록: %s" % SUM)


if __name__ == "__main__":
    main()

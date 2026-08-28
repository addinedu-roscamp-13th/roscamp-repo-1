#!/usr/bin/env python3
"""라이브(pythonanywhere) Web Service 로그를 내 터미널로 실시간 tail.

웹서비스가 메모리에 쌓아 즉시 내주는 /api/v1/weblog 를 0.7초마다 폴링 →
버튼 누르면 1초 안에 여기 뜬다. (pythonanywhere 서버로그 flush 지연 회피)

실행: python3 web_log_tail.py
"""
import os
import time
import json
import urllib.request

L = os.environ.get("WEB_SERVICE_URL", "https://geonsulee.pythonanywhere.com").rstrip("/")


def fetch(since):
    req = urllib.request.Request(L + "/api/v1/weblog?since=%d" % since)
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read().decode())


def main():
    print("──────────────────────────────────────────────", flush=True)
    print(" 🖥  라이브 Web Service 로그 (%s)" % L, flush=True)
    print("     버튼 누르면 여기에 [Web] ▶명령큐 / ◀농장폴링 / ◀콜백 이 흐릅니다", flush=True)
    print("──────────────────────────────────────────────", flush=True)
    # 시작 시점의 마지막 seq를 기준선으로 → 과거 로그 안 뿌리고 '지금부터'만
    try:
        since = fetch(0).get("last", 0)
    except Exception:   # noqa: BLE001
        since = 0
    print("… 준비 완료. 이제 버튼을 누르면 여기에 새 [Web] 로그가 뜹니다 …", flush=True)
    while True:
        try:
            d = fetch(since)
        except Exception as e:   # noqa: BLE001
            print("(조회 재시도: %s)" % e, flush=True)
            time.sleep(1.5)
            continue
        for ln in d.get("lines", []):
            print(ln["text"], flush=True)
            since = ln["seq"]
        time.sleep(0.7)


if __name__ == "__main__":
    main()

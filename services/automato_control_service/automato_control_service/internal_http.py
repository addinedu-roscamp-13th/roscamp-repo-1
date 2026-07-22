#!/usr/bin/env python3
"""내부 서비스 간 HTTP 발송 유틸 — ACS → Automato Web Service.

왜 stdlib urllib 인가: 형제 서비스(automato_web_service)와 같은 관례이고, 보내는 것이
내부망 JSON POST 뿐이라 requests/httpx 로 의존성을 늘릴 이유가 없다.

이 모듈은 **'보내는 방법'만** 안다. 무엇을 어디로 어떤 재시도 정책으로 보낼지는
호출부가 정한다(detection_service = waypoint 1건 단위, patrol_notify = task 1건 단위).
로그도 두지 않는다 — 순수 모듈이라 로깅은 호출부가 맥락과 함께 남긴다.
"""
import json
import urllib.request


def post_json(url: str, payload: dict, timeout: float) -> int:
    """JSON 을 POST 하고 HTTP 상태코드를 반환. 비2xx/네트워크 오류는 예외로 올린다.

    ensure_ascii=False: 한글 메시지가 \\uXXXX 로 부풀지 않게 그대로 UTF-8 로 보낸다.
    """
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        url, data=data,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return int(getattr(r, "status", 200))

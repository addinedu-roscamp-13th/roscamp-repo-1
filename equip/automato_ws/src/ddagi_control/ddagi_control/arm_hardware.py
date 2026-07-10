#!/usr/bin/env python3
"""pymycobot 실물 연결 싱글턴 — telemetry/controller 공용.

시리얼 포트는 프로세스당 단일 인스턴스만 열 수 있으므로, 헬스 조회
(telemetry_publisher)와 팔 제어(arm_controller)가 이 함수로 얻은
동일 객체를 공유해야 한다.
"""
from pymycobot import MyCobot280

PORT = "/dev/ttyUSB0"
BAUD = 1000000  # 실측 확인값 (115200 응답 없음, 1000000에서 정상 통신)

_arm_instance = None


def get_arm():
    global _arm_instance
    if _arm_instance is None:
        _arm_instance = MyCobot280(PORT, BAUD)
    return _arm_instance

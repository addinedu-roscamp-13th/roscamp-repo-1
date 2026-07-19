#!/usr/bin/env python3
"""텔레메트리 캐시 — 로봇별 '그 로봇의 전체 상태' 1건을 메모리에 보관(수신마다 덮어씀).

FleetTelemetry(1Hz) 콜백 스레드가 쓰고, FastAPI 스레드가 읽으므로 락으로 보호한다.
patrol_node(ROS 표면)에서 분리한 '순수 데이터 저장소' — ROS 노드를 참조하지 않아
단독으로 테스트할 수 있다.
"""
import copy
import threading

from automato_interfaces.msg import FleetTelemetry


class TelemetryCache:
    def __init__(self):
        self._lock = threading.Lock()   # 콜백 스레드(쓰기) ↔ API 스레드(읽기) 보호
        self._data = {}                 # robot_id -> entry dict

    def update_from_fleet(self, msg: FleetTelemetry, rx_wall: float) -> None:
        """FleetTelemetry 1건을 받아 로봇별로 병합 저장한다.

        같은 메시지가 ddagos/ddagis 두 배열을 함께 담고 있어 각각 순회하며 robot_id로 매칭.
        이번 메시지에 없는 로봇은 지우지 않고 이전 값을 유지 → ddago_stamp가 늙어
        staleness(미수신)로 자연히 드러난다. dg_03처럼 팔이 없으면 ddagi는 계속 None(정상).
        """
        with self._lock:
            for d in msg.ddagos:
                entry = self._data.setdefault(d.robot_id, {"robot_id": d.robot_id})
                entry["ddago"] = {
                    "nav_status": d.nav_status,
                    "is_charging": bool(d.is_charging),   # 스냅샷엔 담되 판정엔 안 씀
                    "task_id": int(d.task_id),
                    "x": float(d.x), "y": float(d.y), "yaw": float(d.yaw),
                    "battery_percent": float(d.battery_percent),
                    "battery_voltage": float(d.battery_voltage),
                    "us_range_m": float(d.us_range_m),
                }
                entry["ddago_stamp"] = (
                    d.header.stamp.sec + d.header.stamp.nanosec * 1e-9)
                entry["local_rx"] = rx_wall
            for a in msg.ddagis:
                entry = self._data.setdefault(a.robot_id, {"robot_id": a.robot_id})
                entry["ddagi"] = {
                    "is_paused": bool(a.is_paused),
                    "task_id": int(a.task_id),
                    "joint_angles": [float(v) for v in a.joint_angles],
                    "tcp_coords": [float(v) for v in a.tcp_coords],
                    "servo_health": [
                        {
                            "joint_no": int(s.joint_no),
                            "voltage_ok": bool(s.voltage_ok),
                            "temperature": int(s.temperature),
                            "current": float(s.current),
                            "overload": bool(s.overload),
                            "gripper_value": int(s.gripper_value),
                        }
                        for s in a.servo_health
                    ],
                }
                entry["ddagi_stamp"] = (
                    a.header.stamp.sec + a.header.stamp.nanosec * 1e-9)

    def get(self, robot_id: str):
        """가용 판정용 얕은 복사본(없으면 None)."""
        with self._lock:
            entry = self._data.get(robot_id)
            return dict(entry) if entry else None

    def snapshot(self, robot_id: str):
        """DB 스냅샷 저장용 깊은 복사본(JSON 직렬화 대상). 없으면 None."""
        with self._lock:
            entry = self._data.get(robot_id)
            return copy.deepcopy(entry) if entry else None

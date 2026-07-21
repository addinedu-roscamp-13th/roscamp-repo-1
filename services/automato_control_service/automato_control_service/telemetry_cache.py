#!/usr/bin/env python3
"""텔레메트리 캐시 — 로봇별 '그 로봇의 전체 상태' 1건을 메모리에 보관(수신마다 덮어씀).

구독 콜백 스레드가 쓰고, FastAPI 스레드가 읽으므로 락으로 보호한다.
patrol_node(ROS 표면)에서 분리한 '순수 데이터 저장소' — ROS 노드를 참조하지 않아
단독으로 테스트할 수 있다.

RP-114 로 입력이 로봇별 RobotTelemetry(/{robot_id}/telemetry)로 바뀌었다. 옛 경로
(FleetTelemetry 하나에 3대분)는 팀원의 DG 이전이 끝날 때까지 update_from_fleet() 으로
함께 받는다. 어느 쪽으로 들어오든 저장 형태(entry dict)는 같아서 판정 코드는 그대로다.
"""
import copy
import threading

from automato_interfaces.msg import FleetTelemetry, RobotTelemetry


def _stamp_sec(header) -> float:
    """ROS 시각(sec+nanosec)을 epoch 초(float)로.

    '3초 미수신' 판정의 기준이라 로봇이 찍은 원본 stamp 를 그대로 쓴다 — 중간 계층이
    다시 찍으면 죽은 로봇이 영영 신선해 보인다.
    """
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def _ddago_fields(d) -> dict:
    """DdagoTelemetry 를 판정·스냅샷용 순수 dict 로 (ROS 메시지 의존을 여기서 끊는다)."""
    return {
        "nav_status": d.nav_status,
        "is_charging": bool(d.is_charging),   # 스냅샷엔 담되 판정엔 안 씀
        "task_id": int(d.task_id),
        "x": float(d.x), "y": float(d.y), "yaw": float(d.yaw),
        "battery_percent": float(d.battery_percent),
        "battery_voltage": float(d.battery_voltage),
        "us_range_m": float(d.us_range_m),
    }


def _ddagi_fields(a) -> dict:
    """DdagiTelemetry 를 판정·스냅샷용 순수 dict 로."""
    return {
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


class TelemetryCache:
    def __init__(self):
        self._lock = threading.Lock()   # 콜백 스레드(쓰기) ↔ API 스레드(읽기) 보호
        self._data = {}                 # robot_id -> entry dict

    def update_from_robot(self, robot_id: str, msg: RobotTelemetry,
                          rx_wall: float) -> None:
        """RP-114 주 경로: 로봇 하나의 RobotTelemetry 를 병합 저장한다.

        어느 로봇인지는 토픽 네임스페이스가 말해주므로 robot_id 를 인자로 받는다
        (메시지 안에는 없다).

        ddagos/ddagis 는 길이 0 또는 1이다 — ROS2 메시지에 옵셔널 필드가 없어 '팔이 없음'을
        빈 배열로 표현하기 때문. 팔이 없는 세트(dg_03)는 ddagi 가 계속 None 으로 남는다(정상).
        이번 메시지에 없는 부분은 지우지 않고 이전 값을 유지 → stamp 가 늙어 미수신으로 드러난다.
        """
        with self._lock:
            entry = self._data.setdefault(robot_id, {"robot_id": robot_id})
            for d in msg.ddagos:
                entry["ddago"] = _ddago_fields(d)
                entry["ddago_stamp"] = _stamp_sec(d.header)
                entry["local_rx"] = rx_wall
            for a in msg.ddagis:
                entry["ddagi"] = _ddagi_fields(a)
                entry["ddagi_stamp"] = _stamp_sec(a.header)

    def update_from_fleet(self, msg: FleetTelemetry, rx_wall: float) -> None:
        """[삭제 예정] 옛 경로: FleetTelemetry 1건(로봇 3대분)을 로봇별로 병합 저장한다.

        옛 구조에는 네임스페이스가 없어 로봇 구분이 payload 의 robot_id 뿐이다.
        robot_id 가 빈 항목은 어느 로봇인지 알 수 없어 건너뛴다.
        팀원의 DG 이전이 끝나면 이 메서드를 제거한다.
        """
        with self._lock:
            for d in msg.ddagos:
                if not d.robot_id:
                    continue
                entry = self._data.setdefault(d.robot_id, {"robot_id": d.robot_id})
                entry["ddago"] = _ddago_fields(d)
                entry["ddago_stamp"] = _stamp_sec(d.header)
                entry["local_rx"] = rx_wall
            for a in msg.ddagis:
                if not a.robot_id:
                    continue
                entry = self._data.setdefault(a.robot_id, {"robot_id": a.robot_id})
                entry["ddagi"] = _ddagi_fields(a)
                entry["ddagi_stamp"] = _stamp_sec(a.header)

    def robot_ids(self) -> list:
        """지금까지 한 번이라도 텔레메트리를 받은 robot_id 목록(정렬).

        '어떤 로봇이 살아있나'를 밖에서 알려면 내부 dict 를 훑어야 하는데,
        그걸 외부에 시키면 락 없이 순회하게 된다(RuntimeError: dict changed size).
        그래서 목록 뽑기는 캐시가 직접 락 안에서 한다.
        """
        with self._lock:
            return sorted(self._data.keys())

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

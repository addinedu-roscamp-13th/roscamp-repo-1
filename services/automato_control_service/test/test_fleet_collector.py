#!/usr/bin/env python3
"""RP-114  fleet_collector 단위 테스트 — ROS 노드 없이 취합 로직만 검증.

fleet_collector 는 rclpy 를 import 하지 않는 순수 모듈이라 rclpy.init() 없이 돌릴 수 있다
(메시지 타입만 필요하므로 automato_interfaces 소싱은 있어야 한다).

실행 (TESTING.md 규약):
  source /opt/ros/jazzy/setup.bash
  source <automato_interfaces install>/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_fleet_collector.py -v
"""
import os
import sys

from automato_interfaces.msg import (
    DdagiTelemetry,
    DdagoTelemetry,
    FleetTelemetry,
    RobotTelemetry,
    ServoStatus,
)
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from automato_control_service.fleet_collector import (  # noqa: E402
    FleetCollector,
    robot_telemetry_topic,
)


# --------------------------------------------------------------------------- #
# 픽스처 헬퍼
# --------------------------------------------------------------------------- #
def _make_ddago(robot_id='', stamp_sec=100, battery=78.5, nav='NAVIGATING'):
    """로봇이 보내는 형태. robot_id 는 기본이 빈 문자열 — 물리망 분리 후 로봇은 채우지 않는다."""
    m = DdagoTelemetry()
    m.header.frame_id = 'map'
    m.header.stamp.sec = stamp_sec
    m.robot_id = robot_id
    m.task_id = 1024
    m.nav_status = nav
    m.is_charging = False
    m.x, m.y, m.yaw = 3.21, 1.05, 1.57
    m.battery_percent = battery
    m.battery_voltage = 12.1
    m.us_range_m = 0.42
    return m


def _make_ddagi(robot_id='', stamp_sec=100):
    m = DdagiTelemetry()
    m.header.frame_id = 'base_link'
    m.header.stamp.sec = stamp_sec
    m.robot_id = robot_id
    m.task_id = 1024
    m.is_paused = False
    m.joint_angles = [10.2, -30.5, 45.0, 0.0, -12.3, 5.5]
    m.tcp_coords = [160.0, 30.0, 200.0, 0.0, 0.0, 0.0]
    servos = []
    for j in range(7):
        s = ServoStatus()
        s.joint_no = j + 1
        s.voltage_ok = True
        s.temperature = 40 - j
        s.current = 0.2 + j * 0.01
        s.overload = False
        s.gripper_value = 85 if j == 6 else 0   # 7번(그리퍼)만 값 있음
        servos.append(s)
    m.servo_health = servos
    return m


def _make_robot_telemetry(with_arm=True, **kwargs):
    """DG 가 /{robot_id}/telemetry 로 올리는 형태(메시지 안에 robot_id 없음)."""
    m = RobotTelemetry()
    m.ddagos = [_make_ddago(**kwargs)]
    if with_arm:
        # ddagi 는 배터리·주행상태가 없으므로 공통 인자만 골라 넘긴다.
        arm_kwargs = {k: v for k, v in kwargs.items()
                      if k in ('robot_id', 'stamp_sec')}
        m.ddagis = [_make_ddagi(**arm_kwargs)]
    return m


def _stamp(sec):
    from builtin_interfaces.msg import Time
    t = Time()
    t.sec = sec
    return t


# --------------------------------------------------------------------------- #
# 토픽 이름
# --------------------------------------------------------------------------- #
def test_topic_name_uses_namespace():
    assert robot_telemetry_topic('dg_01') == '/dg_01/telemetry'


# --------------------------------------------------------------------------- #
# 캐시 기본 동작
# --------------------------------------------------------------------------- #
def test_update_and_snapshot_sorted():
    """robot_id 오름차순으로 나온다 — QT 화면에서 로봇 순서가 매 프레임 바뀌지 않게."""
    c = FleetCollector()
    c.update('dg_02', _make_robot_telemetry())
    c.update('dg_01', _make_robot_telemetry())
    assert [rid for rid, _ in c.snapshot()] == ['dg_01', 'dg_02']
    assert c.robot_ids() == ['dg_01', 'dg_02']


def test_update_overwrites_same_robot():
    c = FleetCollector()
    c.update('dg_01', _make_robot_telemetry(battery=90.0))
    c.update('dg_01', _make_robot_telemetry(battery=40.0))
    assert len(c.snapshot()) == 1
    assert c.get('dg_01').ddagos[0].battery_percent == pytest.approx(40.0)


def test_disconnected_robot_is_not_dropped():
    """끊긴 로봇도 배열에서 빼지 않는다(문서 E0 ④ 발행 규칙).

    빼버리면 QT 화면에서 로봇이 사라졌다 나타났다 깜빡이고, '연결 끊김'과
    '존재하지 않음'을 구분할 수 없다. 마지막 값이 남아 stamp 가 늙는 것으로 드러나야 한다.
    """
    c = FleetCollector()
    c.update('dg_01', _make_robot_telemetry(stamp_sec=100))
    c.update('dg_02', _make_robot_telemetry(stamp_sec=100))
    # dg_02 만 계속 갱신 — dg_01 은 끊긴 상황
    c.update('dg_02', _make_robot_telemetry(stamp_sec=200))

    ids = c.robot_ids()
    assert ids == ['dg_01', 'dg_02'], '끊긴 로봇이 배열에서 빠졌다'
    assert c.get('dg_01').ddagos[0].header.stamp.sec == 100, 'stamp 가 갱신돼 버렸다'


def test_arm_less_set_is_valid():
    """팔이 없는 세트(dg_03)는 ddagis 가 빈 배열 — 정상이다."""
    c = FleetCollector()
    c.update('dg_03', _make_robot_telemetry(with_arm=False))
    assert len(c.get('dg_03').ddagos) == 1
    assert len(c.get('dg_03').ddagis) == 0


# --------------------------------------------------------------------------- #
# 발행 메시지 조립
# --------------------------------------------------------------------------- #
def test_build_fleet_message_fills_robots():
    c = FleetCollector()
    c.update('dg_01', _make_robot_telemetry())
    c.update('dg_02', _make_robot_telemetry(with_arm=False))

    out = c.build_fleet_message(_stamp(555))

    assert out.header.stamp.sec == 555
    assert [m.robot_id for m in out.robots] == ['dg_01', 'dg_02']
    assert len(out.robots[0].telemetry.ddagos) == 1
    assert len(out.robots[0].telemetry.ddagis) == 1
    assert len(out.robots[1].telemetry.ddagis) == 0


def test_build_preserves_robot_stamp():
    """개별 로봇 stamp 는 원본 그대로 — 취합 시각으로 덮어쓰면 죽은 로봇이 신선해 보인다."""
    c = FleetCollector()
    c.update('dg_01', _make_robot_telemetry(stamp_sec=100))
    out = c.build_fleet_message(_stamp(999))
    assert out.robots[0].telemetry.ddagos[0].header.stamp.sec == 100


def test_build_backfills_robot_id_into_legacy_fields():
    """[삭제 예정] 하위호환의 핵심 — 옛 필드로 평탄화할 때 ACS 가 robot_id 를 채워 넣는다.

    로봇은 더 이상 robot_id 를 채우지 않으므로(네임스페이스로 대체) 그대로 흘리면
    옛 필드를 읽는 QT 가 로봇을 구분하지 못한다.
    """
    c = FleetCollector()
    c.update('dg_01', _make_robot_telemetry())          # robot_id 는 빈 문자열
    c.update('dg_02', _make_robot_telemetry(with_arm=False))

    out = c.build_fleet_message(_stamp(1))

    assert [d.robot_id for d in out.ddagos] == ['dg_01', 'dg_02']
    assert [a.robot_id for a in out.ddagis] == ['dg_01']
    # 옛 필드에도 원본 값이 손실 없이 실린다(QT 가 진단용으로 전부 읽는다).
    assert out.ddagos[0].nav_status == 'NAVIGATING'
    assert out.ddagis[0].servo_health[6].gripper_value == 85


def test_build_does_not_mutate_cached_original():
    """robot_id 를 채우는 건 복사본에만 — 캐시 원본을 건드리면 다음 프레임이 오염된다."""
    c = FleetCollector()
    c.update('dg_01', _make_robot_telemetry())
    c.build_fleet_message(_stamp(1))
    assert c.get('dg_01').ddagos[0].robot_id == '', '캐시 원본이 변조됐다'


def test_build_empty_when_nothing_received():
    c = FleetCollector()
    out = c.build_fleet_message(_stamp(1))
    assert out.robots == []
    assert out.ddagos == []


# --------------------------------------------------------------------------- #
# [삭제 예정] 옛 경로 폴백 — 팀원의 DG 이전 전까지 살아 있어야 하는 길
# --------------------------------------------------------------------------- #
def _make_legacy_fleet(ddagos, ddagis):
    fleet = FleetTelemetry()
    fleet.header.frame_id = 'automato'
    fleet.ddagos = ddagos
    fleet.ddagis = ddagis
    return fleet


def test_legacy_fleet_split_by_payload_robot_id():
    """옛 구조엔 네임스페이스가 없어 payload robot_id 로만 로봇을 가를 수 있다."""
    c = FleetCollector()
    skipped = c.update_from_legacy_fleet(_make_legacy_fleet(
        [_make_ddago('dg_01'), _make_ddago('dg_02')],
        [_make_ddagi('dg_01')],
    ))

    assert skipped == (0, 0)
    assert c.robot_ids() == ['dg_01', 'dg_02']
    assert len(c.get('dg_01').ddagos) == 1
    assert len(c.get('dg_01').ddagis) == 1
    assert len(c.get('dg_02').ddagis) == 0


def test_legacy_fleet_skips_empty_robot_id():
    """robot_id 가 빈 항목은 건너뛴다 — 이름 없는 유령 로봇이 가용 목록에 뜨면
    순찰이 존재하지 않는 로봇에 배차를 시도한다. 건너뛴 수는 호출부가 경고하도록 반환."""
    c = FleetCollector()
    skipped = c.update_from_legacy_fleet(_make_legacy_fleet(
        [_make_ddago('dg_01'), _make_ddago('')],   # 두 번째는 robot_id 없음
        [_make_ddagi('')],
    ))

    assert skipped == (1, 1)
    assert c.robot_ids() == ['dg_01'], '빈 robot_id 가 캐시에 들어갔다'


def test_legacy_and_new_path_merge_into_same_cache():
    """DG 가 섞여 있어도(일부만 새 버전) 같은 캐시에 병합돼 한 배열로 발행된다."""
    c = FleetCollector()
    c.update('dg_01', _make_robot_telemetry())                       # 새 경로
    c.update_from_legacy_fleet(_make_legacy_fleet(                   # 옛 경로
        [_make_ddago('dg_02')], []))

    out = c.build_fleet_message(_stamp(1))
    assert [m.robot_id for m in out.robots] == ['dg_01', 'dg_02']

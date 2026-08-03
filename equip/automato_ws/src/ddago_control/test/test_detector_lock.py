#!/usr/bin/env python3
"""반사마커 검출기의 목표 lock 추적(detector_node.MarkerDetector) 단위 테스트.

로봇·라이다 없이 검증한다. `/scan` 을 흘리는 대신 select_target 에 마커 후보를
직접 먹이고, odom 은 필드에 꽂아 원하는 좌표계 상황을 만든다.

지키려는 것:
  * 획득: 정면 후보가 연속 N프레임 안정돼야 lock 이 잡힌다(첫 프레임 오lock 방지).
  * 게이트: lock 에서 먼 마커(옆 충전소)는 채택하지 않는다.
  * **TTL**: 목표를 오래 못 보면 lock 이 풀린다 — 이게 없으면 순찰 중 잡은 lock 이
    도킹까지 살아남고, 그 사이 odom 이 드리프트해 게이트를 영영 못 넘는다.
    (2026-08-02 도킹 실패 재현: lock 은 odom 좌표라 좌표계가 밀리면 마커가 눈앞에
     있어도 '내 목표가 아니다'로 걸러져 /docking_marker_pose 가 끊긴다.)
  * 도킹 중에는 TTL 이 안 터진다(계속 채택되므로 기준 시각이 갱신된다).

실행:
  source /opt/ros/jazzy/setup.bash
  cd equip/automato_ws && source install/setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/ddago_control/test/test_detector_lock.py -v
"""
import time

import pytest
import rclpy
from rclpy.parameter import Parameter

from ddago_control.reflective_dock import detector_node as D


@pytest.fixture
def det():
    """검출기 노드 하나. 로봇이 없어도 뜬다(구독·발행만 만들고 콜백은 안 돈다)."""
    rclpy.init()
    node = D.MarkerDetector()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _marker(x, y):
    """select_target 이 보는 최소 마커. 라이다 프레임 좌표만 있으면 된다."""
    return {"x": x, "y": y}


def _acquire(node, marker, frames=None):
    """정면 후보를 연속으로 먹여 lock 을 잡게 한다."""
    for _ in range(frames or D.LOCK_ACQUIRE_FRAMES):
        node.select_target([marker])


# ------------------------------------------------------------------ 획득 --- #
def test_lock_needs_stable_frames(det):
    """한 프레임만으로는 lock 이 안 잡힌다(오lock 방지)."""
    det.odom = (0.0, 0.0, 0.0)
    det.select_target([_marker(-0.30, 0.0)])
    assert det.lock is None

    _acquire(det, _marker(-0.30, 0.0))
    assert det.lock is not None


def test_gate_rejects_far_marker(det):
    """lock 에서 게이트(12cm)보다 먼 마커는 채택하지 않는다 — 옆 충전소 차단."""
    det.odom = (0.0, 0.0, 0.0)
    _acquire(det, _marker(-0.30, 0.0))

    # 옆 충전소: 실측 간격 15cm > 게이트 12cm.
    assert det.select_target([_marker(-0.30, 0.15)]) is None


# -------------------------------------------------------------- TTL 회귀 --- #
def test_stale_lock_blocks_docking_without_ttl(det):
    """TTL 을 끄면(=옛 동작) odom 드리프트만으로 도킹이 영구 차단된다.

    이것이 2026-08-02 통합 실패의 재현이다. 로봇도 마커도 그대로인데 좌표계만
    밀렸을 뿐인데, 검출기가 '내 목표가 아니다'로 걸러 pose 발행이 끊긴다.
    """
    det.set_parameters([Parameter("lock_ttl_sec", value=0.0)])  # 무기한 유지
    det.odom = (0.0, 0.0, 0.0)
    m = _marker(-0.30, 0.0)
    _acquire(det, m)
    assert det.select_target([m]) is not None        # 지금은 잘 채택된다

    # 순찰 한 바퀴 — 로봇은 제자리로 돌아왔지만 odom 이 15cm 밀렸다.
    det.odom = (0.15, 0.0, 0.0)
    assert det.select_target([m]) is None            # 눈앞의 마커를 거부
    assert det.select_target([m]) is None            # 재시도해도 마찬가지(영구)


def test_ttl_releases_stale_lock(det):
    """TTL 이 켜져 있으면 목표를 오래 못 본 뒤 lock 이 풀린다."""
    det.odom = (0.0, 0.0, 0.0)
    m = _marker(-0.30, 0.0)
    _acquire(det, m)
    assert det.lock is not None

    ttl = float(det.get_parameter("lock_ttl_sec").value)
    det._expire_lock(time.monotonic() + ttl + 0.1)
    assert det.lock is None
    assert det.acq_count == 0        # 획득 상태도 같이 초기화된다


def test_ttl_recovers_after_drift(det):
    """lock 이 풀린 뒤에는 드리프트한 좌표계에서 다시 획득해 도킹이 살아난다."""
    det.odom = (0.0, 0.0, 0.0)
    m = _marker(-0.30, 0.0)
    _acquire(det, m)

    ttl = float(det.get_parameter("lock_ttl_sec").value)
    det._expire_lock(time.monotonic() + ttl + 0.1)   # 주행 중 자연 해제
    det.odom = (0.15, 0.0, 0.0)                      # 드리프트한 좌표계

    _acquire(det, m)                                 # 새 좌표로 재획득
    assert det.lock is not None
    assert det.select_target([m]) is not None        # 다시 발행된다


# ----------------------------------------------------------- 정지 게이트 --- #
def test_no_lock_while_moving(det):
    """주행 중에는 찜하지 않는다 — 그때의 '가장 정면'이 목표가 아닐 수 있다."""
    det.odom = (0.0, 0.0, 0.0)
    det.speed = 0.20                       # 주행 중(임계 0.03 m/s)
    _acquire(det, _marker(-0.30, 0.0), frames=D.LOCK_ACQUIRE_FRAMES * 3)
    assert det.lock is None

    # 잠정 발행은 계속된다(도킹 FSM 이 아직 돌지 않아 무해하고, 끊으면 진단이 어렵다).
    assert det.select_target([_marker(-0.30, 0.0)]) is not None


def test_lock_acquired_after_stop(det):
    """멈추면 그때 찜한다 — 도킹은 주행이 끝난 뒤 시작하므로 이 순서가 맞다."""
    det.odom = (0.0, 0.0, 0.0)
    m = _marker(-0.30, 0.0)

    det.speed = 0.20
    _acquire(det, m)
    assert det.lock is None

    det.speed = 0.0                        # 도착·정지
    _acquire(det, m)
    assert det.lock is not None


def test_rotation_also_blocks_acquire(det):
    """제자리 회전도 이동으로 본다(선속도는 0이어도 정면이 계속 바뀐다)."""
    det.odom = (0.0, 0.0, 0.0)
    det.speed = 0.0
    det.omega = 0.5                        # 회전 중(임계 0.10 rad/s)
    _acquire(det, _marker(-0.30, 0.0))
    assert det.lock is None


def test_gate_falls_back_when_stuck(det):
    """정지 판정이 오래 안 나오면 경고 후 그냥 획득한다(도킹 전면 차단 방지).

    odom 이 튀어 속도 추정이 늘 커 보이는 로봇에서도 도킹은 되어야 한다.
    """
    det.odom = (0.0, 0.0, 0.0)
    det.speed = 0.20
    wait = float(det.get_parameter("lock_acquire_wait_sec").value)
    det.acq_block_t = time.monotonic() - (wait + 1.0)   # 이미 오래 막혀 있었다

    _acquire(det, _marker(-0.30, 0.0))
    assert det.lock is not None
    assert det.acq_fallback_warned is True


def test_gate_can_be_disabled(det):
    """임계를 0 이하로 두면 게이트가 꺼져 옛 동작이 된다(탈출구)."""
    det.set_parameters([
        Parameter("lock_acquire_max_speed", value=0.0),
        Parameter("lock_acquire_max_omega", value=0.0),
    ])
    det.odom = (0.0, 0.0, 0.0)
    det.speed = 1.0
    _acquire(det, _marker(-0.30, 0.0))
    assert det.lock is not None


def test_ttl_does_not_expire_while_docking(det):
    """도킹 중에는 매 프레임 채택되므로 TTL 이 터지지 않는다."""
    det.odom = (0.0, 0.0, 0.0)
    m = _marker(-0.30, 0.0)
    _acquire(det, m)

    now = time.monotonic()
    ttl = float(det.get_parameter("lock_ttl_sec").value)
    for i in range(20):                  # 10Hz 로 2초간 계속 보임
        t = now + i * 0.1
        det.last_target_t = t            # on_scan 이 채택할 때마다 갱신하는 값
        det._expire_lock(t)
    assert det.lock is not None

    det._expire_lock(now + 20 * 0.1 + ttl + 0.1)   # 그 뒤 시야에서 사라지면
    assert det.lock is None


# ------------------------------------------------- 도킹 시작 알림 리셋 --- #
def test_dock_start_resets_lock(det):
    """도킹 시작 알림을 받으면 찜해 둔 목표를 버린다.

    TTL(시간 기반)만으로는 부족했다 — 복귀 주행 도중 잡은 lock 이 도킹 시작까지
    살아남을 수 있고, 움직이며 본 마커라 옆 충전소이거나 법선이 뒤집혀 있을 수 있다.
    (2026-08-03 실사고: 도킹 7초 전, 초속 5cm 로 회전 중에 정지 게이트가 폴백으로
     뚫려 lock 이 잡혔고, 그 값으로 사전정렬이 뒤집혔다.)
    도킹 시작은 **로봇이 확실히 멈춘 시점**이라 여기서 버리면 제대로 다시 잡는다.
    """
    det.odom = (0.0, 0.0, 0.0)
    _acquire(det, _marker(-0.30, 0.0))
    assert det.lock is not None

    det.on_lock_reset(None)
    assert det.lock is None
    # 딸린 상태도 함께 비워야 한다 — 하나라도 남으면 다음 획득이 옛 값에 끌려간다.
    assert det.last_target_t is None
    assert det.acq_world is None and det.acq_count == 0
    assert det.acq_block_t is None

    # 버린 뒤에는 멈춘 자리에서 새로 잡을 수 있다.
    _acquire(det, _marker(-0.30, 0.0))
    assert det.lock is not None


def test_dock_start_reset_is_safe_without_lock(det):
    """lock 이 없을 때 알림이 와도 예외 없이 넘어간다(도킹을 막지 않는다)."""
    det.on_lock_reset(None)
    assert det.lock is None

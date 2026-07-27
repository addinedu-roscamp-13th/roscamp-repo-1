#!/usr/bin/env python3
"""ChArUco 정밀 도킹 — Dock 액션을 내리고 재시도하며 결과를 기다린다.

RP-116(순찰 종료 후 충전소 복귀)이 PatrolDispatcher 안에 만든 도킹 로직을, 시나리오2
수확도 쓰게 되면서 밖으로 뺀 것이다. 도킹 지점은 셋이다:
  · 충전소 (E4 순찰 종료 복귀 / 22-1 막힘 실패 복귀)
  · 수확지 (S2 E2)
  · 예냉실 (S2 E5)
셋 다 charuco_boards 에 보드 한 장이 붙어 있고 절차가 같다 — 마커 탐색 → 중심선 정렬 →
전진 접근 → 180도 회전 → 후진 접붙임. 순찰 디스패처 안에 두면 수확이 순찰을 부르는
이상한 모양이 되고, 주행 엔진(RouteRunner)에 두면 '예약하며 이동한다'는 책임이 흐려진다.

클래스가 아니라 모듈 함수인 이유: 도킹은 들고 다닐 상태가 없다. 필요한 것(로거·액션
클라이언트·마커 정보·하트비트 대상)을 전부 인자로 받는다.

⚠️ 예약은 여기서 관리하지 않는다. 도킹은 수십 초 걸려 그동안 로봇이 쥔 자리 예약의
   TTL(RESERVATION_TTL_SEC, 기본 15초)이 만료되는데, 그걸 막는 하트비트만 대신 쳐 준다
   (heartbeat 인자). 무엇을 쥐고 언제 놓을지는 호출부(오케스트레이션)가 정한다.
"""
import threading
import time

from automato_interfaces.action import Dock

from automato_control_service.patrol_config import (
    DOCK_RESULT_TIMEOUT_SEC,
    DOCK_RETRY_MAX,
    GOAL_ACCEPT_TIMEOUT_SEC,
    HEARTBEAT_SEC,
    SERVER_WAIT_SEC,
)
from automato_control_service.route_runner import spin_wait


def dock(log, task_id, robot_id, task_point_id, marker, dock_client,
         heartbeat=None):
    """작업 지점 진입 노드에 도착한 로봇을 정밀 도킹시킨다. 실패 시 N_dock 회 재시도.

    task_point_id: 도킹 대상 지점('CHARGE_01' / 'HARVEST_01' / 'PRECOOL_01').
        Dock Goal 에 그대로 실린다 — 로봇이 어느 도크에 붙는지 로그로 남기기 위해서다.
    marker: get_dock_marker 결과 dict(마커번호·딕셔너리·칸 구성·크기·도킹 오프셋).
        None 이면 '도킹 불가'로 즉시 실패 반환한다 — 값도 없는 Goal 을 만들어 로봇이
        엉뚱하게 움직이지 않게 한다(마커 시드는 도킹 튜닝 후 채워진다).
    heartbeat=(engine, [cid...], robot_id): 도킹은 마커 탐색~후진까지 수십 초 걸려,
        결과를 기다리는 동안 로봇이 쥔 자리 예약 TTL 이 만료돼 회수되지 않게 갱신한다.
        예약 관리는 호출부(오케스트레이션)가 넘겨준다.
    반환: (success: bool, result_code, message).
      success=True  → 도킹 성공(다음: 예약 전부 해제, nav_status=IDLE).
      success=False → N_dock 소진/마커 없음/서버 미기동 → task_failed(DOCK_FAILED).
    """
    if marker is None:
        log.warn(
            f"지점 {task_point_id} 마커 미등록 → 도킹 불가 task={task_id}")
        return False, None, "마커 정보 없음(charuco_boards 미시드)"
    if not dock_client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
        log.warn(f"{robot_id} Dock 액션 서버 미기동 task={task_id}")
        return False, None, "Dock 액션 서버 미기동"

    goal = Dock.Goal()
    goal.task_id = int(task_id)
    goal.task_point_id = str(task_point_id)
    goal.marker_id = str(marker["marker_id"])
    goal.dictionary = str(marker["dictionary"])
    goal.squares_x = int(marker["squares_x"])
    goal.squares_y = int(marker["squares_y"])
    goal.square_size_m = float(marker["square_size_m"])
    goal.marker_size_m = float(marker["marker_size_m"])
    goal.dock_offset_x = float(marker["dock_offset_x"])
    goal.dock_offset_y = float(marker["dock_offset_y"])
    goal.dock_offset_yaw = float(marker["dock_offset_yaw"])

    last_code, last_msg = None, ""
    for attempt in range(1, DOCK_RETRY_MAX + 1):
        log.info(
            f"도킹 시도 {attempt}/{DOCK_RETRY_MAX} task={task_id} {robot_id} "
            f"@ {task_point_id}")
        code, msg = _send_dock_goal(log, dock_client, goal, task_id, heartbeat)
        last_code, last_msg = code, msg
        if code == 0:
            log.info(
                f"도킹 성공 task={task_id} {robot_id} @ {task_point_id}")
            return True, 0, msg
        log.warn(
            f"도킹 실패(code={code}) task={task_id} "
            f"시도 {attempt}/{DOCK_RETRY_MAX}: {msg}")
    return False, last_code, last_msg


def _send_dock_goal(log, dock_client, goal, task_id, heartbeat):
    """Dock Goal 을 한 번 하달하고 결과를 기다린다. 반환: (result_code, message).

    Goal 거부/수락 타임아웃은 (1, ...) 로 취급한다 — 마커 미검출(1)과 같은 '재시도
    가능' 등급으로 묶어 상위 재시도 루프가 다시 시도하게 한다.
    """
    def _fb(msg):
        """ROS executor 스레드 — 도킹 단계(phase) 등을 로그로만 남긴다."""
        try:
            fb = msg.feedback
            log.debug(
                f"도킹 진행 task={task_id} phase={fb.phase} "
                f"marker={fb.marker_detected} dist={fb.distance_to_marker_m:.2f}m")
        except Exception as exc:  # noqa: BLE001
            log.warn(f"도킹 피드백 처리 예외(무시): {exc}")

    goal_handle = spin_wait(
        dock_client.send_goal_async(goal, feedback_callback=_fb),
        GOAL_ACCEPT_TIMEOUT_SEC)
    if goal_handle is None or not goal_handle.accepted:
        log.warn(f"Dock Goal 거부/수락 타임아웃 task={task_id}")
        return 1, "Dock Goal 거부/수락 타임아웃"
    return _await_dock_result(log, goal_handle.get_result_async(), heartbeat)


def _await_dock_result(log, result_future, heartbeat):
    """도킹 결과 대기. 대기 중 HEARTBEAT_SEC 마다 쥔 자리 예약을 갱신한다.

    주행(RouteRunner._await_result)과 달리 룩어헤드가 없고 타임아웃이
    DOCK_RESULT_TIMEOUT_SEC 다. Dock Result 는 (result_code, message) 만 꺼낸다 —
    오차 축(final_*)은 로봇이 판정·기록하는 값이라 ACS 의 재시도 판정에는 result_code 로
    충분하다.
    반환: (result_code, message). 타임아웃/파싱 실패는 (1, ...).
    """
    done = threading.Event()
    result_future.add_done_callback(lambda _f: done.set())
    deadline = time.monotonic() + DOCK_RESULT_TIMEOUT_SEC
    while not done.wait(HEARTBEAT_SEC):
        if heartbeat is not None:
            engine, cids, robot_id = heartbeat
            for cid in cids:
                engine.heartbeat(cid, robot_id)
        if time.monotonic() >= deadline:
            log.warn("도킹 결과 대기 타임아웃 → 실패 취급")
            return 1, "도킹 결과 대기 타임아웃"
    try:
        res = result_future.result().result
        return int(res.result_code), str(res.message)
    except Exception:  # noqa: BLE001
        return 1, "도킹 결과 파싱 실패"

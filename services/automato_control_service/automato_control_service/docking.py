#!/usr/bin/env python3
"""정밀 후진 도킹 — 지점에 맞는 도킹 액션을 내리고 재시도하며 결과를 기다린다.

도킹 지점은 셋이고, 지점마다 붙은 마커 종류가 달라 절차·액션이 다르다:
  · 충전소   (E4 순찰 종료 복귀 / 22-1 막힘 실패 복귀) → 반사테이프  (ReflectiveDock)
  · 수확지   (S2 E2)                                   → 바닥 H 마커 (FloorDock)
  · 예냉실   (S2 E5)                                   → 바닥 H 마커 (FloorDock)
'어느 지점이 어느 방식인가'는 method_for() 한 곳에서만 정한다(단일 출처).

세 방식은 Result 구조가 같다(result_code 0 성공 / 1~4 실패 + message). 그래서 재시도·
하트비트·결과 판정 로직은 공유하고, 다른 것은 **어떤 Goal 을 어떤 액션으로 보내느냐**뿐이다
(_build_goal + _ACTION_SPEC). charuco(Dock) 방식 코드는 남겨 두되(휴면) 현재는 아무 지점도
가리키지 않는다 — charuco_boards 마커 시드가 걷힌 뒤에도 이 모듈은 그대로 동작한다.

클래스가 아니라 모듈 함수인 이유: 도킹은 들고 다닐 상태가 없다. 필요한 것(로거·액션
클라이언트·마커 정보·하트비트 대상)을 전부 인자로 받는다.

⚠️ 예약은 여기서 관리하지 않는다. 도킹은 수십 초 걸려 그동안 로봇이 쥔 자리 예약의
   TTL(RESERVATION_TTL_SEC, 기본 15초)이 만료되는데, 그걸 막는 하트비트만 대신 쳐 준다
   (heartbeat 인자). 무엇을 쥐고 언제 놓을지는 호출부(오케스트레이션)가 정한다.
"""
import math
import threading
import time

from automato_interfaces.action import Dock, FloorDock, ReflectiveDock

from automato_control_service.patrol_config import (
    DOCK_RESULT_TIMEOUT_SEC,
    DOCK_RETRY_MAX,
    GOAL_ACCEPT_TIMEOUT_SEC,
    HEARTBEAT_SEC,
    SERVER_WAIT_SEC,
)
from automato_control_service.route_runner import spin_wait


# 도킹 방식 3종.
#   charuco   : 전면 카메라로 ChArUco 보드(마커 규격 필요) — 휴면(현재 아무 지점도 안 씀)
#   floor     : 전면 카메라로 바닥 청색 H 마커(마커리스) — 수확지/예냉실
#   reflective: 2D 라이다로 재귀반사테이프 코너 마커(마커리스) — 충전소
METHOD_CHARUCO = "charuco"
METHOD_FLOOR = "floor"
METHOD_REFLECTIVE = "reflective"

# 방식 -> (액션 타입, 액션 이름 suffix). /{robot_id}/{suffix} 로 액션 클라이언트가 붙는다.
# 클라이언트 생성(action_spec)과 Goal 조립(_build_goal)이 이 표 하나에서 갈린다.
_ACTION_SPEC = {
    METHOD_CHARUCO: (Dock, "dock"),
    METHOD_FLOOR: (FloorDock, "floor_dock"),
    METHOD_REFLECTIVE: (ReflectiveDock, "reflective_dock"),
}


def method_for(task_point_id):
    """작업 지점 id -> 도킹 방식. task_point_id → 방식 매핑의 **유일한 출처**다.

    CHARGE_*  -> reflective (충전소 반사테이프)
    HARVEST_* -> floor      (수확지 바닥 H 마커)
    PRECOOL_* -> floor      (예냉실 바닥 H 마커)
    그 외     -> None       (도킹 방식 미정의 → 호출부가 '도킹 불가'로 처리)

    charuco 폴백은 두지 않는다: 방식이 안 잡히는 지점을 조용히 charuco 로 보내면
    charuco_boards 를 걷어낸 뒤 엉뚱하게 실패하거나 로봇이 헤맨다. 모르면 명시적으로 막는다.
    """
    tp = str(task_point_id).upper()
    if tp.startswith("CHARGE"):
        return METHOD_REFLECTIVE
    if tp.startswith("HARVEST") or tp.startswith("PRECOOL"):
        return METHOD_FLOOR
    return None


def action_spec(method):
    """도킹 방식 -> (액션 타입, suffix). 미정의 방식이면 None.

    노드가 이걸로 /{robot_id}/{suffix} 액션 클라이언트를 만든다(액션 타입만 알면 된다).
    """
    return _ACTION_SPEC.get(method)


def dock(log, task_id, robot_id, task_point_id, method, dock_client,
         marker=None, heartbeat=None):
    """작업 지점 진입 노드에 도착한 로봇을 정밀 도킹시킨다. 실패 시 N_dock 회 재시도.

    method: 'charuco' | 'floor' | 'reflective' (docking.method_for 로 고른다).
        방식마다 Goal 이 다르다 — charuco 만 마커 규격을 싣고, floor/reflective 는
        마커리스라 task_id·task_point_id 만 싣는다(스테이션별 미세값은 로봇 노드 기본값).
    task_point_id: 도킹 대상 지점('CHARGE_01' / 'HARVEST_01' / 'PRECOOL_01').
        Goal 에 실려 로봇이 어느 도크에 붙는지 로그로 남는다.
    marker: charuco 방식에서만 쓰는 보드 정보 dict(마커번호·칸 구성·오프셋).
        floor/reflective 는 무시한다(마커 정보 불필요). charuco 인데 None 이면 '도킹 불가'로
        즉시 실패한다 — 값 없는 Goal 로 로봇을 엉뚱하게 움직이지 않게 한다.
    heartbeat=(engine, [cid...], robot_id): 도킹은 수십 초 걸려, 결과 대기 중 쥔 자리
        예약 TTL 이 만료돼 회수되지 않게 갱신한다. 예약 관리는 호출부가 넘겨준다.
    반환: (success: bool, result_code, message).
      success=True  → 도킹 성공.
      success=False → 방식 미정의/마커 없음(charuco)/서버 미기동/N_dock 소진 → DOCK_FAILED.
    """
    if method not in _ACTION_SPEC:
        log.warn(
            f"지점 {task_point_id} 도킹 방식 미정의(method={method}) → 도킹 불가 "
            f"task={task_id}")
        return False, None, f"도킹 방식 미정의: {task_point_id}"
    if method == METHOD_CHARUCO and marker is None:
        log.warn(
            f"지점 {task_point_id} 마커 미등록 → charuco 도킹 불가 task={task_id}")
        return False, None, "마커 정보 없음(charuco_boards 미시드)"
    if not dock_client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
        log.warn(f"{robot_id} {method} 도킹 액션 서버 미기동 task={task_id}")
        return False, None, "Dock 액션 서버 미기동"

    goal = _build_goal(method, task_id, task_point_id, marker)

    last_code, last_msg = None, ""
    for attempt in range(1, DOCK_RETRY_MAX + 1):
        log.info(
            f"도킹 시도 {attempt}/{DOCK_RETRY_MAX} task={task_id} {robot_id} "
            f"@ {task_point_id} ({method})")
        code, msg, metrics = _send_dock_goal(log, dock_client, goal, task_id, heartbeat)
        last_code, last_msg = code, msg
        # 오차 요약은 성공·실패 양쪽에 붙인다. 실패(특히 code=2 정차 오차 초과)일 때야말로
        # 어느 축이 나빴는지가 진단의 전부다.
        detail = f" | {metrics}" if metrics else ""
        if code == 0:
            log.info(
                f"도킹 성공 task={task_id} {robot_id} @ {task_point_id} ({method})"
                f"{detail}")
            return True, 0, msg
        log.warn(
            f"도킹 실패(code={code}) task={task_id} "
            f"시도 {attempt}/{DOCK_RETRY_MAX}: {msg}{detail}")
    return False, last_code, last_msg


def _build_goal(method, task_id, task_point_id, marker):
    """방식에 맞는 Dock/FloorDock/ReflectiveDock Goal 을 조립한다.

    floor/reflective 의 정차 간격은 0 으로 둔다 — 0 은 '로봇 노드 기본값 사용'을 뜻해,
    스테이션별 미세값을 ACS/DB 가 아니라 로봇별 config yaml 이 갖게 한다(액션 정의의 규약).
    ACS 는 '어디에 붙일지(task_point_id)'만 알려주면 된다.
    """
    if method == METHOD_REFLECTIVE:
        goal = ReflectiveDock.Goal()
        goal.task_id = int(task_id)
        goal.task_point_id = str(task_point_id)
        goal.stop_gap_m = 0.0            # 0 → 로봇 노드 기본 stop_gap_m
        return goal
    if method == METHOD_FLOOR:
        goal = FloorDock.Goal()
        goal.task_id = int(task_id)
        goal.task_point_id = str(task_point_id)
        goal.wall_gap_m = 0.0            # 0 → 로봇 노드 기본 wall_gap_target
        goal.lateral_offset_m = 0.0      # 0 → 로봇 노드 기본 lateral_offset
        return goal
    # charuco (휴면) — 마커 규격을 실어 보낸다.
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
    return goal


# 오차 축 중 좌우(final_lateral_m)·스큐(final_yaw_error)는 세 액션이 이름을 공유하지만,
# 거리 축만 이름과 의미가 갈린다. 맞는 것 하나를 찾아 그 라벨로 표시한다.
_DIST_FIELDS = (
    ("final_wall_gap_m", "벽간격"),     # FloorDock      — 후면~벽
    ("final_gap_m", "마커간격"),        # ReflectiveDock — 뒤끝~마커
    ("final_error_m", "위치오차"),      # Dock(charuco)  — 목표 대비 총 오차
)


def _format_metrics(res):
    """도킹 Result 의 오차 축을 사람이 읽는 한 줄로. 남길 게 없으면 None.

    로봇 쪽 노드는 이 값을 로그로 남기지 않는다 — ACS 가 안 찍으면 **어디에도 안 남는다**.
    도킹 정확도(특히 좌우 이탈)는 한 회차만 봐선 편향인지 산포인지 못 가리므로 회차를
    모을 수 있게 성공·실패 양쪽에 찍는다.

    표시 전용이라 어떤 예외도 도킹을 막지 않게 통째로 감싼다(액션 정의가 바뀌어 필드가
    사라져도 도킹은 그대로 돌아야 한다).
    """
    try:
        parts = []
        for field, label in _DIST_FIELDS:
            val = getattr(res, field, None)
            if val is not None:
                parts.append(f"{label} {float(val) * 100:+.1f}cm")
                break
        lateral = getattr(res, "final_lateral_m", None)
        skew = getattr(res, "final_yaw_error", None)
        if lateral is None or skew is None:
            pass                      # 두 축이 없는 타입 — 거리 축만 남긴다
        elif float(lateral) == 0.0 and float(skew) == 0.0:
            # 로봇은 마커를 한 번도 못 봤거나 각이 신뢰 밖(|b|>90)이면 이 두 축을 아예
            # 안 채운다(0.0 그대로). '+0.0cm' 로 찍으면 '완벽 정렬' 과 구분이 안 돼
            # 실패 회차를 오독하게 되므로 미측정임을 명시한다. 실제 계산값(d·sin σ)이
            # 둘 다 정확히 0.0 으로 떨어지는 일은 사실상 없어 오탐 걱정은 없다.
            parts.append("좌우·스큐 미측정")
        else:
            parts.append(f"중심선이탈 {float(lateral) * 100:+.1f}cm")
            parts.append(f"스큐 {math.degrees(float(skew)):+.1f}°")
        return " / ".join(parts) if parts else None
    except Exception:  # noqa: BLE001
        return None


def _send_dock_goal(log, dock_client, goal, task_id, heartbeat):
    """Dock Goal 을 한 번 하달하고 결과를 기다린다. 반환: (result_code, message, metrics).

    metrics 는 오차 요약 문자열(없으면 None) — 로그 표시 전용이라 재시도 판정에는 쓰지
    않는다. 결과를 못 받은 경로(거부·타임아웃)에는 당연히 없다.

    Goal 거부/수락 타임아웃은 (1, ...) 로 취급한다 — 마커 미검출(1)과 같은 '재시도
    가능' 등급으로 묶어 상위 재시도 루프가 다시 시도하게 한다.
    """
    def _fb(msg):
        """ROS executor 스레드 — 도킹 단계(phase) 등을 로그로만 남긴다."""
        try:
            fb = msg.feedback
            # 방식마다 거리 필드 이름이 다르다: charuco/reflective 는 마커까지,
            # floor 는 벽까지. 둘 중 있는 값을 표시한다.
            dist = getattr(fb, "distance_to_marker_m", None)
            if dist is None:
                dist = getattr(fb, "distance_to_wall_m", 0.0)
            log.debug(
                f"도킹 진행 task={task_id} phase={fb.phase} "
                f"marker={fb.marker_detected} dist={dist:.2f}m")
        except Exception as exc:  # noqa: BLE001
            log.warn(f"도킹 피드백 처리 예외(무시): {exc}")

    goal_handle = spin_wait(
        dock_client.send_goal_async(goal, feedback_callback=_fb),
        GOAL_ACCEPT_TIMEOUT_SEC)
    if goal_handle is None or not goal_handle.accepted:
        log.warn(f"Dock Goal 거부/수락 타임아웃 task={task_id}")
        return 1, "Dock Goal 거부/수락 타임아웃", None
    return _await_dock_result(log, goal_handle.get_result_async(), heartbeat)


def _await_dock_result(log, result_future, heartbeat):
    """도킹 결과 대기. 대기 중 HEARTBEAT_SEC 마다 쥔 자리 예약을 갱신한다.

    주행(RouteRunner._await_result)과 달리 룩어헤드가 없고 타임아웃이
    DOCK_RESULT_TIMEOUT_SEC 다. **재시도 판정에는 result_code 만** 쓴다 — 오차 축(final_*)
    이 얼마든 성공은 성공이고, 어느 축이 나빴는지로 재시도를 가르지 않는다.
    다만 그 오차를 버리면 도킹 정확도가 아무 데도 안 남아(로봇도 안 찍는다) 튜닝할
    근거가 사라지므로, 판정에는 안 쓰되 로그용으로 함께 꺼낸다.
    반환: (result_code, message, metrics). 타임아웃/파싱 실패는 (1, ..., None).
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
            return 1, "도킹 결과 대기 타임아웃", None
    try:
        res = result_future.result().result
        return int(res.result_code), str(res.message), _format_metrics(res)
    except Exception:  # noqa: BLE001
        return 1, "도킹 결과 파싱 실패", None

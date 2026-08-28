#!/usr/bin/env python3
"""RP-79 순찰 task 단위 발송 — 순찰 종료(E2 9-1)와 작업 실패 알림(E2 13).

detection_service 가 'waypoint 1건'마다 보내는 것과 달리, 여기는 **task 하나가 끝났을 때
딱 한 번** 나가는 것들을 다룬다. Web Service 는 이 둘을 받아 Farm Admin App 의
WebSocket 이벤트(`patrol_completed` / `task_failed`)로 중계한다.

발송 정책이 서로 다르다 — 놓쳤을 때의 대가가 다르기 때문이다:
  · patrol_completed : fire-and-forget. 비200이어도 재시도하지 않고 로그만 남긴다(문서 명시).
                       순찰 결과는 DB(tasks)에 이미 남아 있어 화면을 새로 고치면 보인다.
  · task_failed      : 최대 3회 재시도. 놓치면 관리자가 로봇이 멈춰 선 것을 모르고,
                       DB 를 직접 들여다보기 전까지 아무도 현장에 나가지 않는다.

COMPLETED_PARTIAL 이어도 task_failed 는 **보내지 않는다.** 순찰은 끝났고 일부를 못 간
것이지 실패한 것이 아니다(문서 9-1 · 23번).

이 모듈은 ROS/DB 를 모른다 — payload 를 만들고 보내기만 한다. 그래서 로봇 없이
단위테스트할 수 있다.
"""
import time

from automato_control_service.internal_http import post_json

# 경로는 서비스 간 계약이라 상수로 고정한다(base URL 만 설정값).
PATROL_COMPLETED_PATH = "/internal/v1/patrol/completed"
TASK_FAILED_PATH = "/internal/v1/alerts/task-failed"

# 문서 13번 reason enum. 값 자체가 계약이라 오타를 코드 단계에서 잡는다.
FAIL_REASONS = ("BLOCKED", "BLOCKED_UNRECOVERABLE", "DOCK_FAILED",
                "BATTERY_DEPLETED", "HARDWARE_ERROR")
# 문서 13번 recovery_action enum.
RECOVERY_ACTIONS = ("RETURN_TO_CHARGER", "NONE")


# --------------------------------------------------------------------------- #
# payload 구성 (순수 함수 — 네트워크/시각 의존 없음)
# --------------------------------------------------------------------------- #
def build_completed_payload(*, task_id, robot_id, status,
                            unvisited_waypoint_ids, completed_at,
                            summary) -> dict:
    """순찰 종료 전달(9-1) 몸통.

    status: 'COMPLETED' | 'COMPLETED_PARTIAL' — tasks 에 마감한 값과 같아야 한다.
    unvisited_waypoint_ids: sweep 후에도 못 간 순찰 지점. COMPLETED 면 빈 배열.
    summary: 이번 task 탐지 기록의 평균치 4개(automato_db.get_detection_summary).
    """
    return {
        "task_id": int(task_id),
        "robot_id": robot_id,
        "status": status,
        "unvisited_waypoint_ids": [int(w) for w in unvisited_waypoint_ids],
        "completed_at": completed_at.isoformat(),
        "summary": dict(summary),
    }


def build_task_failed_payload(*, task_id, robot_id, reason, failed_at,
                              task_type="PATROL", recovery_action="NONE",
                              blocked_corridor=None, blocked_by_robot_id=None,
                              robot_position=None, waited_sec=None) -> dict:
    """작업 실패 알림(13) 몸통.

    막힘 관련 필드(blocked_corridor / blocked_by_robot_id / waited_sec)는 교통관제로
    실패했을 때만 값이 있다. 그 외 사유(HARDWARE_ERROR 등)에서는 null 로 나간다 —
    필드를 빼버리면 수신측이 'reason 마다 다른 스키마'를 다뤄야 해서 그냥 비워 보낸다.

    예외: enum 에 없는 reason/recovery_action 이면 ValueError.
    """
    if reason not in FAIL_REASONS:
        raise ValueError(f"허용되지 않은 reason: {reason}")
    if recovery_action not in RECOVERY_ACTIONS:
        raise ValueError(f"허용되지 않은 recovery_action: {recovery_action}")
    return {
        "task_id": int(task_id),
        "robot_id": robot_id,
        "task_type": task_type,
        "reason": reason,
        "blocked_corridor": blocked_corridor,
        "blocked_by_robot_id": blocked_by_robot_id,
        "robot_position": robot_position,
        "waited_sec": waited_sec,
        "recovery_action": recovery_action,
        "failed_at": failed_at.isoformat(),
    }


# --------------------------------------------------------------------------- #
# 발송
# --------------------------------------------------------------------------- #
def send_patrol_completed(base_url: str, payload: dict, timeout: float = 3.0,
                          log=None) -> bool:
    """순찰 종료를 1회 발송. 비200/예외여도 **재시도 없이** 로그만 남긴다(문서 정책)."""
    url = base_url.rstrip("/") + PATROL_COMPLETED_PATH
    try:
        status = post_json(url, payload, timeout)
        if log is not None:
            log.info(
                f"patrol_completed 발송 OK({status}) task={payload.get('task_id')} "
                f"status={payload.get('status')} "
                f"미방문={payload.get('unvisited_waypoint_ids')}")
        return True
    except Exception as exc:  # noqa: BLE001
        if log is not None:
            log.warn(f"patrol_completed 실패(재시도 안 함) {url}: {exc}")
        return False


def send_task_failed(base_url: str, payload: dict, timeout: float = 3.0,
                     retries: int = 3, retry_delay: float = 0.5,
                     log=None, sleep=time.sleep) -> bool:
    """작업 실패 알림을 최대 retries 회 시도. 재시도 사이 retry_delay 초 대기.

    호출부가 백그라운드 스레드로 던지므로 여기의 sleep 이 순찰 루프를 막지 않는다.
    """
    url = base_url.rstrip("/") + TASK_FAILED_PATH
    attempts = max(1, int(retries))
    last_err = None
    for i in range(1, attempts + 1):
        try:
            status = post_json(url, payload, timeout)
            if log is not None:
                log.info(
                    f"task_failed 발송 OK({status}) task={payload.get('task_id')} "
                    f"reason={payload.get('reason')} (시도 {i}/{attempts})")
            return True
        except Exception as exc:  # noqa: BLE001
            last_err = exc
            if i < attempts:
                sleep(retry_delay)
    if log is not None:
        log.error(f"task_failed 최종 실패({attempts}회) {url}: {last_err}")
    return False

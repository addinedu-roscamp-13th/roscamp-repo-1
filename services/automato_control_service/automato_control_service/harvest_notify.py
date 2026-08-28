#!/usr/bin/env python3
"""RP-123 시나리오2 — 수확 진행/완료 통지(ACS → Web Service).

E4 진행 상황(harvest_progress)과 E6 완료(harvest_completed)를 Web Service 로 보낸다.
Web Service 는 이를 Farm Admin App 의 WebSocket 이벤트로 중계한다.

둘 다 fire-and-forget 이다 — 놓쳐도 실적은 DB(tasks / harvest_batches)에 남아 있어
화면을 새로 고치면 보인다(순찰 patrol_completed 와 같은 정책). 반면 막힘·도킹 실패 같은
'실패 알림'은 시나리오1의 task_failed 규격을 그대로 재사용하므로(patrol_notify.send_task_failed)
여기 두지 않는다 — 그쪽은 놓치면 관리자가 로봇 정지를 모르므로 재시도가 필요하고, 이미 그
정책이 patrol_notify 에 있다.

이 모듈은 ROS/DB 를 모른다 — payload 를 만들고 보내기만 한다(로봇 없이 단위테스트 가능).
"""
from automato_control_service.internal_http import post_json

# 경로는 서비스 간 계약이라 상수로 고정한다(base URL 만 설정값).
HARVEST_PROGRESS_PATH = "/internal/v1/harvest/progress"
HARVEST_COMPLETED_PATH = "/internal/v1/harvest/completed"


# --------------------------------------------------------------------------- #
# payload 구성 (순수 함수 — 네트워크/시각 의존 없음)
# --------------------------------------------------------------------------- #
def build_progress_payload(*, task_id, robot_id, round, normal_count,
                           discard_count, failed_count, remaining_in_round,
                           reported_at) -> dict:
    """수확 진행 상황(E4) 몸통. Harvest 액션 Feedback 값을 그대로 옮긴다."""
    return {
        "task_id": int(task_id),
        "robot_id": robot_id,
        "round": int(round),
        "normal_count": int(normal_count),
        "discard_count": int(discard_count),
        "failed_count": int(failed_count),
        "remaining_in_round": int(remaining_in_round),
        "reported_at": reported_at.isoformat(),
    }


def build_completed_payload(*, task_id, robot_id, batch_id, normal_count,
                            discard_count, failed_count, exit_reason,
                            completed_at) -> dict:
    """수확 완료(E6) 몸통. harvest_batches 집계 + batch_id 를 싣는다."""
    return {
        "task_id": int(task_id),
        "robot_id": robot_id,
        "batch_id": int(batch_id),
        "normal_count": int(normal_count),
        "discard_count": int(discard_count),
        "failed_count": int(failed_count),
        "exit_reason": exit_reason,
        "completed_at": completed_at.isoformat(),
    }


# --------------------------------------------------------------------------- #
# 발송 (둘 다 fire-and-forget — 비200/예외여도 재시도 없이 로그만)
# --------------------------------------------------------------------------- #
def send_harvest_progress(base_url: str, payload: dict, timeout: float = 3.0,
                          log=None) -> bool:
    """수확 진행 상황을 1회 발송. 진행 보고는 잦고 놓쳐도 다음 보고가 덮으므로 재시도 안 함."""
    url = base_url.rstrip("/") + HARVEST_PROGRESS_PATH
    try:
        status = post_json(url, payload, timeout)
        if log is not None:
            log.info(
                f"harvest_progress 발송 OK({status}) task={payload.get('task_id')} "
                f"round={payload.get('round')} "
                f"N/D/F={payload.get('normal_count')}/{payload.get('discard_count')}"
                f"/{payload.get('failed_count')}")
        return True
    except Exception as exc:  # noqa: BLE001
        if log is not None:
            log.warn(f"harvest_progress 실패(재시도 안 함) {url}: {exc}")
        return False


def send_harvest_completed(base_url: str, payload: dict, timeout: float = 3.0,
                           log=None) -> bool:
    """수확 완료를 1회 발송. 결과는 DB(tasks/harvest_batches)에 이미 있어 재시도 안 함."""
    url = base_url.rstrip("/") + HARVEST_COMPLETED_PATH
    try:
        status = post_json(url, payload, timeout)
        if log is not None:
            log.info(
                f"harvest_completed 발송 OK({status}) task={payload.get('task_id')} "
                f"batch={payload.get('batch_id')} exit={payload.get('exit_reason')}")
        return True
    except Exception as exc:  # noqa: BLE001
        if log is not None:
            log.warn(f"harvest_completed 실패(재시도 안 함) {url}: {exc}")
        return False

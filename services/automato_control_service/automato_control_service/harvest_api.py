#!/usr/bin/env python3
"""RP-123 시나리오2 E1 — 수확 접수 HTTP 엔드포인트(POST /internal/v1/tasks/harvest).

순찰 접수(patrol_api.accept_patrol)와 같은 골격이다: 가용 판정 → 로봇 선정 → DB 접수 → 디스패치.
수확 고유분은 셋뿐:
  (1) 수확 위치(harvest_location) 검증 — task_points 에 있고 point_type=HARVEST 여야 한다.
  (2) 위치별 중복 차단 — 같은 위치에 진행 중 수확이 있으면 409 HARVEST_IN_PROGRESS.
  (3) 배터리 임계값을 HARVEST 기준(기본 50)으로 조회.

가용 판정/선정 순수 함수(judge_all·select_auto)는 patrol_api 것을 재사용한다.
patrol_api.create_app 이 이 모듈의 register(app, node, pool) 를 호출해 라우트를 얹는다
(traffic_debug.register 와 동일한 패턴 — 순찰·수확이 한 HTTP 서버를 공유).
"""
import json
import time
from datetime import datetime, timezone
from typing import Literal, Optional

from fastapi.responses import JSONResponse
from pydantic import BaseModel

from automato_control_service import automato_db
from automato_control_service.patrol_api import judge_all, select_auto


class HarvestRequest(BaseModel):
    """POST /internal/v1/tasks/harvest 요청 몸통.

    robot_selection : "auto"(시스템이 고름) | "manual"(robot_id 지정)
    robot_id        : manual 일 때만 사용(auto 면 무시).
    harvest_location: 수확 위치 task_point_id ('HARVEST_01' / 'HARVEST_02').
    """
    robot_selection: Literal["auto", "manual"] = "auto"
    robot_id: Optional[str] = None
    harvest_location: str


def _iso(ts: float) -> str:
    """epoch 초 -> UTC ISO8601 문자열."""
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()


def register(app, node, pool) -> None:
    """수확 접수 라우트를 기존 FastAPI 앱에 등록한다(patrol_api.create_app 에서 호출)."""
    log = node.get_logger()

    @app.post("/internal/v1/tasks/harvest")
    def accept_harvest(req: HarvestRequest):
        try:
            snap = automato_db.get_availability_snapshot(pool, "HARVEST")
        except Exception as exc:  # noqa: BLE001
            return JSONResponse(
                status_code=503,
                content={"status": "ERROR", "reason": "DB_UNAVAILABLE",
                         "message": str(exc)})
        now = time.time()

        # --- (1) 수확 위치 검증 — 로봇 고르기 전에 잘못된 요청을 거른다 ---
        try:
            tp = automato_db.get_task_point(pool, req.harvest_location)
        except Exception as exc:  # noqa: BLE001
            return JSONResponse(
                status_code=503,
                content={"status": "ERROR", "reason": "DB_ERROR",
                         "message": str(exc)})
        if tp is None or tp["point_type"] != "HARVEST":
            log.warning(f"수확 접수 거절: 잘못된 위치 {req.harvest_location}")
            return JSONResponse(
                status_code=400,
                content={"status": "REJECTED", "reason": "INVALID_HARVEST_LOCATION",
                         "message": f"{req.harvest_location} 은(는) 수확 위치가 아닙니다"})

        # --- 로봇 선정 (순찰과 동일 로직 재사용) ---
        judged = judge_all(node, snap, now)
        if req.robot_selection == "manual":
            rid = req.robot_id
            if not rid:
                return JSONResponse(
                    status_code=400,
                    content={"status": "REJECTED", "reason": "ROBOT_ID_REQUIRED",
                             "message": "manual 선정은 robot_id가 필요합니다"})
            j = judged.get(rid)
            if j is None:
                return JSONResponse(
                    status_code=404,
                    content={"status": "REJECTED", "reason": "UNKNOWN_ROBOT",
                             "message": f"{rid} 로봇을 찾을 수 없습니다"})
            if not j["available"]:
                return JSONResponse(
                    status_code=409,
                    content={"status": "REJECTED", "reason": j["unavailable_reason"],
                             "message": f"{rid} 배정 불가"})
            selected = rid
        else:  # auto
            selected = select_auto(list(judged.values()))
            if selected is None:
                return JSONResponse(
                    status_code=409,
                    content={"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                             "message": "가용 로봇이 없습니다"})

        # --- 배정 근거 스냅샷(명령 직전 상태 전체)을 JSON 문자열로 ---
        snapshot = node.cache.snapshot(selected) or {"robot_id": selected}
        snapshot["captured_at"] = _iso(now)
        snapshot_json = json.dumps(snapshot, ensure_ascii=False)

        # --- (2)(3) DB 접수(HARVEST/WAITING → snapshot → IN_PROGRESS).
        #     위치 중복은 ux_tasks_active_harvest_location, 로봇 중복은 ux_tasks_active_robot 이 최종 방어 ---
        try:
            task_id = automato_db.accept_harvest_task(
                pool, selected, req.harvest_location, snapshot_json)
        except automato_db.HarvestInProgressError:
            return JSONResponse(
                status_code=409,
                content={"status": "REJECTED", "reason": "HARVEST_IN_PROGRESS",
                         "message": f"{req.harvest_location} 에서 이미 수확이 진행 중입니다"})
        except automato_db.RobotBusyError:
            return JSONResponse(
                status_code=409,
                content={"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                         "message": f"{selected} 이미 활성 task 보유"})
        except Exception as exc:  # noqa: BLE001
            return JSONResponse(
                status_code=503,
                content={"status": "ERROR", "reason": "DB_ERROR",
                         "message": str(exc)})

        # --- 노드에 디스패치 시작 요청(비동기; 즉시 200 반환) ---
        node.start_harvest(task_id, selected, req.harvest_location)
        log.info(f"수확 접수: task={task_id} robot={selected} 위치={req.harvest_location}")
        return {
            "task_id": task_id,
            "assigned_robot_id": selected,
            "status": "ACCEPTED",
            "message": f"{selected} 수확 접수 (위치 {req.harvest_location})",
        }

#!/usr/bin/env python3
"""RP-78 ① API 계층 — FastAPI로 순찰 가용 조회/접수 HTTP 엔드포인트를 제공한다.

엔드포인트:
  GET  /internal/v1/robots/patrol/available  가용 로봇 조회(4조건 판정 결과)
  POST /internal/v1/tasks/patrol             순찰 접수(auto/manual 로봇 선정)
  GET  /health                               헬스체크

이 계층의 책임은 '판단'이다:
  - 가용 판정(judge_robot): 4개 조건을 AND로 확인
  - 로봇 선정(select_auto): 가용 후보 중 배터리 최고, 동점 시 robot_id 오름차순
데이터는 두 곳에서 온다:
  - DB(automato_db): 활성 task 여부, 배터리 임계값        ← '영속' 상태
  - 노드 캐시(node.cache): 로봇 최신 텔레메트리           ← '순간' 상태

judge_robot / select_auto 는 외부 의존(ROS/DB) 없는 순수 함수라 단위테스트가 쉽다.
create_app(node, pool) 이 노드/DB를 주입받아 엔드포인트에 연결한다.
"""
import json
import time
from datetime import datetime, timezone
from typing import Literal, Optional

from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from automato_control_service import automato_db, traffic_debug

# 텔레메트리가 이 시간(초)보다 오래되면 '미수신'으로 본다(ddago header.stamp 기준).
# 이 상태의 대외 사유명은 ROBOT_OFFLINE — 시나리오1 문서 E0 5)의 enum 을 따른다
# (같은 ACS 의 telemetry_ws 방송도 동일 이름을 쓴다).
STALE_SEC = 3.0


class PatrolRequest(BaseModel):
    """POST /internal/v1/tasks/patrol 요청 몸통.

    robot_selection: "auto"(시스템이 고름) | "manual"(robot_id 지정)
    robot_id: manual 일 때만 사용(auto 면 무시).
    """
    robot_selection: Literal["auto", "manual"] = "auto"
    robot_id: Optional[str] = None


def _iso(ts: float) -> str:
    """epoch 초 -> UTC ISO8601 문자열."""
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()


# --------------------------------------------------------------------------- #
# 가용 판정 (순수 함수) — 4개 조건 AND
#   1) (DB)   해당 로봇에 활성 task 없음
#   2) (캐시) nav_status == 'IDLE'
#   3) (캐시) battery_percent >= 임계값
#   4) (캐시) 최근 STALE_SEC 초 이내 수신(ddago header.stamp)
#   ※ is_charging 은 판정에 넣지 않는다(현재 항상 false 고정 → 넣으면 향후 함정).
# --------------------------------------------------------------------------- #
def judge_robot(robot_id: str, entry: Optional[dict], has_active_task: bool,
                threshold: float, now: float,
                stale_sec: float = STALE_SEC) -> dict:
    """로봇 1대의 가용 여부와 (불가 시) 사유를 판정해 응답 dict로 만든다.

    entry: 노드 캐시의 해당 로봇 항목(없으면 None). 형태는 patrol_node.TelemetryCache 참고.
    반환 예: {"robot_id","status","battery_percent","current_position","available"[,"unavailable_reason"]}

    사유 우선순위(여럿 겹칠 때):
      활성 task > 텔레메트리 없음/오래됨 > nav!=IDLE > 배터리 부족
    (활성 task는 DB 사실이라 캐시 신선도와 무관하게 최우선. 미수신이면 캐시값을 못 믿으므로
     nav/battery보다 먼저 ROBOT_OFFLINE으로 처리.)
    """
    ddago = entry.get("ddago") if entry else None
    status = ddago["nav_status"] if ddago else None
    battery = ddago["battery_percent"] if ddago else None
    position = {"x": ddago["x"], "y": ddago["y"]} if ddago else None

    reason = None
    if has_active_task:
        reason = "ROBOT_BUSY"
    elif ddago is None:
        reason = "ROBOT_OFFLINE"            # 한 번도 수신 못 함
    else:
        stamp = entry.get("ddago_stamp")
        age = (now - stamp) if stamp is not None else None
        if age is None or age > stale_sec:
            reason = "ROBOT_OFFLINE"        # stale_sec 초 이상 미수신
        elif status != "IDLE":
            reason = "ROBOT_BUSY"
        elif battery is None or battery < threshold:
            reason = "BATTERY_TOO_LOW"

    out = {
        "robot_id": robot_id,
        "status": status,
        "battery_percent": battery,
        "current_position": position,
        "available": reason is None,
    }
    if reason is not None:
        out["unavailable_reason"] = reason
    return out


def select_auto(judged: list) -> Optional[str]:
    """auto 선정: 가용 후보 중 배터리 최댓값, 동점 시 robot_id 오름차순 첫 번째.

    judged: judge_robot 결과 dict들의 리스트. 후보 없으면 None.
    """
    candidates = [j for j in judged if j["available"]]
    if not candidates:
        return None
    # battery 내림차순, robot_id 오름차순 → 첫 번째가 정답
    candidates.sort(key=lambda j: (-j["battery_percent"], j["robot_id"]))
    return candidates[0]["robot_id"]


# --------------------------------------------------------------------------- #
# FastAPI 앱 팩토리
# --------------------------------------------------------------------------- #
def create_app(node, pool) -> FastAPI:
    """노드(텔레메트리 캐시/디스패치)와 DB 풀을 주입받아 FastAPI 앱을 만든다.

    node: patrol_node.PatrolControlNode  (node.cache, node.start_patrol 사용)
    pool: psycopg_pool.ConnectionPool
    """
    app = FastAPI(title="Automato Control Service — Patrol (RP-78)")

    # 교통관제 관측용 읽기 전용 라우트(GET /internal/v1/debug/traffic).
    # 검증 화면(verify_web)의 LIVE 모드가 이걸 폴링한다. 순찰 판단 로직과 무관하다.
    traffic_debug.register(app, node)

    def _judge_all(snap: dict, now: float) -> dict:
        """robot_id -> 판정 dict. available/접수 양쪽에서 재사용."""
        return {
            rid: judge_robot(
                rid, node.cache.get(rid), rid in snap["active"],
                snap["threshold"], now,
            )
            for rid in snap["robots"]
        }

    @app.get("/health")
    def health():
        return {"ok": True, "service": "Automato Control Service (Patrol)"}

    # ------- API 1: 가용 로봇 조회 -------
    @app.get("/internal/v1/robots/patrol/available")
    def available():
        try:
            snap = automato_db.get_availability_snapshot(pool)
        except Exception as exc:  # noqa: BLE001
            return JSONResponse(
                status_code=503,
                content={"error": "DB_UNAVAILABLE", "message": str(exc)},
            )
        now = time.time()
        robots = list(_judge_all(snap, now).values())
        return {
            "requested_at": _iso(now),
            "min_battery_percent": snap["threshold"],
            "robots": robots,
        }

    # ------- API 2: 순찰 접수 -------
    @app.post("/internal/v1/tasks/patrol")
    def accept_patrol(req: PatrolRequest):
        try:
            snap = automato_db.get_availability_snapshot(pool)
        except Exception as exc:  # noqa: BLE001
            return JSONResponse(
                status_code=503,
                content={"status": "ERROR", "reason": "DB_UNAVAILABLE",
                         "message": str(exc)},
            )
        now = time.time()
        judged = _judge_all(snap, now)

        # --- 로봇 선정 ---
        if req.robot_selection == "manual":
            rid = req.robot_id
            if not rid:
                return JSONResponse(
                    status_code=400,
                    content={"status": "REJECTED", "reason": "ROBOT_ID_REQUIRED",
                             "message": "manual 선정은 robot_id가 필요합니다"},
                )
            j = judged.get(rid)
            if j is None:
                return JSONResponse(
                    status_code=404,
                    content={"status": "REJECTED", "reason": "UNKNOWN_ROBOT",
                             "message": f"{rid} 로봇을 찾을 수 없습니다"},
                )
            # 배정 직전 재검증 — 미달이면 해당 사유로 409
            if not j["available"]:
                return JSONResponse(
                    status_code=409,
                    content={"status": "REJECTED",
                             "reason": j["unavailable_reason"],
                             "message": f"{rid} 배정 불가"},
                )
            selected = rid
        else:  # auto
            selected = select_auto(list(judged.values()))
            if selected is None:
                return JSONResponse(
                    status_code=409,
                    content={"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                             "message": "가용 로봇이 없습니다"},
                )

        # --- 배정 근거 스냅샷(명령 직전 상태 전체)을 JSON 문자열로 ---
        snapshot = node.cache.snapshot(selected) or {"robot_id": selected}
        snapshot["captured_at"] = _iso(now)
        snapshot_json = json.dumps(snapshot, ensure_ascii=False)

        # --- DB 트랜잭션 접수(①~④). 중복배정은 유니크 위반 -> 409 ---
        try:
            task_id, waypoints = automato_db.accept_patrol_task(
                pool, selected, snapshot_json)
        except automato_db.RobotBusyError:
            return JSONResponse(
                status_code=409,
                content={"status": "REJECTED", "reason": "NO_AVAILABLE_ROBOT",
                         "message": f"{selected} 이미 활성 task 보유"},
            )
        except automato_db.PatrolInProgressError:
            return JSONResponse(
                status_code=409,
                content={"status": "REJECTED", "reason": "PATROL_IN_PROGRESS",
                         "message": "이미 순찰이 진행 중입니다(동시 1대 제약)"},
            )
        except Exception as exc:  # noqa: BLE001
            return JSONResponse(
                status_code=503,
                content={"status": "ERROR", "reason": "DB_ERROR",
                         "message": str(exc)},
            )

        # --- 노드에 세그먼트 디스패치 시작 요청(비동기; 즉시 200 반환) ---
        node.start_patrol(task_id, selected, waypoints)
        return {
            "task_id": task_id,
            "assigned_robot_id": selected,
            "status": "ACCEPTED",
            "message": f"{selected} 순찰 접수 (waypoint {len(waypoints)}개)",
        }

    return app

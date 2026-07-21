#!/usr/bin/env python3
"""교통관제 관측용 읽기 전용 스냅샷 API (RP-EX 검증 화면용).

  GET /internal/v1/debug/traffic   지금 이 순간의 통로 예약표 + 로봇 위치

왜 별도 파일인가 —
  patrol_api.py 는 '판단'(가용 판정·로봇 선정·접수)을 담당하는 검증 끝난 코드다.
  관측용 부가 기능을 그 안에 섞으면 나중에 순찰 로직을 읽을 때 잡음이 된다.
  여기로 빼면 patrol_api 는 import 1줄 + 등록 1줄만 늘어난다.

설계 원칙 두 가지 —
  1) 관찰이 대상을 바꾸지 않는다.
     RoutingEngine 은 첫 순찰 때 DB 그래프를 읽어 lazy 생성된다. 이 API 가
     node._get_engine() 을 호출하면 '아무도 순찰을 안 시켰는데 그래프가 로드되는'
     부작용이 생긴다. 그래서 이미 만들어진 엔진만 들여다보고, 없으면
     engine_ready=false 로 정직하게 답한다(순찰 전엔 예약도 당연히 없다).
  2) 쓰기 API 를 하나도 만들지 않는다.
     예약을 강제로 풀거나 통로를 막는 조작은 실물 로봇을 실제로 위험하게 만든다.
     검증 화면의 '조작'은 시뮬(verify_web SIM 모드)에서만 한다.

정확도 —
  예전에는 reserved_corridors() 로 목록을 받고 holder_of() 로 보유자를 다시 묻는
  2단계라 그 사이에 해제되면 항목이 빠졌다. 노드 자리까지 관측 대상이 되면서 왕복이
  배로 늘고 '음수면 자리'라는 규칙을 관측하는 쪽마다 다시 구현하게 되어,
  engine.reservation_snapshot() 으로 한 락 안에서 원자적으로 덤프하도록 바꿨다.
  TTL 지난 죽은 예약은 엔진이 걸러서 준다.
"""
import time

from fastapi.responses import JSONResponse


# 텔레메트리가 이보다 오래되면 '미수신'으로 표시한다(patrol_api.STALE_SEC 과 동일 의미).
STALE_SEC = 3.0


def _robot_view(robot_id: str, entry, now: float) -> dict:
    """텔레메트리 캐시 1건 -> 화면이 쓸 수 있는 평평한 dict.

    ddago(주행 로봇) 부분만 쓴다. 팔(ddagi)은 이 화면의 관심사가 아니다.
    한 번도 못 받은 로봇은 online=False 로 남기고 좌표를 None 으로 둔다
    — 0,0 으로 채우면 화면 구석에 '있지도 않은 로봇'이 그려진다.
    """
    ddago = (entry or {}).get("ddago")
    stamp = (entry or {}).get("ddago_stamp")
    age = (now - stamp) if stamp else None
    if ddago is None:
        return {"robot_id": robot_id, "online": False, "age_sec": None,
                "x": None, "y": None, "yaw": None,
                "nav_status": None, "task_id": None, "battery_percent": None}
    return {
        "robot_id": robot_id,
        # 살아있음의 기준은 '값이 있느냐'가 아니라 '최근 것이냐'다.
        "online": age is not None and age <= STALE_SEC,
        "age_sec": round(age, 2) if age is not None else None,
        "x": ddago["x"], "y": ddago["y"], "yaw": ddago["yaw"],
        "nav_status": ddago["nav_status"],
        "task_id": ddago["task_id"] or None,
        "battery_percent": ddago["battery_percent"],
        "is_charging": ddago["is_charging"],
    }


def snapshot(node) -> dict:
    """지금 이 순간의 교통관제 상태 한 장. 순수 읽기."""
    now = time.time()

    # --- 로봇: 텔레메트리 캐시(1Hz 로 갱신되는 '순간' 상태) ---
    robots = [_robot_view(rid, node.cache.get(rid), now)
              for rid in node.cache.robot_ids()]

    # --- 예약: 엔진이 이미 만들어져 있을 때만. 통로와 '지점 자리'를 갈라서 낸다 ---
    engine = node._engine                        # noqa: SLF001  (읽기 전용 관찰)
    reservations, node_holders = {}, {}
    avoiding, avoiding_nodes = [], []
    if engine is not None:
        snap = engine.reservation_snapshot()     # 한 락 안의 원자적 덤프
        reservations = {str(cid): rid for cid, rid in snap["corridors"].items()}
        node_holders = {str(n): rid for n, rid in snap["nodes"].items()}

        # --- 회피 중(블랙리스트): 막힘/양보로 재계획에서 잠시 빠진 통로/지점 ---
        try:
            view = node._dispatcher.blacklist_view(engine)        # noqa: SLF001
            avoiding, avoiding_nodes = view["corridors"], view["nodes"]
        except Exception:                        # noqa: BLE001
            pass

    return {
        "server_time": now,
        "engine_ready": engine is not None,
        "robots": robots,
        "reservations": reservations,
        # 지점 자리 점유 {노드id: 로봇}. 통로 예약과 키 공간이 겹쳐 따로 낸다.
        "node_holders": node_holders,
        "avoiding": avoiding,
        "avoiding_nodes": avoiding_nodes,
        "stale_sec": STALE_SEC,
    }


def register(app, node) -> None:
    """create_app 에서 호출. 라우트 1개만 붙인다."""

    @app.get("/internal/v1/debug/traffic")
    def traffic():
        try:
            return snapshot(node)
        except Exception as exc:                 # noqa: BLE001
            # 관측 API 가 죽어도 순찰 API 는 멀쩡해야 한다 → 500 대신 503+사유.
            return JSONResponse(
                status_code=503,
                content={"error": "SNAPSHOT_FAILED", "message": str(exc)},
            )

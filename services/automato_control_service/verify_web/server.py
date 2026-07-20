#!/usr/bin/env python3
"""ACS 교통관제 검증 웹 — 서버(1단계: 그래프 API).

목적:
  routing_engine / patrol_dispatcher 가 '실제로' 어떻게 움직이는지를 눈으로 보기 위한
  검증 도구. 로봇 없이(가짜 로봇으로) 경로 탐색·통로 예약/해제를 실시간 관찰한다.

가장 중요한 원칙 — 검증 대상 코드는 한 줄도 고치지 않는다:
  RoutingEngine 과 PatrolDispatcher 를 '그대로 import 해서 실행'한다. 경로 탐색을
  JavaScript 로 다시 구현하면 '웹 페이지가 잘 도는 것'만 증명될 뿐, 정작 검증하려는
  파이썬 코드는 하나도 검증되지 않기 때문이다. 가짜인 것은 로봇(액션 클라이언트)뿐이다.

위치가 왜 패키지(automato_control_service/) '밖'인가:
  이 파일은 ROS 노드가 아니라 평범한 웹 서버다. 패키지 안에 넣으면 setup.py 의
  data_files 로 정적 파일(html/js)까지 install 트리에 복사해야 하고, colcon 빌드에
  영향을 준다. 밖에 두면 find_packages() 가 잡지 않아 빌드 리스크가 0이고,
  정적 파일도 그냥 상대경로로 읽으면 된다.

실행:
  cd <리포 루트>
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash          # PYTHONPATH 에 automato_control_service 등록
  python3 services/automato_control_service/verify_web/server.py
"""
import asyncio
import contextlib
import logging
import os
import sys
from pathlib import Path

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

from automato_control_service import automato_db
from automato_control_service.patrol_config import PATROL_START_WAYPOINT_ID

# 이 파일과 같은 폴더의 모듈(map_layout 등)을 import 하기 위한 경로 추가.
# verify_web 은 colcon 패키지가 아니라 '스크립트 묶음'이라 패키지 import 가 안 된다.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import map_layout  # noqa: E402
from live import LiveSource  # noqa: E402
from sim import VerifySim  # noqa: E402

log = logging.getLogger("verify_web")

STATIC_DIR = Path(__file__).resolve().parent / "static"

# 상태 방송 주기(초). 로봇 위치가 0.1초마다 갱신되므로 같은 주기로 내보낸다.
BROADCAST_SEC = 0.1
# 가짜 로봇 기본 속도. 맵이 작아서(통로 0.03~0.44m) 느려야 통로 점유가 눈에 보인다.
SIM_SPEED_MPS = float(os.environ.get("VERIFY_SPEED_MPS", "0.06"))
SIM_SPIN_RPS = float(os.environ.get("VERIFY_SPIN_RPS", "0.9"))

# ACS(8200)·텔레메트리 WS(8000)와 겹치지 않는 포트. 셋을 동시에 띄울 수 있어야 한다.
PORT = int(os.environ.get("VERIFY_WEB_PORT", "8300"))

# LIVE 모드가 들여다볼 실물 ACS 주소. ACS 는 0.0.0.0:8200 에 뜬다(patrol_node.main).
ACS_BASE = os.environ.get("ACS_BASE_URL", "http://127.0.0.1:8200")
# 시작 모드. 기본이 SIM 인 이유 — 로봇이 없어도 항상 동작하는 쪽이 기본이어야 한다.
START_MODE = os.environ.get("VERIFY_MODE", "SIM").upper()

# 바인딩 주소. 기본은 루프백(이 머신에서만 접근) — 검증 도구를 실수로 네트워크에
# 노출하지 않기 위한 안전한 기본값이다.
# 다른 기기(예: Tailscale 로 연결한 노트북)에서 보려면 그 인터페이스 주소를 준다:
#   VERIFY_WEB_HOST=$(tailscale ip -4) python3 .../server.py
# → tailnet 안에서만 열리고 같은 공유기(LAN)에는 노출되지 않는다.
# 0.0.0.0 은 모든 인터페이스(LAN 포함)에 여는 것이라 필요할 때만 쓴다.
HOST = os.environ.get("VERIFY_WEB_HOST", "127.0.0.1")

# 순찰 순서(patrol_order)만 automato_db.load_graph 가 돌려주지 않아 여기서 따로 읽는다.
# load_graph 는 ACS 런타임이 쓰는 공용 함수라 검증 도구 편의로 수정하지 않는다
# (읽기 전용 추가 조회는 이 파일 안에 가둔다 → ACS 동작에 영향 0).
_SELECT_PATROL_ORDER = (
    "SELECT waypoint_id, patrol_order FROM waypoints "
    " WHERE patrol_order IS NOT NULL"
)

# 충전소/수확대/예냉실의 '실제 위치'. task_points 가 waypoint_id FK 를 갖게 되면서
# 이 시설들의 좌표를 DB 에서 그대로 유도할 수 있다(예전엔 화면에 손으로 찍어야 했다).
# 로봇의 전용 충전소(robots.charge_point_id)도 같이 묶어 어느 칸이 누구 자리인지 표시한다.
_SELECT_TASK_POINTS = (
    "SELECT t.task_point_id, t.point_type, t.waypoint_id, "
    "       w.x_coord, w.y_coord, "
    "       (SELECT string_agg(r.robot_id, ',' ORDER BY r.robot_id) "
    "          FROM robots r WHERE r.charge_point_id = t.task_point_id) AS robots "
    "  FROM task_points t "
    "  JOIN waypoints w ON w.waypoint_id = t.waypoint_id "
    " ORDER BY t.point_type, t.task_point_id"
)


def load_map(pool) -> dict:
    """DB에서 맵(노드 + 통로 + 시설)을 읽어 웹이 그릴 수 있는 형태로 만든다.

    노드=waypoints, 간선=corridors 라는 그래프 구조 자체는 ACS 가 쓰는 것과 완전히
    동일하다(automato_db.load_graph 재사용 — pair_of 포함). 여기에 화면 표시용으로
    patrol_order 와 시설(task_points)을 덧붙인다.

    ★ 짝(pair) 처리가 중요하다:
      18(부모 10), 19(부모 13)은 '같은 자리에서 방향만 180° 돌려 한 번 더 찍는' 전용 행이라
      부모와 x·y 가 완전히 같고 corridors 에 등장하지 않는다. 그래서
        - routing_nodes  : 짝을 뺀 것 (patrol_node 와 동일. 넣으면 고립 노드가 섞인다)
        - waypoints      : 짝까지 전부 (화면이 부모 위에 회전 표시를 그려야 하므로 좌표가 필요)
      두 벌을 따로 내려준다. 화면은 짝을 '별도 점'이 아니라 부모에 붙는 ↻ 배지로 그린다.

    반환에 추가되는 것:
      routing_node_ids : 경로 탐색 그래프에 실제로 들어가는 노드 id 목록
      pairs            : [{"parent","pair","yaw"}, ...] 짝 관계
      task_points      : [{"task_point_id","point_type","waypoint_id","x","y","robots"}, ...]
      patrol_start     : {robot_id: waypoint_id} 로봇별 순찰 출발 노드(전용 충전소)
    """
    graph = automato_db.load_graph(pool)

    with pool.connection() as conn:
        orders = {
            r["waypoint_id"]: r["patrol_order"]
            for r in conn.execute(_SELECT_PATROL_ORDER).fetchall()
        }
        task_points = [
            {"task_point_id": r["task_point_id"], "point_type": r["point_type"],
             "waypoint_id": r["waypoint_id"], "x": r["x_coord"], "y": r["y_coord"],
             "robots": (r["robots"] or "").split(",") if r["robots"] else []}
            for r in conn.execute(_SELECT_TASK_POINTS).fetchall()
        ]
    for w in graph["waypoints"]:
        w["patrol_order"] = orders.get(w["waypoint_id"])

    # patrol_node.py 와 동일한 규칙으로 라우팅 노드를 추린다(짝 제외).
    graph["routing_node_ids"] = [
        w["waypoint_id"] for w in graph["waypoints"] if w["pair_of"] is None]
    graph["pairs"] = [
        {"parent": w["pair_of"], "pair": w["waypoint_id"], "yaw": w["yaw"]}
        for w in graph["waypoints"] if w["pair_of"] is not None]
    graph["task_points"] = task_points

    # 로봇별 출발 노드 = 전용 충전소의 진입 노드(robots.charge_point_id 사슬).
    # 조회 실패/미등록 로봇은 전역 상수로 폴백 — ACS 와 같은 규칙.
    starts = {}
    for tp in task_points:
        for rid in tp["robots"]:
            starts[rid] = tp["waypoint_id"]
    graph["patrol_start"] = starts
    graph["patrol_start_fallback"] = PATROL_START_WAYPOINT_ID
    return graph


def create_app(pool, sim, live=None) -> FastAPI:
    """DB 풀과 두 데이터 소스를 주입받아 앱을 만든다(테스트 시 가짜를 넣을 수 있게 팩토리 형태).

    sim  : VerifySim   — 가짜 로봇 + 이 프로세스 안의 RoutingEngine (조작 가능)
    live : LiveSource  — 실물 ACS 폴링 결과 (관측 전용). None 이면 LIVE 전환 불가.

    둘은 snapshot(since_seq) / last_seq() 라는 같은 인터페이스를 가진다. 그래서
    아래 방송 루프와 WebSocket 은 '지금 어느 쪽인지' 신경 쓰지 않고 current() 만 부른다.
    """
    app = FastAPI(title="ACS 교통관제 검증 웹")
    clients: set = set()          # 접속 중인 WebSocket
    state = {"seq": 0, "mode": "SIM"}   # 마지막 방송 seq + 현재 데이터 출처

    def current():
        """지금 화면에 흘려보낼 데이터 소스."""
        return live if (state["mode"] == "LIVE" and live is not None) else sim

    @app.get("/health")
    def health():
        return {"ok": True, "service": "acs-traffic-verify-web"}

    @app.get("/api/graph")
    def api_graph():
        """맵 데이터. 페이지 로드 시 1회 호출한다(맵은 실행 중 바뀌지 않는다).

        실시간으로 변하는 것(로봇 위치·통로 예약 상태)은 4단계에서 WebSocket 으로
        따로 흘려보낸다. 안 변하는 것과 변하는 것을 분리해야 화면 갱신이 가벼워진다.
        """
        try:
            graph = load_map(pool)
        except Exception as exc:  # noqa: BLE001
            log.error(f"그래프 로드 실패: {exc}")
            return JSONResponse(
                status_code=503,
                content={"error": "DB_UNAVAILABLE", "message": str(exc)},
            )
        log.info(
            f"그래프 응답: 노드 {len(graph['waypoints'])}개"
            f"(라우팅 {len(graph['routing_node_ids'])}, 짝 {len(graph['pairs'])} 제외) / "
            f"통로 {len(graph['corridors'])}개 / 시설 {len(graph['task_points'])}개 / "
            f"출발 {graph['patrol_start']}")
        return graph

    @app.get("/api/layout")
    def api_layout():
        """맵 배경(방 외곽 + 구조물). DB 에 없는 장식 정보라 map_layout.py 상수에서 온다."""
        return map_layout.get_layout()

    # ---------------------- 실시간 상태 방송 (WebSocket) ---------------------- #
    @app.websocket("/ws/state")
    async def ws_state(ws: WebSocket):
        """맵 화면이 여기에 붙어 로봇 위치·통로 예약 상태를 실시간으로 받는다.

        폴링(HTTP 반복 요청) 대신 WebSocket 인 이유: 10Hz 로 상태가 바뀌는데 폴링은
        매번 연결·헤더 비용이 붙고, '서버가 밀어주는' 모델이 아니라 늘 한 박자 늦는다.
        """
        await ws.accept()
        clients.add(ws)
        # 새로 붙은 화면은 최근 이벤트까지 한 번 받아 맥락을 잡는다.
        src = current()
        await ws.send_json(src.snapshot(since_seq=max(0, src.last_seq() - 40)))
        try:
            while True:
                await ws.receive_text()      # 클라이언트는 보내는 게 없다(끊김 감지용)
        except WebSocketDisconnect:
            pass
        except Exception:  # noqa: BLE001
            pass
        finally:
            clients.discard(ws)

    async def broadcast_loop():
        """BROADCAST_SEC 마다 스냅샷을 모든 화면에 밀어준다."""
        while True:
            await asyncio.sleep(BROADCAST_SEC)
            if not clients:
                continue
            try:
                snap = current().snapshot(since_seq=state["seq"])
            except Exception as exc:  # noqa: BLE001
                log.error(f"스냅샷 실패: {exc}")
                continue
            state["seq"] = snap["seq"]
            dead = []
            for ws in list(clients):
                try:
                    await ws.send_json(snap)
                except Exception:  # noqa: BLE001
                    dead.append(ws)
            for ws in dead:
                clients.discard(ws)

    @app.on_event("startup")
    async def _start():
        app.state.bc = asyncio.create_task(broadcast_loop())
        # 환경변수로 LIVE 부팅을 요청했다면 여기서 폴링을 켠다. 로봇을 붙여둔 채
        # 서버를 재시작할 때 매번 버튼을 누르지 않아도 되게.
        if START_MODE == "LIVE" and live is not None:
            live.start()
            state["mode"] = "LIVE"
            log.info("시작 모드 LIVE — ACS 폴링 시작")
        log.info("상태 방송 시작")

    @app.on_event("shutdown")
    async def _stop():
        app.state.bc.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await app.state.bc
        sim.shutdown()
        if live is not None:
            live.stop()

    # ---------------------------- 모드 ---------------------------- #
    @app.get("/api/mode")
    def api_mode_get():
        return {
            "mode": state["mode"],
            "live_available": live is not None,
            "acs_base": live.acs_base if live is not None else None,
        }

    @app.post("/api/mode/{mode}")
    def api_mode_set(mode: str):
        """SIM ↔ LIVE 전환. 데이터 출처만 갈아끼운다(맵도 프런트도 그대로).

        전환 시 seq 를 0 으로 되돌리는 이유: 두 소스는 각자 독립된 이벤트 번호를 쓴다.
        SIM 의 seq=120 을 LIVE 에 그대로 물어보면 LIVE 이벤트가 통째로 걸러진다.
        """
        mode = mode.upper()
        if mode not in ("SIM", "LIVE"):
            return JSONResponse(status_code=400,
                                content={"ok": False, "reason": "UNKNOWN_MODE"})
        if mode == "LIVE" and live is None:
            return JSONResponse(status_code=409,
                                content={"ok": False, "reason": "LIVE_UNAVAILABLE"})
        if mode == state["mode"]:
            return {"ok": True, "mode": mode}

        if mode == "LIVE":
            live.start()          # 폴링 스레드 기동
        elif live is not None:
            live.stop()           # SIM 으로 돌아가면 ACS 를 더 두드리지 않는다
        state["mode"] = mode
        state["seq"] = 0
        log.info(f"모드 전환 → {mode}")
        return {"ok": True, "mode": mode}

    # ---------------------------- 조작 ---------------------------- #
    def _sim_only():
        """LIVE 에서는 조작을 거절한다. None 이면 통과, 아니면 그대로 응답으로 쓴다.

        왜 막나 — 이 화면의 조작은 '가짜 로봇 세계'를 흔드는 것이다. LIVE 는 실물
        ACS 를 들여다보기만 하므로, 여기서 순찰 버튼을 눌러도 시뮬 로봇이 움직일 뿐
        화면(실물)에는 아무 변화가 없다. 조용히 아무 일도 안 일어나는 것보다
        '지금은 관측 모드라 안 된다'고 말해주는 편이 훨씬 덜 헷갈린다.
        """
        if state["mode"] != "LIVE":
            return None
        return JSONResponse(
            status_code=409,
            content={"ok": False, "reason": "LIVE_READONLY",
                     "message": "LIVE(실물 관측) 모드에서는 조작할 수 없습니다"})

    @app.post("/api/patrol/{robot_id}")
    def api_patrol(robot_id: str):
        """이 로봇에 전체 순찰을 시킨다(진짜 PatrolDispatcher.run_patrol 이 돈다)."""
        if (deny := _sim_only()) is not None:
            return deny
        r = sim.start_patrol(robot_id)
        log.info(f"순찰 시작 요청 {robot_id} → {r}")
        return JSONResponse(status_code=200 if r["ok"] else 409, content=r)

    @app.post("/api/goto/{robot_id}/{waypoint_id}")
    def api_goto(robot_id: str, waypoint_id: int):
        """한 지점으로만 보낸다 — 통로 경합을 손으로 만들 때 쓴다."""
        if (deny := _sim_only()) is not None:
            return deny
        r = sim.goto(robot_id, waypoint_id)
        log.info(f"이동 요청 {robot_id}→wp{waypoint_id} : {r}")
        return JSONResponse(status_code=200 if r["ok"] else 409, content=r)

    @app.post("/api/corridor/{corridor_id}/{action}")
    def api_corridor(corridor_id: int, action: str):
        """통로를 진짜 막힘으로 만들거나 푼다(로봇이 그 구간에서 code=1 을 보고)."""
        if (deny := _sim_only()) is not None:
            return deny
        if action not in ("block", "unblock"):
            return JSONResponse(status_code=400, content={"ok": False})
        sim.set_corridor_blocked(corridor_id, action == "block")
        return {"ok": True, "corridor_id": corridor_id, "blocked": action == "block"}

    @app.post("/api/place/{robot_id}/{waypoint_id}")
    def api_place(robot_id: str, waypoint_id: int):
        """로봇을 특정 노드에 즉시 세운다(시나리오 초기 배치용, 주행 아님)."""
        if (deny := _sim_only()) is not None:
            return deny
        r = sim.place(robot_id, waypoint_id)
        return JSONResponse(status_code=200 if r["ok"] else 409, content=r)

    @app.post("/api/scenario/deadlock")
    def api_scenario_deadlock():
        """두 로봇을 통로 사슬 양 끝에서 마주보게 출발시켜 대기 사이클을 만든다."""
        if (deny := _sim_only()) is not None:
            return deny
        r = sim.scenario_deadlock()
        log.info(f"데드락 시나리오 시작: {r}")
        return r

    @app.post("/api/reset")
    def api_reset():
        if (deny := _sim_only()) is not None:
            return deny
        sim.reset()
        state["seq"] = 0
        return {"ok": True}

    # 정적 파일은 맨 마지막에 마운트한다 — "/" 를 통째로 잡아먹기 때문에,
    # 먼저 선언된 /api/* 라우트가 우선 매칭되도록 순서를 지켜야 한다.
    app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")
    return app


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    pool = automato_db.create_pool()   # DATABASE_URL 또는 services/database/.env
    sim = VerifySim(pool, speed_mps=SIM_SPEED_MPS, spin_rps=SIM_SPIN_RPS)
    log.info(
        f"시뮬 준비: 로봇 {len(sim._robots)}대 / 순찰지점 {len(sim.targets)}개 / "
        f"통로 {len(sim.corridor_ids)}개 / 속도 {SIM_SPEED_MPS}m/s")

    # LIVE 소스는 만들어만 두고 폴링은 시작하지 않는다(모드 전환 시 start()).
    # 좌표→노드 역추정에 쓸 wp_meta 는 시뮬이 DB 에서 이미 읽어 둔 것을 그대로 빌린다
    # — 실물이든 시뮬이든 맵은 같은 DB 에서 오므로 두 벌 읽을 이유가 없다.
    live = LiveSource(ACS_BASE, sim.dispatcher.wp_meta)
    log.info(f"LIVE 소스 준비(대기): {live.url}")

    app = create_app(pool, sim, live)
    log.info(f"ACS 교통관제 검증 웹 → http://{HOST}:{PORT} (시작 모드 {START_MODE})")
    try:
        uvicorn.run(app, host=HOST, port=PORT, log_level="info")
    finally:
        try:
            pool.close()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()

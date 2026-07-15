#!/usr/bin/env python3
"""RP-78 ③ DB 저장 계층 — 순찰 접수/조회/종료의 모든 SQL을 여기 모은다.

이 파일은 "DB에 무엇을 어떻게 쓰고 읽는가"만 담당한다(순수 데이터 계층).
가용 판정/로봇 선정 같은 '판단'은 API 계층(patrol_api.py)이,
로봇에 명령을 내리는 '동작'은 노드(patrol_node.py)가 맡는다.

배경 지식(왜 이렇게 나눴나):
  - ROS2/로봇 코드와 DB 코드가 한 파일에 섞이면 테스트·디버깅이 어렵다.
  - DB 계층을 분리하면 "SQL만" 따로 검증할 수 있고, 나중에 드라이버를 바꿔도
    여기만 고치면 된다.

드라이버: psycopg v3 (RP-82 database 서비스와 동일). 접속은 DATABASE_URL(env).
  - SQLAlchemy 표기(postgresql+psycopg://)는 libpq 표기(postgresql://)로 정규화해서 쓴다.
  - 커넥션 풀(psycopg_pool)을 써서 요청마다 새 연결을 만드는 비용을 줄인다.

핵심: 순찰 접수(①~④)는 '하나의 트랜잭션'으로 묶는다.
  중간에 실패하면 전부 롤백돼 '반쯤 만들어진 task'가 남지 않는다(원자성).
"""
import os

import psycopg
from psycopg.rows import dict_row
from psycopg_pool import ConnectionPool


class RobotBusyError(Exception):
    """대상 로봇에 이미 활성(WAITING/IN_PROGRESS) task가 있어 배정할 수 없음.

    DB의 부분 유니크 인덱스(ux_tasks_active_robot) 위반을 애플리케이션 예외로
    바꿔 던진다. API 계층이 이걸 잡아 409(NO_AVAILABLE_ROBOT)로 응답한다.
    (GUI가 1차로 막더라도, 처리 시점 상태 변동을 DB 인덱스로 최종 방어)
    """


# --------------------------------------------------------------------------- #
# 접속 정보(DSN) 로딩
# --------------------------------------------------------------------------- #
def _load_dsn() -> str:
    """DATABASE_URL(우선) 또는 POSTGRES_* 조합으로 libpq DSN을 만든다.

    services/database/.env 를 best-effort 로 읽어온다(같은 값 공유).
    smoke_check.py 와 동일한 규칙이라 DB 서비스와 접속 설정이 어긋나지 않는다.
    """
    try:
        from dotenv import load_dotenv

        here = os.path.dirname(__file__)
        db_env = os.path.normpath(
            os.path.join(here, "..", "..", "database", ".env"))
        load_dotenv(db_env)
    except Exception:  # noqa: BLE001  (dotenv 없거나 파일 없어도 무시)
        pass

    url = os.environ.get("DATABASE_URL")
    if url:
        # SQLAlchemy 표기 -> libpq 표기 (psycopg 직접 연결용)
        return url.replace("postgresql+psycopg://", "postgresql://", 1)

    user = os.environ.get("POSTGRES_USER", "automato")
    pw = os.environ.get("POSTGRES_PASSWORD", "automato")
    host = os.environ.get("POSTGRES_HOST", "localhost")
    port = os.environ.get("POSTGRES_PORT", "5432")
    db = os.environ.get("POSTGRES_DB", "automato")
    return f"postgresql://{user}:{pw}@{host}:{port}/{db}"


def create_pool() -> ConnectionPool:
    """커넥션 풀 생성. row_factory=dict_row 라 결과를 컬럼명으로 접근한다.

    open(wait=False): DB가 아직 안 떠 있어도 서비스 자체는 기동된다.
    실제 연결 실패는 쿼리 시점(pool.connection())에 드러난다.
    """
    dsn = _load_dsn()
    pool = ConnectionPool(
        conninfo=dsn,
        min_size=1,
        max_size=4,
        kwargs={"row_factory": dict_row},
        open=False,
    )
    pool.open(wait=False)
    return pool


# --------------------------------------------------------------------------- #
# 가용 판정 입력 조회 (available API / 접수 API 공통)
# --------------------------------------------------------------------------- #
def get_availability_snapshot(pool: ConnectionPool) -> dict:
    """가용 판정에 필요한 'DB쪽' 입력을 한 번에 모아 온다.

    반환:
      robots    : 전체 로봇 id 목록(robots 테이블, 정렬됨)
      active    : 지금 활성 task를 가진 robot_id 집합(WAITING/IN_PROGRESS)
      threshold : PATROL 배터리 임계값(operation_battery_thresholds, 기본 70)

    캐시(nav_status/battery/staleness)는 노드가 들고 있으므로 여기선 안 읽는다.
    """
    with pool.connection() as conn:
        robots = [
            r["robot_id"]
            for r in conn.execute(
                "SELECT robot_id FROM robots ORDER BY robot_id"
            ).fetchall()
        ]
        active = {
            r["assigned_robot_id"]
            for r in conn.execute(
                "SELECT assigned_robot_id FROM tasks "
                "WHERE assigned_robot_id IS NOT NULL "
                "AND status IN ('WAITING','IN_PROGRESS')"
            ).fetchall()
        }
        row = conn.execute(
            "SELECT min_battery_percent FROM operation_battery_thresholds "
            "WHERE task_type = 'PATROL'"
        ).fetchone()
        threshold = row["min_battery_percent"] if row else 70
    return {"robots": robots, "active": active, "threshold": threshold}


# --------------------------------------------------------------------------- #
# RP-90 텔레메트리 방송용 DB 사실 (1Hz 로 호출) — 활성 task 종류 + 배터리 임계값
# --------------------------------------------------------------------------- #
def get_telemetry_state(pool: ConnectionPool) -> tuple:
    """RP-90 방송에 필요한 'DB쪽' 사실을 한 번에 모아 온다.

    반환: (active_types, threshold)
      active_types : {robot_id: task_type}. 활성(WAITING/IN_PROGRESS) task 가 있는 로봇만.
                     값이 그 로봇의 진행 중 task 종류(PATROL/HARVEST/TRANSFER)다 →
                     여기 있으면 ROBOT_BUSY 이고, 그 종류가 그대로 응답 task_type 이 된다.
                     (부분 유니크 인덱스로 로봇당 활성 task 는 최대 1건이라 dict 로 충분.)
      threshold    : 배터리 임계값(operation_battery_thresholds, 기본 70). BATTERY_TOO_LOW
                     기준. 여러 task_type 이 있으나 여기선 PATROL 기준을 '가용 표시용 단일
                     임계값'으로 쓴다(가용 조회 API 와 동일 관례).

    가벼운 SELECT 두 번뿐이라 1Hz 호출에도 부담이 없다(커넥션 풀 재사용).
    """
    with pool.connection() as conn:
        active_types = {
            r["assigned_robot_id"]: r["task_type"]
            for r in conn.execute(
                "SELECT assigned_robot_id, task_type FROM tasks "
                "WHERE assigned_robot_id IS NOT NULL "
                "AND status IN ('WAITING','IN_PROGRESS')"
            ).fetchall()
        }
        row = conn.execute(
            "SELECT min_battery_percent FROM operation_battery_thresholds "
            "WHERE task_type = 'PATROL'"
        ).fetchone()
        threshold = row["min_battery_percent"] if row else 70
    return active_types, threshold


# --------------------------------------------------------------------------- #
# 순찰 접수 트랜잭션 (①~④를 하나로 묶음, 예외 시 전체 롤백)
# --------------------------------------------------------------------------- #
_INSERT_TASK = (
    "INSERT INTO tasks (task_type, status, assigned_robot_id, created_at, updated_at) "
    "VALUES ('PATROL', 'WAITING', %s, NOW(), NOW()) RETURNING task_id"
)

# point_index 는 patrol_order 순서대로 0부터 '연속' 재부여(ROW_NUMBER-1).
# patrol_order 값에 구멍(1,2,4,7...)이 있어도 0,1,2,3...으로 촘촘히 채워진다.
_INSERT_PATHS = (
    "INSERT INTO task_paths (task_id, waypoint_id, point_index, is_visited, created_at, updated_at) "
    "SELECT %s, waypoint_id, ROW_NUMBER() OVER (ORDER BY patrol_order) - 1, "
    "       FALSE, NOW(), NOW() "
    "  FROM waypoints "
    " WHERE is_patrol_point = TRUE "
    " ORDER BY patrol_order"
)

# robot_state_snapshot: 앱에서 JSON 직렬화한 문자열을 %s로 바인딩 후 ::jsonb 캐스팅.
_INSERT_SNAPSHOT = (
    "INSERT INTO task_assignment_snapshot (task_id, robot_id, robot_state_snapshot, assigned_at) "
    "VALUES (%s, %s, %s::jsonb, NOW())"
)

_UPDATE_INPROGRESS = (
    "UPDATE tasks SET status = 'IN_PROGRESS', started_at = NOW(), updated_at = NOW() "
    "WHERE task_id = %s"
)

# 접수 후 디스패치용으로 순서대로 방문할 waypoint(좌표 포함)를 뽑는다.
_SELECT_PATH = (
    "SELECT tp.point_index, tp.waypoint_id, w.x_coord, w.y_coord "
    "  FROM task_paths tp "
    "  JOIN waypoints w ON w.waypoint_id = tp.waypoint_id "
    " WHERE tp.task_id = %s "
    " ORDER BY tp.point_index"
)


def accept_patrol_task(pool: ConnectionPool, robot_id: str,
                       snapshot_json: str) -> tuple:
    """순찰 task를 접수한다. ①~④를 하나의 트랜잭션으로 실행.

    ① tasks INSERT (PATROL/WAITING) -> task_id 확보
    ② task_paths 복사 (patrol_order 순, point_index 0부터 연속)
    ③ task_assignment_snapshot INSERT (명령 직전 로봇 상태 전체 JSONB)
    ④ tasks 를 IN_PROGRESS 로 전환

    반환: (task_id, waypoints)
      waypoints = [{"point_index", "waypoint_id", "x", "y"}, ...]  (디스패치용)

    예외:
      RobotBusyError  — 동일 로봇 활성 task 중복(부분 유니크 인덱스 위반).
                        트랜잭션은 자동 롤백된다.
    """
    try:
        with pool.connection() as conn:
            with conn.transaction():   # BEGIN ~ COMMIT/ROLLBACK 자동 관리
                task_id = conn.execute(_INSERT_TASK, (robot_id,)).fetchone()["task_id"]
                conn.execute(_INSERT_PATHS, (task_id,))
                conn.execute(_INSERT_SNAPSHOT, (task_id, robot_id, snapshot_json))
                conn.execute(_UPDATE_INPROGRESS, (task_id,))
            # 커밋 후(같은 커넥션의 새 트랜잭션)에서 경로를 읽어 반환
            rows = conn.execute(_SELECT_PATH, (task_id,)).fetchall()
        waypoints = [
            {
                "point_index": r["point_index"],
                "waypoint_id": r["waypoint_id"],
                "x": r["x_coord"],
                "y": r["y_coord"],
            }
            for r in rows
        ]
        return task_id, waypoints
    except psycopg.errors.UniqueViolation as exc:
        # ①의 INSERT가 ux_tasks_active_robot 을 위반한 경우
        raise RobotBusyError(str(exc)) from exc


# --------------------------------------------------------------------------- #
# 순찰 종료 반영
# --------------------------------------------------------------------------- #
def finish_patrol_task(pool: ConnectionPool, task_id: int,
                       result_code: int) -> None:
    """(Phase 1) Patrol result_code 에 따라 tasks 를 DONE/FAILED 로 마감한다.

    result_code == 0 -> DONE, 그 외(1 실패/막힘, 2 중단) -> FAILED.
    Phase 2 에서는 건너뜀이 생길 수 있어 set_task_status 로 PARTIAL 도 쓴다.
    """
    sql = (
        "UPDATE tasks "
        "   SET status = CASE WHEN %s = 0 THEN 'DONE' ELSE 'FAILED' END, "
        "       ended_at = NOW(), updated_at = NOW() "
        " WHERE task_id = %s"
    )
    with pool.connection() as conn:
        conn.execute(sql, (int(result_code), int(task_id)))


_VALID_END_STATUS = ("DONE", "FAILED", "PARTIAL")


def set_task_status(pool: ConnectionPool, task_id: int, status: str) -> None:
    """(Phase 2) tasks 를 명시 상태(DONE/PARTIAL/FAILED)로 마감한다.

    순찰 지점 일부만 방문(우회 실패로 건너뜀)한 경우 PARTIAL 을 쓴다.
    tasks.status CHECK 제약이 이 세 값을 허용한다(스키마 0001).
    """
    if status not in _VALID_END_STATUS:
        raise ValueError(f"허용되지 않은 종료 상태: {status}")
    sql = (
        "UPDATE tasks "
        "   SET status = %s, ended_at = NOW(), updated_at = NOW() "
        " WHERE task_id = %s"
    )
    with pool.connection() as conn:
        conn.execute(sql, (status, int(task_id)))


# --------------------------------------------------------------------------- #
# 라우팅 그래프 로드 (Phase 2) — corridors + waypoints 를 메모리 그래프 재료로 반환
# --------------------------------------------------------------------------- #
def load_graph(pool: ConnectionPool) -> dict:
    """토폴로지 그래프(노드=waypoints, 간선=corridors)를 읽어온다.

    반환:
      waypoints: [{"waypoint_id","x","y"}, ...]   (좌표는 하달 WaypointGoal 용)
      corridors: [{"corridor_id","a","b"}, ...]   (무방향 간선; a<b 관례로 저장돼 있음)

    RoutingEngine 은 이 두 리스트만으로 그래프를 구성한다(엔진은 DB를 모른다).
    corridors 가 비어 있으면(시드 미보강) 순찰 이동이 모두 skip 될 수 있다.
    """
    with pool.connection() as conn:
        waypoints = [
            {"waypoint_id": r["waypoint_id"], "x": r["x_coord"], "y": r["y_coord"]}
            for r in conn.execute(
                "SELECT waypoint_id, x_coord, y_coord FROM waypoints"
            ).fetchall()
        ]
        corridors = [
            {"corridor_id": r["corridor_id"],
             "a": r["waypoint_a_id"], "b": r["waypoint_b_id"]}
            for r in conn.execute(
                "SELECT corridor_id, waypoint_a_id, waypoint_b_id FROM corridors"
            ).fetchall()
        ]
    return {"waypoints": waypoints, "corridors": corridors}

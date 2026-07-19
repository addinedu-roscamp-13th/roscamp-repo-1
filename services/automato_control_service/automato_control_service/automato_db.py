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

핵심: 순찰 접수(①~③)는 '하나의 트랜잭션'으로 묶는다.
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


class PatrolInProgressError(Exception):
    """이미 활성(WAITING/IN_PROGRESS) PATROL task가 있어 새 순찰을 받을 수 없음.

    순찰은 전역에서 동시에 1건만 허용한다(운영 원칙). DB의 부분 유니크 인덱스
    (ux_tasks_single_active_patrol) 위반을 애플리케이션 예외로 바꿔 던진다.
    API 계층이 이걸 잡아 409(PATROL_IN_PROGRESS)로 응답한다. 로봇별 제약
    (RobotBusyError)과 달리 '어느 로봇이든 순찰이 이미 돌고 있으면' 거부한다.
    """


# --------------------------------------------------------------------------- #
# 접속 정보(DSN) 로딩
# --------------------------------------------------------------------------- #
def _find_env_file():
    """현재 작업 디렉터리(CWD)부터 위로 올라가며 services/database/.env 를 찾는다.

    왜 CWD 기준 상위 탐색인가:
      과거엔 이 파일(__file__) 기준 상대경로로 .env 를 찾았는데, colcon 이 코드를
      install/ 트리로 복사하면 그 상대경로가 깨져(.env 없는 곳을 가리켜) 조용히
      실패했다. 반면 개발자는 보통 리포 안에서 명령을 실행하므로, CWD 에서 위로
      올라가며 찾으면 소스/설치 트리 어디서 ros2 run 하든 리포 안이기만 하면
      .env 를 안정적으로 발견한다.
    """
    d = os.getcwd()
    while True:
        cand = os.path.join(d, "services", "database", ".env")
        if os.path.isfile(cand):
            return cand
        parent = os.path.dirname(d)
        if parent == d:          # 파일시스템 루트까지 갔는데 못 찾음
            return None
        d = parent


def _load_dsn() -> str:
    """실행 시 DATABASE_URL 환경변수로 접속한다(12-factor 관례: 설정은 환경에서).

    순서:
      ① 환경변수 DATABASE_URL 이 있으면 그대로 쓴다(운영/CI/명시 export 우선).
      ② 없으면 개발 편의로 리포의 services/database/.env 를 찾아 환경에 주입한다.
      ③ 그래도 없으면 '조용히 틀린 기본값으로 접속'하지 않고 명확한 에러로 죽는다.
         (과거엔 automato/automato 로 폴백해 엉뚱한 인증 실패의 원인 파악이 어려웠다.)

    SQLAlchemy 표기(postgresql+psycopg://)는 libpq 표기(postgresql://)로 정규화한다.
    """
    url = os.environ.get("DATABASE_URL")
    if not url:
        env_file = _find_env_file()
        if env_file:
            try:
                from dotenv import load_dotenv
                load_dotenv(env_file)
            except Exception:  # noqa: BLE001  (dotenv 미설치 등은 무시)
                pass
        url = os.environ.get("DATABASE_URL")

    if not url:
        raise RuntimeError(
            "DATABASE_URL 환경변수가 필요합니다. 리포 안에서 실행하거나"
            "(services/database/.env 자동 탐색), 직접 주입하세요: "
            "set -a; source services/database/.env; set +a")

    # SQLAlchemy 표기 -> libpq 표기 (psycopg 직접 연결용)
    return url.replace("postgresql+psycopg://", "postgresql://", 1)


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

# robot_state_snapshot: 앱에서 JSON 직렬화한 문자열을 %s로 바인딩 후 ::jsonb 캐스팅.
_INSERT_SNAPSHOT = (
    "INSERT INTO task_assignment_snapshot (task_id, robot_id, robot_state_snapshot, assigned_at) "
    "VALUES (%s, %s, %s::jsonb, NOW())"
)

_UPDATE_INPROGRESS = (
    "UPDATE tasks SET status = 'IN_PROGRESS', started_at = NOW(), updated_at = NOW() "
    "WHERE task_id = %s"
)

# 접수 후 디스패치용으로 순서대로 방문할 순찰 waypoint(좌표 포함)를 뽑는다.
# RP-88: 경로는 휘발성(실행 중 재계획)이라 task_paths 에 저장하지 않고,
# 순찰점(is_patrol_point)을 patrol_order 순으로 매번 직접 조회한다.
# point_index 는 0부터 '연속' 재부여(ROW_NUMBER-1) — patrol_order 에 구멍이 있어도 촘촘히.
_SELECT_PATROL_WAYPOINTS = (
    "SELECT ROW_NUMBER() OVER (ORDER BY patrol_order) - 1 AS point_index, "
    "       waypoint_id, x_coord, y_coord "
    "  FROM waypoints "
    " WHERE is_patrol_point = TRUE "
    " ORDER BY patrol_order"
)


def accept_patrol_task(pool: ConnectionPool, robot_id: str,
                       snapshot_json: str) -> tuple:
    """순찰 task를 접수한다. ①~③을 하나의 트랜잭션으로 실행.

    ① tasks INSERT (PATROL/WAITING) -> task_id 확보
    ② task_assignment_snapshot INSERT (명령 직전 로봇 상태 전체 JSONB)
    ③ tasks 를 IN_PROGRESS 로 전환
    커밋 후, 방문할 순찰 waypoint 목록을 waypoints 에서 직접 조회해 반환한다
    (RP-88: 경로는 휘발성이라 task_paths 로 저장하지 않음).

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
                conn.execute(_INSERT_SNAPSHOT, (task_id, robot_id, snapshot_json))
                conn.execute(_UPDATE_INPROGRESS, (task_id,))
            # 커밋 후, 방문할 순찰점을 waypoints 에서 직접 조회(경로는 저장 안 함)
            rows = conn.execute(_SELECT_PATROL_WAYPOINTS).fetchall()
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
        # ①의 INSERT 가 '어느' 부분 유니크 인덱스를 위반했는지로 사유를 구분한다.
        #   ux_tasks_single_active_patrol → 이미 순찰 진행 중(전역 1건 제약)
        #   ux_tasks_active_robot         → 그 로봇이 이미 활성 task 보유
        # psycopg 는 위반한 인덱스명을 exc.diag.constraint_name 으로 알려준다.
        if getattr(exc.diag, "constraint_name", None) == "ux_tasks_single_active_patrol":
            raise PatrolInProgressError(str(exc)) from exc
        raise RobotBusyError(str(exc)) from exc


# --------------------------------------------------------------------------- #
# 순찰 종료 반영
# --------------------------------------------------------------------------- #
_VALID_END_STATUS = ("COMPLETED", "FAILED", "COMPLETED_PARTIAL")


def set_task_status(pool: ConnectionPool, task_id: int, status: str) -> None:
    """(Phase 2) tasks 를 명시 상태(COMPLETED/COMPLETED_PARTIAL/FAILED)로 마감한다.

    순찰 지점 일부만 방문(우회 실패로 건너뜀)한 경우 COMPLETED_PARTIAL 을 쓴다.
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
      waypoints: [{"waypoint_id","x","y","yaw","is_patrol_point"}, ...]
                  (하달 Waypoint 용. yaw=지점 방향(rad, 비순찰점은 None),
                   is_patrol_point → Waypoint.capture 판정에 사용)
      corridors: [{"corridor_id","a","b","length"}, ...]  (무방향 간선; a<b 관례.
                  length = 간선 비용(두 waypoint 유클리드 거리, m). Dijkstra 가 사용)

    RoutingEngine 은 이 두 리스트만으로 그래프를 구성한다(엔진은 DB를 모른다).
    corridors 가 비어 있으면(시드 미보강) 순찰 이동이 모두 skip 될 수 있다.
    """
    with pool.connection() as conn:
        waypoints = [
            {"waypoint_id": r["waypoint_id"], "x": r["x_coord"], "y": r["y_coord"],
             "yaw": r["yaw_coord"], "is_patrol_point": r["is_patrol_point"]}
            for r in conn.execute(
                "SELECT waypoint_id, x_coord, y_coord, yaw_coord, is_patrol_point "
                "FROM waypoints"
            ).fetchall()
        ]
        corridors = [
            {"corridor_id": r["corridor_id"],
             "a": r["waypoint_a_id"], "b": r["waypoint_b_id"],
             "length": r["length"]}
            for r in conn.execute(
                "SELECT corridor_id, waypoint_a_id, waypoint_b_id, length FROM corridors"
            ).fetchall()
        ]
    return {"waypoints": waypoints, "corridors": corridors}

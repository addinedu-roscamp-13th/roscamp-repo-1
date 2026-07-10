"""initial schema — Automato DB (시나리오 1: 순찰·관제)

Confluence "DB ERD" (v27) 기준 12개 테이블 + updated_at 트리거 + 인덱스.
MySQL 표기 -> PostgreSQL 변환:
  - INT AUTO_INCREMENT            -> INTEGER GENERATED ALWAYS AS IDENTITY
  - ENUM('A','B')                 -> VARCHAR + CHECK (팀 결정)
  - TIMESTAMP ON UPDATE ...       -> set_updated_at() 트리거
  - FLOAT                         -> DOUBLE PRECISION
  - TIMESTAMP                     -> TIMESTAMPTZ (UTC 시점 저장, 타임존 버그 예방)
ERD 오타 정정:
  - task_paths.point_index 의 DEFAULT FALSE -> DEFAULT 0
ERD v27 반영:
  - tasks.status 에 'PARTIAL' 추가
  - detection_logs.disease_image_path (VARCHAR(255)) 추가
  - corridors 테이블 신설 (waypoint 간 무방향 간선). 문서의
    "waypoint_a_id < waypoint_b_id 저장 관례"를 CHECK 제약으로 강제.
  - waypoints.yaw_coord (DOUBLE PRECISION, NULLABLE) 추가. 순찰점만 방향이
    필요하고(카메라가 오른쪽에 달려 왼쪽만 촬영) 비순찰점은 NULL.

Revision ID: 0001
Revises:
Create Date: 2026-07-07
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0001"
down_revision: Union[str, None] = None
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None

# updated_at 이 있어 트리거를 붙일 테이블 (unload_logs/event_logs/snapshot 은 updated_at 없음)
_UPDATED_AT_TABLES = [
    "waypoints",
    "robots",
    "task_points",
    "tasks",
    "task_paths",
    "detection_logs",
    "harvest_batches",
    "operation_battery_thresholds",
    "corridors",
]

# downgrade 시 FK 자식 -> 부모 역순으로 DROP
_DROP_ORDER = [
    "corridors",
    "task_assignment_snapshot",
    "event_logs",
    "unload_logs",
    "harvest_batches",
    "detection_logs",
    "task_paths",
    "operation_battery_thresholds",
    "tasks",
    "task_points",
    "robots",
    "waypoints",
]


def upgrade() -> None:
    # 0) updated_at 자동 갱신 트리거 함수
    op.execute(
        """
        CREATE OR REPLACE FUNCTION set_updated_at() RETURNS trigger AS $$
        BEGIN
            NEW.updated_at = now();
            RETURN NEW;
        END;
        $$ LANGUAGE plpgsql;
        """
    )

    # 1) 마스터 테이블 (FK 부모, 의존 없음) -----------------------------
    op.execute(
        """
        CREATE TABLE waypoints (
            waypoint_id     INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            x_coord         DOUBLE PRECISION NOT NULL,
            y_coord         DOUBLE PRECISION NOT NULL,
            yaw_coord       DOUBLE PRECISION,
            is_patrol_point BOOLEAN NOT NULL DEFAULT FALSE,
            patrol_order    INTEGER,
            created_at      TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at      TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )
    op.execute(
        """
        CREATE TABLE robots (
            robot_id   VARCHAR(50) PRIMARY KEY,
            robot_name VARCHAR(50),
            created_at TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )
    op.execute(
        """
        CREATE TABLE task_points (
            task_point_id VARCHAR(50) PRIMARY KEY,
            x_coord       DOUBLE PRECISION NOT NULL,
            y_coord       DOUBLE PRECISION NOT NULL,
            yaw_coord     DOUBLE PRECISION NOT NULL,
            created_at    TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at    TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )

    # 2) tasks (task_points, robots 참조) ------------------------------
    op.execute(
        """
        CREATE TABLE tasks (
            task_id           INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            task_type         VARCHAR(20) NOT NULL
                              CHECK (task_type IN ('PATROL','HARVEST','TRANSFER')),
            status            VARCHAR(20) NOT NULL DEFAULT 'WAITING'
                              CHECK (status IN ('WAITING','IN_PROGRESS','DONE','FAILED','PARTIAL')),
            priority          INTEGER NOT NULL DEFAULT 0,
            task_point_id     VARCHAR(50) REFERENCES task_points(task_point_id),
            assigned_robot_id VARCHAR(50) REFERENCES robots(robot_id),
            started_at        TIMESTAMPTZ,
            ended_at          TIMESTAMPTZ,
            scheduled_at      TIMESTAMPTZ,
            created_at        TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at        TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )

    # 3) tasks 에 의존하는 자식 테이블들 --------------------------------
    op.execute(
        """
        CREATE TABLE task_paths (
            task_path_id INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            task_id      INTEGER NOT NULL REFERENCES tasks(task_id),
            waypoint_id  INTEGER NOT NULL REFERENCES waypoints(waypoint_id),
            point_index  INTEGER NOT NULL DEFAULT 0,
            is_visited   BOOLEAN,
            created_at   TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at   TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )
    op.execute(
        """
        CREATE TABLE detection_logs (
            detection_id    INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            task_id         INTEGER REFERENCES tasks(task_id),
            robot_id        VARCHAR(50) NOT NULL REFERENCES robots(robot_id),
            ripe_percent    INTEGER,
            unripe_percent  INTEGER,
            rotten_percent  INTEGER,
            disease_percent INTEGER,
            disease_image_path VARCHAR(255),
            waypoint_id     INTEGER NOT NULL REFERENCES waypoints(waypoint_id),
            detected_at     TIMESTAMPTZ NOT NULL,
            created_at      TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at      TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )
    op.execute(
        """
        CREATE TABLE harvest_batches (
            batch_id      INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            task_id       INTEGER NOT NULL REFERENCES tasks(task_id),
            robot_id      VARCHAR(50) NOT NULL REFERENCES robots(robot_id),
            normal_count  INTEGER NOT NULL DEFAULT 0,
            discard_count INTEGER NOT NULL DEFAULT 0,
            created_at    TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at    TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )
    op.execute(
        """
        CREATE TABLE unload_logs (
            unload_id   INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            task_id     INTEGER NOT NULL REFERENCES tasks(task_id),
            robot_id    VARCHAR(50) NOT NULL REFERENCES robots(robot_id),
            normal_qty  INTEGER NOT NULL,
            discard_qty INTEGER NOT NULL,
            created_at  TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )
    op.execute(
        """
        CREATE TABLE event_logs (
            event_id   INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            robot_id   VARCHAR(50) REFERENCES robots(robot_id),
            task_id    INTEGER REFERENCES tasks(task_id),
            event_type VARCHAR(20) NOT NULL
                       CHECK (event_type IN ('BATTERY_LOW','OBSTACLE_STOP','TRAFFIC_CONTROL','HARDWARE_ERROR')),
            severity   VARCHAR(10) NOT NULL
                       CHECK (severity IN ('INFO','WARN','CRITICAL')),
            message    TEXT NOT NULL,
            created_at TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )
    op.execute(
        """
        CREATE TABLE task_assignment_snapshot (
            id                   BIGINT GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            task_id              INTEGER NOT NULL REFERENCES tasks(task_id),
            robot_id             VARCHAR(50) NOT NULL REFERENCES robots(robot_id),
            assigned_at          TIMESTAMPTZ NOT NULL DEFAULT now(),
            robot_state_snapshot JSONB NOT NULL
        );
        """
    )

    # 4) 설정 테이블 (FK 없음) -----------------------------------------
    op.execute(
        """
        CREATE TABLE operation_battery_thresholds (
            threshold_id        INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            task_type           VARCHAR(20) NOT NULL UNIQUE
                                CHECK (task_type IN ('PATROL','HARVEST','TRANSFER')),
            min_battery_percent INTEGER NOT NULL,
            created_at          TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at          TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )

    # 4-1) corridors — 통로 그래프의 무방향 간선 (waypoints 참조) --------
    #   waypoint_a_id < waypoint_b_id 로만 저장(관례)해 한 쌍당 1행.
    #   CHECK 로 순서를 강제하고 UNIQUE 로 중복 간선을 막는다.
    op.execute(
        """
        CREATE TABLE corridors (
            corridor_id   INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            waypoint_a_id INTEGER NOT NULL REFERENCES waypoints(waypoint_id),
            waypoint_b_id INTEGER NOT NULL REFERENCES waypoints(waypoint_id),
            created_at    TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at    TIMESTAMPTZ NOT NULL DEFAULT now(),
            CONSTRAINT ck_corridors_order CHECK (waypoint_a_id < waypoint_b_id),
            CONSTRAINT ux_corridors_pair  UNIQUE (waypoint_a_id, waypoint_b_id)
        );
        """
    )

    # 5) updated_at 트리거 부착 ----------------------------------------
    for tbl in _UPDATED_AT_TABLES:
        op.execute(
            f"""
            CREATE TRIGGER trg_{tbl}_updated_at
            BEFORE UPDATE ON {tbl}
            FOR EACH ROW EXECUTE FUNCTION set_updated_at();
            """
        )

    # 6) 부분 유니크 인덱스 — 동일 로봇 활성(대기/수행중) task 중복 배정 방지
    op.execute(
        """
        CREATE UNIQUE INDEX ux_tasks_active_robot ON tasks (assigned_robot_id)
        WHERE assigned_robot_id IS NOT NULL
          AND status IN ('WAITING','IN_PROGRESS');
        """
    )

    # 7) 조회/조인 성능용 인덱스 (PG는 FK에 인덱스를 자동 생성하지 않음)
    op.execute("CREATE INDEX idx_tasks_status ON tasks (status);")
    op.execute("CREATE INDEX idx_tasks_assigned_robot ON tasks (assigned_robot_id);")
    op.execute("CREATE INDEX idx_tasks_task_point ON tasks (task_point_id);")
    op.execute("CREATE INDEX idx_task_paths_task ON task_paths (task_id);")
    op.execute("CREATE INDEX idx_task_paths_waypoint ON task_paths (waypoint_id);")
    op.execute("CREATE INDEX idx_detection_logs_task ON detection_logs (task_id);")
    op.execute("CREATE INDEX idx_detection_logs_robot ON detection_logs (robot_id);")
    op.execute("CREATE INDEX idx_detection_logs_waypoint ON detection_logs (waypoint_id);")
    op.execute("CREATE INDEX idx_detection_logs_detected_at ON detection_logs (detected_at);")
    op.execute("CREATE INDEX idx_harvest_batches_task ON harvest_batches (task_id);")
    op.execute("CREATE INDEX idx_harvest_batches_robot ON harvest_batches (robot_id);")
    op.execute("CREATE INDEX idx_unload_logs_task ON unload_logs (task_id);")
    op.execute("CREATE INDEX idx_unload_logs_robot ON unload_logs (robot_id);")
    op.execute("CREATE INDEX idx_event_logs_robot ON event_logs (robot_id);")
    op.execute("CREATE INDEX idx_event_logs_task ON event_logs (task_id);")
    op.execute("CREATE INDEX idx_event_logs_created_at ON event_logs (created_at);")
    op.execute("CREATE INDEX idx_snapshot_task ON task_assignment_snapshot (task_id);")
    op.execute("CREATE INDEX idx_snapshot_robot ON task_assignment_snapshot (robot_id);")
    # corridors: (a,b) 복합은 UNIQUE 제약이 인덱스를 자동 생성 → a 선두 조회는 그걸로 커버.
    # 반대 끝(b) 단독 조회용 인덱스만 별도로 추가한다.
    op.execute("CREATE INDEX idx_corridors_b ON corridors (waypoint_b_id);")


def downgrade() -> None:
    # 테이블을 FK 역순으로 DROP (인덱스/트리거는 테이블과 함께 삭제됨)
    for tbl in _DROP_ORDER:
        op.execute(f"DROP TABLE IF EXISTS {tbl} CASCADE;")
    op.execute("DROP FUNCTION IF EXISTS set_updated_at();")

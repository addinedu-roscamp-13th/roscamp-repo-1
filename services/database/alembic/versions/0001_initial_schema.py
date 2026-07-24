"""initial schema — Automato DB (시나리오 1·2: 순찰·관제·수확)

Confluence "DB ERD" (v33) 기준 12개 테이블 + updated_at 트리거 + 인덱스.
MySQL 표기 -> PostgreSQL 변환:
  - INT AUTO_INCREMENT            -> INTEGER GENERATED ALWAYS AS IDENTITY
  - ENUM('A','B')                 -> VARCHAR + CHECK (팀 결정)
  - TIMESTAMP ON UPDATE ...       -> set_updated_at() 트리거
  - FLOAT                         -> DOUBLE PRECISION
  - TIMESTAMP                     -> TIMESTAMPTZ (UTC 시점 저장, 타임존 버그 예방)
ERD v33(시나리오 2 설계) 반영 — RP-88:
  - task_paths 테이블 폐기(삭제). 경로는 실행 중 재계획되는 휘발성 데이터라
    ACS 메모리에서 관리하고, DB에는 요청(tasks)과 결과만 영속화한다.
  - charuco_boards 테이블 신설. 정밀 도킹용 ChArUco 보드/도킹 오프셋(task_points 1:1).
    단일 ArUco 가 아니라 보드를 쓰므로 보드 구성(squares_x/y, square_size_m)까지 담는다.
  - robots.charge_point_id (FK -> task_points, NULLABLE) 추가. 로봇별 전용 충전소.
  - task_points.point_type (ENUM HARVEST/PRECOOL/CHARGE) 추가. 작업 위치 종류.
  - task_points 의 좌표(x/y/yaw) -> waypoint_id (FK -> waypoints, UNIQUE) 로 교체.
    작업 지점의 위치를 좌표로 중복 보관하지 않고 '진입 노드'로 가리킨다.
  - harvest_batches.failed_count / exit_reason 추가. 수확 성공률·종료 사유.
  - tasks.status 값을 DONE/PARTIAL -> COMPLETED/COMPLETED_PARTIAL 로 문서와 일치시킴.
ERD 후속 반영 — 촬영 짝(pair) 지점:
  - waypoints.pair_waypoint_id (자기참조 FK, NULLABLE) 추가. 같은 위치에서 촬영 방향(yaw)만
    다른 '짝' 지점을 표현한다. 카메라가 로봇 한쪽에 고정돼 통로를 한 번 지나면 한쪽 베드만
    찍히므로, 같은 자리에서 180° 돌려 한 번 더 촬영한다. 값이 있는 행 = 짝(경로 탐색 대상
    아님, corridors 에 등장하지 않음), NULL = 그래프에 포함되는 독립 지점.
이전(v27)부터 이미 반영된 것:
  - tasks.status 다상태화, detection_logs.disease_image_path (VARCHAR(255)),
  - corridors 테이블(waypoint 간 무방향 간선, a<b CHECK + UNIQUE),
  - waypoints.yaw_coord(NULLABLE, 순찰점만 방향값·비순찰점 NULL).

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
    "detection_logs",
    "harvest_batches",
    "operation_battery_thresholds",
    "corridors",
    "charuco_boards",
]

# downgrade 시 FK 자식 -> 부모 역순으로 DROP
# (charuco_boards, robots 모두 task_points 를 참조하므로 task_points 보다 먼저 지운다)
_DROP_ORDER = [
    "corridors",
    "charuco_boards",
    "task_assignment_snapshot",
    "event_logs",
    "unload_logs",
    "harvest_batches",
    "detection_logs",
    "operation_battery_thresholds",
    "tasks",
    "robots",
    "task_points",
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

    # 1) 마스터 테이블 -------------------------------------------------
    #   robots.charge_point_id 가 task_points 를 참조하므로 task_points 를 먼저 만든다.
    op.execute(
        """
        CREATE TABLE waypoints (
            waypoint_id      INTEGER GENERATED ALWAYS AS IDENTITY PRIMARY KEY,
            x_coord          DOUBLE PRECISION NOT NULL,
            y_coord          DOUBLE PRECISION NOT NULL,
            yaw_coord        DOUBLE PRECISION,
            is_patrol_point  BOOLEAN NOT NULL DEFAULT FALSE,
            -- 같은 위치에서 촬영 방향만 다른 '짝' 지점의 부모를 가리키는 자기참조.
            -- NULL 이면 경로 탐색 그래프에 포함되는 독립 지점, 값이 있으면 추가 촬영 전용 행.
            -- FK 를 거는 이유: 부모를 지웠을 때 짝이 없는 id 를 가리키는 고아 행이 되는 걸
            -- DB 가 막아준다. ON DELETE CASCADE 는 일부러 안 건다 — 순찰 지점 하나를 실수로
            -- 지웠을 때 두 행이 조용히 사라지면 미촬영 구간이 소리 없이 생긴다.
            pair_waypoint_id INTEGER REFERENCES waypoints(waypoint_id),
            patrol_order     INTEGER,
            created_at       TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at       TIMESTAMPTZ NOT NULL DEFAULT now(),
            -- 자기 자신을 짝으로 가리키는 행 차단(ERD 명시 제약).
            CONSTRAINT chk_pair_not_self CHECK (pair_waypoint_id <> waypoint_id)
        );
        """
    )
    op.execute(
        """
        CREATE TABLE task_points (
            task_point_id VARCHAR(50) PRIMARY KEY,
            point_type    VARCHAR(20) NOT NULL
                          CHECK (point_type IN ('HARVEST','PRECOOL','CHARGE')),
            -- 이 작업 지점에 접근하기 위한 '진입 노드'. 좌표를 여기에 또 적지 않고
            -- waypoints 를 가리킨다 — 같은 위치가 두 테이블에 따로 적히면 지도를 고칠 때
            -- 한쪽만 바뀌어 소리 없이 어긋난다(단일 진실 공급원).
            -- ACS 는 이 노드를 목적지로 경로를 계산하고, 도착 후 Dock 액션으로 전환한다.
            -- UNIQUE: 진입 노드 하나를 두 작업 지점이 나눠 쓰지 않는다(1:1).
            waypoint_id   INTEGER NOT NULL UNIQUE REFERENCES waypoints(waypoint_id),
            created_at    TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at    TIMESTAMPTZ NOT NULL DEFAULT now()
        );
        """
    )
    op.execute(
        """
        CREATE TABLE robots (
            robot_id        VARCHAR(50) PRIMARY KEY,
            robot_name      VARCHAR(50),
            charge_point_id VARCHAR(50) REFERENCES task_points(task_point_id),
            created_at      TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at      TIMESTAMPTZ NOT NULL DEFAULT now()
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
                              CHECK (status IN ('WAITING','IN_PROGRESS','COMPLETED','FAILED','COMPLETED_PARTIAL')),
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
            failed_count  INTEGER NOT NULL DEFAULT 0,
            exit_reason   VARCHAR(20)
                          CHECK (exit_reason IN ('DEPLETED','FULL','MAX_ROUNDS_EXCEEDED')),
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

    # 4-2) charuco_boards — 정밀 도킹용 ChArUco 보드 (task_points 참조) ---
    #   단일 ArUco 가 아니라 ChArUco(체스판에 마커를 심은 보드)를 쓴다. 자세를 마커
    #   네 귀퉁이가 아니라 체스판 코너에서 얻어 서브픽셀 정밀도가 나오고, 접붙임 직전
    #   보드가 잘려도 남은 코너로 계산된다. 그래서 마커 크기만이 아니라 보드 구성
    #   (칸수·칸 크기)까지 저장해야 검출기가 보드 모델을 만들 수 있다.
    #   marker_id 는 보드가 점유하는 마커 ID 범위의 '시작 번호'(자연키)라 IDENTITY 를
    #   쓰지 않는다. 보드 1장이 floor(squares_x*squares_y/2) 개를 연속 점유한다.
    #   task_point_id UNIQUE — 작업 위치 1곳당 보드 1장.
    op.execute(
        """
        CREATE TABLE charuco_boards (
            marker_id       VARCHAR(20) PRIMARY KEY,
            task_point_id   VARCHAR(50) NOT NULL UNIQUE
                            REFERENCES task_points(task_point_id),
            dictionary      VARCHAR(30) NOT NULL DEFAULT 'DICT_5X5_1000',
            squares_x       INTEGER NOT NULL,
            squares_y       INTEGER NOT NULL,
            square_size_m   DOUBLE PRECISION NOT NULL,
            marker_size_m   DOUBLE PRECISION NOT NULL,
            dock_offset_x   DOUBLE PRECISION NOT NULL,
            dock_offset_y   DOUBLE PRECISION NOT NULL,
            dock_offset_yaw DOUBLE PRECISION NOT NULL,
            created_at      TIMESTAMPTZ NOT NULL DEFAULT now(),
            updated_at      TIMESTAMPTZ NOT NULL DEFAULT now(),
            -- 마커는 체스판 칸 안에 인쇄되므로 물리적으로 항상 칸보다 작다.
            -- 뒤집어 넣어도 검출은 되고 거리 추정만 조용히 틀어져서 제약으로 막는다.
            CONSTRAINT ck_charuco_marker_smaller
                CHECK (marker_size_m < square_size_m),
            CONSTRAINT ck_charuco_board_min_size
                CHECK (squares_x >= 3 AND squares_y >= 3)
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
    op.execute("CREATE INDEX idx_robots_charge_point ON robots (charge_point_id);")
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

"""seed — 초기 시드 데이터

- operation_battery_thresholds: PATROL=70, HARVEST=50, TRANSFER=50  (RP-82 명시)
- robots: dg_01~03  (후속 작업 편의용 마스터. 불필요하면 이 블록만 삭제)
- waypoints: 실제 맵 기반 19개 점 (순찰점 12 + 비순찰점 7).
    · CSV(waypoints_final.csv) 순서대로 INSERT → waypoint_id 1~19 부여.
    · 순찰점만 yaw_coord/patrol_order 값이 있고, 비순찰점은 둘 다 NULL.
    · patrol_order 는 방문 순서대로 1~12 연속 부여(순찰점 12개).

Revision ID: 0002
Revises: 0001
Create Date: 2026-07-07
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0002"
down_revision: Union[str, None] = "0001"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # 1) 기능별 배터리 임계값 (기능당 1행, task_type UNIQUE)
    op.execute(
        """
        INSERT INTO operation_battery_thresholds (task_type, min_battery_percent) VALUES
            ('PATROL',   70),
            ('HARVEST',  50),
            ('TRANSFER', 50);
        """
    )

    # 2) 로봇 마스터 (Ddago 주행 로봇 3대)
    op.execute(
        """
        INSERT INTO robots (robot_id, robot_name) VALUES
            ('dg_01', 'ddagoddagi_01'),
            ('dg_02', 'ddagoddagi_02'),
            ('dg_03', 'ddagoddagi_03');
        """
    )

    # 3) 실제 맵 기반 순찰/경로 지점 19개 (CSV 순서 = waypoint_id 1~19)
    #    순찰점(is_patrol_point=TRUE)만 yaw_coord/patrol_order 를 가진다.
    op.execute(
        """
        INSERT INTO waypoints (x_coord, y_coord, yaw_coord, is_patrol_point, patrol_order) VALUES
            ( 0.7,   -0.446,  0.001, TRUE,   4),   -- id 1
            ( 0.416, -0.443, -0.017, TRUE,   3),   -- id 2
            ( 0.024, -0.397,  NULL,  FALSE,  NULL),-- id 3
            ( 0.715, -0.003,  NULL,  FALSE,  NULL),-- id 4
            ( 0.66,  -0.018,  3.111, TRUE,   1),   -- id 5
            ( 0.389, -0.008,  0.017, TRUE,   2),   -- id 6
            ( 0.354, -0.002,  NULL,  FALSE,  NULL),-- id 7
            ( 0.037, -0.005,  NULL,  FALSE,  NULL),-- id 8
            ( 0.75,   0.263,  1.496, TRUE,   9),   -- id 9
            ( 0.314,  0.261, -1.626, TRUE,   5),   -- id 10
            (-0.011,  0.25,  -1.559, TRUE,   8),   -- id 11
            ( 0.738,  0.498,  1.609, TRUE,  10),   -- id 12
            ( 0.323,  0.491, -1.649, TRUE,   6),   -- id 13
            (-0.013,  0.461, -1.581, TRUE,   7),   -- id 14
            ( 0.706,  0.791,  NULL,  FALSE,  NULL),-- id 15
            ( 0.355,  0.755,  NULL,  FALSE,  NULL),-- id 16
            (-0.016,  0.798,  NULL,  FALSE,  NULL),-- id 17
            ( 0.382,  0.282,  1.493, TRUE,  12),   -- id 18
            ( 0.4,    0.495,  1.493, TRUE,  11);   -- id 19
        """
    )


def downgrade() -> None:
    # 시드 롤백 (역순). 참조가 걸리기 전(클린 상태)에만 안전하게 삭제됨.
    # waypoints 는 이 시드로만 채워지므로(순찰점+비순찰점 전부) 통째로 비운다.
    op.execute("DELETE FROM waypoints;")
    op.execute("DELETE FROM robots WHERE robot_id IN ('dg_01','dg_02','dg_03');")
    op.execute(
        "DELETE FROM operation_battery_thresholds "
        "WHERE task_type IN ('PATROL','HARVEST','TRANSFER');"
    )

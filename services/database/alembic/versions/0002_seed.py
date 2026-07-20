"""seed — 초기 시드 데이터

- operation_battery_thresholds: PATROL=70, HARVEST=50, TRANSFER=50  (RP-82 명시)
- waypoints: 실제 맵 기반 25개 점 (순찰점 12 + 통로 경유점 7 + 도킹 진입점 6).
    · CSV(waypoints_260720.csv) 순서대로 INSERT → waypoint_id 1~25 부여.
    · 순찰점(is_patrol_point=TRUE)과 도킹 진입점은 yaw_coord 를 갖는다.
      순찰점의 yaw 는 '촬영 방향'(카메라가 한쪽에 고정), 도킹 진입점의 yaw 는 '진입 헤딩'.
      순수 통로 경유점(3·4·7·8·15·16·17)만 yaw 가 NULL 이다.
    · 짝(pair) 지점 2개: 18(부모 10), 19(부모 13). 부모와 x·y 가 완전히 동일하고 yaw 만 다르다.
      로봇은 '직전 waypoint 와 좌표가 같으면 제자리 회전(Spin), 다르면 주행'으로 분기하므로
      좌표가 몇 cm 라도 어긋나면 회전 대신 주행을 시도하다 좁은 통로에서 실패한다.
    · patrol_order 는 부모 순찰점에만 부여한다(짝은 NULL). 짝은 부모 직후에 방문하도록
      ACS 가 끼워 넣기 때문에 별도 순서를 갖지 않는다.
      결번(6·8)은 짝이 빠진 자리 — 조회 측이 ROW_NUMBER 로 0부터 다시 매기므로 무해하다.
- task_points: 작업 위치 6곳. 좌표 대신 '진입 노드'(waypoints 20~25)를 FK 로 가리킨다.
- robots: dg_01~03 + 로봇별 전용 충전소(CHARGE_01~03) 연결.

삽입 순서가 곧 FK 의존 순서다: waypoints → task_points → robots.
(robots.charge_point_id → task_points.task_point_id → waypoints.waypoint_id)

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

    # 2) 실제 맵 기반 지점 25개 (CSV 순서 = waypoint_id 1~25)
    #    waypoint_id 는 GENERATED ALWAYS AS IDENTITY 라 명시 삽입이 불가능하다.
    #    → 'CSV 행 순서 = 부여될 id' 라는 전제로 넣는다. 짝(18·19)이 가리키는 부모(10·13)가
    #      앞에 있어 자기참조 FK 도 이 순서 그대로 만족한다.
    op.execute(
        """
        INSERT INTO waypoints
            (x_coord, y_coord, yaw_coord, is_patrol_point, pair_waypoint_id, patrol_order) VALUES
            -- ── 순찰점 + 통로 경유점 (1~17) ──────────────────────────────────
            ( 0.7,   -0.446,  0.001, TRUE,  NULL,  12),   -- id 1
            ( 0.416, -0.443, -0.017, TRUE,  NULL,  11),   -- id 2
            ( 0.024, -0.397,  NULL,  FALSE, NULL,  NULL), -- id 3
            ( 0.715, -0.003,  NULL,  FALSE, NULL,  NULL), -- id 4
            ( 0.66,  -0.018,  3.111, TRUE,  NULL,   3),   -- id 5
            ( 0.389, -0.008,  0.017, TRUE,  NULL,   4),   -- id 6
            ( 0.354, -0.002,  NULL,  FALSE, NULL,  NULL), -- id 7
            ( 0.037, -0.005,  NULL,  FALSE, NULL,  NULL), -- id 8
            ( 0.75,   0.263,  1.496, TRUE,  NULL,   2),   -- id 9
            ( 0.314,  0.261, -1.626, TRUE,  NULL,   5),   -- id 10  (짝 18 의 부모)
            (-0.011,  0.25,  -1.559, TRUE,  NULL,  10),   -- id 11
            ( 0.738,  0.498,  1.609, TRUE,  NULL,   1),   -- id 12  순찰 첫 지점
            ( 0.323,  0.491, -1.649, TRUE,  NULL,   7),   -- id 13  (짝 19 의 부모)
            (-0.013,  0.461, -1.581, TRUE,  NULL,   9),   -- id 14
            ( 0.706,  0.791,  NULL,  FALSE, NULL,  NULL), -- id 15  순찰 시작(충전소 앞) 노드
            ( 0.355,  0.755,  NULL,  FALSE, NULL,  NULL), -- id 16
            (-0.016,  0.798,  NULL,  FALSE, NULL,  NULL), -- id 17
            -- ── 짝 지점 (18·19): 부모와 x·y 동일, yaw 만 반대. corridors 에 넣지 않는다 ──
            ( 0.314,  0.261,  1.493, TRUE,    10,  NULL), -- id 18  = 10 의 반대 방향 촬영
            ( 0.323,  0.491,  1.493, TRUE,    13,  NULL), -- id 19  = 13 의 반대 방향 촬영
            -- ── 도킹 진입 노드 (20~25): 주행 그래프 노드이지만 순찰 대상은 아니다 ────
            ( 0.717, -0.449, -1.711, FALSE, NULL,  NULL), -- id 20
            (-0.059, -0.438, -1.616, FALSE, NULL,  NULL), -- id 21
            ( 0.77,   0.92,   1.45,  FALSE, NULL,  NULL), -- id 22  충전소
            ( 0.633,  0.91,   1.448, FALSE, NULL,  NULL), -- id 23  충전소
            ( 0.468,  0.921,  1.523, FALSE, NULL,  NULL), -- id 24  충전소(로봇3)
            (-0.026,  0.863,  1.512, FALSE, NULL,  NULL); -- id 25
        """
    )

    # 2-1) 자체 검증 — 짝의 좌표가 부모와 다르면 여기서 멈춘다(트랜잭션 롤백).
    #      좌표가 어긋나면 로봇이 '제자리 회전' 대신 '주행'으로 분기해 좁은 통로에서 실패하는데,
    #      현장에서는 원인을 찾기가 매우 어렵다. 넣는 시점에 막는 편이 훨씬 싸다.
    op.execute(
        """
        DO $$
        BEGIN
            IF EXISTS (
                SELECT 1
                  FROM waypoints c
                  JOIN waypoints p ON p.waypoint_id = c.pair_waypoint_id
                 WHERE c.x_coord <> p.x_coord OR c.y_coord <> p.y_coord
            ) THEN
                RAISE EXCEPTION '짝 waypoint 의 좌표가 부모와 다릅니다 — 제자리 회전이 아니라 주행으로 분기해 실패합니다';
            END IF;
        END $$;
        """
    )

    # 3) 작업 위치 6곳 — 각 지점의 '진입 노드'(waypoints 20~25)를 가리킨다.
    #    waypoints 보다 뒤에 넣어야 FK 를 만족한다(진입 노드가 먼저 존재해야 함).
    #    수확 2곳(HARVEST_01·02), 로봇별 전용 충전소 3곳(CHARGE_01~03), 예냉실 1곳(PRECOOL_01).
    op.execute(
        """
        INSERT INTO task_points (task_point_id, point_type, waypoint_id) VALUES
            ('HARVEST_01', 'HARVEST', 20),
            ('HARVEST_02', 'HARVEST', 21),
            ('CHARGE_01',  'CHARGE',  22),
            ('CHARGE_02',  'CHARGE',  23),
            ('CHARGE_03',  'CHARGE',  24),
            ('PRECOOL_01', 'PRECOOL', 25);
        """
    )

    # 4) 로봇 마스터 (Ddago 주행 로봇 3대) — 로봇별 전용 충전소를 함께 지정한다.
    #    charge_point_id 가 task_points 를 참조하므로 3) 뒤에 와야 한다.
    op.execute(
        """
        INSERT INTO robots (robot_id, robot_name, charge_point_id) VALUES
            ('dg_01', 'ddagoddagi_01', 'CHARGE_01'),
            ('dg_02', 'ddagoddagi_02', 'CHARGE_02'),
            ('dg_03', 'ddagoddagi_03', 'CHARGE_03');
        """
    )


def downgrade() -> None:
    # 시드 롤백 — 참조하는 쪽부터 지운다(upgrade 의 정확한 역순).
    #   robots ─(charge_point_id)→ task_points ─(waypoint_id)→ waypoints
    # 순서를 어기면 FK 위반으로 롤백 자체가 실패한다.
    op.execute("DELETE FROM robots WHERE robot_id IN ('dg_01','dg_02','dg_03');")
    op.execute("DELETE FROM task_points;")
    # waypoints 는 이 시드로만 채워지므로(순찰점+비순찰점 전부) 통째로 비운다.
    op.execute("DELETE FROM waypoints;")
    op.execute(
        "DELETE FROM operation_battery_thresholds "
        "WHERE task_type IN ('PATROL','HARVEST','TRANSFER');"
    )

"""seed corridors — 실제 맵 기반 통로(corridor) 데이터 (RP-78 라우팅 전제)

0002 가 넣은 waypoint 사이의 무방향 간선을, 확정된 맵에서 뽑은 25쌍으로 직접 넣는다.
(이전엔 patrol_order 로 통로를 자동 생성했으나 — 순번 공백이 있으면 루프가 안 닫히는 문제가
 있어 — 실제 맵 토폴로지를 명시 데이터로 교체함.)

⚠️ 짝 지점(18·19)은 여기에 절대 넣지 않는다.
   "짝은 경로 탐색에 등장하지 않는다"가 이 스키마 전체의 전제이고, pair_waypoint_id 는
   논리적 표시일 뿐 실제 차단은 '짝이 corridors 에 없다'는 사실이 담당한다. 짝으로의 이동은
   부모 위에서의 제자리 회전이라 통로가 필요 없다.

corridors 는 (waypoint_a_id < waypoint_b_id) 관례로 한 쌍당 1행만 저장한다.
아래 데이터는 이미 a < b 로 정렬돼 있어 그대로 INSERT 하면 CHECK/UNIQUE 제약을 만족한다.

Revision ID: 0003
Revises: 0002
Create Date: 2026-07-08
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0003"
down_revision: Union[str, None] = "0002"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # 실제 맵에서 뽑은 무방향 간선 25쌍 (모두 a < b, corridor_id 는 IDENTITY 자동 부여)
    # CSV(corridors_260720.csv) 순서 그대로 = corridor_id 1~25.
    op.execute(
        """
        INSERT INTO corridors (waypoint_a_id, waypoint_b_id) VALUES
            ( 1,  2), ( 1, 20), ( 2,  3), ( 3, 21), ( 3,  8),
            ( 4,  5), ( 4,  9), ( 5,  6), ( 6,  7), ( 7,  8),
            ( 7, 10), ( 8, 11), ( 9, 12), (10, 13), (11, 14),
            (12, 15), (13, 16), (14, 17), (15, 16), (15, 22),
            (15, 23), (16, 17), (16, 24), (17, 25), (15, 24);
        """
    )

    # 자체 검증 — 짝 지점이 통로에 섞여 들어오면 여기서 멈춘다(트랜잭션 롤백).
    # 짝이 그래프 노드가 되면 Dijkstra 가 '도달은 되지만 가면 안 되는' 목적지를 잡을 수 있다.
    op.execute(
        """
        DO $$
        BEGIN
            IF EXISTS (
                SELECT 1 FROM corridors c
                  JOIN waypoints w
                    ON w.waypoint_id IN (c.waypoint_a_id, c.waypoint_b_id)
                 WHERE w.pair_waypoint_id IS NOT NULL
            ) THEN
                RAISE EXCEPTION '짝 waypoint 가 corridors 에 있습니다 — 짝은 경로 탐색 대상이 아닙니다';
            END IF;
        END $$;
        """
    )


def downgrade() -> None:
    # 이 프로젝트에서 corridors 는 이 시드로만 채워지므로 전체 비움으로 되돌린다.
    op.execute("DELETE FROM corridors;")

"""seed corridors — 실제 맵 기반 통로(corridor) 데이터 (RP-78 라우팅 전제)

0002 가 넣은 waypoint(1~19) 사이의 무방향 간선을, 확정된 맵에서 뽑은 19쌍으로 직접 넣는다.
(이전엔 patrol_order 로 통로를 자동 생성했으나 — 순번 공백이 있으면 루프가 안 닫히는 문제가
 있어 — 실제 맵 토폴로지를 명시 데이터로 교체함.)

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
    # 실제 맵에서 뽑은 무방향 간선 19쌍 (모두 a < b, corridor_id 는 IDENTITY 자동 부여)
    op.execute(
        """
        INSERT INTO corridors (waypoint_a_id, waypoint_b_id) VALUES
            ( 1,  5), ( 2,  6), ( 3,  8), ( 4,  9), ( 7, 10),
            ( 8, 11), ( 9, 12), (10, 13), (11, 14), (12, 15),
            (13, 16), (14, 17), ( 4,  5), ( 5,  6), ( 6,  7),
            ( 7,  8), (15, 16), (16, 17), ( 1,  2);
        """
    )


def downgrade() -> None:
    # 이 프로젝트에서 corridors 는 이 시드로만 채워지므로 전체 비움으로 되돌린다.
    op.execute("DELETE FROM corridors;")

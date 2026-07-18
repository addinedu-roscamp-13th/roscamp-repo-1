"""add corridors 16-19, 19-18 — 순찰 촬영점 18/19 그래프 편입

waypoint 18/19 는 10/13 과 거의 같은 칸에서 '반대편(위) 베드'를 촬영하는 순찰점인데,
0003 시드에 이들과 이어지는 corridor 가 없어 라우팅이 도달할 수 없었다(고립 노드).
그 결과 Dijkstra 가 18/19 로 가는 경로를 못 찾아 순찰 촬영이 스킵된다.

순찰 순서(patrol_order 11→12)의 진입 경로 '16 → 19 → 18' 을 성립시키기 위해
corridor 두 개(16-19, 18-19)를 추가한다. length 는 두 끝점 waypoint 좌표로
유클리드 거리를 계산해 채운다(0004 와 동일한 방식; length 는 NOT NULL).

corridors 는 (a < b) 관례라 (16,19),(18,19) 모두 그대로 만족한다.

Revision ID: 0005
Revises: 0004
Create Date: 2026-07-18
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0005"
down_revision: Union[str, None] = "0004"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # (16,19),(18,19) 삽입. length 는 waypoints 좌표로 즉시 계산해 채운다.
    #   VALUES 로 추가할 쌍을 만들고, 각 끝점을 waypoints 에 조인해 거리를 구한다.
    op.execute(
        """
        INSERT INTO corridors (waypoint_a_id, waypoint_b_id, length)
        SELECT v.a, v.b,
               sqrt(power(wb.x_coord - wa.x_coord, 2)
                  + power(wb.y_coord - wa.y_coord, 2))
          FROM (VALUES (16, 19), (18, 19)) AS v(a, b)
          JOIN waypoints wa ON wa.waypoint_id = v.a
          JOIN waypoints wb ON wb.waypoint_id = v.b;
        """
    )


def downgrade() -> None:
    # 0004 직후 상태로 되돌린다(추가한 두 통로만 제거).
    op.execute(
        """
        DELETE FROM corridors
         WHERE (waypoint_a_id, waypoint_b_id) IN ((16, 19), (18, 19));
        """
    )

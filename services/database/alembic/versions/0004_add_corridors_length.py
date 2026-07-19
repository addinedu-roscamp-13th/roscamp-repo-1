"""add corridors.length — 다익스트라 간선 비용용 통로 길이(유클리드 거리)

경로 탐색을 BFS(홉 수 최소)에서 Dijkstra(거리 비용 최소)로 바꾸기 위한 전제.
corridors 에 length 컬럼을 추가하고, 두 끝점 waypoint 의 좌표(x_coord, y_coord)로
유클리드 거리를 계산해 채운다. 값의 단위는 waypoints 좌표계와 동일(설계상 m).

NOT NULL 컬럼을 '기존 행이 있는' 테이블에 바로 추가할 수 없으므로 3단계로 나눈다.
  1) length 를 nullable 로 추가
  2) 두 끝점 좌표로 거리 계산해 backfill(UPDATE)
  3) 모든 행이 채워졌으니 NOT NULL 제약 부여

length 는 waypoints(다른 테이블) 좌표에 의존하므로 Postgres GENERATED 컬럼으로는
만들 수 없다(생성 컬럼은 같은 테이블 컬럼만 참조 가능). → 지도(waypoints/corridors)를
바꾸는 시드 마이그레이션에서 이 값도 함께 채워야 한다. 고정 지도라 자주 바뀌지 않는다.

Revision ID: 0004
Revises: 0003
Create Date: 2026-07-18
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0004"
down_revision: Union[str, None] = "0003"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # 1) nullable 로 컬럼 추가 — 기존 19행이 있어 NOT NULL 을 바로 걸 수 없다.
    #    (FLOAT == DOUBLE PRECISION in Postgres. 좌표 컬럼과 타입을 맞춘다.)
    op.execute("ALTER TABLE corridors ADD COLUMN length DOUBLE PRECISION;")

    # 2) 두 끝점 waypoint 좌표로 유클리드 거리를 계산해 채운다.
    #    length = sqrt((xb - xa)^2 + (yb - ya)^2)
    #    corridors 를 waypoints 에 두 번 조인(a끝=wa, b끝=wb)한다.
    op.execute(
        """
        UPDATE corridors AS c
           SET length = sqrt(
                   power(wb.x_coord - wa.x_coord, 2)
                 + power(wb.y_coord - wa.y_coord, 2)
               )
          FROM waypoints AS wa,
               waypoints AS wb
         WHERE c.waypoint_a_id = wa.waypoint_id
           AND c.waypoint_b_id = wb.waypoint_id;
        """
    )

    # 3) 모든 행이 채워졌으니 NOT NULL 로 잠근다(이후 통로는 length 없이 못 들어옴).
    op.execute("ALTER TABLE corridors ALTER COLUMN length SET NOT NULL;")


def downgrade() -> None:
    # 컬럼만 떨어뜨리면 0003 직후 상태로 되돌아간다.
    op.execute("ALTER TABLE corridors DROP COLUMN length;")

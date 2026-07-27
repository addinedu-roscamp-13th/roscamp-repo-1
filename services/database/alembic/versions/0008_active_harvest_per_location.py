"""active harvest per-location guard — 같은 수확 위치에 활성(WAITING/IN_PROGRESS) HARVEST 1건만 허용

수확(HARVEST)은 순찰과 달리 '동시에 여러 대'가 가능하다(서로 다른 수확 위치에서). 따라서
순찰의 전역 1건 제약(ux_tasks_single_active_patrol)은 맞지 않고, '수확 위치(task_point_id)별
1건'이어야 한다. 두 로봇이 같은 HARVEST_01 로 동시에 배정되면 물리적으로 충돌하므로 이를 막는다.
로봇당 활성 task 1건은 기존 ux_tasks_active_robot(task_type 무관)이 이미 막으므로 여기선 위치만 본다.

기법(부분 유니크 인덱스): WHERE 로 '활성 HARVEST' 행만 인덱스 대상으로 좁히고, 그 대상 안에서
task_point_id 에 UNIQUE 를 건다. 같은 위치의 활성 수확이 2건이 될 수 없다. 상류/GUI 가 1차로
막더라도, 접수 시점의 상태 변동은 DB 인덱스만이 원자적으로 막을 수 있다(순찰 0006 과 같은 원칙).

Revision ID: 0008
Revises: 0007
Create Date: 2026-07-25
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0008"
down_revision: Union[str, None] = "0007"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # 활성(WAITING/IN_PROGRESS) HARVEST 행만 대상으로 task_point_id 에 UNIQUE →
    # 같은 수확 위치에는 활성 수확 task 가 최대 1건만 존재할 수 있다(2대 동시수확은
    # 서로 다른 위치라 허용된다).
    op.execute(
        """
        CREATE UNIQUE INDEX ux_tasks_active_harvest_location ON tasks (task_point_id)
        WHERE task_type = 'HARVEST'
          AND status IN ('WAITING','IN_PROGRESS');
        """
    )


def downgrade() -> None:
    op.execute("DROP INDEX IF EXISTS ux_tasks_active_harvest_location;")

"""single active patrol guard — 전역에서 활성(WAITING/IN_PROGRESS) PATROL task 1건만 허용

순찰(PATROL)은 운영상 '동시에 로봇 1대'만 수행한다. 기존 ux_tasks_active_robot 은
'로봇당' 활성 task 1건만 막을 뿐이라, 로봇 A가 순찰 중이어도 idle 로봇 B에게 순찰이
또 접수될 수 있었다. 이 전역 제약을 DB 레벨에서 최종 방어한다(상류/GUI가 1차로 막더라도,
접수 시점의 상태 변동은 DB 인덱스만이 원자적으로 막을 수 있다).

기법(부분 유니크 인덱스): WHERE 로 '활성 PATROL' 행만 인덱스 대상으로 좁히고, 그 대상
안에서 task_type 컬럼에 UNIQUE 를 건다. 대상 행은 전부 task_type='PATROL'(같은 값)이라
유일성이 곧 '전역 최대 1행'을 강제한다. (교통관제/통로 예약은 여러 로봇이 서로 다른
작업으로 통로를 공유하기 때문에 필요한 것이라, 이 제약과 무관하게 그대로 유지된다.)

Revision ID: 0006
Revises: 0005
Create Date: 2026-07-19
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0006"
down_revision: Union[str, None] = "0005"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # 활성(WAITING/IN_PROGRESS) PATROL 행만 대상으로 task_type 에 UNIQUE →
    # 그런 행은 전역에서 최대 1개만 존재할 수 있다.
    op.execute(
        """
        CREATE UNIQUE INDEX ux_tasks_single_active_patrol ON tasks (task_type)
        WHERE task_type = 'PATROL'
          AND status IN ('WAITING','IN_PROGRESS');
        """
    )


def downgrade() -> None:
    op.execute("DROP INDEX IF EXISTS ux_tasks_single_active_patrol;")

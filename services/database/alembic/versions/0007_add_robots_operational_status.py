"""add robots.operational_status — 로봇의 '일을 줘도 되는가' 축

텔레메트리의 주행 상태(nav_status)와는 별개의 축이다. 갇힌 로봇도 물리적으로는
멈춰 있으므로 nav_status 는 IDLE 이지만, 배정 후보로 올리면 같은 막힘을 반복한다.
"멈춰 있다"와 "일을 줘도 되는가"는 다른 질문이라 컬럼을 나눈다.
로봇 선정 시 이 값이 NORMAL 인 로봇만 후보가 된다.

  NORMAL       기본값. 정상 가용.
  IMMOBILIZED  ACS 가 자동 설정. 통로에 갇혀 충전소 복귀조차 실패한 경우
               (시나리오 1 E2 22-2). 관리자가 로봇을 물리적으로 옮긴 뒤 NORMAL 로 되돌린다.
  MAINTENANCE  관리자가 수동 설정. 점검·수리 중이라 작업에서 빼두고 싶을 때.

왜 ACS 메모리가 아니라 DB인가. IMMOBILIZED 는 정의상 사람이 손으로 풀어줘야 하는
상태다. ACS 메모리에 두면 서비스가 재기동되는 순간 조용히 초기화되고, 아직 통로에
갇혀 있는 로봇이 다시 배정 후보로 올라온다. 프로세스 수명과 무관하게 남아야 하므로 DB.

ENUM 이 아니라 VARCHAR + CHECK 인 이유는 0001 상단의 팀 결정을 따른 것이다.
(Postgres 의 CREATE TYPE ... AS ENUM 은 값을 빼거나 이름을 바꾸는 게 사실상 불가능해
 마이그레이션이 무거워진다. CHECK 는 제약만 갈아끼우면 된다.)

0004(corridors.length)와 달리 nullable 추가 → backfill → NOT NULL 의 3단계를 밟지
않아도 되는 이유: 기존 행에 채울 값이 'NORMAL' 하나로 정해져 있어 DEFAULT 로 줄 수
있다. 0004 의 length 는 두 끝점 좌표로 계산해야 나오는 값이라 DEFAULT 가 불가능했다.

DEFAULT 를 컬럼 생성 후에도 남겨두는 이유: 새 로봇을 등록할 때 '정상 가용'이 자연스러운
기본값이고, INSERT 하는 쪽이 이 컬럼을 몰라도 안전하게 동작한다.

인덱스는 만들지 않는다. 로봇이 3대뿐이라 WHERE operational_status='NORMAL' 에
인덱스를 걸어도 플래너가 순차 스캔을 고른다. 관리 비용만 늘어난다.

Revision ID: 0007
Revises: 0006
Create Date: 2026-07-21
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0007"
down_revision: Union[str, None] = "0006"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # DEFAULT 'NORMAL' 덕분에 기존 3행(dg_01~03)이 자동으로 채워져
    # NOT NULL 을 같은 문장에서 바로 걸 수 있다. 0002 시드는 손대지 않는다.
    op.execute(
        """
        ALTER TABLE robots
            ADD COLUMN operational_status VARCHAR(20) NOT NULL DEFAULT 'NORMAL'
            CHECK (operational_status IN ('NORMAL','IMMOBILIZED','MAINTENANCE'));
        """
    )


def downgrade() -> None:
    # CHECK 제약은 컬럼에 딸려 있어 컬럼과 함께 사라진다.
    op.execute("ALTER TABLE robots DROP COLUMN operational_status;")

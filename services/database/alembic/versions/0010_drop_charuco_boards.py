"""drop charuco_boards — ChArUco 정밀 도킹 폐기로 테이블 제거

도킹 방식이 지점별 라우팅(충전소=반사테이프, 수확·예냉실=H마커 바닥 도킹)으로 바뀌면서
ChArUco 보드 도킹을 더 이상 쓰지 않는다. `charuco_boards`(0001 신설)는 그 마커 규격/도킹
오프셋을 담던 테이블인데, 이제 어떤 도킹 경로도 이 값을 읽지 않으므로 걷어낸다.

안전성:
  - `charuco_boards`는 leaf 테이블이다(다른 테이블이 이걸 FK 로 참조하지 않는다). 반대로
    이 테이블이 task_points 를 참조할 뿐이라, DROP 해도 참조 무결성이 깨지지 않는다.
  - 0001 에서 붙인 updated_at 트리거(trg_charuco_boards_updated_at)는 테이블에 종속되어
    DROP TABLE 시 함께 사라진다. 별도 DROP TRIGGER 가 필요 없다.
  - 트리거가 부르는 set_updated_at() 함수는 다른 테이블도 공유하므로 그대로 둔다.

downgrade 는 0001 의 정의를 그대로 되살린다(테이블 + 트리거). 다만 데이터(행)는 복원하지
않는다 — 시드 마이그레이션이 아니라 앱/수동으로 들어온 값이라 원복 대상이 아니다.

Revision ID: 0010
Revises: 0009
Create Date: 2026-07-30
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0010"
down_revision: Union[str, None] = "0009"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # 트리거는 테이블에 딸려 자동 소멸하므로 테이블만 지운다.
    op.execute("DROP TABLE charuco_boards;")


def downgrade() -> None:
    # 0001 의 정의를 그대로 복원한다(테이블 구조 + 제약 + updated_at 트리거).
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
            CONSTRAINT ck_charuco_marker_smaller
                CHECK (marker_size_m < square_size_m),
            CONSTRAINT ck_charuco_board_min_size
                CHECK (squares_x >= 3 AND squares_y >= 3)
        );
        """
    )
    # updated_at 트리거 재부착 (set_updated_at() 함수는 0001 에서 만든 것을 재사용).
    op.execute(
        """
        CREATE TRIGGER trg_charuco_boards_updated_at
        BEFORE UPDATE ON charuco_boards
        FOR EACH ROW EXECUTE FUNCTION set_updated_at();
        """
    )

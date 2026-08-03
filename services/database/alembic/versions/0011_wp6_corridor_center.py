"""wp6 을 통로 중앙 쪽으로 1cm 올림 — 좁은 구간에서 벽에 붙어 경로가 끊기는 것 방지

2026-08-03 순찰 통합테스트에서 wp6→wp5 구간(통로 8)이 '막힘'으로 판정돼 순찰이
우회했다. 현장에는 아무것도 없었고, 원인은 **도착 위치가 벽에 너무 가까웠던 것**이다.

  · Nav2 는 목표 10cm 안에 들면 도착으로 친다(xy_goal_tolerance). 어느 방향에서
    오느냐에 따라 그 오차가 어느 쪽으로 남는지 달라진다.
  · 로봇이 차지하는 것으로 계산되는 반경 = footprint 6cm + inflation 3cm = 9cm.
  · 실측(bag): wp6 에 (0.36,-0.04) 로 도착 → 벽까지 **9.0cm** → 플래너가 '로봇이
    장애물 안'으로 보고 경로를 못 냈다. 40초간 0 속도만 내다 실패했다.
    3분 뒤 같은 통로를 (0.38,+0.04) 로 지날 때는 벽까지 17cm 라 멀쩡했다.

wp5·wp6 사이 x 0.50~0.55 구간은 통로 폭이 **26cm** 뿐이다(위쪽 세로 베드가 y=+0.14
까지 내려온다). 로봇 18cm 를 빼면 좌우 여유가 4cm 씩이다.

**+1cm(y: -0.008 → +0.002) 인 이유** — 진입 방향별 도착 오차 실측을 넣어 양쪽 벽에
똑같이 걸리는 지점을 잡았다:
    wp8 →(+x 통과) -3.2cm → y=-0.030 → 아래벽까지 9.0cm
    wp10→(-y 통과) +4.8cm → y=+0.050 → 병목 위벽까지 9.0cm
대칭점이라 이보다 위로든 아래로든 옮기면 한쪽이 반드시 나빠진다. 통로 기하학적
중앙(+0.010)보다 살짝 아래인 것은 오차 분포가 위로 치우쳐 있기 때문이다.

**wp5 는 건드리지 않는다.** 목표 -0.018 일 때 실제 도착이 +0.032 로 이미 병목 중앙
근처다(위벽 여유 10.8cm). 실측 두 번 모두 +5cm 로 떴다 — wp9 에서 -y 로 내려오며
생기는 계통 오차이고, 목표를 올리면 그만큼 같이 올라가 벽으로 밀린다(실제로 +5cm
올려 시험했다가 병목에서 위벽 7cm 까지 붙어 접촉했다).

촬영 화각 영향은 확인했다: 베드까지 12.0cm → 13.0cm(+8%). 같은 지점 전후 사진의
선명도가 1035 → 1084 로 **오히려 좋아졌다**(현재가 초점보다 가까웠던 것). 대조군
wp5 도 1235 → 1302 로 재현됐다.

⚠️ ACS 는 그래프·좌표를 첫 순찰 때 1회 캐시한다 → 이 마이그레이션 뒤 **ACS 재기동 필요**.
"""

from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0011"
down_revision: Union[str, None] = "0010"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    # y 만 바꾼다. x·yaw·촬영방향은 그대로 — 화각은 이 정도 거리 변화에 견딘다.
    op.execute("UPDATE waypoints SET y_coord = 0.002 WHERE waypoint_id = 6;")


def downgrade() -> None:
    op.execute("UPDATE waypoints SET y_coord = -0.008 WHERE waypoint_id = 6;")

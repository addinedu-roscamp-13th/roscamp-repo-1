"""wp24(CHARGE_03) 를 마커 법선 위로 3.5cm 옮김 — 순찰 복귀 도킹이 매번 실패하던 원인

2026-08-04 통합에서 순찰 후 충전소 도킹이 `code=1`(마커 상실)로 반복 실패했고,
도킹 기동 중 **옆 충전소의 로봇에 접촉**했다. 로봇 단독 도킹(손으로 놓고 시작)은
같은 날에도 멀쩡했다 — 차이는 '어느 자리에서 도킹을 시작하느냐' 하나였다.

원인은 **wp24 좌표가 반사테이프 마커의 법선에서 3.5cm 치우쳐 있던 것**이다.
그 자리에 아무리 정확히 서도 법선에서 3.5cm 벗어난 채 도킹이 시작된다.

  · 반사 도킹 FSM 은 법선 이탈이 MIN_MOVE_M(2cm) 이상이면 TURN1→DRIVE 로 **옆걸음**을
    한 뒤 후진한다. 충전소 좌우 여유는 1.5cm 뿐이라 그 옆걸음이 곧 접촉이 된다.
  · 2cm 미만이면 옆걸음 없이 바로 후진한다. 단독 도킹이 성공하던 경로가 이것이다
    (실측 '법선까지 0.2cm' → 뒤끝 여유 2cm 로 완료).

**3.5cm 를 어떻게 구했나** — 진입 노드에 주행으로 세운 뒤 로봇을 움직이지 않고 두 값을
짝으로 재면, 노드 자체의 치우침 C 가 `법선이탈 = C + lat` 으로 분리된다
(lat = 노드 대비 좌우 오차, `tf2_echo map base_footprint`; 법선이탈 = `/docking_marker_pose`
의 `e = -x·sin(yaw) + y·cos(yaw)`, `yaw = 2·atan2(qz, qw)`).

    회차 A   lat +1.4cm → 법선 4.90cm → C = 3.50cm
    회차 B   lat -1.3cm → 법선 2.15cm → C = 3.45cm

두 회차가 C≈3.5cm 로 일치했고, 이 모델로 A 의 법선 이탈을 역산하면 4.9cm 로 로그와
같다. 회차 B 의 법선 이탈이 오히려 작았던 것은 로봇이 우연히 **치우친 쪽 반대로**
1.3cm 빗나가 법선에 가까워졌기 때문이다 — 주행이 정확할수록 도킹이 나빠지는 상태였다.

**보정**: 진입 방향(yaw=1.523)에 수직으로 3.5cm.
    x: 0.373 + 0.035·(-sin 1.523) = 0.338
    y: 0.878 + 0.035·( cos 1.523) = 0.880
보정 후 실기에서 도킹 성공을 확인했다.

**yaw 는 건드리지 않는다.** 마커 법선과의 차이는 3.35° 뿐이고, 그 정도는 도킹 FSM 의
사전정렬(TURN2)이 잡는다. 여기서 yaw 를 만지면 복귀 주행의 hold_yaw 목표까지 흔들린다.

`corridors.length` 는 두 끝점 좌표로 계산된 Dijkstra 비용이라 함께 갱신한다
(wp16↔wp24 0.12431→0.12615, wp15↔wp24 0.30753→0.34258). 안 하면 경로 비용이 옛 좌표
기준으로 남는다.

⚠️ ACS 는 좌표·그래프를 첫 요청 때 1회 캐시한다 → 이 마이그레이션 뒤 **ACS 재기동 필요**.
⚠️ CHARGE_01(wp22)·CHARGE_02(wp23)는 같은 치우침이 있는지 **아직 검증하지 않았다.**
   위 두-값 측정법을 그대로 쓰면 로봇 한 대당 몇 분이면 확인된다.
"""

from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0012"
down_revision: Union[str, None] = "0011"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def _recompute_wp24_corridor_lengths():
    """wp24 에 닿는 통로의 길이를 두 끝점 좌표로 다시 채운다(0009 와 같은 식)."""
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
           AND c.waypoint_b_id = wb.waypoint_id
           AND (c.waypoint_a_id = 24 OR c.waypoint_b_id = 24);
        """
    )


def upgrade() -> None:
    op.execute(
        "UPDATE waypoints SET x_coord = 0.338, y_coord = 0.880 "
        " WHERE waypoint_id = 24;"
    )
    _recompute_wp24_corridor_lengths()


def downgrade() -> None:
    op.execute(
        "UPDATE waypoints SET x_coord = 0.373, y_coord = 0.878 "
        " WHERE waypoint_id = 24;"
    )
    _recompute_wp24_corridor_lengths()

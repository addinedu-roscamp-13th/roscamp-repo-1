"""patrol route rework — 현장 실측 좌표·순서로 순찰 경로 재정의 (Phase 2)

RP-EX 순찰 촬영 구도 개선의 데이터 쪽 절반이다. 코드 쪽(로봇 navigate_server 의 출발정렬·
정밀조준)은 현장 34/34 로 검증했고, 그 주행이 따라간 좌표·순서를 여기서 DB 로 옮긴다.

바꾸는 것 6가지:

1) **노드 4·7 삭제 + 통로 재배선.** 4·7 이 각각 5·6 과 3cm 안쪽으로 붙어 있어 그래프에
   의미 없는 홉만 늘렸다(로봇 몸통 12cm 보다 가까우면 별개의 자리가 아니다). 이미 손으로
   지운 개발 DB 와 새로 만든 DB 가 같은 모양이 되도록 여기 기록한다.
       삭제 통로: (4,5) (4,9) (6,7) (7,8) (7,10)
       추가 통로: (5,9) (6,8) (6,10)

2) **순찰점 좌표·촬영 방향을 실측값으로.** waypoint_tuner 로 라이브뷰를 보며 잡고 7차 주행
   까지 재현 확인한 값이다. wp6 은 좌표는 그대로고 **촬영 방향만** 동쪽 → 서쪽으로 바꾼다.
   아래행을 서쪽으로 지나며 5·6 을 찍고, 되돌아오는 길에 2·1 을 동쪽으로 찍는 구성이라
   6 을 동향으로 두면 그 자리에서 180° 를 돌아야 한다.

3) **짝(18·19)의 좌표를 부모와 분리.** 이 마이그레이션의 핵심이다.
   예전 전제는 "짝은 부모와 x·y 가 완전히 같고 로봇이 제자리에서 180° 돌아 반대쪽을 찍는다"
   였다(0002). 그런데 현장에서 그 회전이 **물리적으로 불가능**함이 확인됐다 — 로봇(12cm
   정사각)이 45° 기울면 옆으로 8.49cm 튀어나오는데 통로 여유가 7.5cm 다. 90° 이상의 회전은
   반드시 45° 를 지나므로 파라미터로 못 고친다.
   → 같은 자리 반대 방향 촬영은 **왕복하며 지나가는 김에** 한 장씩 찍는 방식으로 바뀐다.
   그러면 두 장의 정차 위치가 6~8cm 벌어진다. 웹캠 렌즈가 바퀴축보다 약 3.5cm 뒤에 있어,
   렌즈를 같은 지점에 두려면 로봇이 **진행 방향으로 그만큼 더 가서** 서야 하기 때문이다.

   ⚠️ 좌표가 갈라져도 **짝은 여전히 corridors 에 넣지 않는다.** 예약이 다루는 것은 지도상의
   점이 아니라 '로봇 한 대가 차지하는 공간'인데, 7cm 는 로봇 몸통보다 작아 물리적으로 같은
   자리다. 짝을 그래프 노드로 만들면 "10번은 찼지만 18번은 비었다"는 거짓말이 가능해진다.
   예약은 부모 자리를 그대로 쓰고(ACS `_parent_of`), 갈라진 좌표는 **주행 목표로만** 쓴다.

4) **짝을 순찰 목표로 승격**(patrol_order 부여). 예전에는 부모 바로 뒤에 끼워 넣는 부속물
   이라 순서가 없었으나, 이제 두 장이 서로 다른 시점·다른 자리에서 찍히므로 각자 목표다.
   "12장 다 찍었나"를 촬영 단위로 세야 미방문 보고가 정확해진다.
   ※ pair_waypoint_id 는 그대로 둔다 — '부모 자리를 쓴다(통로 예약은 부모 것)'는 표시다.

5) **patrol_order 재배치.** 옛 순서는 오른쪽 열을 **내려가면서** 북향 촬영을 하게 되어 있어
   그 자리에서 180° 회전이 필요했다. 실측 경로대로 **올라가면서** 찍는 순서로 바꾼다:
       5 → 6 → 18 → 19 → 14 → 11 → 2 → 1 → 9 → 12 → 13 → 10

6) **진입 노드(20~25) 위치 갱신 + corridors.length 전체 재계산.**
   진입 노드는 x·y 만 바꾸고 yaw 는 손대지 않는다 — 어차피 쓰이지 않기 때문이다. ACS 는
   촬영하지 않는 노드의 yaw 를 진행 방향으로 덮어쓰고(route_runner._travel_yaw), 도킹은
   Dock 액션이 마커 기준(marker_id/dock_offset_*)으로 하므로 waypoint yaw 를 받지 않는다.
   length 는 좌표에서 유도된 저장 컬럼인데 갱신 트리거가 없다(0004). 좌표를 바꾸고 이걸
   빠뜨리면 Dijkstra 가 옛 거리로 최단 경로를 고른다 — 20·21 은 20~31cm 나 움직였다.

이미 손으로 고쳐 둔 개발 DB 에서도 그대로 돌도록 삭제·삽입은 멱등하게 쓴다.

Revision ID: 0009
Revises: 0008
Create Date: 2026-07-29
"""
from typing import Sequence, Union

from alembic import op

# revision identifiers, used by Alembic.
revision: str = "0009"
down_revision: Union[str, None] = "0008"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


# (waypoint_id, x, y, yaw) — yaw 가 None 이면 좌표만 바꾸고 방향은 유지한다.
_NEW_POSES = [
    # ── 순찰점 (촬영 방향 = 로봇 몸이 향할 방향. 카메라는 그 왼쪽 90° 를 본다) ──
    (1,   0.700, -0.410,  0.000),   # 동향. y 를 올려 아래 벽에서 180° 회전 여유 확보
    (2,   0.453, -0.410, -0.040),   # 동향. wp1 과 수평으로 이어 직진 주행
    (5,   0.660, -0.018,  3.110),   # 서향 (좌표 변경 없음)
    (6,   0.389, -0.008,  3.110),   # 서향 ← 동향(0.017)에서 뒤집음. 좌표는 그대로
    (9,   0.732,  0.293,  1.600),   # 북향
    (10,  0.344,  0.216, -1.526),   # 남향 — 남쪽으로 지날 때 서는 자리
    (11, -0.011,  0.250, -1.560),   # 남향 (좌표 변경 없음)
    (12,  0.719,  0.515,  1.640),   # 북향
    (13,  0.368,  0.453, -1.560),   # 남향 — 남쪽으로 지날 때 서는 자리
    (14, -0.013,  0.461, -1.580),   # 남향 (좌표 변경 없음)
    # ── 짝: 부모와 같은 '자리'지만 반대 방향으로 지날 때 서는 위치 ──
    (18,  0.352,  0.293,  1.571),   # 10 의 북향 촬영 (부모와 7.7cm)
    (19,  0.355,  0.518,  1.550),   # 13 의 북향 촬영 (부모와 6.5cm)
    # ── 통로 경유점 (yaw 없음) ──
    (15,  0.680,  0.860,  None),    # 벽에서 떼되 베드 모서리도 피한 자리
    (17,  0.017,  0.798,  None),    # 벽 8.7cm 에서 90° 회전이 안 돼 통로 중앙으로
    # ── 작업 지점 진입 노드: 위치만 (yaw 는 쓰이지 않으므로 유지) ──
    (20,  0.484, -0.425,  None),    # HARVEST_01
    (21,  0.254, -0.402,  None),    # HARVEST_02
    (22,  0.778,  0.830,  None),    # CHARGE_01 (dg_01)
    (23,  0.544,  0.886,  None),    # CHARGE_02 (dg_02)
    (24,  0.373,  0.878,  None),    # CHARGE_03 (dg_03)
    (25, -0.013,  0.718,  None),    # PRECOOL_01
]

# 실측 순찰 루프의 촬영 순서. 짝(18·19)도 각자 한 자리를 차지한다.
_PATROL_ORDER = [
    (5, 1), (6, 2), (18, 3), (19, 4), (14, 5), (11, 6),
    (2, 7), (1, 8), (9, 9), (12, 10), (13, 11), (10, 12),
]


def _update_pose(wp_id, x, y, yaw):
    """좌표(+선택적으로 yaw)를 갱신하는 UPDATE 한 줄."""
    yaw_set = "" if yaw is None else f", yaw_coord = {yaw}"
    op.execute(
        f"UPDATE waypoints SET x_coord = {x}, y_coord = {y}{yaw_set} "
        f" WHERE waypoint_id = {wp_id};"
    )


def _recompute_corridor_lengths():
    """두 끝점 좌표로 통로 길이를 다시 채운다(0004 의 backfill 과 같은 식)."""
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


def upgrade() -> None:
    # 1) 노드 4·7 제거 — 통로가 waypoints 를 FK 로 참조하므로 통로부터 지운다.
    #    (ON DELETE CASCADE 를 일부러 안 걸어 뒀다: 노드를 실수로 지웠을 때 통로가 조용히
    #     사라지면 그래프가 끊긴 걸 아무도 모른다. 그래서 여기서 명시적으로 지운다.)
    op.execute(
        "DELETE FROM corridors "
        " WHERE waypoint_a_id IN (4, 7) OR waypoint_b_id IN (4, 7);"
    )
    op.execute("DELETE FROM waypoints WHERE waypoint_id IN (4, 7);")

    # 4·7 을 건너뛰고 이웃을 직접 잇는다. 이미 손으로 넣어 둔 개발 DB 에서도 돌도록
    # ON CONFLICT 로 넘어간다(ux_corridors_pair 가 중복을 막는다).
    op.execute(
        """
        INSERT INTO corridors (waypoint_a_id, waypoint_b_id, length) VALUES
            (5, 9, 0), (6, 8, 0), (6, 10, 0)
        ON CONFLICT (waypoint_a_id, waypoint_b_id) DO NOTHING;
        """
    )

    # 2)~4) 좌표·촬영 방향 갱신 (짝의 좌표 분리 포함)
    for wp_id, x, y, yaw in _NEW_POSES:
        _update_pose(wp_id, x, y, yaw)

    # 5) 순찰 순서 재배치. patrol_order 에 UNIQUE 가 없어 중간 상태 충돌 걱정이 없다.
    #    먼저 전부 비우고 다시 채운다 — 옛 순번이 남아 있으면 지금 빠진 지점이 조용히
    #    순찰 목록에 계속 낀다.
    op.execute("UPDATE waypoints SET patrol_order = NULL;")
    for wp_id, order in _PATROL_ORDER:
        op.execute(
            f"UPDATE waypoints SET patrol_order = {order} "
            f" WHERE waypoint_id = {wp_id};"
        )

    # 6) 좌표가 바뀌었으니 통로 길이를 전부 다시 계산한다(위 INSERT 의 0 도 여기서 채워진다).
    _recompute_corridor_lengths()

    # --- 자체 검증: 어긋난 채로 커밋되면 현장에서 원인 찾기가 매우 어렵다 ---
    op.execute(
        """
        DO $$
        DECLARE
            n INTEGER;
            d DOUBLE PRECISION;
        BEGIN
            -- (a) 짝은 경로 탐색 그래프에 없어야 한다. 좌표가 갈라져도 이 전제는 그대로다
            --     — 짝은 '새 장소'가 아니라 '같은 자리의 다른 자세'이기 때문이다.
            IF EXISTS (
                SELECT 1 FROM corridors c
                  JOIN waypoints w
                    ON w.waypoint_id IN (c.waypoint_a_id, c.waypoint_b_id)
                 WHERE w.pair_waypoint_id IS NOT NULL
            ) THEN
                RAISE EXCEPTION '짝 waypoint 가 corridors 에 있습니다 — 짝은 경로 탐색 대상이 아닙니다';
            END IF;

            -- (b) 짝과 부모는 '같은 자리'로 볼 수 있는 범위 안이어야 한다. 통로 예약을
            --     부모 자리로 대신하기 때문이다. 로봇 몸통이 12cm 라 그보다 크게 벌어지면
            --     두 대가 동시에 설 수 있는 별개의 자리라는 뜻이고, 예약이 거짓말이 된다.
            SELECT max(sqrt(power(c.x_coord - p.x_coord, 2)
                          + power(c.y_coord - p.y_coord, 2)))
              INTO d
              FROM waypoints c
              JOIN waypoints p ON p.waypoint_id = c.pair_waypoint_id;
            IF d IS NOT NULL AND d > 0.12 THEN
                RAISE EXCEPTION '짝과 부모의 거리가 %m 로 로봇 몸통(0.12m)보다 큽니다 — 같은 자리로 볼 수 없어 통로 예약이 어긋납니다', round(d::numeric, 3);
            END IF;

            -- (c) 순찰 목표는 12곳(순찰점 10 + 짝 2)이고 1~12 가 빠짐없이 채워져야 한다.
            SELECT count(*) INTO n FROM waypoints WHERE patrol_order IS NOT NULL;
            IF n <> 12 THEN
                RAISE EXCEPTION 'patrol_order 가 부여된 지점이 %개입니다 (12개여야 함)', n;
            END IF;
            SELECT count(*) INTO n
              FROM generate_series(1, 12) g
             WHERE NOT EXISTS (SELECT 1 FROM waypoints w WHERE w.patrol_order = g);
            IF n <> 0 THEN
                RAISE EXCEPTION 'patrol_order 에 빠진 순번이 %개 있습니다', n;
            END IF;

            -- (d) patrol_order 를 받은 지점은 전부 촬영 지점이어야 한다.
            IF EXISTS (SELECT 1 FROM waypoints
                        WHERE patrol_order IS NOT NULL AND is_patrol_point = FALSE) THEN
                RAISE EXCEPTION '순찰 지점이 아닌 노드에 patrol_order 가 붙었습니다';
            END IF;

            -- (e) 통로 길이가 좌표와 어긋나면 Dijkstra 가 엉뚱한 경로를 고른다.
            SELECT count(*) INTO n
              FROM corridors c
              JOIN waypoints wa ON wa.waypoint_id = c.waypoint_a_id
              JOIN waypoints wb ON wb.waypoint_id = c.waypoint_b_id
             WHERE abs(c.length - sqrt(power(wb.x_coord - wa.x_coord, 2)
                                     + power(wb.y_coord - wa.y_coord, 2))) > 1e-9;
            IF n <> 0 THEN
                RAISE EXCEPTION 'corridors.length 가 좌표와 어긋난 행이 %개 있습니다', n;
            END IF;
        END $$;
        """
    )


def downgrade() -> None:
    # 0002/0003 시점의 좌표·순서로 되돌린다.
    old_poses = [
        (1,   0.700, -0.446,  0.001),
        (2,   0.416, -0.443, -0.017),
        (5,   0.660, -0.018,  3.111),
        (6,   0.389, -0.008,  0.017),
        (9,   0.750,  0.263,  1.496),
        (10,  0.314,  0.261, -1.626),
        (11, -0.011,  0.250, -1.559),
        (12,  0.738,  0.498,  1.609),
        (13,  0.323,  0.491, -1.649),
        (14, -0.013,  0.461, -1.581),
        (18,  0.314,  0.261,  1.493),
        (19,  0.323,  0.491,  1.493),
        (15,  0.706,  0.791,  None),
        (17, -0.016,  0.798,  None),
        (20,  0.717, -0.449,  None),
        (21, -0.059, -0.438,  None),
        (22,  0.770,  0.920,  None),
        (23,  0.633,  0.910,  None),
        (24,  0.468,  0.921,  None),
        (25, -0.026,  0.863,  None),
    ]
    for wp_id, x, y, yaw in old_poses:
        _update_pose(wp_id, x, y, yaw)

    # 노드 4·7 복구. waypoint_id 가 GENERATED ALWAYS AS IDENTITY 라 명시 삽입에는
    # OVERRIDING SYSTEM VALUE 가 필요하다(자동 부여를 이번만 건너뛰겠다는 선언).
    op.execute(
        """
        INSERT INTO waypoints
            (waypoint_id, x_coord, y_coord, yaw_coord, is_patrol_point,
             pair_waypoint_id, patrol_order)
        OVERRIDING SYSTEM VALUE VALUES
            (4, 0.715, -0.003, NULL, FALSE, NULL, NULL),
            (7, 0.354, -0.002, NULL, FALSE, NULL, NULL)
        ON CONFLICT (waypoint_id) DO NOTHING;
        """
    )
    op.execute(
        "DELETE FROM corridors WHERE (waypoint_a_id, waypoint_b_id) "
        " IN ((5, 9), (6, 8), (6, 10));"
    )
    op.execute(
        """
        INSERT INTO corridors (waypoint_a_id, waypoint_b_id, length) VALUES
            (4, 5, 0), (4, 9, 0), (6, 7, 0), (7, 8, 0), (7, 10, 0)
        ON CONFLICT (waypoint_a_id, waypoint_b_id) DO NOTHING;
        """
    )

    # 옛 순찰 순서(짝은 순서를 갖지 않았다).
    op.execute("UPDATE waypoints SET patrol_order = NULL;")
    for wp_id, order in [(12, 1), (9, 2), (5, 3), (6, 4), (10, 5), (13, 7),
                         (14, 9), (11, 10), (2, 11), (1, 12)]:
        op.execute(
            f"UPDATE waypoints SET patrol_order = {order} "
            f" WHERE waypoint_id = {wp_id};"
        )

    _recompute_corridor_lengths()

#!/usr/bin/env python3
"""파지 시퀀스 — send_coords 기반 (MoveIt2 미사용, 제조사 IK 솔버).

한 개의 토마토를 집어 바구니에 넣는 1회 시도. 동작 분할(매니퓰레이션 정석):
    관측 → 수확준비자세(관절이동, 경유) → pre-grasp(직선) → 직선 접근 → 그립
    → 파지 판정 → 직선 후퇴 → 바구니 투하 → 복귀
관측→수확준비만 관절보간(mode=0). 그 뒤 접근/후퇴는 전부 직선(mode=1)으로 옆 열매 회피.
직선 IK가 안 풀리는 자세는 관절(mode=0)로 자동 폴백.

- 방향(자세)은 고정: 고정방향 직선접근이 우리 선택(빠르고 튼튼). AI approach
  벡터로 손목 정렬하는 건 이후 확장(M6).
- send_coords mode=1(직선)로 접근/후퇴해 옆 열매를 안 건드린다.
- 재시도(3회)는 이 함수 밖(harvest 루프)에서. 여기선 '1회 시도'만.

⚠️ 아래 상수는 전부 **실물에서 튜닝**한다. 기본값은 검증된 시퀀스
(arm_solver_direct)·taught_path 좌표에서 가져온 출발점.
좌표 mm / 각도 deg.
"""
from __future__ import annotations

import time

from ddagi_harvest import approach_model
from ddagi_harvest.arm_backend import ArmBackend

# ---- 튜닝 상수 (실물에서 확정) --------------------------------------------- #
# 기본 그리퍼 자세 (rx,ry,rz). AI 기울기값이 오면 pick(orientation=...)로 덮어쓴다.
# 아래 값은 tune_pick.py 실물 튜닝 결과 (2026-07-24, grasp값 12→39로 확실히 잡힘).
GRIPPER_ORI = [-93.6, 9.7, -106.4]
DESCEND_OFFSET = [0.0, 0.0, 0.0]          # 0 — 파지점이 이제 tf_transform 의 관측자세
                                          # camera→flange 피팅 결과 그대로다(손끝이 열매에
                                          # 닿는 flange 를 직접 측정해 피팅). 파지점 오프셋·
                                          # 처짐·TCP 가 모두 그 측정에 포함돼 있어 추가 보정 불필요.
PREGRASP_OFFSET = [-45.0, 20.0, -5.0]     # standoff(캐노피 밖·도달가능) — base 대비 뒤·좌·아래.
                                          # y(+20)는 '열매 왼쪽에 서서 오른쪽으로 진입'을 뜻한다.
                                          # 줄기 기준 반대쪽 열매엔 이 y 부호를 뒤집는다(아래 참조).
                                          # tf_verify 조그 실측(2026-07-25).
RETREAT_OFFSET = [-45.0, 0.0, 0.0]        # 그립 후 후퇴 — **순수 후진**(좌우·상하 0).
                                          # 진입 경로를 그대로 되짚으면 좌우 20mm 성분이
                                          # 물고 있는 열매에 전단력을 줘 빠뜨린다(실측:
                                          # 파지값 25·33 이 상승 후 0). 물었을 때는 흔들지
                                          # 않는 게 우선이라 접근축으로만 뺀다.

# ---- 줄기 기준 진입 방향 ----------------------------------------------------- #
# 베드에 줄기가 2개(우측줄·좌측줄). 열매가 '어느 줄기의 어느 쪽'에 달렸는지에 따라
# 진입 방향이 달라야 한다 — 줄기 쪽에서 들어가면 줄기까지 함께 물어 다른 열매를 떨군다.
# base 좌표에서 +y 가 좌측줄 방향(우측줄 y≈-30~45, 좌측줄 y≈95~155).
#   열매 y > 줄기 y → 줄기의 '왼쪽'에 달림  → 왼쪽 바깥에서 진입 (PREGRASP y = +)
#   열매 y < 줄기 y → 줄기의 '오른쪽'에 달림 → 오른쪽 바깥에서 진입 (PREGRASP y = -)
# 값은 tf_verify 로 각 줄기를 클릭해 실측한 base y 를 넣는다. 비워두면(빈 리스트)
# 종전 동작(항상 왼쪽 진입) 그대로 — 값이 채워질 때만 방향 분기가 켜진다.
# tf_verify 클릭 실측 — 베드 테두리에 그리퍼가 막혀 두 줄기를 가운데로 모은 뒤 재측정
# (2026-07-28 저녁): 우측 줄기 base=[272.7, 13.6, 295.3], 좌측 줄기 base=[278.5, 107.9, 309.9]
#   줄기 간격 110.5 → 94.3mm 로 좁아짐.
# 이 값은 camera→flange 피팅 좌표계 기준이라 별도 보정 없이 그대로 쓴다
# (종전 [22.7, 133.2] 은 옛 TF 체계에서 잰 값 + 보정을 얹은 것이었다).
# 검증법: tf_verify 에서 열매를 클릭하면 구역·진입방향이 표시된다. 눈으로 본 좌우와
#   다르면 이 값을 조정한다(줄기를 다시 클릭해 재측정하는 게 가장 정확).
# ⚠ 식물을 옮기면 여기와 J1 조준(AIM_J1_LEFT)만 재확인하면 된다. camera→flange 피팅과
#   손목 자세(ORI_BY_ZONE)는 식물 위치와 무관하므로 그대로 유효하다.
STEM_REFS_Y: list[float] = [13.6, 107.9]
# 속도(1~100). 사이클 시간의 대부분이 이동이라 여기가 최대 레버. 정밀이 필요한 구간만
# 중간 속도로 두고, 자세 경유·상승·바구니 같은 '이동만 하는' 구간은 빠르게.
# 파지 구간(수확준비→pre-grasp→그랩점→후퇴)은 30 고정 — 파지율이 9/28 로 가장 좋았던
# 설정이고, 55/35 로 올릴 때마다 정확도가 떨어졌다(종단 정확도가 파지율을 지배).
APPROACH_SPEED = 30       # 수확준비·J1 aim·pre-grasp·그랩점 진입
RETREAT_SPEED = 30        # 후퇴·안전 상승
TRANSIT_SPEED = 85        # 바구니 왕복·관측 복귀 — 파지 정확도와 무관해 빠르게
GRIP_THRESHOLD = 6                    # 닫은 뒤 값이 이보다 크면 파지 성공(토마토 걸림).
                                      # 실측(gripper_check, 2026-07-27): 빈손=0, 작은토마토=12
                                      # → 중간 6. 큰 토마토는 12↑라 자동 통과. 그리퍼 재캘리 금지
                                      # (닫힘=0 정상 상태 기준값이라, 캘리 건드리면 다시 재야 함).
GRIP_SPEED = 100                      # 그리퍼 개폐 속도(1~100). 파지 지연 줄이려 최대로.
SETTLE = 0.2                          # 각 단계 후 정착 대기(s). 짧을수록 빠르나 너무 짧으면 값 오독

# 위치별 손티칭 접근모델(taught_approaches) 사용 여부. True 면 시연점 근처(approach_model.
# LOCAL_RADIUS_MM 내)의 토마토만 그 시연 접근을 쓰고, 시연 없는 곳은 고정 접근으로 폴백.
# → '문제 구역(오른쪽 끝 등)만 국소 티칭'하는 타깃 오버라이드. 나머진 고정 접근 유지.
# ⚠ 끔(2026-07-28) — 시연 15개의 base 는 옛 TF 체계(보정 20~60mm 오차)에서 기록돼
# 새 피팅 좌표계와 정합되지 않는다. 진입 방향은 아래 구역 분기가 담당하므로 기능 손실은
# 크지 않다. 다시 쓰려면 새 좌표계에서 재티칭 필요.
USE_APPROACH_MODEL = False

# 관측/대기 자세 — 카메라 라이브뷰로 베드 프레이밍해 확정 (observe_setup.py, 2026-07-24)
# 이 자세의 get_coords는 tf_transform.OBSERVE_COORDS 와 반드시 짝을 이룬다(같이 갱신).
OBSERVE_ANGLES = [-3.2, 87.2, -26.4, -64.5, 5.6, -3.4]     # 검출 시 카메라가 베드 보는 자세
# 2026-07-28 재티칭 — 종전 자세는 카메라가 약간 왼쪽을 봐 프레이밍이 치우쳤다.
STOW_ANGLES = [-3.2, 87.2, -26.4, -64.5, 5.6, -3.4]        # TODO 미티칭 — 임시로 관측자세. E5 이송용 별도 티칭 필요

# 수확 준비 자세 — 관측↔pre-grasp 사이 '경유'. 관측→여기는 관절이동(mode=0)으로 한 번,
# 여기서부터 pre-grasp·접근·후퇴는 전부 직선(mode=1)으로 간다. 큰 관절 스윙을 이 한 번에
# 가두고 접근은 직선으로 → 옆 열매 회피. 베드 위·그리퍼 접근방향 정렬 자세.
# tf_verify 't' 드래그 티칭(2026-07-25). 관측자세를 바꾸면 이 자세 도달성도 재확인.
STAGING_ANGLES = [-2.7, -36.0, 113.5, -78.1, 1.1, 6.8]

# J1 수평 aim 보간 — 공통 staging에서 J1만 목표 방향으로 돌려(캐노피 위 순수 회전) 그 줄을
# 바라본 뒤 직선접근. 그래야 mode=1이 '가로 스윕' 없이 반경방향으로 풀린다.
# 두 줄의 (base y, J1) 2점 선형보간, 두 줄 밖은 가까운 줄로 클램프.
#   y 앵커는 STEM_REFS_Y 를 그대로 쓴다 — 열 위치를 한 곳에서만 관리해 관측자세/TF 가
#   바뀔 때 두 군데를 따로 고치다 어긋나는 일을 막는다.
#   J1 값은 '그 열을 바라보는 팔 각도'라 TF 변경과 무관(재측정 불필요).
AIM_J1_RIGHT = STAGING_ANGLES[0]   # 우측줄을 보는 J1 (=staging 기본값)
AIM_J1_LEFT = 38.8                 # 좌측줄을 보는 J1 (드래그 티칭 실측)

# 바구니 투하 — 'J1 수평회전 → 접근 → 놓기' 3단계.
# 바구니가 파지 지점보다 ~19.5cm 위라, 저위치 파지 후 대각선 상승하면 벽을 친다.
# 그래서 ①J1만 수평 회전(높이 유지) → ②접근 → ③놓기.
# 바구니 2개가 한 곳에 반반 → '접근'은 공용, '놓기'만 좌/우로 나뉜다.
# teach_basket.py 드래그 티칭 (2026-07-27, 바구니 위치 이동 후 재티칭).
BASKET_APPROACH_ANGLES = [-128.6, -49.4, 91.9, -44.0, 10.9, 5.5]   # 공용
BASKET_DROP_ANGLES = {
    "NORMAL":  [-128.6, -46.4, 17.7, -31.8, 0.3, 4.7],
    "DISCARD": [-120.2, -57.5, 20.5, -15.2, -2.3, 4.7],
}


# myCobot 280 좌표 가동범위(mm) — send_coords 전 검사해 범위 밖이면 안전하게 거부.
COORD_LIMITS = {0: (-281.45, 281.45), 1: (-281.45, 281.45), 2: (-70.0, 412.67)}

# 손끝→플랜지 보정: TF는 토마토 손끝(카메라가 본) 위치를 주는데 send_coords는
# 플랜지를 옮긴다. 손끝-플랜지 차(TCP, 고정 GRIPPER_ORI 기준 상수)를 빼 플랜지
# 목표로 바꾼다. 실측 시작값 — hover 잔차로 미세조정.
TCP_CORRECTION = [97.7, 75.9, -3.0]


# ---- 도착 편향 역보정 (서보 처짐) ------------------------------------------ #
# 실측 25건(2026-07-28 수확 런 도착오차 로그) 회귀. '도착오차 = 실제 − 명령':
#   x ≈ +3.90 (상수, 잔차std 2.9)
#   y ≈ +3.72 + 0.0864·y_명령   (R²=0.82, 잔차std 4.8→2.1)
#   z ≈ -4.14 + 0.0411·y_명령   (R²=0.65, 잔차std 2.6→1.5)
# z 는 25/25 전부 음수(평균 -6.8mm) — 팔은 항상 명령보다 아래에 도착한다. y·z 편향이
# '명령 y'에 비례하는 건 J1 이 옆으로 돌아갈수록 처짐·편향이 커지기 때문.
#
# ⚠ 이 보정은 **적용하지 않는다**(USE_ARRIVAL_COMP=False). 실물에서 켜 보니 도착오차는
# 의도대로 0 근처가 됐지만 파지율이 떨어지고 열매를 떨어뜨렸다. 이유 — 이 처짐은 '오차'가
# 아니라 **이미 캘리브레이션에 녹아 있던 값**이다. TF 보정(게이지 조그)·DESCEND_OFFSET·
# TCP_CORRECTION 이 모두 서보 ON 상태에서 손끝이 실제로 닿는 위치를 보고 맞춘 값이라,
# '명령 좌표 → 실제 손끝' 관계에 처짐이 포함돼 있다. 처짐을 없애면 그 관계가 7mm 위로
# 밀려 그리퍼가 열매 위·꼭지를 쳐서 떨어뜨린다. 측정값은 기록으로 남긴다(자세별 오프셋
# 매핑이나 0점 재캘리 때 참고).
USE_ARRIVAL_COMP = False
ARRIVAL_X_BIAS = 3.90
ARRIVAL_Y_A, ARRIVAL_Y_B = 3.72, 0.0864
ARRIVAL_Z_A, ARRIVAL_Z_B = -4.14, 0.0411


def compensate_arrival(target6):
    """목표 flange 좌표(6) → 도착 편향을 역보정한 '명령' 좌표(6).

    y 는 편향이 명령값에 비례하므로 역산: 실제 = y_c + (A + B·y_c) = 목표.
    z 는 그 보정된 y 를 써서 편향을 뺀다. 자세(rx,ry,rz)는 그대로.
    """
    x, y, z = float(target6[0]), float(target6[1]), float(target6[2])
    y_cmd = (y - ARRIVAL_Y_A) / (1.0 + ARRIVAL_Y_B)
    z_cmd = z - (ARRIVAL_Z_A + ARRIVAL_Z_B * y_cmd)
    return [x - ARRIVAL_X_BIAS, y_cmd, z_cmd] + list(target6[3:6])


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def flange_target(tomato_xyz):
    """카메라(손끝) 기준 목표 → send_coords(플랜지) 기준 목표."""
    return [tomato_xyz[i] - TCP_CORRECTION[i] for i in range(3)]


def _try_move(arm, coords, speed, mode, quiet=False) -> bool:
    """이동 시도. 도달 불가(RuntimeError)면 False 반환하고 건너뛴다.
    quiet=True 면 실패 로그를 찍지 않는다(pre-grasp 폴백처럼 정상 흐름일 때)."""
    try:
        arm.move_coords(coords, speed, mode)
        return True
    except RuntimeError as e:
        if not quiet:
            print(f"  (건너뜀) {e}")
        return False


# 자세 대안 — send_coords 는 '위치 + 자세'를 동시에 만족하는 IK 해를 요구하므로,
# 고정 자세 하나로는 위치가 닿는데도 해가 없는 사각지대가 생긴다(실측: z 200~215 낮은
# 열매). 손으로 옮기면 쉽게 닿는 이유는 손목이 다른 각도를 자유롭게 잡기 때문.
# 손목 요(rz)·피치(ry)를 조금씩 틀어 해가 있는 자세를 찾는다. 도달 불가 판정이 1.2초로
# 빨라졌기에 몇 번 시도해도 비용이 작다. 순서는 원본 자세를 최우선(캘리 기준).
# ⚠ 자세가 바뀌면 손끝-플랜지 관계도 바뀌어 위치가 미세하게 어긋난다 → 원본 자세로
#   되는 열매는 반드시 원본을 쓴다. 대안은 '못 따는 것보다 낫다' 수준의 폴백.
ORI_FALLBACK_DELTAS = [
    (0, 0), (0, -15), (0, 15), (-10, 0), (10, 0),
    (0, -30), (0, 30), (-10, -15), (10, 15),
]
# 자세 후보를 몇 개까지 시도할지 — 후보마다 'standoff + 진입' 쌍이라 비용이 있어 제한.
ORI_FALLBACK_TRIES = 5
# 손목을 세팅할 standoff 의 접근축 후퇴량(mm). -45 가 IK 안 풀리면 짧은 쪽으로.
STANDOFF_BACKOFFS = [-45.0, -30.0, -20.0]
# standoff 를 하나도 못 잡으면 그 열매를 **포기**한다(직접 진입 금지).
# 실측 근거: 손목을 못 세우고 들어간 3건에서 잎·가지를 물어(파지값 26·92·95) 놓쳤고,
# 그 과정에서 옆 열매가 여러 개 떨어져 수확 대상 자체가 사라졌다. 하나를 포기해
# 여러 개를 지키는 쪽이 이득. False 로 두면 종전처럼 밀고 들어간다.
REQUIRE_STANDOFF = True


def ori_candidates(ori):
    """원본 자세부터, 손목 (ry, rz) 를 조금씩 틀어본 자세 후보들."""
    return [[ori[0], ori[1] + dry, ori[2] + drz]
            for dry, drz in ORI_FALLBACK_DELTAS]


def move_with_ori_fallback(arm, xyz, ori, speed, mode):
    """xyz 로 이동. 자세는 원본 우선, 실패하면 손목을 틀어 재시도.

    성공한 자세를 반환(실패면 None). 어느 자세로 됐는지 로그로 남겨 두면 나중에
    구역별 자세 표나 AI 기울기 연동의 참고 자료가 된다.
    """
    for i, cand in enumerate(ori_candidates(ori)):
        try:
            arm.move_coords(list(xyz) + list(cand), speed, mode)
            if i:
                print(f"    (자세 대안 #{i} 사용: ry{cand[1] - ori[1]:+.0f}"
                      f" rz{cand[2] - ori[2]:+.0f})")
            return cand
        except RuntimeError:
            continue
    return None


def in_workspace(xyz) -> bool:
    return all(lo <= xyz[i] <= hi for i, (lo, hi) in COORD_LIMITS.items())


def move_observe(arm: ArmBackend, speed: int = TRANSIT_SPEED) -> None:
    """관측 자세로 복귀 (검출 직전)."""
    arm.move_angles(OBSERVE_ANGLES, speed)


def move_staging(arm: ArmBackend, speed: int = TRANSIT_SPEED) -> None:
    """수확 준비 자세로 (관측→접근 사이 경유). 이후부터 직선(mode=1) 접근."""
    arm.move_angles(STAGING_ANGLES, speed)


def zone_of(target_y: float):
    """열매가 속한 구역 (줄기 인덱스, 쪽). 줄기 미설정이면 None.

    쪽: +1 = 줄기의 왼쪽(+y), -1 = 오른쪽. 시연 매칭도 이 구역 안에서만 한다.
    """
    if not STEM_REFS_Y:
        return None
    i = min(range(len(STEM_REFS_Y)), key=lambda k: abs(target_y - STEM_REFS_Y[k]))
    return (i, 1 if target_y >= STEM_REFS_Y[i] else -1)


# 구역별 TF 잔차 보정 — 전역 보정(tf_transform.TF_OBSERVE_CORRECTION_MM) 위에 더한다.
# 왜 구역별인가: 전역 보정 후에도 남는 잔차가 위치마다 +35 ~ -10mm 로 크고, y 를 따라
# 선형이 아니다(기울기가 구간마다 8배 차이). 직선을 억지로 맞추면 측정 안 한 구간에서
# 크게 틀어지므로, **외삽 없이 각 구역에서 잰 값만** 쓴다.
# tf_verify 게이지 실측(2026-07-28). 미측정 구역은 0(=전역 보정만).
#   (0,-1) 우측줄 오른쪽편 : 계산 base y=-20.3 에서 잔차 [0, +35, 0]
#   (0,+1) 우측줄 왼쪽편   : y≈30 에서 잔차 ≈0 (전역 보정을 여기서 쟀음)
#   (1,+1) 좌측줄 왼쪽편   : y=152.2 에서 잔차 [+20, -10, -5]
#   (1,-1) 좌측줄 오른쪽편 : 미측정
# ⚠ 구역당 1점이라 구역 안에서도 잔차가 남는다. 근본 원인(핸드아이 추정)은 별도 과제.
# ⚠ 전부 0 (2026-07-28) — 관측자세 camera→flange 직접 피팅이 이 잔차들을 이미 흡수한다.
# 값은 기록으로만 남긴다(옛 체계: (0,-1)=[0,35,0], (1,1)=[20,-10,-5]).
ZONE_CORRECTION = {
    (0, -1): [0.0, 0.0, 0.0],
    (0, 1): [0.0, 0.0, 0.0],
    (1, 1): [0.0, 0.0, 0.0],
    (1, -1): [0.0, 0.0, 0.0],
}


def zone_correct(base):
    """전역 보정된 base 에 구역별 잔차 보정을 더한다. 줄기 미설정이면 그대로."""
    z = zone_of(base[1])
    if z is None:
        return list(base)
    d = ZONE_CORRECTION.get(z, [0.0, 0.0, 0.0])
    return [float(base[i]) + d[i] for i in range(3)]


# ---- 측면별 손목 자세 (상수 기울기) ---------------------------------------- #
# 열매는 구형이라 접근축 기준으로 회전 대칭 — 기울기가 의미 있는 건 '줄기·이웃을 피하는
# 방향'뿐이고, 그건 줄기 기준점으로 결정론적으로 안다(AI 기울기 검출 불필요).
# 그래서 측면(줄기의 왼쪽편/오른쪽편)마다 손목 자세를 상수로 둔다.
#   +1 (줄기의 왼쪽편) : 왼쪽 바깥에서 진입
#   -1 (줄기의 오른쪽편): 오른쪽 바깥에서 진입
#
# ⚠ 왜 구역 분기만으론 부족했나 — entry_sign 은 standoff 를 좌우 20mm 옮길 뿐이고,
#   그리퍼가 '어느 쪽을 향하는지'는 자세(rz)가 정한다. 자세가 모든 구역 동일하면
#   반대편 열매에선 줄기 쪽으로 파고든다(실측: 좌-오 구역 6연속 실패).
#
# ⚠ 자세가 바뀌면 손끝-플랜지 관계가 바뀌므로, 관측자세 피팅(camera→flange)에
#   **자세별 상수 보정**을 더해야 한다. 두 자세 모두 고정 회전이라 그 차이는 정확히
#   상수다 → 같은 열매를 두 자세로 터치해 한 번 재면 끝.
#   tf_verify 'a' 티칭이 (자세 + 그때의 flange) 를 함께 기록하므로 그걸로 산출한다.
# ⚠ 자세와 flange 보정은 **반드시 짝으로** 쓴다. 보정은 '그 자세에서의 손끝-플랜지
#   관계'라, 여러 티칭의 자세와 보정을 각각 평균하면 서로 안 맞는 조합이 된다.
#   TCP 가 ~110mm 이므로 자세 19° 불일치 = 최대 35mm 오차(노이즈 바닥 9mm보다 나쁨).
#   그래서 평균 없이 **구역별 측정 짝**을 그대로 쓴다.
# tf_verify 'w' 티칭(2026-07-28). 왼쪽편은 피팅의 기준 자세라 보정 0.
# 4구역 모두 실측. ry(손목 피치) 부호가 측면에 따라 깔끔하게 뒤집히고(왼쪽편 +36,
# 오른쪽편 -48~-60), 왼쪽편 두 구역은 rz 도 -88 로 거의 같다 — 설계가 물리와 맞는 신호.
ORI_BY_ZONE = {
    (1, 1): [-86.8, 37.3, -88.9],     # 좌-왼 (좌측줄기의 왼쪽)   — 왼쪽에서 진입
    (0, 1): [-89.5, 35.9, -88.2],     # 우-왼 (우측줄기의 왼쪽)   — 왼쪽에서 진입
    (1, -1): [-96.2, -60.2, -75.7],   # 좌-오 (좌측줄기의 오른쪽) — 오른쪽에서 진입
    (0, -1): [-90.9, -47.6, -94.7],   # 우-오 (우측줄기의 오른쪽) — 오른쪽에서 진입
}
FLANGE_DELTA_BY_ZONE = {
    (1, 1): [4.2, -10.2, -17.4],
    (0, 1): [-3.3, -20.0, -20.4],
    (1, -1): [2.4, -34.0, -7.6],
    (0, -1): [6.8, -19.2, -11.9],
}
# ---- 구역별 도착 편향 보정 -------------------------------------------------- #
# 명령한 좌표와 실제 도착(get_coords) 사이의 계통 편차. '도착오차 = 실제 − 명령' 이므로
# 명령에서 그만큼 빼면 실제가 목표에 온다.
#
# 왜 이제는 보정이 맞는가 (낮에 껐던 USE_ARRIVAL_COMP 와 다른 이유):
#   · 그때는 TF보정·DESCEND·TCP 가 처짐이 있는 상태로 end-to-end 튜닝돼 있어 이중 보정이었다.
#   · 지금 피팅의 기준값은 5mm 씩 조그해 터치한 위치 — 짧은 이동이라 처짐이 거의 없다.
#     반면 실제 파지는 standoff→그랩점 45mm 이동이라 처짐이 5~12mm 새로 생긴다.
#   · 직접 증거: 같은 열매가 도착오차 y=4.8 일 때 성공, y=12.4 일 때 실패(2026-07-28).
#
# 자세가 구역마다 다르면 팔 자세도 달라 편차도 달라진다 → 구역별로 따로 잰다.
# 실측(tf_verify 'p', 2026-07-28): 좌-왼 4건 평균 [1.2, 9.4, -4.6] / 우-왼 3건 [6.7, 1.3, -4.7]
# 좌-오·우-오 는 새 자세에서 미측정 → 0. 로그의 '도착오차' 를 모아 채운다.
ARRIVAL_COMP_BY_ZONE = {
    (1, 1): [1.2, 9.4, -4.6],     # 좌-왼  — 검증됨(보정 후 잔차 [-1.5,-5.2,2.9], 파지 성공)
    (0, 1): [6.7, 1.3, -4.7],     # 우-왼  — 검증됨(보정 후 잔차 [0.1,-0.6,0.1], 파지 성공)
    (0, -1): [6.3, -7.5, -11.1],  # 우-오  — 2건 평균 [6.1,-7.0,-10.7]/[6.4,-7.9,-11.5].
                                  #          z -11 = '아래로 쳐짐'의 정체. 검증 대기
    (1, -1): [0.0, 0.0, 0.0],     # 좌-오  — 미측정. 로그의 도착오차로 채운다
}

# 되돌리기 참고 — 왼쪽편 구역의 종전 값(피팅 기준 자세, 보정 0):
#   (1,1)/(0,1) ORI = GRIPPER_ORI = [-93.6, 9.7, -106.4],  DELTA = [0, 0, 0]
#   왼쪽편은 그 값으로도 파지에 성공했으므로, 새 자세가 더 나쁘면 해당 구역만 복구한다.


def entry_sign(target_y: float) -> float:
    """진입 쪽 부호 — standoff 를 열매의 어느 쪽에 둘지. 줄기 미설정이면 +1(종전)."""
    z = zone_of(target_y)
    return 1.0 if z is None else float(z[1])


def j1_aim(target_y: float) -> float:
    """목표 base y로 J1 수평 aim 각도(두 줄기 사이 선형보간). 두 줄 밖은 클램프.

    앵커는 STEM_REFS_Y(우측줄, 좌측줄). 줄기 미설정이면 staging J1 그대로.
    """
    if len(STEM_REFS_Y) < 2:
        return AIM_J1_RIGHT
    y_r, y_l = STEM_REFS_Y[0], STEM_REFS_Y[1]
    if y_l == y_r:
        return AIM_J1_RIGHT
    t = max(0.0, min(1.0, (target_y - y_r) / (y_l - y_r)))
    return AIM_J1_RIGHT + t * (AIM_J1_LEFT - AIM_J1_RIGHT)


def move_stow(arm: ArmBackend, speed: int = TRANSIT_SPEED) -> None:
    """이송 중 안전 자세 (만차 후)."""
    arm.move_angles(STOW_ANGLES, speed)


def drop_to_basket(arm: ArmBackend, grade: str, speed: int = TRANSIT_SPEED) -> bool:
    """바구니 투하 — 접근(바구니 위) → 놓기 → 접근 복귀. 항상 True.

    호출 시점엔 이미 staging 높이(캐노피 위)로 안전 상승한 상태다. 그래서 예전에 있던
    'J1만 수평회전' 단계는 불필요해져 제거했다(저위치 파지 자세에서 곧장 바구니로 갈 때
    캐노피를 훑는 걸 막으려던 단계 — 이제 상승이 그 역할을 한다). 이동 1회 절약.
    """
    approach = BASKET_APPROACH_ANGLES   # 공용 (NORMAL/DISCARD 공통)
    drop = BASKET_DROP_ANGLES.get(grade, BASKET_DROP_ANGLES["NORMAL"])

    # ① 놓기 좋은 접근 포지션 (바구니 위)
    arm.move_angles(approach, speed)
    time.sleep(SETTLE)

    # ③ 바구니에 놓기 (놓기 직전 재확인은 제거 — 상승 후 게이트가 이미 빈손을 걸러
    #    바구니행을 막으므로 중복이고, 픽당 1~2초가 사이클에 크게 누적된다)
    arm.move_angles(drop, speed)
    time.sleep(SETTLE)
    arm.open_gripper(GRIP_SPEED)
    time.sleep(SETTLE)

    # ③ 바구니에서 빠져나오기 — 놓기 자세(바구니 안)에서 곧장 관측으로 가면 바구니 벽을
    #    친다. 접근 자세(바구니 위)로 먼저 복귀해 이후 큰 이동이 바구니 위에서 시작되게.
    arm.move_angles(approach, speed)
    return True


def pick(arm: ArmBackend, target_xyz, grade: str, orientation=None,
         to_basket: bool = True) -> bool:
    """토마토 1개 파지(+투하) 1회 시도. 성공하면 True.

    target_xyz  : base 기준 [x,y,z] (mm) — tf_transform.camera_to_base 결과
    grade       : 'NORMAL' / 'DISCARD' → 투하 바구니 결정
    orientation : 그리퍼 자세 [rx,ry,rz]. None이면 기본 GRIPPER_ORI.
                  AI 기울기값이 있으면 그걸 넘긴다(고정방향 → 방향정렬 확장 지점).
    to_basket   : False면 바구니 투하 생략(파지→후퇴까지만). TF/파지 검증용.
    """
    # 접근 자세 결정: 위치별 티칭 모델(taught_approaches 보간)이 있으면 그걸로 — 끝
    # 토마토는 바깥에서 접근하는 등 위치마다 다른 자세가 필요. 시연 없으면 고정 오프셋 폴백.
    plan = (approach_model.plan(target_xyz, zone_fn=lambda b: zone_of(b[1]))
            if USE_APPROACH_MODEL else None)
    if plan is not None:
        pregrasp, approach = plan               # flange 6벡터(위치+자세, TCP·처짐 포함)
        retreat = list(pregrasp)                # 후퇴는 pre-grasp 로(왔던 길 역순)
        if orientation is not None:             # AI 기울기 오면 자세(rx,ry,rz)만 덮어씀
            pregrasp = pregrasp[:3] + list(orientation)
            approach = approach[:3] + list(orientation)
            retreat = retreat[:3] + list(orientation)
    else:
        # 줄기 기준 진입 쪽 → 손목 자세와 standoff 방향을 함께 결정한다.
        # 자세만 바꾸면 손끝 위치가 어긋나므로 자세별 flange 보정을 같이 더한다.
        z = zone_of(target_xyz[1])
        s = int(entry_sign(target_xyz[1]))
        ori = (list(orientation) if orientation is not None
               else list(ORI_BY_ZONE.get(z, GRIPPER_ORI)))
        d = (FLANGE_DELTA_BY_ZONE.get(z, [0.0, 0.0, 0.0])
             if orientation is None else [0.0, 0.0, 0.0])
        tgt = [flange_target(target_xyz)[i] + d[i] for i in range(3)]
        pre_off = [PREGRASP_OFFSET[0], PREGRASP_OFFSET[1] * s, PREGRASP_OFFSET[2]]
        ret_off = [RETREAT_OFFSET[0], RETREAT_OFFSET[1] * s, RETREAT_OFFSET[2]]
        pregrasp = _add(tgt, pre_off) + ori
        approach = _add(tgt, DESCEND_OFFSET) + ori
        retreat = None   # 그랩 명령이 확정된 뒤 그 좌표 기준으로 만든다(순수 후진 보장)

    # 그랩점(approach)은 반드시 도달 가능해야 파지가 성립한다. 범위 밖이면 거부.
    if not in_workspace(approach):
        print(f"  [거부] 접근점 {[round(c,1) for c in approach[:3]]} 가동범위 밖 — 목표/TF 확인")
        return False

    # 0) 관측→공통 staging(정면 unfold, 안전) → J1만 목표 방향으로 수평 aim
    #    (캐노피 위 순수 회전). 이후 pre-grasp·접근·후퇴는 반경방향 직선(mode=1)으로
    #    풀려 옆 열매를 안 쓴다. 우측줄이면 aim≈STAGING J1이라 스윙 ≈0.
    move_staging(arm, APPROACH_SPEED)
    aimed = [j1_aim(target_xyz[1])] + list(STAGING_ANGLES[1:])
    if abs(aimed[0] - STAGING_ANGLES[0]) > 1.0:   # 우측줄은 aim≈staging → 이동 생략
        arm.move_angles(aimed, APPROACH_SPEED)

    # 1~2) '손목 세팅(standoff) → 순수 병진 진입' 을 자세 후보마다 쌍으로 시도한다.
    #
    # ⚠ 손목은 **반드시 standoff 에서 미리 돌려놓고** 그 자세로 직진해야 한다. 진입
    #   도중에 손목을 돌리면 그리퍼가 열매·줄기를 쓸어버린다. 그래서 자세 대안을
    #   '그랩점에서 회전'으로 처리하면 안 되고, 후보 자세별로 standoff 부터 다시 잡는다.
    #
    # standoff 거리도 짧은 쪽으로 폴백한다(-45 가 IK 안 풀리면 -30). standoff 를
    # 아예 못 잡으면 손목이 이동 중 돌아가므로 최후 수단으로만 직접 진입한다.
    arm.open_gripper(GRIP_SPEED)
    time.sleep(SETTLE)
    # 그랩점 명령 = 목표 − 구역별 도착 편향 (실제 도착이 목표에 오도록)
    _ac = ARRIVAL_COMP_BY_ZONE.get(zone_of(target_xyz[1]), [0.0, 0.0, 0.0])
    grasp_xyz = [approach[i] - _ac[i] for i in range(3)]
    entered = False
    for i, cand in enumerate(ori_candidates(ori)[:ORI_FALLBACK_TRIES]):
        for back in STANDOFF_BACKOFFS:          # 손목을 세팅할 standoff 후보
            pre = [approach[0] + back, pregrasp[1], pregrasp[2]] + cand
            if (_try_move(arm, pre, APPROACH_SPEED, 1, quiet=True)
                    or _try_move(arm, pre, APPROACH_SPEED, 0, quiet=True)):
                break                            # 손목 세팅 완료
        else:
            if REQUIRE_STANDOFF:
                if i == 0:
                    print("  standoff 도달불가 — 이 자세 후보 건너뜀")
                continue          # 손목을 못 세우면 진입하지 않는다(옆 열매 보호)
            if i == 0:
                print("  standoff 도달불가 — 손목 세팅 없이 직접 진입(옆 열매 주의)")
        # 손목이 이미 목표 자세인 상태에서 순수 병진 진입
        if _try_move(arm, grasp_xyz + cand, APPROACH_SPEED, 0, quiet=True):
            entered = True
            if i:
                print(f"    (자세 대안 #{i} 사용: ry{cand[1] - ori[1]:+.0f}"
                      f" rz{cand[2] - ori[2]:+.0f})")
            ori = cand                           # 후퇴도 같은 자세로
            break
    if not entered:
        raise RuntimeError(
            f"진입 불가(standoff 확보 실패): 목표 {[round(c, 1) for c in grasp_xyz]}"
            f" — 손목을 세울 자리가 없어 포기(옆 열매 보호)")
    grasp_cmd = grasp_xyz + ori
    # 후퇴는 '실제 명령한 그랩 좌표'에서 접근축으로만 뺀다 — 목표 기준으로 계산하면
    # 도착보정분이 좌우/상하 성분으로 섞여 물고 있는 열매에 전단력이 생긴다.
    if retreat is None or True:
        retreat = _add(grasp_xyz, ret_off) + ori
    cur = arm.get_coords() or []
    if cur:
        derr = [round(cur[i] - approach[i], 1) for i in range(3)]
        print(f"    도착오차 {derr} mm (목표기준) "
              f"명령 {[round(c, 1) for c in grasp_cmd[:3]]}")
    arm.close_gripper(GRIP_SPEED)
    time.sleep(SETTLE)
    # 이 값은 참고·튜닝용(캐노피 안이라 잎·줄기 접촉으로 오염될 수 있음).
    # 실제 판정은 아래 '상승 후 확인'(1차)과 바구니 투하 직전(최종)에서 한다.
    grip_val = arm.gripper_value()
    print(f"    그리퍼값 {grip_val} (임계 {GRIP_THRESHOLD}) → "
          f"{'파지O' if grip_val > GRIP_THRESHOLD else '파지X'} (참고)")

    # 3) 후퇴(직선 우선, 관절 폴백) — 파지물을 접근축 역방향으로 곧게 뺀다.
    if not _try_move(arm, retreat, RETREAT_SPEED, mode=1):
        _try_move(arm, retreat, RETREAT_SPEED, mode=0)

    # 4) 안전 상승 — staging 높이(캐노피 위)로 복귀. 낮은 retreat 자세에서 곧장
    #    큰 이동(바구니/관측)을 하면 J1 스윙이 캐노피를 훑는다. 여기서 먼저 올라오면
    #    이후 어떤 큰 이동이든 캐노피 위에서 시작 → 옆 열매 안 침. (aimed = step 0의 조준 staging)
    arm.move_angles(aimed, RETREAT_SPEED)

    # 5) 파지 판정(1차 게이트) — 캐노피 위에서 동일 닫힘 명령 재차 + 값 읽기.
    #    여기서 빈손이면 **바구니 왕복을 건너뛴다**(사이클 시간 절약). 파지 직후 값은
    #    잎·줄기 접촉으로 오염될 수 있어 이 지점 값을 판정에 쓴다(캐노피 밖 = 깨끗).
    arm.close_gripper(GRIP_SPEED)
    time.sleep(SETTLE)
    lift_val = arm.gripper_value()
    held = lift_val > GRIP_THRESHOLD
    print(f"    [상승 후 확인] 그리퍼값 {lift_val} (임계 {GRIP_THRESHOLD}) → "
          f"{'파지O' if held else '빈손X — 바구니 생략'}")
    if not held:
        return False

    # 6) 바구니 투하 (검증 모드면 생략). 투하 직전 재확인이 최종 게이트 —
    #    이송 중 떨어졌으면 거기서 빈손으로 잡혀 False.
    if to_basket:
        return drop_to_basket(arm, grade)
    return True


if __name__ == "__main__":
    # FakeArm으로 시퀀스 흐름 검증 (하드웨어 불필요)
    from ddagi_harvest.arm_backend import make_arm

    print("=== 파지 성공 시나리오 (grip_result=39 > 임계값) ===")
    arm = make_arm("fake", grip_result=39)
    ok = pick(arm, [150.0, -40.0, 250.0], "NORMAL")
    print(f"결과: {'성공' if ok else '실패'}  (명령 {len(arm.log)}개)\n")

    print("=== 파지 실패 시나리오 (grip_result=0, 빈 그리퍼) ===")
    arm2 = make_arm("fake", grip_result=0)
    ok2 = pick(arm2, [150.0, -40.0, 250.0], "NORMAL")
    print(f"결과: {'성공' if ok2 else '실패(투하 스킵 확인)'}  (명령 {len(arm2.log)}개)")

    assert ok and not ok2, "FAIL: 성공/실패 판정이 기대와 다름"
    print("\nPASS — 파지 시퀀스 흐름 검증 통과")

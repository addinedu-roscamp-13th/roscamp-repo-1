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
DESCEND_OFFSET = [8.0, 5.0, 0.0]          # 파지점 오프셋 — base 대비 그리퍼 손끝이
                                          # 실제로 토마토를 감싸무는 위치(앞+8,좌+5,높이0).
                                          # tf_verify 게이지 조그 실측(2026-07-25, 중앙 토마토).
                                          # 15는 너무 깊어 8로↓(우측줄 실측). 서보 처짐분 포함.
PREGRASP_OFFSET = [-45.0, 20.0, -5.0]     # standoff(캐노피 밖·도달가능) — base 대비 뒤·좌·아래.
                                          # 여기서 mode=1 직선으로 그랩점 진입(옆 열매 회피).
                                          # tf_verify 조그 실측(2026-07-25).
RETREAT_OFFSET = [-45.0, 20.0, -5.0]      # 그립 후 후퇴 = standoff 로 직선 복귀(왔던 길)
# 속도(1~100). 사이클 시간의 대부분이 이동이라 여기가 최대 레버. 정밀이 필요한 구간만
# 중간 속도로 두고, 자세 경유·상승·바구니 같은 '이동만 하는' 구간은 빠르게.
APPROACH_SPEED = 55       # pre-grasp→그랩점. 종단 정확도가 파지율이라 과하게 올리지 않음
RETREAT_SPEED = 70        # 파지물 빼기 — 정밀 불필요
TRANSIT_SPEED = 85        # staging·J1 aim·상승·바구니 왕복 — 정밀 불필요, 제일 빠르게
GRIP_THRESHOLD = 6                    # 닫은 뒤 값이 이보다 크면 파지 성공(토마토 걸림).
                                      # 실측(gripper_check, 2026-07-27): 빈손=0, 작은토마토=12
                                      # → 중간 6. 큰 토마토는 12↑라 자동 통과. 그리퍼 재캘리 금지
                                      # (닫힘=0 정상 상태 기준값이라, 캘리 건드리면 다시 재야 함).
GRIP_SPEED = 100                      # 그리퍼 개폐 속도(1~100). 파지 지연 줄이려 최대로.
SETTLE = 0.2                          # 각 단계 후 정착 대기(s). 짧을수록 빠르나 너무 짧으면 값 오독

# 위치별 손티칭 접근모델(taught_approaches) 사용 여부. True 면 시연점 근처(approach_model.
# LOCAL_RADIUS_MM 내)의 토마토만 그 시연 접근을 쓰고, 시연 없는 곳은 고정 접근으로 폴백.
# → '문제 구역(오른쪽 끝 등)만 국소 티칭'하는 타깃 오버라이드. 나머진 고정 접근 유지.
USE_APPROACH_MODEL = True

# 관측/대기 자세 — 카메라 라이브뷰로 베드 프레이밍해 확정 (observe_setup.py, 2026-07-24)
# 이 자세의 get_coords는 tf_transform.OBSERVE_COORDS 와 반드시 짝을 이룬다(같이 갱신).
OBSERVE_ANGLES = [-8.9, 55.0, 1.6, -73.2, 19.2, -3.7]     # 검출 시 카메라가 베드 보는 자세
STOW_ANGLES = [-8.9, 55.0, 1.6, -73.2, 19.2, -3.7]        # TODO 미티칭 — 임시로 관측자세. E5 이송용 별도 티칭 필요

# 수확 준비 자세 — 관측↔pre-grasp 사이 '경유'. 관측→여기는 관절이동(mode=0)으로 한 번,
# 여기서부터 pre-grasp·접근·후퇴는 전부 직선(mode=1)으로 간다. 큰 관절 스윙을 이 한 번에
# 가두고 접근은 직선으로 → 옆 열매 회피. 베드 위·그리퍼 접근방향 정렬 자세.
# tf_verify 't' 드래그 티칭(2026-07-25). 관측자세를 바꾸면 이 자세 도달성도 재확인.
STAGING_ANGLES = [-2.7, -36.0, 113.5, -78.1, 1.1, 6.8]

# J1 수평 aim 보간 — 공통 staging에서 J1만 목표 방향으로 돌려(캐노피 위 순수 회전) 그 줄을
# 바라본 뒤 직선접근. 그래야 mode=1이 '가로 스윕' 없이 반경방향으로 풀린다. 두 줄을 실측한
# (base y, J1) 2점으로 선형보간, 두 줄 밖은 가까운 줄로 클램프. tf_verify 실측(2026-07-25):
#   우측줄 y=8.4 → J1=-2.7(=STAGING),  좌측줄 y=110.3 → J1=38.8.
AIM_Y_RIGHT, AIM_J1_RIGHT = 8.4, STAGING_ANGLES[0]
AIM_Y_LEFT, AIM_J1_LEFT = 110.3, 38.8

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


def in_workspace(xyz) -> bool:
    return all(lo <= xyz[i] <= hi for i, (lo, hi) in COORD_LIMITS.items())


def move_observe(arm: ArmBackend, speed: int = TRANSIT_SPEED) -> None:
    """관측 자세로 복귀 (검출 직전)."""
    arm.move_angles(OBSERVE_ANGLES, speed)


def move_staging(arm: ArmBackend, speed: int = TRANSIT_SPEED) -> None:
    """수확 준비 자세로 (관측→접근 사이 경유). 이후부터 직선(mode=1) 접근."""
    arm.move_angles(STAGING_ANGLES, speed)


def j1_aim(target_y: float) -> float:
    """목표 base y로 J1 수평 aim 각도(두 줄 실측 2점 선형보간). 두 줄 밖은 클램프."""
    t = (target_y - AIM_Y_RIGHT) / (AIM_Y_LEFT - AIM_Y_RIGHT)
    t = max(0.0, min(1.0, t))
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
    plan = approach_model.plan(target_xyz) if USE_APPROACH_MODEL else None
    if plan is not None:
        pregrasp, approach = plan               # flange 6벡터(위치+자세, TCP·처짐 포함)
        retreat = list(pregrasp)                # 후퇴는 pre-grasp 로(왔던 길 역순)
        if orientation is not None:             # AI 기울기 오면 자세(rx,ry,rz)만 덮어씀
            pregrasp = pregrasp[:3] + list(orientation)
            approach = approach[:3] + list(orientation)
            retreat = retreat[:3] + list(orientation)
    else:
        ori = list(orientation) if orientation is not None else GRIPPER_ORI
        tgt = flange_target(target_xyz)   # 손끝 목표 → 플랜지 목표
        pregrasp = _add(tgt, PREGRASP_OFFSET) + ori
        approach = _add(tgt, DESCEND_OFFSET) + ori
        retreat = _add(tgt, RETREAT_OFFSET) + ori

    # 그랩점(approach)은 반드시 도달 가능해야 파지가 성립한다. 범위 밖이면 거부.
    if not in_workspace(approach):
        print(f"  [거부] 접근점 {[round(c,1) for c in approach[:3]]} 가동범위 밖 — 목표/TF 확인")
        return False

    # 0) 관측→공통 staging(정면 unfold, 안전) → J1만 목표 방향으로 수평 aim
    #    (캐노피 위 순수 회전). 이후 pre-grasp·접근·후퇴는 반경방향 직선(mode=1)으로
    #    풀려 옆 열매를 안 쓴다. 우측줄이면 aim≈STAGING J1이라 스윙 ≈0.
    move_staging(arm, TRANSIT_SPEED)
    aimed = [j1_aim(target_xyz[1])] + list(STAGING_ANGLES[1:])
    if abs(aimed[0] - STAGING_ANGLES[0]) > 1.0:   # 우측줄은 aim≈staging → 이동 생략
        arm.move_angles(aimed, TRANSIT_SPEED)

    # 1) 그리퍼 열고 pre-grasp(standoff)로. 직선 우선, 안되면 관절. 둘 다 도달 불가면
    #    standoff 생략하고 그랩점으로 직접 간다(그랩점만 되면 파지 성립). 로그 한 줄로.
    arm.open_gripper(GRIP_SPEED)
    time.sleep(SETTLE)
    if not (_try_move(arm, pregrasp, APPROACH_SPEED, 1, quiet=True)
            or _try_move(arm, pregrasp, APPROACH_SPEED, 0, quiet=True)):
        print("  pre-grasp standoff 도달불가 → 그랩점으로 직접 접근(직선접근 생략)")

    # 2) 접근 → 그립. mode=0(관절)로 간다 — 실측상 mode=1(직선)은 종단이 살짝 왼쪽으로
    #    치우친다(카티전 보간 오차; 같은 목표라도 mode=0='g'는 정확). pre-grasp가 접근축에
    #    정렬돼 있어 이 짧은 구간은 mode=0도 거의 직선이라 옆 열매도 안 친다.
    arm.move_coords(approach, APPROACH_SPEED, mode=0)
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
    arm.move_angles(aimed, TRANSIT_SPEED)

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

#!/usr/bin/env python3
"""파지 시퀀스 — send_coords 기반 (MoveIt2 미사용, 제조사 IK 솔버).

한 개의 토마토를 집어 바구니에 넣는 1회 시도. 동작 분할(매니퓰레이션 정석):
    관측/대기 → pre-grasp(방향 세팅+뒤로) → 직선 접근(mode=1) → 그립
    → 파지 판정 → 직선 후퇴 → 바구니 투하 → 복귀

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

from ddagi_harvest.arm_backend import ArmBackend

# ---- 튜닝 상수 (실물에서 확정) --------------------------------------------- #
# 기본 그리퍼 자세 (rx,ry,rz). AI 기울기값이 오면 pick(orientation=...)로 덮어쓴다.
# 아래 값은 tune_pick.py 실물 튜닝 결과 (2026-07-24, grasp값 12→39로 확실히 잡힘).
GRIPPER_ORI = [-93.6, 9.7, -106.4]
PREGRASP_OFFSET = [-80.1, -14.3, -18.8]   # 목표 대비 pre-grasp(접근축으로 물러남)
DESCEND_OFFSET = [-5.0, -4.0, 12.0]       # 처짐 보정 — 서보 실행 시 팔이 아래+오른쪽으로
                                          # 쳐지는 것 상쇄(팀 공통 "보정치"). 위로+12, 좌로 −5·−4
RETREAT_OFFSET = [-78.3, -14.8, -19.8]    # 그립 후 후퇴(왔던 접근축 역방향)
APPROACH_SPEED = 30
RETREAT_SPEED = 30
GRIP_THRESHOLD = 10                   # 닫은 뒤 값이 이보다 크면 파지 성공(토마토 걸림)
SETTLE = 0.3

# 관측/대기 자세 — 티칭값 (capture_poses.py, 2026-07-24)
OBSERVE_ANGLES = [-11.7, 51.8, -12.4, -53.4, 10.2, 2.7]   # 검출 시 카메라가 베드 보는 자세
STOW_ANGLES = [-11.7, 51.8, -12.4, -53.4, 10.2, 2.7]      # TODO 미티칭 — 임시로 관측자세. E5 이송용 별도 티칭 필요

# 바구니 투하 — 'J1 수평회전 → 접근 → 놓기' 3단계.
# 바구니가 파지 지점보다 ~19.5cm 위라, 저위치 파지 후 대각선 상승하면 벽을 친다.
# 그래서 ①J1만 수평 회전(높이 유지) → ②접근 → ③놓기.
# 바구니 2개가 한 곳에 반반 → '접근'은 공용, '놓기'만 좌/우로 나뉜다.
BASKET_APPROACH_ANGLES = [-138.3, -62.0, 72.9, -12.0, 5.8, 3.8]   # 공용
BASKET_DROP_ANGLES = {
    "NORMAL":  [-138.5, -41.3, -5.5, -13.2, 5.4, 2.8],
    "DISCARD": [-130.5, -51.6, -5.5, 11.2, 2.5, 2.5],
}


def _add(a, b):
    return [a[i] + b[i] for i in range(3)]


def move_observe(arm: ArmBackend, speed: int = 30) -> None:
    """관측 자세로 복귀 (검출 직전)."""
    arm.move_angles(OBSERVE_ANGLES, speed)


def move_stow(arm: ArmBackend, speed: int = 30) -> None:
    """이송 중 안전 자세 (만차 후)."""
    arm.move_angles(STOW_ANGLES, speed)


def drop_to_basket(arm: ArmBackend, grade: str, speed: int = 30) -> None:
    """바구니 투하 — J1 수평회전 → 접근 → 놓기 3단계.

    저위치 파지 후 곧장 바구니로 대각선 상승하면 바구니 벽을 치므로,
    먼저 J1만 돌려 바구니 방향으로 정렬(현재 높이 유지)한 뒤 올라간다.
    """
    approach = BASKET_APPROACH_ANGLES   # 공용 (NORMAL/DISCARD 공통)
    drop = BASKET_DROP_ANGLES.get(grade, BASKET_DROP_ANGLES["NORMAL"])

    # ① J1만 바구니 방향으로 회전 (다른 관절 유지 → 높이 유지, 수평 스윙)
    cur = arm.get_angles()
    if cur:
        j1_rotate = list(cur)
        j1_rotate[0] = approach[0]
        arm.move_angles(j1_rotate, speed)
        time.sleep(SETTLE)

    # ② 놓기 좋은 접근 포지션
    arm.move_angles(approach, speed)
    time.sleep(SETTLE)

    # ③ 바구니에 놓기
    arm.move_angles(drop, speed)
    time.sleep(SETTLE)
    arm.open_gripper()
    time.sleep(SETTLE)


def pick(arm: ArmBackend, target_xyz, grade: str, orientation=None) -> bool:
    """토마토 1개 파지+투하 1회 시도. 성공(파지+투하)하면 True.

    target_xyz  : base 기준 [x,y,z] (mm) — tf_transform.camera_to_base 결과
    grade       : 'NORMAL' / 'DISCARD' → 투하 바구니 결정
    orientation : 그리퍼 자세 [rx,ry,rz]. None이면 기본 GRIPPER_ORI.
                  AI 기울기값이 있으면 그걸 넘긴다(고정방향 → 방향정렬 확장 지점).
    """
    ori = list(orientation) if orientation is not None else GRIPPER_ORI
    pregrasp = _add(target_xyz, PREGRASP_OFFSET) + ori
    approach = _add(target_xyz, DESCEND_OFFSET) + ori
    retreat = _add(target_xyz, RETREAT_OFFSET) + ori

    # 1) 그리퍼 열고 pre-grasp(방향 세팅된 채 뒤/위로 접근)
    arm.open_gripper()
    time.sleep(SETTLE)
    arm.move_coords(pregrasp, APPROACH_SPEED, mode=1)

    # 2) 직선 접근 → 그립
    arm.move_coords(approach, APPROACH_SPEED, mode=1)
    arm.close_gripper()
    time.sleep(SETTLE)
    grabbed = arm.grasped(GRIP_THRESHOLD)

    # 3) 직선 후퇴(들어올림)
    arm.move_coords(retreat, RETREAT_SPEED, mode=1)

    if not grabbed:
        # 파지 실패 → 잡은 것 없이 후퇴만. 투하 안 함.
        return False

    # 4) 바구니 투하
    drop_to_basket(arm, grade)
    return True


if __name__ == "__main__":
    # FakeArm으로 시퀀스 흐름 검증 (하드웨어 불필요)
    from ddagi_harvest.arm_backend import make_arm

    print("=== 파지 성공 시나리오 (grip_result=18) ===")
    arm = make_arm("fake", grip_result=18)
    ok = pick(arm, [150.0, -40.0, 250.0], "NORMAL")
    print(f"결과: {'성공' if ok else '실패'}  (명령 {len(arm.log)}개)\n")

    print("=== 파지 실패 시나리오 (grip_result=0, 빈 그리퍼) ===")
    arm2 = make_arm("fake", grip_result=0)
    ok2 = pick(arm2, [150.0, -40.0, 250.0], "NORMAL")
    print(f"결과: {'성공' if ok2 else '실패(투하 스킵 확인)'}  (명령 {len(arm2.log)}개)")

    assert ok and not ok2, "FAIL: 성공/실패 판정이 기대와 다름"
    print("\nPASS — 파지 시퀀스 흐름 검증 통과")

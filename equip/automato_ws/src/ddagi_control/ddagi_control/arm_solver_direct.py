#!/usr/bin/env python3
"""send_coords 기반 자체 솔버 경로 (「4. 로봇팔로 집기」 검증된 시퀀스).

send_coords()는 fire-and-forget(비동기)라 고정 sleep으로 완료를 가정하면
다음 명령이 이전 이동을 끊어버릴 수 있다. sync_send_coords()로 실제
목표 도달(is_in_position)까지 대기한다.
"""
import time

GRIPPER_ORIENTATION = [-133, 7, -100]

# 티칭으로 확보한 홈 포지션(관절 각도) — 매 pick을 항상 이 자세에서 시작/종료해
# IK 경로가 매번 달라지는 문제(임의 시작 자세 → 특이점/도달불가 경로)를 피한다.
HOME_ANGLES = [-9.05, 3.42, 1.31, -4.3, 17.13, 5.8]

# sync_send_coords/sync_send_angles 기본 timeout(15s)이 is_in_position() 판정
# 오차 때문에 매번 다 채워지는 경향이 있어 실측으로 낮춘 값.
MOVE_TIMEOUT = 3.0


def _wait_gripper(arm, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if arm.is_gripper_moving() == 0:
            return
        time.sleep(0.1)


def move_home(arm, speed=50, timeout=MOVE_TIMEOUT):
    arm.sync_send_angles(HOME_ANGLES, speed, timeout=timeout)


def pick_direct(arm, coord_xyz, speed=30, timeout=MOVE_TIMEOUT):
    x, y, z = coord_xyz

    move_home(arm, speed, timeout)

    arm.set_gripper_value(100, 50)
    _wait_gripper(arm)

    arm.sync_send_coords([x, y, z + 60] + GRIPPER_ORIENTATION, speed, 1, timeout=timeout)
    arm.sync_send_coords([x, y, z - 7] + GRIPPER_ORIENTATION, speed, 1, timeout=timeout)

    arm.set_gripper_value(0, 50)
    _wait_gripper(arm)

    arm.sync_send_coords([x, y, z + 93] + GRIPPER_ORIENTATION, speed, 1, timeout=timeout)
    grabbed = arm.get_gripper_value() > 10

    move_home(arm, speed, timeout)
    return grabbed

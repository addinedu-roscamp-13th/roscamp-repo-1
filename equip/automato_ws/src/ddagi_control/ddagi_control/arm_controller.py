#!/usr/bin/env python3
"""선택적 전환(자체 솔버 vs MoveIt2) 진입점.

use_moveit 플래그는 AI 서비스의 초기 스캔 결과(HarvestTarget.use_moveit)로
미리 결정되어 전달됨. 로컬 재판단/재시도 없음.
"""
from ddagi_control.arm_hardware import get_arm
from ddagi_control.arm_solver_direct import pick_direct


def pick_tomato(coord_xyz, use_moveit) -> bool:
    arm = get_arm()
    if use_moveit:
        raise NotImplementedError("moveit_bridge는 다음 단계에서 구현 예정")
    return pick_direct(arm, coord_xyz)

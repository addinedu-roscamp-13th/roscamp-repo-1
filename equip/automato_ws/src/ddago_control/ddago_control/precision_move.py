#!/usr/bin/env python3
"""정밀 주행 계산 — 순수 함수 모음(ROS 의존 없음).

navigate_server 가 쓰는 세 가지 계산만 모았다.
  ① 출발 정렬  : Nav2 에 목표를 주기 **전에** 이동 방향으로 미리 도는 각도
  ② 목표 방향  : Nav2 에 넘길 '도착 시 방향' = 목표 yaw 가 아니라 **가는 방향**
  ③ 정밀 조준  : Nav2 가 멈춘 뒤 목표 좌표로 좁힐 때의 오차 분해와 속도

rclpy·tf 를 임포트하지 않으므로 로봇 없이 단위 테스트할 수 있고, 로그도 남기지
않는다(무엇을 왜 했는지는 호출부인 노드가 남긴다).

--- 왜 이런 보정이 필요한가 (현장 실측 근거) ---
Nav2 의 도착 판정은 xy 5cm / yaw 11.5° 안에 들면 멈춘다(SimpleGoalChecker).
그런데 그 오차는 랜덤이 아니라 **한쪽으로 치우친 편향**이다 — 촬영 12곳 전부
목표에 못 미쳐 멈췄고 평균 +4.3cm 였다. 순찰 카메라는 로봇 옆 90° 를 보므로
이 4~5cm 가 그대로 **사진의 가로 밀림**이 된다(촬영 폭의 20~30%).

goal_checker 의 tolerance 를 좁히는 것은 답이 아니다. 그건 '달리면서' 맞추는
단계라 좁힐수록 좌우 왕복(헌팅)이 심해진다(실측 3→8→14회). 대신 **멈춘 뒤 따로**
좁히면 헌팅과 무관하다. 실측: 도착 fwd +4.3cm / yaw ±29° → 보정 후 ±0.5cm / ±1.1°.
"""
import math

# ── 기본 상수 (현장 7차 주행까지 검증된 값. 노드가 파라미터로 덮어쓸 수 있다) ──
# 속도 하한이 있는 이유: 너무 느리면 바퀴가 정지 마찰을 못 이겨 아예 안 움직인다.
V_MIN, V_MAX = 0.035, 0.060          # [m/s] 전후진 속도 범위
W_MIN, W_MAX = 0.150, 0.450          # [rad/s] 제자리 회전 속도 범위
K_LIN, K_ANG = 1.5, 2.0              # 남은 거리·각도에 곱하는 감속 계수
TOL_LIN = 0.005                      # [m] 이 안에 들어오면 됐다고 본다
TOL_ANG = math.radians(1.0)          # [rad] 위와 같음
MAX_FIX_LIN = 0.12                   # [m] 이보다 크게 어긋났으면 손대지 않는다(로그만)
MAX_FIX_ANG = math.radians(180.0)    # [rad] 코너 촬영은 90° 넘게 돌아야 해 넉넉히
MIN_TRAVEL = 0.10                    # [m] 이보다 짧으면 '가는 방향' 계산이 부정확


def normalize_angle(a):
    """각도를 -pi ~ +pi 로 접는다.

    접지 않으면 '-179° 로 가라'를 '+181° 만큼 돌아라'로 읽어 로봇이 먼 쪽으로 돈다.
    """
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quaternion(x, y, z, w):
    """쿼터니언 → yaw(rad). 평면 주행이라 Z 축 회전만 쓴다."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """yaw(rad) → 쿼터니언 (z, w). Z 축 회전만 있으므로 x=y=0 이다."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def pose_error(cur, target):
    """목표 대비 오차를 (전방, 횡, 각) 으로 분해한다. cur/target 은 (x, y, yaw).

    그냥 (dx, dy) 로 두지 않고 **목표 헤딩 기준으로 돌려서** 나누는 이유는, 옆을 보는
    카메라에서는 성분마다 사진에 미치는 영향이 전혀 다르기 때문이다.
      fwd  앞뒤 오차 → 화면이 가로로 밀리는 양.   +면 목표보다 덜 왔다.
      lat  좌우 오차 → 피사체까지의 거리.        +면 그만큼 멀다.
                       차동구동(옆으로 못 감)이라 못 고친다 — 측정해 로그로만 남긴다.
      dyaw 각도 오차.                            +면 덜 돌았다.
    """
    dx, dy = target[0] - cur[0], target[1] - cur[1]
    fwd = dx * math.cos(target[2]) + dy * math.sin(target[2])
    lat = -dx * math.sin(target[2]) + dy * math.cos(target[2])
    return fwd, lat, normalize_angle(target[2] - cur[2])


def travel_yaw(cur_xy, target_xy, min_travel=MIN_TRAVEL):
    """cur → target 으로 '가는 방향'(rad). 거리가 min_travel 미만이면 None.

    Nav2 에 넘길 도착 방향으로 쓴다. 목표 yaw 를 그대로 주면 Nav2 의 컨트롤러(RPP)가
    도착해서 제자리 회전을 하는데, 그 회전이 **등속**(rotate_to_heading_angular_vel)
    이라 목표각을 지나쳤다 되돌아오길 반복한다 = 두리번거림. 가는 방향을 주면 직진해
    도착한 순간 이미 그 방향이라 Nav2 가 돌 일이 없다. 방향 맞추기는 도착 후 정밀
    조준이 감속하며 대신 한다.

    None 을 돌려주는 경우(너무 짧은 이동)에는 호출부가 목표 yaw 를 그대로 쓴다 —
    몇 cm 이동에서 atan2 를 쓰면 위치 노이즈가 그대로 방향 오차가 되기 때문이다.
    """
    dx, dy = target_xy[0] - cur_xy[0], target_xy[1] - cur_xy[1]
    if math.hypot(dx, dy) < min_travel:
        return None
    return math.atan2(dy, dx)


def approach_speed(remain, gain, v_min, v_max):
    """남은 거리(또는 각도)에 비례한 속도의 **크기**. 방향은 호출부가 붙인다.

    남을수록 빠르고 가까울수록 느린 이 감속이 헌팅을 막는 핵심이다. Nav2 의 제자리
    회전은 등속이라 목표를 지나치고, 지나치면 되돌리느라 왕복한다. 우리는 감속하고,
    지나치면 되돌리지 않고 멈춘다(overshot 참고).
    """
    return clamp(gain * abs(remain), v_min, v_max)


def overshot(err, sign0):
    """목표를 지나쳤는가. sign0 은 처음 돌기 시작한 방향(+1/-1).

    지나친 뒤 되돌리면 그 되돌림이 또 지나치고… 를 반복하는 것이 헌팅이다.
    지나쳤으면 그냥 멈추는 편이 낫다 — 1° 안쪽이면 사진에 보이지도 않는다.
    """
    return err * sign0 < 0

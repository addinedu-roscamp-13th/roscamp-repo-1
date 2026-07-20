#!/usr/bin/env python3
"""검증 웹 — 가짜 로봇(Navigate 액션 클라이언트 흉내). 3단계.

이 파일이 이 검증 도구의 핵심 트릭이다.

PatrolDispatcher 는 rclpy 노드를 직접 참조하지 않고 'client' 를 인자로 받는다
(composition). 그래서 client 자리에 진짜 ActionClient 대신 이 가짜를 끼우면,
ROS 를 한 줄도 안 띄우고 진짜 디스패처·진짜 라우팅 엔진이 그대로 돌아간다.
디스패처 입장에선 진짜와 구분이 불가능하다 — 그게 이 검증이 의미를 갖는 이유다.

디스패처가 client 에게 요구하는 것은 딱 이만큼이다(patrol_dispatcher.py 기준):
  ① client.wait_for_server(timeout_sec=...)            -> bool
  ② client.send_goal_async(goal, feedback_callback=..) -> future ①
  ③ future①.result()                                   -> goal_handle (.accepted)
  ④ goal_handle.get_result_async()                     -> future ②
  ⑤ future②.result().result                            -> .result_code / .last_waypoint_id
  그리고 feedback_callback(msg) 의 msg.feedback.current_waypoint_id

future 는 파이썬 표준 concurrent.futures.Future 를 그대로 쓴다. rclpy 의 future 와
필요한 부분(add_done_callback / result)의 사용법이 같아서 흉내낼 필요가 없다.
'누가 future 를 채우는가' 만 다르다:
  진짜 — ROS executor 스레드가 DDS 로 온 로봇 응답을 받아 채운다.
  가짜 — 아래 _drive() 워커 스레드가 이동 시간을 흘려보낸 뒤 채운다.

메시지는 진짜 타입(Navigate.Feedback/Result, automato_interfaces)을 그대로 만든다.
필드명·타입이 어긋나면 여기서 즉시 터지므로 인터페이스 정합성까지 함께 검증된다.
"""
import math
import threading
import time
from concurrent.futures import Future

from automato_interfaces.action import Navigate

# 이동 시뮬 해상도. 0.1초마다 위치를 갱신해 화면에서 로봇이 '흐르듯' 움직이게 한다.
STEP_SEC = 0.1

# 이 거리 이내면 '같은 자리'로 보고 주행 대신 제자리 회전을 한다(짝 촬영).
# 짝은 부모와 좌표가 완전히 같아야 하는 게 스키마 불변식이라 아주 작게 잡아도 되지만,
# 부동소수 왕복(DB→JSON→float) 오차를 흡수할 만큼의 여유는 둔다.
SAME_SPOT_M = 0.005


class FakeRobot:
    """가짜 로봇 1대의 물리 상태. 주행 스레드가 쓰고 상태 방송 스레드가 읽으므로 락으로 보호한다.

    '로봇이 지금 어디 있는가'의 소스오브트루스다. 디스패처 안(_navigate 의 지역변수)에는
    로봇 위치가 남지 않기 때문에, 화면에 그릴 위치는 여기서 읽는다.
    """

    def __init__(self, robot_id: str, x: float, y: float, yaw: float = 0.0,
                 waypoint_id=None):
        self.robot_id = robot_id
        self._lock = threading.Lock()
        self._x = x
        self._y = y
        self._yaw = yaw
        # 마지막으로 도달한 '그래프 노드'. 짝(pair)은 여기 넣지 않는다 — 아래 설명 참고.
        self._wp = waypoint_id
        self._capture_wp = None     # 방금 촬영한 waypoint(짝이면 짝 id). 표시용.
        self._moving = False
        self._spinning = False      # 짝 촬영용 제자리 회전 중

    def set_pose(self, x, y, yaw=None, waypoint_id=None, moving=None,
                 spinning=None, capture_wp=False) -> None:
        """capture_wp=False 는 '건드리지 않음'. None 을 넣으면 지운다(구분 필요)."""
        with self._lock:
            self._x = x
            self._y = y
            if yaw is not None:
                self._yaw = yaw
            if waypoint_id is not None:
                self._wp = waypoint_id
            if capture_wp is not False:
                self._capture_wp = capture_wp
            if moving is not None:
                self._moving = moving
            if spinning is not None:
                self._spinning = spinning

    def snapshot(self) -> dict:
        """화면 방송용 현재 상태(락 짧게 잡고 값만 복사).

        moving 과 spinning 을 나눠 두는 이유: 제자리 회전은 통로를 지나지 않는 동작이라
        '왜 이 로봇은 안 움직이는데 통로를 계속 쥐고 있나'가 화면에서 설명돼야 한다.
        (회전 중에도 예약을 유지한다 — 좁은 통로에서 남이 들어오면 마주치기 때문.)
        """
        with self._lock:
            return {
                "robot_id": self.robot_id,
                "x": self._x, "y": self._y, "yaw": self._yaw,
                "waypoint_id": self._wp, "capture_wp": self._capture_wp,
                "moving": self._moving, "spinning": self._spinning,
            }


class _FakeGoalHandle:
    """진짜 ClientGoalHandle 흉내. 디스패처는 .accepted 와 .get_result_async() 만 쓴다."""

    def __init__(self, result_future: Future):
        self.accepted = True          # 이 시뮬은 Goal 을 항상 수락한다
        self._result_future = result_future

    def get_result_async(self) -> Future:
        return self._result_future


class _FeedbackMessage:
    """rclpy 가 feedback_callback 에 넘기는 래퍼 흉내 — 알맹이는 진짜 Navigate.Feedback."""

    def __init__(self, feedback):
        self.feedback = feedback


class _ResultResponse:
    """result_future.result() 가 돌려주는 래퍼 흉내 — 알맹이는 진짜 Navigate.Result."""

    def __init__(self, result):
        self.result = result


class FakeNavigateClient:
    """Navigate 액션 클라이언트 자리에 끼우는 가짜 로봇 구동기(로봇 1대당 1개)."""

    def __init__(self, robot: FakeRobot, wp_meta: dict, *, speed_mps: float = 0.06,
                 spin_rps: float = 0.9, is_edge_blocked=None, logger=None):
        """
        robot          : 위치를 갱신할 대상(FakeRobot).
        wp_meta        : waypoint_id -> {"x","y","yaw",...}. 목표 좌표를 여기서 얻는다.
        speed_mps      : 주행 속도(m/s). 맵이 작아(통로 0.03~0.44m) 기본을 느리게 잡았다.
        spin_rps       : 제자리 회전 속도(rad/s). 짝 촬영이 화면에서 보이도록 시간을 쓴다.
        is_edge_blocked: fn(from_wp, to_wp) -> bool. True 면 그 구간에서 '진짜 막힘'을
                         만나 result_code=1 로 되돌린다(막힘 우회 시나리오 재현용).
        logger         : .info/.warn 을 가진 로거(없으면 로그 없음).
        """
        self.robot = robot
        self._wp_meta = wp_meta
        self.speed_mps = speed_mps
        self.spin_rps = spin_rps
        self._is_edge_blocked = is_edge_blocked
        self._log = logger
        self._cancel = threading.Event()   # 시뮬 종료 시 주행 스레드를 깨워 빠져나오게 한다

    # --------------------------- 액션 클라이언트 표면 --------------------------- #
    def wait_for_server(self, timeout_sec=None) -> bool:
        """진짜는 액션 서버 발견까지 기다린다. 가짜 로봇은 항상 준비돼 있다."""
        return True

    def send_goal_async(self, goal, feedback_callback=None) -> Future:
        """Goal 을 접수하고 주행 스레드를 띄운다. 즉시 완료된 future ①을 돌려준다.

        진짜 액션에서 future 가 두 개인 이유가 여기서 그대로 재현된다:
          future ① = '접수됐는가'(즉시 결정), future ② = '다 갔는가'(한참 뒤).
        """
        result_future = Future()
        handle = _FakeGoalHandle(result_future)

        worker = threading.Thread(
            target=self._drive, args=(goal, result_future, feedback_callback),
            name=f"fake-drive-{self.robot.robot_id}", daemon=True)
        worker.start()

        goal_future = Future()
        goal_future.set_result(handle)     # 접수는 즉시 확정
        return goal_future

    def shutdown(self) -> None:
        """진행 중인 주행을 중단시킨다(서버 종료·시나리오 리셋용)."""
        self._cancel.set()

    # --------------------------- 주행 시뮬 --------------------------- #
    def _drive(self, goal, result_future: Future, feedback_callback) -> None:
        """waypoint 배열을 순서대로 '이동'하며 Feedback 을 흘리고, 끝나면 future ②를 채운다.

        여기가 진짜에서 'ROS executor + 실제 로봇' 이 하던 일을 대신하는 자리다.
        """
        state = self.robot.snapshot()
        cur_x, cur_y = state["x"], state["y"]
        prev_wp = state["waypoint_id"]
        last_reached = -1          # 하나도 못 갔으면 -1 (Result.last_waypoint_id 규약)
        code = 0
        message = "도착"

        for index, wp in enumerate(goal.waypoints):
            target = self._wp_meta.get(wp.waypoint_id, {})
            tx = float(target.get("x", wp.x))
            ty = float(target.get("y", wp.y))

            # 좌표가 같으면 '짝(pair) 촬영' = 주행이 아니라 제자리 회전이다.
            # 실제 로봇도 "직전 waypoint 와 좌표가 같으면 Spin"으로 분기한다 —
            # 몇 cm만 어긋나도 좁은 통로에서 주행을 시도하다 실패하므로 같은 기준을 쓴다.
            same_spot = math.hypot(tx - cur_x, ty - cur_y) <= SAME_SPOT_M

            # (a) 이 구간이 '막힌' 구간인가 → 진짜 로봇이 막힘을 보고하는 상황 재현.
            #     제자리 회전은 통로를 지나지 않으므로 막힘 판정에서 제외한다.
            if not same_spot and self._is_edge_blocked and prev_wp is not None \
                    and self._is_edge_blocked(prev_wp, wp.waypoint_id):
                code, message = 1, f"{prev_wp}→{wp.waypoint_id} 구간 막힘"
                if self._log:
                    self._log.warn(
                        f"[가짜로봇 {self.robot.robot_id}] {message} "
                        f"→ result_code=1 (도달 {last_reached})")
                break

            # (b) 실제 동작: 회전이면 yaw 를, 주행이면 위치를 시간에 걸쳐 바꾼다.
            if same_spot:
                if self._log:
                    self._log.info(
                        f"[가짜로봇 {self.robot.robot_id}] 짝 촬영 제자리 회전 "
                        f"→ wp{wp.waypoint_id} (yaw {wp.yaw:.3f})")
                ok = self._spin_to(float(wp.yaw))
            else:
                ok = self._move_to(cur_x, cur_y, tx, ty)
            if not ok:
                code, message = 2, "시뮬 중단"      # shutdown() → 중단(2)
                break
            cur_x, cur_y = tx, ty
            last_reached = wp.waypoint_id

            if same_spot:
                # 짝은 '그래프 노드'가 아니다(corridors 에 없다). 로봇의 현재 노드를
                # 짝 id 로 바꾸면 다음 구간의 출발 노드가 그래프에 없는 값이 되어
                #   · 간선 조회(막힘 판정)가 조용히 빗나가고
                #   · 다음 작업의 시작 노드로 쓰면 경로 탐색이 실패한다.
                # 그래서 위치 식별자는 부모 노드 그대로 두고, 촬영 id 만 따로 남긴다.
                self.robot.set_pose(tx, ty, yaw=wp.yaw, capture_wp=wp.waypoint_id,
                                    moving=False, spinning=False)
            else:
                prev_wp = wp.waypoint_id
                self.robot.set_pose(tx, ty, yaw=wp.yaw, waypoint_id=wp.waypoint_id,
                                    capture_wp=None, moving=False, spinning=False)

            # (c) Feedback 발행 — 노드에 도달할 때마다. ACS 는 이걸로 조기 반납을 한다.
            if feedback_callback is not None:
                fb = Navigate.Feedback()
                fb.current_waypoint_id = int(wp.waypoint_id)
                fb.waypoint_index = int(index)
                fb.current_x = float(tx)
                fb.current_y = float(ty)
                fb.current_yaw = float(wp.yaw)
                feedback_callback(_FeedbackMessage(fb))

        self.robot.set_pose(cur_x, cur_y, moving=False)

        result = Navigate.Result()
        result.result_code = int(code)
        result.last_waypoint_id = int(last_reached)
        result.message = message
        if self._log:
            self._log.info(
                f"[가짜로봇 {self.robot.robot_id}] 세그먼트 종료 code={code} "
                f"last_wp={last_reached}")
        result_future.set_result(_ResultResponse(result))

    def _spin_to(self, target_yaw: float) -> bool:
        """제자리에서 target_yaw 까지 회전(짝 촬영). 중단되면 False.

        최단 회전 방향을 쓴다: atan2(sin Δ, cos Δ) 로 Δ를 (-π, π] 로 정규화하면
        "179° 돌기" 대신 "-181°"처럼 먼 쪽으로 도는 일이 없다. 각도를 그냥 빼면
        3.1 → -3.1 같은 경계에서 6.2rad(거의 한 바퀴)를 도는 버그가 난다.
        """
        s = self.robot.snapshot()
        start = s["yaw"]
        x, y = s["x"], s["y"]
        delta = math.atan2(math.sin(target_yaw - start), math.cos(target_yaw - start))
        duration = abs(delta) / self.spin_rps if self.spin_rps > 0 else 0.0
        steps = max(1, int(duration / STEP_SEC))
        for i in range(1, steps + 1):
            if self._cancel.wait(STEP_SEC):
                return False
            self.robot.set_pose(x, y, yaw=start + delta * (i / steps),
                                moving=False, spinning=True)
        self.robot.set_pose(x, y, yaw=target_yaw, moving=False, spinning=False)
        return True

    def _move_to(self, x0, y0, x1, y1) -> bool:
        """(x0,y0) → (x1,y1) 를 speed_mps 로 이동. 중단되면 False.

        한 번에 순간이동시키지 않는 이유: 화면에서 로봇이 통로 위를 지나가는 게 보여야
        '지금 이 통로를 점유 중'이라는 예약 상태와 눈으로 대조할 수 있다.
        """
        dist = math.hypot(x1 - x0, y1 - y0)
        duration = dist / self.speed_mps if self.speed_mps > 0 else 0.0
        steps = max(1, int(duration / STEP_SEC))
        for i in range(1, steps + 1):
            if self._cancel.wait(STEP_SEC):     # 대기 겸 중단 확인
                return False
            t = i / steps
            self.robot.set_pose(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, moving=True)
        return True

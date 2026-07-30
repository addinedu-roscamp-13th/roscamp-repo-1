#!/usr/bin/env python3
"""시나리오2 E6 : Ddagi 예냉실 하역 액션 서버 (/ddagi/unload).

따고가 예냉실에 도킹하면 DG 가 Unload Goal 을 보낸다. 바구니 손잡이를 잡아 들고,
예냉실 위에서 기울여 쏟은 뒤 제자리에 놓고 복귀한다. 진행 상황은 phase 로 올린다.

    Goal ──> [손잡이 파지 → 들기 → 쏟기(대기) → 되돌리기 → 놓기 → 복귀] ──> Result

동작은 티칭한 관절각 경로를 재생하는 것이다(teach_unload.replay). 좌표가 아니라
관절각인 이유는 같은 좌표에도 IK 해가 매번 달라져 경로가 튀기 때문이다 — 짐을 든 채
경로가 바뀌면 예냉실 벽을 친다.

    티칭:  python3 ddagi_harvest/teach_unload.py teach
    검사:  python3 ddagi_harvest/teach_unload.py check
    수동:  python3 ddagi_harvest/teach_unload.py run

■ 티칭 경로는 로봇마다 다르다
unload_path.json 은 그 로봇·그 예냉실 배치에서 손으로 가르친 값이라 .gitignore 로
빠져 있다. **로봇마다 각자 티칭해야 한다.** 파일이 없으면 Goal 을 거절한다.

■ 털기는 빼기로 했다 (2026-07-30)
경로에서 shake 스텝의 act 를 해제했다. 코드는 기능을 남겨 두었으니(do_shake)
되살리려면 unload_path.json 에서 그 스텝의 act 를 "shake" 로 되돌리면 된다.
그래서 Feedback 에 SHAKE phase 는 현재 나오지 않는다.

실행:
    ros2 run ddagi_harvest unload_node
    ros2 launch ddagi_harvest harvest_with_ai.launch.py   # 수확과 함께 (권장)
"""
from __future__ import annotations

import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from automato_interfaces.action import Unload
from ddagi_harvest import log as L
from ddagi_harvest import pick as pk
from ddagi_harvest import teach_unload as tu
from ddagi_harvest.arm_backend import FakeArm, NetworkArm

ACTION_NAME = "/ddagi/unload"      # 로봇 세트 내부라 {robot_id} 네임스페이스 없음


class UnloadActionServer(Node):
    def __init__(self):
        super().__init__("ddagi_unload_node")
        self.declare_parameter("arm", "network")
        self.declare_parameter("arm_ip", tu.ARM_IP)
        # 티칭 파일 경로. 비우면 패키지 기본 위치(teach_unload.PATH_FILE).
        self.declare_parameter("path_file", "")

        self._arm = None
        self._busy = threading.Lock()   # 팔은 1대 — Goal 동시 실행을 막는다

        lg = self.get_logger()
        L.set_sink(info=lg.info, warning=lg.warning, err=lg.error)

        self._server = ActionServer(
            self, Unload, ACTION_NAME,
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        lg.info(f"Ddagi 하역 액션 서버 시작: {ACTION_NAME}")

    # ---- Goal 수락/취소 ------------------------------------------------- #

    def on_goal(self, goal_request):
        if self._busy.locked():
            self.get_logger().warning("하역이 이미 진행 중 — Goal 거절")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        # 실제 중단은 스텝 경계에서 일어난다(replay 의 should_cancel).
        self.get_logger().info("취소 요청 접수 — 현재 스텝을 마치고 중단")
        return CancelResponse.ACCEPT

    # ---- 하드웨어 ------------------------------------------------------- #

    def _ensure_arm(self):
        if self._arm is None:
            if self.get_parameter("arm").value == "fake":
                self.get_logger().warning("팔 = FakeArm (명령 로깅만, 실제 이동 없음)")
                # 18 = 손잡이가 걸려 완전히 안 닫힌 값(파지 성공 시뮬). 임계 6 초과.
                self._arm = FakeArm(start_angles=pk.OBSERVE_ANGLES,
                                    grip_result=18, verbose=False)
            else:
                ip = self.get_parameter("arm_ip").value
                self.get_logger().info(f"로봇팔 연결 {ip}:9010")
                self._arm = NetworkArm(ip)
        return self._arm

    def _load_path(self):
        pf = self.get_parameter("path_file").value
        if pf:
            tu.PATH_FILE = pf          # teach_unload.load() 가 이 경로를 읽는다
        return tu.load()

    # ---- 실행 ----------------------------------------------------------- #

    def execute(self, goal_handle):
        req = goal_handle.request
        result = Unload.Result()

        if not self._busy.acquire(blocking=False):
            goal_handle.abort()
            result.result_code = 2
            result.message = "다른 하역이 진행 중"
            return result

        # shake_delay_sec 은 '들어올린 뒤 대기 시간' 이다(Unload.action). 쏟아지는 걸
        # 기다리는 구간이라 티칭의 wait 스텝 길이로 쓴다. 0 이면 티칭 기본값.
        wait_sec = float(req.shake_delay_sec) or tu.WAIT_SEC
        self.get_logger().info(
            f"하역 시작 task_id={req.task_id} 대기 {wait_sec:.1f}s")

        try:
            data = self._load_path()
            steps = data["steps"]
            arm = self._ensure_arm()

            def on_phase(phase, step_no, total):
                fb = Unload.Feedback()
                fb.phase = phase
                goal_handle.publish_feedback(fb)

            r = tu.replay(
                arm, steps,
                speed=data.get("speed", tu.SPEED),
                shake_cfg=data.get("shake", {}),
                wait_sec=wait_sec,
                on_phase=on_phase,
                should_cancel=lambda: goal_handle.is_cancel_requested,
            )
        except SystemExit as exc:
            # teach_unload.load() 는 파일이 없으면 SystemExit 을 던진다.
            self.get_logger().error(f"하역 실패: {exc}")
            goal_handle.abort()
            result.result_code = 2
            result.message = f"티칭 경로 없음 — {exc}"
            self._busy.release()
            return result
        except Exception as exc:
            self.get_logger().error(f"하역 실패: {exc}")
            # 팔 연결을 버려 다음 Goal 이 새로 붙게 한다. 소켓이 한 번 끊기면
            # 죽은 객체를 계속 들고 있어 이후 모든 Goal 이 같은 오류로 실패한다.
            if self._arm is not None:
                try:
                    self._arm.close()
                except Exception:
                    pass
                self._arm = None
                self.get_logger().warning("팔 연결을 버렸습니다 — 다음 Goal 에서 재접속")
            goal_handle.abort()
            result.result_code = 2
            result.message = str(exc)
            self._busy.release()
            return result
        finally:
            if self._busy.locked():
                self._busy.release()

        result.result_code = int(r["result_code"])
        result.message = r["message"]
        if result.result_code == 2:
            goal_handle.canceled()
        elif result.result_code == 0:
            goal_handle.succeed()
        else:
            # 손잡이 파지 실패(1). 하역은 보너스 기능이라 ACS 가 task 를 FAILED 로
            # 되돌리지 않는다(Unload.action 주석) — 그래도 goal 은 실패로 알린다.
            goal_handle.abort()
        self.get_logger().info(
            f"하역 종료 code={result.result_code} — {result.message}")
        return result

    def destroy_node(self):
        if self._arm is not None:
            self._arm.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnloadActionServer()
    # 실행 콜백이 수십 초 블로킹하므로 단일 스레드면 취소 요청도 처리하지 못한다.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGINT 를 받으면 rclpy 가 컨텍스트를 이미 닫고 spin() 이 돌아온다.
        # 거기서 shutdown() 을 또 부르면 RCLError 로 죽어 종료 코드가 1이 된다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

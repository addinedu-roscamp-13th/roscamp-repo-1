#!/usr/bin/env python3
"""RP-126: 바닥 H 도킹 반복 테스트 클라이언트.

FloorDock 액션(단일 도킹)을 **고정 횟수 순차** 전송한다(1회 완료 → 다음).
floor_pose.py 의 '자동 반복'을 서버 밖(클라이언트)에서 재현 — 서버는 단일 도킹
의미를 유지한다.

'벽에서 후퇴'(다음 회차가 마커를 다시 보게)는 **클라이언트가 회차별로 서버
파라미터 `post_advance_m` 을 설정**해서 제어한다:
  * i < count  → post_advance_m = `post_advance_m`(기본 0.05)  (후퇴 후 다음 도킹)
  * 마지막/단발 → post_advance_m = 0  (**후퇴 없이 도킹 상태로 종료**)
  ※ post_advance_m 은 '고정 후퇴거리'가 아니라 **STAGED 복귀(rev_dist) 뒤 여유(margin)**.
    ADVANCE 실제거리 = rev_dist + margin → 재획득 d ≈ D_STAGE + margin (신뢰창 안).
    신뢰창을 넘으면 데이터가 없어 복구 불가라, margin 은 창 far 끝까지로 클램프된다.

한 회차라도 실패(거절/ABORT/미검출)하면 **즉시 중단하고 요약을 출력**한다.

실행 예 (서버 launch 는 post_advance_m 불필요 — 클라이언트가 회차별로 덮어씀):
  ros2 launch ddago_control ddago_floor_dock.launch.py robot_id:=dg_03
  ros2 run ddago_control floor_dock_repeat_client --ros-args \\
    -p count:=5 -p wall_gap_m:=0.03 -p post_advance_m:=0.05 -p pause_s:=2.0

파라미터:
  count(int=5)          반복 횟수 (1이면 단발 → 후퇴 없이 도킹 종료)
  wall_gap_m(float=0)   목표 후면~벽 [m] (0=서버 기본)
  lateral_offset_m(0)   횡 오프셋 [m] (0=서버 기본)
  post_advance_m(0.05)  STAGED 복귀 후 여유 margin [m] (실제 ADVANCE=rev_dist+margin, 창 클램프)
  task_point_id('TEST') goal 라벨
  pause_s(float=2.0)    회차 사이 대기 [s]
  action_name('/ddago/floor_dock')
  server_node('ddago_floor_dock_server')  파라미터 설정 대상
  stop_on_fail(bool=True)  실패 시 즉시 중단
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from automato_interfaces.action import FloorDock


class FloorDockRepeatClient(Node):
    def __init__(self):
        super().__init__('floor_dock_repeat_client')
        self.declare_parameter('count', 5)
        self.declare_parameter('wall_gap_m', 0.0)
        self.declare_parameter('lateral_offset_m', 0.0)
        self.declare_parameter('task_point_id', 'TEST')
        self.declare_parameter('pause_s', 2.0)
        self.declare_parameter('action_name', '/ddago/floor_dock')
        self.declare_parameter('stop_on_fail', True)
        # 회차 사이 후퇴[m]. 클라이언트가 서버 파라미터를 회차별로 설정한다:
        #  i<count → 이 값(다음 마커 보려 후퇴), 마지막/단발 → 0(도킹 상태로 종료).
        self.declare_parameter('post_advance_m', 0.05)
        self.declare_parameter('server_node', 'ddago_floor_dock_server')

        g = self.get_parameter
        self.count = int(g('count').value)
        self.wall_gap = float(g('wall_gap_m').value)
        self.lateral = float(g('lateral_offset_m').value)
        self.point_id = str(g('task_point_id').value)
        self.pause_s = float(g('pause_s').value)
        self.stop_on_fail = bool(g('stop_on_fail').value)
        self.post_advance = float(g('post_advance_m').value)
        self._action_name = str(g('action_name').value)
        self._client = ActionClient(self, FloorDock, self._action_name)
        self._pcli = AsyncParameterClient(self, str(g('server_node').value))
        self._last_phase = None

    def _set_server_advance(self, val):
        """서버의 post_advance_m 을 val 로 설정(회차별). 실패해도 진행(경고만)."""
        if not self._pcli.wait_for_services(timeout_sec=3.0):
            self.get_logger().warn('서버 파라미터 서비스 없음 — post_advance 설정 생략')
            return
        fut = self._pcli.set_parameters(
            [Parameter('post_advance_m', Parameter.Type.DOUBLE, float(val))])
        rclpy.spin_until_future_complete(self, fut)

    # --- 피드백: phase 바뀔 때만 한 줄 ---
    def _on_feedback(self, msg):
        fb = msg.feedback
        if fb.phase != self._last_phase:
            self._last_phase = fb.phase
            self.get_logger().info(
                '   .. %-10s marker=%d dw=%.1fcm'
                % (fb.phase, int(fb.marker_detected), fb.distance_to_wall_m * 100))

    def _send_one(self, task_id):
        """goal 하나 보내고 결과까지 대기. 반환 (ok, result) — ok=성공여부."""
        goal = FloorDock.Goal()
        goal.task_id = int(task_id)
        goal.task_point_id = self.point_id
        goal.wall_gap_m = self.wall_gap
        goal.lateral_offset_m = self.lateral
        self._last_phase = None

        fut = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().error('   goal 거절됨')
            return False, None
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        wrapped = res_fut.result()
        return (wrapped is not None and wrapped.result.result_code == 0), \
            (wrapped.result if wrapped is not None else None)

    def run(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 없음: %s (서버 떠 있나?)' % self._action_name)
            return
        self.get_logger().info(
            '바닥 H 도킹 반복 시작: count=%d wall_gap=%s pause=%.1fs → %s'
            % (self.count, ('%.3f' % self.wall_gap) if self.wall_gap else 'default',
               self.pause_s, self._action_name))

        gaps, ok_n = [], 0
        for i in range(1, self.count + 1):
            # 마지막(=단발 포함) 회차는 후퇴 없이 도킹 상태로 종료 → 서버 post_advance=0
            adv = self.post_advance if i < self.count else 0.0
            self._set_server_advance(adv)
            self.get_logger().info('── [%d/%d] 도킹 goal 전송 (post_advance=%.2fm) ──'
                                   % (i, self.count, adv))
            ok, res = self._send_one(i)
            if res is not None:
                gap_mm = res.final_wall_gap_m * 1000.0
                yaw_deg = res.final_yaw_error * 57.2958
                if ok:
                    ok_n += 1
                    gaps.append(gap_mm)
                self.get_logger().info(
                    '   결과: rc=%d gap=%.0fmm yaw=%.1f° | %s'
                    % (res.result_code, gap_mm, yaw_deg, res.message))
            if not ok and self.stop_on_fail:
                self.get_logger().error('실패 → 즉시 중단 (%d/%d 완료)' % (ok_n, i))
                break
            if i < self.count:
                time.sleep(self.pause_s)

        # 요약
        self.get_logger().info('══ 반복 종료: 성공 %d/%d ══' % (ok_n, self.count))
        if gaps:
            lo, hi = min(gaps), max(gaps)
            avg = sum(gaps) / len(gaps)
            self.get_logger().info(
                '   벽갭 [mm]: min=%.0f max=%.0f avg=%.0f 편차폭=%.0f  값=%s'
                % (lo, hi, avg, hi - lo, ['%.0f' % g for g in gaps]))


def main(args=None):
    rclpy.init(args=args)
    node = FloorDockRepeatClient()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('Ctrl+C — 중단')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

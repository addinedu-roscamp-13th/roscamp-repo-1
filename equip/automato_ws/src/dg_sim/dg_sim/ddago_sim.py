#!/usr/bin/env python3
"""DdaGo(주행 로봇) Control Service 시뮬레이터.

팀원이 개발 중인 실제 DdaGo Control Service 대역. 즉시-응답 스텁.

담당(시퀀스 다이어그램, 2026-07-14 개정):
  E0    DdagoTelemetry 1Hz 발행                  /ddago/telemetry
  E1/E2 Navigate 액션 서버 (DCS ← )              /ddago/navigate
        - goal(Waypoint[] 경로) 접수 → waypoint 마다 feedback(current_waypoint_id,
          waypoint_index) → 배열 끝까지 주행 → result(result_code=0, last_waypoint_id)
        - **capture==true 노드에서만** RGB 촬영 흉내 → DCS 로 AnalyzeFrame 분석요청 (E2 3단계)
        - 취소(cancel) 요청 시 그 자리에서 중단 → result_code=2, 도달한 마지막 노드 반환
  E2    AnalyzeFrame 서비스 클라이언트 (→ DCS)    /dg/analyze_frame
"""
import os
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger

from automato_interfaces.action import Navigate
from automato_interfaces.msg import DdagoTelemetry
from automato_interfaces.srv import AnalyzeFrame
from sensor_msgs.msg import Image


class DdagoSim(Node):
    def __init__(self, **kwargs):
        super().__init__('ddago_sim', **kwargs)
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('move_delay', 3.0)   # waypoint 이동 처리 시간 시뮬(초)
        self.declare_parameter('auto_telemetry', False)  # 상시 텔레메트리 발행(기본 off)
        self.declare_parameter('burst_sec', 8.0)         # 트리거 시 발행 지속(초)
        # AnalyzeFrame 으로 보낼 RGB 프레임. waypoint별로 image_dir 안의 이미지를 순서대로 사용.
        self.declare_parameter(
            'image_dir',
            '/home/ane/dev_ws/test_data/sample_frames')
        # image_dir 이 비었을 때 쓰는 단일 폴백 이미지
        self.declare_parameter(
            'image_path',
            '/home/ane/dev_ws/test_data/sample_frame.jpg')
        self.declare_parameter('image_max_width', 256)   # 원본 축소 최대 폭(px)
        self.robot_id = self.get_parameter('robot_id').value
        self.move_delay = float(self.get_parameter('move_delay').value)
        self.burst_sec = float(self.get_parameter('burst_sec').value)
        self._cb = ReentrantCallbackGroup()
        self._frames = self._load_frames()   # [(name, Image), ...] waypoint별 프레임

        # 현재 위치/상태(텔레메트리용)
        self._task_id = 0
        self._x, self._y, self._yaw = 0.0, 0.0, 0.0
        # 텔레메트리는 실행(트리거) 시에만 발행. auto_telemetry=true면 상시.
        self._tel_until = float('inf') if self.get_parameter('auto_telemetry').value else 0.0

        self._tel_pub = self.create_publisher(
            DdagoTelemetry, '/ddago/telemetry', 10)   # 연동에 robot_id 미사용
        self.create_timer(1.0, self._tick, callback_group=self._cb)
        self.create_service(Trigger, '/ddago_sim/start_telemetry', self._on_start_tel,
                            callback_group=self._cb)
        self.create_service(Trigger, '/ddago_sim/stop_telemetry', self._on_stop_tel,
                            callback_group=self._cb)

        self._navigate_srv = ActionServer(
            self, Navigate, '/ddago/navigate',   # 연동에 robot_id 미사용
            execute_callback=self._execute,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=self._cb)

        self._analyze_cli = self.create_client(
            AnalyzeFrame, '/dg/analyze_frame', callback_group=self._cb)

        self.get_logger().info('DdaGo 시뮬 시작: /ddago/{telemetry,navigate}')

    # ---- E0 텔레메트리 (실행 트리거 시에만) ----
    def _on_start_tel(self, request, response):
        self._tel_until = float('inf')   # 중지 전까지 상시 발행
        self.get_logger().info('DdaGo 텔레메트리 발행 시작(상시)')
        response.success = True
        response.message = 'ddago telemetry started'
        return response

    def _on_stop_tel(self, request, response):
        self._tel_until = 0.0
        self.get_logger().info('DdaGo 텔레메트리 발행 중지')
        response.success = True
        response.message = 'ddago telemetry stopped'
        return response

    def _tick(self):
        if time.time() > self._tel_until:
            return   # 실행 트리거 전/후에는 발행 안 함
        msg = DdagoTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = self._task_id
        msg.nav_status = 'NAVIGATING' if self._task_id else 'IDLE'
        msg.is_charging = False
        msg.x, msg.y, msg.yaw = self._x, self._y, self._yaw
        msg.battery_percent = 85.0
        msg.battery_voltage = 12.1
        msg.us_range_m = 0.42
        self._tel_pub.publish(msg)

    # ---- E1/E2 Navigate 액션 서버 (경로 배열) ----
    def _execute(self, goal_handle):
        goal = goal_handle.request
        wps = list(goal.waypoints)
        self._task_id = goal.task_id
        self.get_logger().info('DdaGo 구간 주행 시작: task=%d waypoints=%s'
                               % (goal.task_id, [w.waypoint_id for w in wps]))

        last_wp = wps[0].waypoint_id if wps else -1
        code = 0
        for idx, wp in enumerate(wps):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('취소 요청 → 구간 중단 (last_wp=%d)' % last_wp)
                code = 2
                break

            # 이동 흉내: move_delay 동안 feedback 여러 번 발행 후 도착(처리 시간 시뮬)
            steps = 3
            per = self.move_delay / steps if self.move_delay > 0 else 0.0
            for i in range(steps):
                fb = Navigate.Feedback()
                fb.current_waypoint_id = wp.waypoint_id
                fb.waypoint_index = idx
                fb.current_x = wp.x * (i + 1) / steps
                fb.current_y = wp.y * (i + 1) / steps
                fb.current_yaw = 0.0
                goal_handle.publish_feedback(fb)
                if per:
                    time.sleep(per)

            # 도착
            self._x, self._y, self._yaw = wp.x, wp.y, 0.0
            last_wp = wp.waypoint_id

            # capture==true 노드에서만 촬영·분석요청(E2 3단계). 나머지는 통과만 한다.
            # 분석요청은 비동기라 이동을 막지 않는다(fire-and-forget).
            if wp.capture:
                threading.Thread(target=self._request_analyze,
                                 args=(goal.task_id, wp.waypoint_id), daemon=True).start()
            else:
                self.get_logger().info('wp=%d 통과(capture=false)' % wp.waypoint_id)

        result = Navigate.Result()
        result.result_code = code
        result.last_waypoint_id = int(last_wp)
        if code == 2:
            result.message = '중단'
            goal_handle.canceled()
        else:
            result.message = '구간 완주'
            goal_handle.succeed()
        self.get_logger().info('DdaGo 구간 종료: task=%d code=%d last_wp=%d'
                               % (goal.task_id, code, last_wp))
        return result

    # ---- E2 분석 요청 (→ DCS) ----
    def _request_analyze(self, task_id, waypoint_id):
        if not self._analyze_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('DCS AnalyzeFrame 서비스 없음')
            return
        req = AnalyzeFrame.Request()
        req.task_id = int(task_id)
        req.waypoint_id = int(waypoint_id)
        if self._frames:
            name, img = self._frames[waypoint_id % len(self._frames)]   # waypoint별 이미지
            req.image = img
        else:
            name, req.image = 'dummy', self._dummy_image()
        self.get_logger().info('DCS로 분석요청: task=%d wp=%d img=%s (%dx%d)'
                               % (task_id, waypoint_id, name, req.image.width, req.image.height))
        fut = self._analyze_cli.call_async(req)
        fut.add_done_callback(self._on_analyze_ack)

    def _on_analyze_ack(self, future):
        try:
            resp = future.result()
            self.get_logger().info('분석요청 ACK: accepted=%s request_id=%s'
                                   % (resp.accepted, resp.request_id))
        except Exception as e:   # noqa: BLE001
            self.get_logger().error('분석요청 실패: %s' % e)

    def _load_one(self, path):
        """이미지 1장을 sensor_msgs/Image(rgb8)로 로드. header.frame_id=파일명. 실패 시 None."""
        try:
            from PIL import Image as PILImage
            im = PILImage.open(path).convert('RGB')
            im.thumbnail((int(self.get_parameter('image_max_width').value), 100000))
            w, h = im.size
            img = Image()
            img.header.stamp = self.get_clock().now().to_msg()
            img.header.frame_id = os.path.basename(path)   # 어떤 이미지인지 식별용
            img.height = h
            img.width = w
            img.encoding = 'rgb8'
            img.is_bigendian = 0
            img.step = w * 3
            img.data = list(im.tobytes())
            return img
        except Exception as e:   # noqa: BLE001 — 로드 실패해도 더미로 계속
            self.get_logger().warn('이미지 로드 실패(%s): %s' % (path, e))
            return None

    def _load_frames(self):
        """image_dir 안의 이미지들을 정렬해 [(파일명, Image), ...] 로 로드(waypoint별 사용).
        폴더가 없거나 비면 image_path 단일 이미지로 폴백. 둘 다 없으면 빈 리스트(→더미)."""
        frames = []
        d = self.get_parameter('image_dir').value
        if d and os.path.isdir(d):
            for fn in sorted(os.listdir(d)):
                if fn.lower().endswith(('.jpg', '.jpeg', '.png')):
                    img = self._load_one(os.path.join(d, fn))
                    if img is not None:
                        frames.append((fn, img))
        if not frames:
            path = self.get_parameter('image_path').value
            if path and os.path.isfile(path):
                img = self._load_one(path)
                if img is not None:
                    frames.append((os.path.basename(path), img))
        if frames:
            self.get_logger().info('waypoint 이미지 %d장 로드: %s'
                                   % (len(frames), ', '.join(n for n, _ in frames)))
        else:
            self.get_logger().warn('이미지 없음 → 더미 2x2 사용')
        return frames

    @staticmethod
    def _dummy_image():
        img = Image()
        img.height = 2
        img.width = 2
        img.encoding = 'rgb8'
        img.is_bigendian = 0
        img.step = 6
        img.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        return img


def main(args=None):
    rclpy.init(args=args)
    node = DdagoSim()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

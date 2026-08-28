#!/usr/bin/env python3
"""DdaGo(주행 로봇) Control Service 시뮬레이터.

팀원이 개발 중인 실제 DdaGo Control Service 대역. 즉시-응답 스텁.

담당(시퀀스 다이어그램, 2026-07-14 개정):
  E0    DdagoTelemetry 1Hz 발행                  /ddago/telemetry
        - nav_status 는 주행 중에만 NAVIGATING, 끝나면 IDLE 로 복귀(실물과 같은 규약).
          task_id 는 반대로 goal 이 끝나도 유지한다. 자세한 이유는 __init__ 주석 참고.
  E1/E2 Navigate 액션 서버 (DCS ← )              /ddago/navigate
        - goal(Waypoint[] 경로) 접수 → waypoint 마다 feedback(current_waypoint_id,
          waypoint_index) → 배열 끝까지 주행 → result(result_code=0, last_waypoint_id)
        - **capture==true 노드에서만** RGB 촬영 흉내 → DCS 로 AnalyzeFrame 분석요청 (E2 3단계)
        - 취소(cancel) 요청 시 그 자리에서 중단 → result_code=2, 도달한 마지막 노드 반환
  E2    AnalyzeFrame 서비스 클라이언트 (→ DCS)    /dg/analyze_frame
  E4-6  정밀 도킹 액션 서버 3종 (DCS ← )         /ddago/{dock,floor_dock,reflective_dock}
        - 실제 도킹 기동(마커 탐색→정렬→접근→회전→후진)은 하지 않는다. 방식별 phase 를
          순서대로 흘려보내 **DCS 의 중계**(feedback·result·cancel)를 검증한다.
        - 취소 요청 시 그 자리에서 CANCELED + result_code=3
        - dock_mode 로 실패를 주입한다: success(0) / no_marker(1) / error_exceeded(2) /
          hang(무응답 → DCS timeout) / empty_abort(ABORTED 인데 결과는 전 필드 0 —
          로봇 rclpy 7.1.9 재현, DCS 가 status 로 걸러내는지 검증).
          테스트가 self.dock_mode 를 바꿔 5종을 검증한다.
          모드는 3종이 공유한다(같은 로봇의 같은 도킹 실패 양상이라 방식별로 나눌 게 없다).
        - 동시에 실행된 도킹 goal 수를 dock_concurrent_max 에 기록한다 — DCS 가 방식이
          다른 도킹까지 하나의 락으로 직렬화하는지 테스트가 이 값으로 확인한다.
"""
import collections
import functools
import os
import threading
import time

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger

from automato_interfaces.action import Dock, FloorDock, Navigate, ReflectiveDock
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
        # 도킹 결과 시뮬 모드. DCS 중계가 성공/실패/무응답/취소를 모두 그대로 올리는지
        # 검증하기 위한 스위치. 테스트는 인스턴스 속성 self.dock_mode 를 직접 바꿔 케이스별로
        # 재사용한다(하나의 sim 노드로 4종 검증). 값: success/no_marker/error_exceeded/hang
        self.declare_parameter('dock_mode', 'success')
        self.robot_id = self.get_parameter('robot_id').value
        self.move_delay = float(self.get_parameter('move_delay').value)
        self.burst_sec = float(self.get_parameter('burst_sec').value)
        self.dock_mode = self.get_parameter('dock_mode').value
        # 라이브에서 `ros2 param set /ddago_sim dock_mode no_marker` 로 실패를 주입할 수 있게
        # 파라미터 변경을 self.dock_mode 에 반영한다(테스트는 속성을 직접 바꾼다).
        self.add_on_set_parameters_callback(self._on_set_params)
        self._cb = ReentrantCallbackGroup()
        self._frames = self._load_frames()   # [(name, Image), ...] waypoint별 프레임

        # 현재 위치/상태(텔레메트리용)
        # task_id 와 nav_status 는 **별개로** 관리한다(실물 ddago_control 과 같은 규약).
        #   task_id    : goal 을 받을 때 갱신하고 goal 이 끝나도 0 으로 되돌리지 않는다(latched).
        #                ACS 가 한 task 를 여러 구간 goal 로 쪼개 하달하므로 구간 사이마다 0 이
        #                되면 QT 화면이 깜빡이고 복귀 주행 추적이 끊긴다.
        #   nav_status : "지금 움직이는 중인가"만 답하는 일시 상태 → 주행이 끝나면 IDLE 로
        #                돌아와야 한다. DCS 가용 판정이 nav_status=='IDLE' 을 AND 조건으로
        #                요구하므로(telemetry_ws/patrol_api), 여기서 안 돌리면 로봇이 영영
        #                ROBOT_BUSY 로 남아 두 번째 작업을 못 받는다.
        self._task_id = 0
        self._nav_status = 'IDLE'
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

        # E4 도킹 서버 3종. 실기동 대신 방식별 phase 를 순서대로 흘려 DCS 중계를 검증한다.
        self._dock_srv = {}
        for spec in self.DOCK_KINDS:
            self._dock_srv[spec.name] = ActionServer(
                self, spec.action, '/ddago/%s' % spec.name,   # 연동에 robot_id 미사용
                execute_callback=functools.partial(self._dock_execute, spec),
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=self._cb)
        # 동시에 실행 중인 도킹 goal 수(직렬화 확인용). DCS 가 방식이 다른 도킹까지
        # _ddago_lock 하나로 묶으므로 정상이면 최대 1 이다.
        self._dock_active = 0
        self._dock_active_lock = threading.Lock()
        self.dock_concurrent_max = 0

        self._analyze_cli = self.create_client(
            AnalyzeFrame, '/dg/analyze_frame', callback_group=self._cb)

        self.get_logger().info(
            'DdaGo 시뮬 시작: /ddago/{telemetry,navigate,%s} (dock_mode=%s)'
            % (','.join(s.name for s in self.DOCK_KINDS), self.dock_mode))

    VALID_DOCK_MODES = ('success', 'no_marker', 'error_exceeded', 'hang', 'empty_abort')

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'dock_mode':
                if p.value not in self.VALID_DOCK_MODES:
                    return SetParametersResult(
                        successful=False,
                        reason='dock_mode 는 %s 중 하나' % ', '.join(self.VALID_DOCK_MODES))
                self.dock_mode = p.value
                self.get_logger().info('dock_mode 변경 → %s' % p.value)
        return SetParametersResult(successful=True)

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
        msg.nav_status = self._nav_status
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
        self._nav_status = 'NAVIGATING'
        self.get_logger().info('DdaGo 구간 주행 시작: task=%d waypoints=%s'
                               % (goal.task_id, [w.waypoint_id for w in wps]))

        last_wp = wps[0].waypoint_id if wps else -1
        code = 0
        try:
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
        finally:
            # 완주·취소·예외 어느 경로로 끝나든 '움직이는 중' 표시를 내린다. task_id 는
            # 유지(다음 구간 goal 이 같은 task 로 이어진다).
            self._nav_status = 'IDLE'

    # ---- E2 분석 요청 (→ DCS) ----
    # ---- E4-6 정밀 도킹 3종 (DCS ← ) : 실기동 대신 phase 만 흘려 중계를 검증 ----
    # 실제 도킹은 ddago_control 의 dock_server/floor_dock_server/reflective_dock_server 가
    # 카메라·라이다·odom 으로 수행한다. 여기서는 DCS 가 goal 을 그대로 넘기는지,
    # feedback/result 를 손실 없이 되돌리는지, 취소가 끝까지 전파되는지만 본다.
    #
    # phase 표: (phase 이름, 거리 [m], 마커가 보이는가). 값 자체에 의미는 없고 실물이
    # 흘리는 순서를 흉내 낸다. 방식마다 이름 집합이 다르다는 것이 중계 검증의 핵심이다
    # (DCS 가 phase 를 해석하지 않고 문자열 그대로 올려야 한다).
    DOCK_PHASES = [
        ('SEARCHING', 0.50, True),
        ('CENTERING', 0.40, True),
        ('APPROACHING', 0.30, True),
        ('STAGED', 0.24, True),
        ('ROTATING', 0.24, True),
        ('REVERSING', 0.00, False),   # 180도 돈 뒤라 카메라가 보드를 못 본다
    ]
    FLOOR_DOCK_PHASES = [
        ('SEARCH', 0.60, True),
        ('CENTERLINE', 0.50, True),
        ('ALIGN', 0.40, True),
        ('FACE', 0.35, True),
        ('STAGED', 0.30, True),
        ('TURN', 0.30, False),        # 180도 회전 중 — 바닥 H 가 화면에서 벗어난다
        ('REVERSE', 0.00, False),     # 후진 접붙임(뒤는 카메라가 못 본다)
        ('DONE', 0.00, False),
    ]
    REFLECTIVE_DOCK_PHASES = [
        ('SNAP', 0.90, True),         # 멈춘 채 마커 법선 스냅
        ('TURN1', 0.90, True),
        ('DRIVE', 0.60, True),
        ('TURN2', 0.45, True),
        ('APPROACH', 0.20, True),     # 라이다라 가까워져도 계속 보인다
        ('CREEP', 0.05, False),       # 마지막 블라인드 후진
    ]

    # 도킹 방식 3종의 시뮬 사양. 액션 타입·이름·phase 표에 더해, 방식마다 이름과 뜻이
    # 다른 앞뒤 오차 필드(gap_field/gap_ok)와 feedback 거리 필드(dist_field)를 적어 둔다.
    #   charuco    final_error_m     목표 대비 최종 위치 오차
    #   floor      final_wall_gap_m  후면~벽 간격(실측 20~30mm)
    #   reflective final_gap_m       뒤끝~마커 간격
    DockKind = collections.namedtuple(
        'DockKind', 'action name phases gap_field gap_ok dist_field')
    DOCK_KINDS = (
        DockKind(Dock, 'dock', DOCK_PHASES,
                 'final_error_m', 0.012, 'distance_to_marker_m'),
        DockKind(FloorDock, 'floor_dock', FLOOR_DOCK_PHASES,
                 'final_wall_gap_m', 0.025, 'distance_to_wall_m'),
        DockKind(ReflectiveDock, 'reflective_dock', REFLECTIVE_DOCK_PHASES,
                 'final_gap_m', 0.020, 'distance_to_marker_m'),
    )

    def _dock_execute(self, spec, goal_handle):
        req = goal_handle.request
        mode = self.dock_mode      # 실행 시점의 모드(테스트가 케이스마다 바꾼다)
        with self._dock_active_lock:
            self._dock_active += 1
            self.dock_concurrent_max = max(self.dock_concurrent_max, self._dock_active)
        try:
            return self._dock_run(spec, goal_handle, req, mode)
        finally:
            with self._dock_active_lock:
                self._dock_active -= 1

    def _dock_run(self, spec, goal_handle, req, mode):
        # 실물 dock_server 도 도킹 goal 을 받으면 /ddago/current_task 로 task_id 를 알린다
        # → 텔레메트리의 task_id 를 같이 맞춘다. 반면 nav_status 는 건드리지 않는다:
        # 도킹 기동은 Nav2 goal 이 아니라 dock_server 가 cmd_vel 을 직접 내는 것이라
        # 실물도 도킹 중 nav_status 는 IDLE 이다(가용 판정은 DB 의 활성 task 가 막는다).
        self._task_id = req.task_id
        # goal 필드는 방식마다 다르다(charuco 만 마커 규격을 싣고, floor/reflective 는
        # 마커리스라 목표 정차값뿐이다). 공통분(task/point) 밖은 있는 것만 그대로 찍는다.
        extra = ' '.join(
            '%s=%s' % (f, getattr(req, f))
            for f in req.get_fields_and_field_types()
            if f not in ('task_id', 'task_point_id'))
        self.get_logger().info(
            '도킹 goal 수신: 방식=%s task=%d point=%s %s mode=%s'
            % (spec.name, req.task_id, req.task_point_id, extra, mode))

        # empty_abort: **abort 인데 결과는 빈 Result(전 필드 0)** 로 돌아오는 상황.
        #  실기에서 실제로 일어난다 — 로봇의 rclpy 7.1.9 는 인자 없는 abort() 에서 빈 Result 를
        #  먼저 보내고 콜백의 return 값을 버린다(노트북 7.1.11 은 가드가 있어 정상).
        #  result_code 는 0 이 성공이라 **빈 결과 = 성공** 으로 읽히는 fail-open 이 된다.
        #  08-05 에 이것 때문에 도킹 실패인데 E3 게이트가 열려 팔이 움직였다.
        #  로봇 쪽을 고쳐도 통신 이상·서버 예외에서 같은 모양이 나오므로, **DCS 가 status 로
        #  걸러내는지**를 여기서 상시 검증한다(노트북 rclpy 로는 이 모드 없이는 재현 불가).
        if mode == 'empty_abort':
            self.get_logger().warn('빈 결과 abort 시뮬 → status=ABORTED, result 는 전 필드 0')
            r = spec.action.Result()          # 전부 0 — result_code=0(=성공값)
            goal_handle.abort(r)
            return r

        # hang: 결과를 돌려주지 않는다 → DCS 의 dock_result_timeout 검증용.
        # 취소/종료 시 빠져나오도록 유계 루프로 대기(테스트 teardown 안전).
        if mode == 'hang':
            self.get_logger().warn('도킹 무응답 시뮬(hang) — 결과 반환 지연')
            waited = 0.0
            while rclpy.ok() and not goal_handle.is_cancel_requested and waited < 4.0:
                time.sleep(0.1)
                waited += 0.1
            r = spec.action.Result()
            r.result_code = 3
            r.message = 'hang 종료'
            # 종료(shutdown) 중이면 서버가 이미 내려가 상태 전이가 예외를 던진다 → 건드리지 않는다.
            if not rclpy.ok():
                return r
            try:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    r.message = 'hang 취소'
                else:
                    goal_handle.abort()
            except Exception:   # noqa: BLE001 — teardown 경합 시 무시
                pass
            return r

        per = self.move_delay / len(spec.phases) if self.move_delay > 0 else 0.0
        for phase, dist, seen in spec.phases:
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('취소 요청 → 도킹 중단 (phase=%s)' % phase)
                goal_handle.canceled()
                r = spec.action.Result()
                r.result_code = 3            # 3: 중단
                r.message = '취소로 중단 (phase=%s)' % phase
                return r
            fb = spec.action.Feedback()
            fb.phase = phase
            fb.marker_detected = seen
            setattr(fb, spec.dist_field, float(dist))   # 거리 필드 이름이 방식마다 다르다
            goal_handle.publish_feedback(fb)
            if per > 0:
                time.sleep(per)
            # no_marker: 첫 단계(탐색/스냅)에서 마커를 못 찾고 실패(code 1)
            if mode == 'no_marker' and phase == spec.phases[0][0]:
                self.get_logger().warn('마커 미검출 시뮬 → code=1')
                goal_handle.abort()
                r = spec.action.Result()
                r.result_code = 1
                r.message = '마커 미검출(시뮬)'
                return r

        # error_exceeded: 기동은 끝났지만 정차 오차가 기준을 초과(code 2)
        if mode == 'error_exceeded':
            self.get_logger().warn('정차 오차 초과 시뮬 → code=2')
            goal_handle.abort()
            r = spec.action.Result()
            r.result_code = 2
            r.final_lateral_m = 0.085     # 목표(예: 2cm) 크게 초과
            r.final_yaw_error = 0.20
            setattr(r, spec.gap_field, spec.gap_ok)
            r.message = '정차 오차 초과(시뮬)'
            return r

        r = spec.action.Result()
        r.result_code = 0
        # 실장비 실측(ddago03)과 같은 자릿수의 값을 돌려 ACS 쪽 표시를 확인할 수 있게 한다.
        r.final_lateral_m = -0.012        # 중심선/법선 이탈(좌우)
        r.final_yaw_error = 0.021         # 스큐
        setattr(r, spec.gap_field, spec.gap_ok)
        r.message = '도킹 완료(시뮬)'
        goal_handle.succeed()
        self.get_logger().info('도킹 완료(시뮬): 방식=%s task=%d lateral=%.3fm yaw=%.3frad'
                               % (spec.name, req.task_id, r.final_lateral_m, r.final_yaw_error))
        return r

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

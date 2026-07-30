#!/usr/bin/env python3
"""Automato Control Service 시뮬레이터.

팀원이 개발 중인 실제 Automato Control Service(ACS) 대역. 즉시-응답 스텁.

담당(시퀀스 다이어그램, 2026-07-14 개정):
  E0  RobotTelemetry 구독 (DCS ← )               /{robot_id}/telemetry       (로그로 확인)
  E1  Navigate 액션 클라이언트 (→ DCS)           /{robot_id}/navigate
        - **예약 확보된 구간까지만** Waypoint[] 로 하달(루프 주체=ACS)
        - 구간 result(last_waypoint_id) 받으면 다음 구간 하달 (E2 4단계)
        - 순찰 지점(capture=true)과 통과 노드(capture=false)를 섞어서 하달
  E2  SaveDetection 서비스 서버 (DCS ← )         /automato/save_detection    (즉시 success)
        - disease_image 가 있으면(=AI 가 disease>=5 로 판단) 파일로 저장 (저장은 ACS 몫)

실제 ACS 의 통로 예약(try_reserve)·BFS·막힘 판정은 흉내내지 않는다. 여기서는 경로를
seg_size 개씩 끊어 "예약된 구간까지만 하달"하는 형태만 재현한다.

편의:
  - 파라미터 auto_start(기본 true)면 기동 후 auto_delay 초에 순찰 1회 자동 발행.
  - 서비스 /acs_sim/start_patrol (std_srvs/Trigger) 호출로 언제든 순찰 재발행.
"""
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_interfaces.action import Dock, FloorDock, Harvest, Navigate, ReflectiveDock, Unload
from automato_interfaces.msg import RobotTelemetry, Waypoint
from automato_interfaces.srv import SaveDetection
from std_srvs.srv import Trigger

# SaveDetection 으로 받은 병해충 라벨 이미지 저장 폴더 (파일 저장은 ACS 담당).
# 기본값은 홈의 ~/dg_sim_data/acs_recv — dg_ai_sim.SAVE_DIR 과 같은 자리에 모은다.
# 예전 기본값은 사라진 클론 경로였고, 저장 때마다 그 빈 트리가 되살아났다.
ACS_SAVE_DIR = os.environ.get(
    'ACS_SIM_SAVE_DIR',
    os.path.join(os.path.expanduser('~'), 'dg_sim_data', 'acs_recv'))


class AcsSim(Node):
    # 도킹 방식 3종 — (방식, 액션 타입, 액션 이름). ACS docking._ACTION_SPEC 과 같은 표다.
    DOCK_METHODS = (
        ('charuco', Dock, 'dock'),
        ('floor', FloorDock, 'floor_dock'),
        ('reflective', ReflectiveDock, 'reflective_dock'),
    )

    def __init__(self, **kwargs):
        super().__init__('acs_sim', **kwargs)
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('auto_delay', 3.0)
        # auto_start 시 실행할 시나리오:
        #   'patrol'(S1) | 'harvest-move'(S2 E2 수확지 이동+도킹) | 'return-dock'(E4 복귀+충전)
        self.declare_parameter('scenario', 'patrol')
        self.declare_parameter('num_waypoints', 6)   # 경로 전체 노드 수
        self.declare_parameter('seg_size', 3)        # 한 번에 하달할 구간 크기(예약 흉내)
        # S2 E2 수확 이동 파라미터
        self.declare_parameter('harvest_waypoints', 4)
        self.declare_parameter('harvest_seg', 2)
        self.declare_parameter('dock_point', 'HARVEST_01')
        # E4 순찰 종료 후 복귀 파라미터 (복귀 주행 → 충전소 도킹). 지점이 CHARGE_* 이므로
        # dock_method='auto' 가 반사테이프(ReflectiveDock)를 고른다.
        self.declare_parameter('return_waypoints', 4)
        self.declare_parameter('return_seg', 2)
        self.declare_parameter('charger_point', 'CHARGE_01')
        # 도킹 방식: auto | charuco | floor | reflective
        # 기본 'auto' = 실제 ACS(docking.method_for)와 같은 규칙으로 **지점 id 로 고른다** —
        # CHARGE_*→reflective(반사테이프), HARVEST_*/PRECOOL_*→floor(바닥 H 마커).
        # 그래서 S2 E2(수확지)·E5/E6(예냉실) 시나리오는 H마커로, E4 복귀는 반사테이프로 간다.
        # 방식별 중계만 따로 찔러 보고 싶을 때는 방식 이름을 직접 지정한다(테스트가 그렇게 쓴다).
        self.declare_parameter('dock_method', 'auto')
        self.robot_id = self.get_parameter('robot_id').value
        self._cb = ReentrantCallbackGroup()
        self._task_seq = 1024
        self.saved = []   # 수신한 SaveDetection 누적(검증/디버깅용)
        self.fleet_count = 0
        self.last_fleet = None
        self.last_result = None
        # 순찰 루프 상태: 구간(Waypoint[])을 하달하고 result를 받으면 다음 구간 발행
        self._patrol = None        # {'task_id', 'wps', 'seg', 'seg_size'}
        self.patrol_done = False   # 마지막 구간까지 완료 여부(검증용)
        self.last_waypoint_id = -1  # 마지막 구간 result의 last_waypoint_id(검증용)
        self.capture_ids = []      # 이번 순찰의 촬영 지점(capture=true) 목록(검증용)

        # S2 E2 수확 이동 상태: 수확 위치까지 capture=false 로 이동 → 도착 후 도킹 하달
        # (수확지·예냉실은 H마커=FloorDock. 방식은 dock_method='auto' 가 지점 id 로 고른다)
        self._harvest = None            # {'task_id','wps','seg','seg_size','dock_point'}
        self.harvest_move_done = False  # 수확 이동(주행) 완료 여부(검증용)
        self.dock_done = False          # 도킹 result 수신 여부(검증용)
        self.last_dock_result = None    # 마지막 도킹 Result(검증용)
        self.dock_feedback_phases = []  # 도킹 중 받은 phase 목록(중계 확인용)
        self._dock_gh = None            # 진행 중 도킹 goal handle(취소 테스트용)
        # 이번에 쓸 도킹 방식. 테스트는 이 속성을 바꿔 방식별 중계를 검증한다.
        self.dock_method = self.get_parameter('dock_method').value

        # S2 E3~E5 Harvest 상태
        self._last_docked_task = None   # 도킹 성공한 마지막 task(수확 goal 대상)
        self.harvest_accepted = None    # 수확 goal 수락 여부(None=미발행, True/False)
        self.harvest_feedback = []      # 받은 Harvest Feedback 목록(라운드 중계 확인용)
        self.harvest_result = None      # 마지막 Harvest.Result(검증용)
        self.harvest_done = False       # 수확 result 수신 여부
        self._harvest_gh = None         # 진행 중 Harvest goal handle(취소 테스트용)

        # S2 E6 Unload 상태 (예냉실 도킹 성공 후 바구니 하역)
        self.unload_accepted = None     # 하역 goal 수락 여부(None=미발행, True/False)
        self.unload_feedback = []       # 받은 Unload Feedback(phase) 목록(중계 확인용)
        self.unload_result = None       # 마지막 Unload.Result(검증용)
        self.unload_done = False        # 하역 result 수신 여부
        self._unload_gh = None          # 진행 중 Unload goal handle(취소 테스트용)

        # E0 RobotTelemetry 구독 — DG 는 자기 세트분만 /{robot_id}/telemetry 로 보낸다
        self.create_subscription(
            RobotTelemetry, '/%s/telemetry' % self.robot_id,
            self._on_fleet, 10, callback_group=self._cb)

        # E1 Navigate 액션 클라이언트
        self._navigate_cli = ActionClient(
            self, Navigate, '/%s/navigate' % self.robot_id, callback_group=self._cb)

        # 정밀 도킹 액션 클라이언트 3종 (지점 도착 후 하달). 방식마다 액션이 달라
        # 셋 다 미리 만들어 두고 하달 직전에 고른다(실제 ACS 도 방식별 클라이언트를 둔다).
        self._dock_cli = {
            m: ActionClient(self, act, '/%s/%s' % (self.robot_id, name),
                            callback_group=self._cb)
            for m, act, name in self.DOCK_METHODS}

        # S2 E3~E5 Harvest 액션 클라이언트 (도킹 성공 후 수확 시작 하달)
        self._harvest_cli = ActionClient(
            self, Harvest, '/%s/harvest' % self.robot_id, callback_group=self._cb)

        # S2 E6 Unload 액션 클라이언트 (예냉실 도킹 성공 후 하역 하달)
        self._unload_cli = ActionClient(
            self, Unload, '/%s/unload' % self.robot_id, callback_group=self._cb)

        # E2 SaveDetection 서비스 서버
        self.create_service(
            SaveDetection, '/automato/save_detection',
            self._on_save_detection, callback_group=self._cb)

        # 수동 트리거
        self.create_service(
            Trigger, '/acs_sim/start_patrol', self._on_trigger, callback_group=self._cb)
        # S2 E2 수확 이동+도킹 트리거 (이동+도킹은 'harvest-move', 실제 수확은 'harvest')
        self.create_service(
            Trigger, '/acs_sim/start_harvest_move', self._on_trigger_harvest_move,
            callback_group=self._cb)
        # S2 E3 수확 시작 트리거 (도킹 성공한 마지막 task 로 Harvest 하달)
        self.create_service(
            Trigger, '/acs_sim/start_harvest', self._on_trigger_harvest, callback_group=self._cb)
        # S2 E6 하역 시작 트리거 (예냉실 도킹 성공한 마지막 task 로 Unload 하달)
        self.create_service(
            Trigger, '/acs_sim/start_unload', self._on_trigger_unload, callback_group=self._cb)
        # E4 순찰 종료 후 복귀 트리거 (복귀 주행 → 충전소 도킹, 2단계)
        self.create_service(
            Trigger, '/acs_sim/start_return', self._on_trigger_return, callback_group=self._cb)

        self.get_logger().info(
            'ACS 시뮬 시작: Navigate클라·도킹클라 /%s/{navigate,%s}, SaveDetection서버, '
            'Telemetry구독 | scenario=%s dock_method=%s'
            % (self.robot_id, ','.join(n for _m, _a, n in self.DOCK_METHODS),
               self.get_parameter('scenario').value, self.dock_method))

        if self.get_parameter('auto_start').value:
            delay = float(self.get_parameter('auto_delay').value)
            self.create_timer(delay, self._auto_start_once, callback_group=self._cb)

    # ---- E0 ----
    def _on_fleet(self, msg):
        self.fleet_count += 1
        self.last_fleet = msg
        self.get_logger().info('Fleet 수신: ddago=%d대 ddagi=%d대'
                               % (len(msg.ddagos), len(msg.ddagis)))

    # ---- E2 SaveDetection 서버 ----
    def _on_save_detection(self, request, response):
        has_image = request.disease_image.height > 0 and request.disease_image.width > 0
        self.saved.append({
            'task_id': request.task_id, 'waypoint_id': request.waypoint_id,
            'ripe': request.ripe_percent, 'unripe': request.unripe_percent,
            'rotten': request.rotten_percent, 'disease': request.disease_percent,
            'has_image': has_image})
        self.get_logger().info(
            'SaveDetection 저장: task=%d wp=%d ripe=%d unripe=%d rotten=%d disease=%d image=%s'
            % (request.task_id, request.waypoint_id, request.ripe_percent,
               request.unripe_percent, request.rotten_percent, request.disease_percent,
               ('%dx%d' % (request.disease_image.width, request.disease_image.height)) if has_image else '없음'))
        if has_image:
            self._save_image(request)
        response.success = True
        response.message = '저장 완료(sim)'
        return response

    def _save_image(self, request):
        """SaveDetection 으로 받은 sensor_msgs/Image(rgb8) 를 파일로 저장(수신 확인용)."""
        try:
            from PIL import Image as PILImage
            im = PILImage.frombytes('RGB', (request.disease_image.width, request.disease_image.height),
                                    bytes(request.disease_image.data))
            os.makedirs(ACS_SAVE_DIR, exist_ok=True)
            path = os.path.join(ACS_SAVE_DIR, 'save_task%d_wp%d.jpg'
                                % (request.task_id, request.waypoint_id))
            im.save(path)
            self.get_logger().info('SaveDetection 이미지 저장: %s (%dx%d)'
                                   % (path, im.width, im.height))
        except Exception as e:   # noqa: BLE001
            self.get_logger().warn('SaveDetection 이미지 저장 실패: %s' % e)

    # ---- E1 순찰 발행 ----
    def _auto_start_once(self):
        # 타이머는 1회만 쓰기 위해 즉시 취소
        for t in list(self.timers):
            t.cancel()
        scenario = self.get_parameter('scenario').value
        if scenario == 'harvest-move':
            self.send_harvest_move(
                num_waypoints=int(self.get_parameter('harvest_waypoints').value),
                seg_size=int(self.get_parameter('harvest_seg').value),
                dock_point=self.get_parameter('dock_point').value)
        elif scenario == 'return-dock':
            self.send_return_to_charger()
        else:
            self.send_patrol()

    def _on_trigger(self, request, response):
        task_id = self.send_patrol()
        response.success = True
        response.message = '순찰 발행 task_id=%d' % task_id
        return response

    def _on_trigger_harvest_move(self, request, response):
        """S2 E2: 수확 위치 이동(전 구간 capture=false) → 도착 후 도킹까지 한 번에 발행."""
        task_id = self.send_harvest_move(
            num_waypoints=int(self.get_parameter('harvest_waypoints').value),
            seg_size=int(self.get_parameter('harvest_seg').value),
            dock_point=self.get_parameter('dock_point').value)
        if task_id is None:
            response.success = False
            response.message = 'ROBOT_BUSY 또는 서버 없음 — 수확 이동 발행 실패'
        else:
            response.success = True
            response.message = '수확 이동+도킹 발행 task_id=%d' % task_id
        return response

    def _on_trigger_return(self, request, response):
        """E4: 복귀 주행(전 구간 capture=false) → 충전소 도킹까지 2단계로 발행."""
        task_id = self.send_return_to_charger()
        if task_id is None:
            response.success = False
            response.message = 'ROBOT_BUSY 또는 서버 없음 — 복귀 발행 실패'
        else:
            response.success = True
            response.message = '복귀 주행+충전소 도킹 발행 task_id=%d' % task_id
        return response

    def send_return_to_charger(self, num_waypoints=None, seg_size=None, charger_point=None):
        """E4 순찰 종료 후 복귀 및 충전 — 복귀 주행 → 충전소 도킹, 2단계.

        수확지 이동(S2 E2)과 같은 '이동 → 도킹' 흐름이라 같은 구현을 쓴다. 다른 것은
        목적지가 충전소(CHARGE_*)라는 점이고, 그래서 도킹 방식도 자동으로 반사테이프가
        된다(dock_method='auto'). 복귀 주행은 촬영이 목적이 아니므로 전 구간
        capture=false — E2 20번 capture 판정식은 순찰 정상 경로에만 적용된다.
        """
        return self.send_harvest_move(
            num_waypoints=(int(self.get_parameter('return_waypoints').value)
                           if num_waypoints is None else num_waypoints),
            seg_size=(int(self.get_parameter('return_seg').value)
                      if seg_size is None else seg_size),
            dock_point=(self.get_parameter('charger_point').value
                        if charger_point is None else charger_point),
            label='복귀 주행')

    def send_patrol(self, num_waypoints=None, seg_size=None):
        """순찰 경로를 만들어 첫 구간(Waypoint[])을 하달. 이후 구간 result마다 다음 구간을 하달.

        capture 규칙(시뮬): 홀수 waypoint_id = 순찰 지점(capture=true, 촬영·분석),
        짝수 = 통과 노드(capture=false). 실제 ACS 는 예약 결과에 따라 정한다."""
        if self._patrol is not None:
            # 실제 ACS 는 진행 중 task 가 있는 로봇을 배정하지 않는다(unavailable_reason=ROBOT_BUSY).
            self.get_logger().warn('ROBOT_BUSY — 진행 중인 task=%d 있음, 새 순찰 발행 안 함'
                                   % self._patrol['task_id'])
            return self._patrol['task_id']
        if num_waypoints is None:
            num_waypoints = int(self.get_parameter('num_waypoints').value)
        if seg_size is None:
            seg_size = int(self.get_parameter('seg_size').value)
        self._task_seq += 1
        task_id = self._task_seq
        wps = []
        for i in range(num_waypoints):
            wp = Waypoint()
            wp.waypoint_id = i
            wp.x = float(i + 1)          # 첫 waypoint가 (0,0)이 되지 않도록 1부터
            wp.y = float(i + 1) * 0.5
            wp.capture = (i % 2 == 1)    # 순찰 지점에서만 촬영
            wps.append(wp)

        if not self._navigate_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DCS Navigate 서버 없음 — 발행 취소')
            return task_id
        self._patrol = {'task_id': task_id, 'wps': wps, 'seg': 0, 'seg_size': max(1, seg_size)}
        self.patrol_done = False
        self.last_waypoint_id = -1
        self.capture_ids = [w.waypoint_id for w in wps if w.capture]
        self.get_logger().info('순찰 시작: task_id=%d waypoints=%d 구간크기=%d 촬영지점=%s'
                               % (task_id, num_waypoints, seg_size, self.capture_ids))
        self._send_next_segment()
        return task_id

    def _send_next_segment(self):
        """예약 확보된 구간(seg_size 개)만큼 잘라서 하달 (E2 20번 흉내)."""
        p = self._patrol
        if p is None:
            return
        start = p['seg'] * p['seg_size']
        seg = p['wps'][start:start + p['seg_size']]
        if not seg:
            return
        goal = Navigate.Goal(task_id=p['task_id'], waypoints=seg)
        self.get_logger().info('구간 하달: task=%d waypoints=%s (구간 %d)'
                               % (p['task_id'], [w.waypoint_id for w in seg], p['seg'] + 1))
        fut = self._navigate_cli.send_goal_async(goal, feedback_callback=self._on_navigate_fb)
        fut.add_done_callback(self._on_navigate_goal_response)

    def _on_navigate_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('DCS가 Navigate goal 거부')
            self._patrol = None
            return
        gh.get_result_async().add_done_callback(self._on_navigate_result)

    def _on_navigate_fb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info('순찰 진행: wp=%d idx=%d (%.2f,%.2f)'
                               % (fb.current_waypoint_id, fb.waypoint_index,
                                  fb.current_x, fb.current_y))

    def _on_navigate_result(self, future):
        res = future.result().result
        self.last_result = res
        self.last_waypoint_id = res.last_waypoint_id
        p = self._patrol
        if p is None:
            return
        self.get_logger().info('구간 결과: code=%d last_wp=%d msg=%s'
                               % (res.result_code, res.last_waypoint_id, res.message))
        p['seg'] += 1
        remaining = len(p['wps']) - p['seg'] * p['seg_size']
        if res.result_code == 0 and remaining > 0:
            self._send_next_segment()   # 재계획 후 다음 구간 하달 (E2 18~20번)
        else:
            self.patrol_done = True
            self.get_logger().info('순찰 완료: task=%d last_wp=%d code=%d'
                                   % (p['task_id'], res.last_waypoint_id, res.result_code))
            self._patrol = None

    # ---- '이동 → 도킹' 2단계 시나리오 (Navigate 전 구간 capture=false → 도착 후 도킹) ----
    # S2 E2 수확지 이동과 E4 충전소 복귀는 **같은 모양**이다: 전 구간 촬영 없이 주행하고,
    # 도착하면 지점에 맞는 도킹을 하달한다. 다른 것은 목적지(지점 id)와 로그 이름뿐이라
    # 한 벌로 처리하고 label 로만 구분한다(도킹 방식은 dock_method='auto' 가 지점에서 고른다).
    def send_harvest_move(self, num_waypoints=4, seg_size=2, dock_point='HARVEST_01',
                          label='수확 이동'):
        """지점까지 이동한 뒤 도킹까지 이어서 하달한다(S2 E2 수확지 / E4 충전소 복귀 공용).

        순찰과 다른 점은 두 가지다: (1) 전 구간 capture=false — 이동 중 촬영·분석이 없다,
        (2) 마지막 구간 도착 후 도킹을 하달한다. 실제 ACS 는 task_points 에서 목적지 좌표를,
        지점 종류에서 도킹 방식을 얻는다. 여기서는 좌표만 고정값으로 흉내낸다.

        ⚠️ 실제 E4 복귀는 **새 task 를 만들지 않고** 끝난 순찰의 task_id 를 그대로 재사용한다
        (E4 원칙). 시뮬은 단독 실행을 전제로 매번 새 task_id 를 쓴다 — 중계 검증에는 영향이
        없지만, 로그의 task_id 를 실제 흐름과 1:1로 읽으면 안 된다."""
        if self._harvest is not None or self._patrol is not None:
            self.get_logger().warn('ROBOT_BUSY — 진행 중 task 있음, %s 발행 안 함' % label)
            return None
        if not self._navigate_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DCS Navigate 서버 없음 — 발행 취소')
            return None
        self._task_seq += 1
        task_id = self._task_seq
        wps = []
        for i in range(num_waypoints):
            wp = Waypoint()
            wp.waypoint_id = i
            wp.x = float(i + 1)
            wp.y = float(i + 1) * 0.5
            wp.capture = False        # 이동 구간은 전 구간 촬영 없음
            wps.append(wp)
        self._harvest = {'task_id': task_id, 'wps': wps, 'seg': 0,
                         'seg_size': max(1, seg_size), 'dock_point': dock_point,
                         'label': label}
        self.harvest_move_done = False
        self.dock_done = False
        self.last_dock_result = None
        self.dock_feedback_phases = []
        self._dock_gh = None
        self.get_logger().info('%s 시작: task=%d 목적지=%s waypoints=%d (전 구간 capture=false)'
                               % (label, task_id, dock_point, num_waypoints))
        self._send_next_harvest_segment()
        return task_id

    def _send_next_harvest_segment(self):
        h = self._harvest
        if h is None:
            return
        start = h['seg'] * h['seg_size']
        seg = h['wps'][start:start + h['seg_size']]
        if not seg:
            return
        goal = Navigate.Goal(task_id=h['task_id'], waypoints=seg)
        self.get_logger().info('%s 구간 하달: task=%d waypoints=%s'
                               % (h['label'], h['task_id'], [w.waypoint_id for w in seg]))
        fut = self._navigate_cli.send_goal_async(goal, feedback_callback=self._on_navigate_fb)
        fut.add_done_callback(self._on_harvest_nav_goal_response)

    def _on_harvest_nav_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('DCS가 이동 Navigate goal 거부')
            self._harvest = None
            return
        gh.get_result_async().add_done_callback(self._on_harvest_nav_result)

    def _on_harvest_nav_result(self, future):
        res = future.result().result
        self.last_result = res
        self.last_waypoint_id = res.last_waypoint_id
        h = self._harvest
        if h is None:
            return
        self.get_logger().info('%s 구간 결과: code=%d last_wp=%d'
                               % (h['label'], res.result_code, res.last_waypoint_id))
        if res.result_code != 0:
            # 이동 실패/중단 → 도킹으로 넘어가지 않는다(실제 ACS 는 실패 처리·충전소 복귀).
            self.harvest_move_done = True
            self._harvest = None
            return
        h['seg'] += 1
        remaining = len(h['wps']) - h['seg'] * h['seg_size']
        if remaining > 0:
            self._send_next_harvest_segment()
        else:
            # 목적지 도착 → 도킹 하달 (S2 E2 6~7 / E4 6). 주행 성공 이후에만 도킹한다.
            self.harvest_move_done = True
            self._send_dock(h['task_id'], h['dock_point'])

    @staticmethod
    def method_for(task_point_id):
        """지점 id -> 도킹 방식. 실제 ACS(docking.method_for)와 같은 규칙.

        charuco 폴백은 두지 않는다 — 실제 ACS 도 방식이 안 잡히는 지점을 charuco 로
        보내지 않고 명시적으로 막는다. 여기서는 수확지·예냉실 외 지점이 들어올 일이
        없으므로 나머지를 floor 로 본다."""
        tp = str(task_point_id).upper()
        if tp.startswith('CHARGE'):
            return 'reflective'      # 충전소 = 반사테이프
        return 'floor'               # 수확지(HARVEST_*)·예냉실(PRECOOL_*) = 바닥 H 마커

    def _send_dock(self, task_id, task_point_id, method=None):
        """지점에 맞는 도킹 액션을 골라 하달한다.

        방식마다 Goal 이 다르다 — charuco 만 마커 규격을 싣고, floor/reflective 는
        마커리스라 목표 정차값만 싣는다. **정차값 0 은 '로봇 노드 기본값을 쓴다'는
        뜻**이라 시뮬도 0 으로 보내 그 경로를 그대로 태운다.
        """
        method = method or self.dock_method
        if method == 'auto':
            method = self.method_for(task_point_id)
        cli = self._dock_cli.get(method)
        if cli is None:
            self.get_logger().error('알 수 없는 도킹 방식: %s' % method)
            self._harvest = None
            return
        if not cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DCS %s 도킹 서버 없음 — 도킹 취소' % method)
            self._harvest = None
            return
        if method == 'charuco':
            goal = Dock.Goal()
            goal.task_id = task_id
            goal.task_point_id = task_point_id
            # 마커(ChArUco 보드) 정보 — 실제 ACS 는 DB(charuco_boards)에서 조회해 채운다.
            goal.marker_id = '23'
            goal.dictionary = 'DICT_5X5_1000'
            goal.squares_x = 6
            goal.squares_y = 5
            goal.square_size_m = 0.024
            goal.marker_size_m = 0.018
            goal.dock_offset_x = 0.0
            goal.dock_offset_y = 0.0
            goal.dock_offset_yaw = 0.0
        elif method == 'floor':
            goal = FloorDock.Goal()
            goal.task_id = task_id
            goal.task_point_id = task_point_id
            goal.wall_gap_m = 0.0        # 0 → 로봇 노드 기본 wall_gap_target
            goal.lateral_offset_m = 0.0  # 0 → 로봇 노드 기본 lateral_offset
        else:
            goal = ReflectiveDock.Goal()
            goal.task_id = task_id
            goal.task_point_id = task_point_id
            goal.stop_gap_m = 0.0        # 0 → 로봇 config yaml 기본 stop_gap_m
        self.get_logger().info('도킹 하달: 방식=%s task=%d point=%s'
                               % (method, task_id, task_point_id))
        fut = cli.send_goal_async(goal, feedback_callback=self._on_dock_fb)
        fut.add_done_callback(self._on_dock_goal_response)

    def _on_dock_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('DCS가 Dock goal 거부')
            self._harvest = None
            return
        self._dock_gh = gh
        gh.get_result_async().add_done_callback(self._on_dock_result)

    def _on_dock_fb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.dock_feedback_phases.append(fb.phase)
        # 거리 필드 이름이 방식마다 다르다(마커까지 / 벽까지). 있는 쪽을 찍는다.
        dist = getattr(fb, 'distance_to_marker_m', None)
        if dist is None:
            dist = getattr(fb, 'distance_to_wall_m', 0.0)
        self.get_logger().info('도킹 진행: phase=%s marker=%s dist=%.2fm'
                               % (fb.phase, fb.marker_detected, dist))

    def _on_dock_result(self, future):
        res = future.result().result
        self.last_dock_result = res
        self.dock_done = True
        if res.result_code == 0 and self._harvest is not None:
            self._last_docked_task = self._harvest['task_id']   # 수확 goal 대상
        self.get_logger().info('도킹 결과: code=%d lateral=%.3fm yaw=%.3frad msg=%s'
                               % (res.result_code, res.final_lateral_m,
                                  res.final_yaw_error, res.message))
        self._harvest = None

    def cancel_dock(self):
        """진행 중인 Dock goal 을 취소한다(E2 22-1 취소 전파 검증용)."""
        if self._dock_gh is not None:
            self.get_logger().warn('ACS 도킹 취소 요청')
            self._dock_gh.cancel_goal_async()

    # ---- S2 E3~E5 수확 시작 (도킹 성공 task 로 Harvest 액션 하달) ----
    def send_harvest_action(self, task_id, max_capacity=7):
        """도킹 성공한 task 로 수확을 시작한다(E3~E5). DG 가 /ddagi/harvest 로 중계한다.
        도킹 안 된 task 면 DG 가 goal 을 거부(reject) → harvest_accepted=False 로 표시."""
        if not self._harvest_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DCS Harvest 서버 없음')
            return None
        self.harvest_accepted = None
        self.harvest_feedback = []
        self.harvest_result = None
        self.harvest_done = False
        self._harvest_gh = None
        goal = Harvest.Goal(task_id=int(task_id), max_capacity=int(max_capacity))
        self.get_logger().info('수확 시작 하달: task=%d max_capacity=%d' % (task_id, max_capacity))
        fut = self._harvest_cli.send_goal_async(goal, feedback_callback=self._on_harvest_fb)
        fut.add_done_callback(self._on_harvest_goal_response)
        return task_id

    def _on_harvest_goal_response(self, future):
        gh = future.result()
        self.harvest_accepted = bool(gh.accepted)
        if not gh.accepted:
            self.get_logger().warn('DCS가 Harvest goal 거부(도킹 안 됨)')
            self.harvest_done = True   # reject 도 종료로 본다(대기 해제용)
            return
        self._harvest_gh = gh
        gh.get_result_async().add_done_callback(self._on_harvest_result)

    def _on_harvest_fb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.harvest_feedback.append(
            {'round': fb.round, 'normal': fb.normal_count, 'discard': fb.discard_count,
             'failed': fb.failed_count, 'remaining': fb.remaining_in_round})
        self.get_logger().info('수확 진행: round=%d normal=%d discard=%d failed=%d remaining=%d'
                               % (fb.round, fb.normal_count, fb.discard_count,
                                  fb.failed_count, fb.remaining_in_round))

    def _on_harvest_result(self, future):
        res = future.result().result
        self.harvest_result = res
        self.harvest_done = True
        self.get_logger().info('수확 결과: exit=%s normal=%d discard=%d failed=%d msg=%s'
                               % (res.exit_reason, res.normal_count, res.discard_count,
                                  res.failed_count, res.message))

    def cancel_harvest(self):
        """진행 중인 Harvest goal 을 취소한다(취소 전파 검증용)."""
        if self._harvest_gh is not None:
            self.get_logger().warn('ACS 수확 취소 요청')
            self._harvest_gh.cancel_goal_async()

    def _on_trigger_harvest(self, request, response):
        """라이브: 도킹 성공한 마지막 task 로 수확(E3) 시작. dashboard.sh 등에서 호출."""
        if self._last_docked_task is None:
            response.success = False
            response.message = '도킹 성공한 task 없음 — 먼저 수확 이동+도킹(start_harvest_move) 실행'
            return response
        tid = self.send_harvest_action(self._last_docked_task)
        response.success = tid is not None
        response.message = ('수확 시작 하달 task=%d' % tid) if tid is not None else 'Harvest 서버 없음'
        return response

    # ---- S2 E6 하역 시작 (예냉실 도킹 성공 task 로 Unload 액션 하달) ----
    def send_unload_action(self, task_id, shake_delay_sec=3.0):
        """예냉실 도킹 성공한 task 로 바구니 하역을 시작한다(E6). DG 가 /ddagi/unload 로 중계한다.
        도킹 안 된 task 면 DG 가 goal 을 거부(reject) → unload_accepted=False 로 표시."""
        if not self._unload_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('DCS Unload 서버 없음')
            return None
        self.unload_accepted = None
        self.unload_feedback = []
        self.unload_result = None
        self.unload_done = False
        self._unload_gh = None
        goal = Unload.Goal(task_id=int(task_id), shake_delay_sec=float(shake_delay_sec))
        self.get_logger().info('하역 시작 하달: task=%d shake_delay=%.1fs'
                               % (task_id, shake_delay_sec))
        fut = self._unload_cli.send_goal_async(goal, feedback_callback=self._on_unload_fb)
        fut.add_done_callback(self._on_unload_goal_response)
        return task_id

    def _on_unload_goal_response(self, future):
        gh = future.result()
        self.unload_accepted = bool(gh.accepted)
        if not gh.accepted:
            self.get_logger().warn('DCS가 Unload goal 거부(도킹 안 됨)')
            self.unload_done = True   # reject 도 종료로 본다(대기 해제용)
            return
        self._unload_gh = gh
        gh.get_result_async().add_done_callback(self._on_unload_result)

    def _on_unload_fb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.unload_feedback.append(fb.phase)
        self.get_logger().info('하역 진행: phase=%s' % fb.phase)

    def _on_unload_result(self, future):
        res = future.result().result
        self.unload_result = res
        self.unload_done = True
        self.get_logger().info('하역 결과: code=%d msg=%s' % (res.result_code, res.message))

    def cancel_unload(self):
        """진행 중인 Unload goal 을 취소한다(취소 전파 검증용)."""
        if self._unload_gh is not None:
            self.get_logger().warn('ACS 하역 취소 요청')
            self._unload_gh.cancel_goal_async()

    def _on_trigger_unload(self, request, response):
        """라이브: 예냉실 도킹 성공한 마지막 task 로 하역(E6) 시작. dashboard.sh 등에서 호출."""
        if self._last_docked_task is None:
            response.success = False
            response.message = '도킹 성공한 task 없음 — 먼저 예냉실 이동+도킹 실행'
            return response
        tid = self.send_unload_action(self._last_docked_task)
        response.success = tid is not None
        response.message = ('하역 시작 하달 task=%d' % tid) if tid is not None else 'Unload 서버 없음'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AcsSim()
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

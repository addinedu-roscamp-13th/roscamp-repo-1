#!/usr/bin/env python3
"""RP-75  E0: DdaGo(주행 로봇) 상시 텔레메트리 스트리밍 Publisher.

순찰 요청 여부와 무관하게 상시 도는 텔레메트리 루프. DdaGo가 자신의
위치/배터리/내비 상태를 1Hz로 HQ에 publish 한다.
DB/메모리/rosbag2 저장 없이 실시간 스트리밍만 수행한다.

여러 소스 토픽을 구독해 콜백마다 "마지막 값"을 캐시하고, 1Hz 타이머에서
캐시값을 DdagoTelemetry 하나로 취합해 발행한다.
(배터리처럼 저주기로 오는 값은 새 값이 올 때까지 마지막 수신값을 유지한다.)

구독 소스 (드라이버/Nav2 토픽은 상대명 — bare 로 뜬 소스와 그대로 매칭):
  amcl_pose                        geometry_msgs/PoseWithCovarianceStamped  위치(맵 절대좌표, 우선)
  odom                             nav_msgs/Odometry                        위치(amcl 없을 때 fallback)
  battery/percent                  std_msgs/Float32                         배터리 퍼센트(0~100)
  battery/voltage                  std_msgs/Float32                         배터리 전압(V)
  us_sensor/range                  sensor_msgs/Range                        전방 초음파 거리(m)
  navigate_to_pose/_action/status  action_msgs/GoalStatusArray              Nav2 주행 상태
  /ddago/current_task              std_msgs/Int64                           진행 중 task_id (내부 신호)

※ 배터리: 핑키의 batt_state(BatteryState).percentage 는 NaN, power_supply_status 는
   항상 UNKNOWN 이라 못 쓴다. 실제 값은 pinky_bringup/battery_publisher 가 발행하는
   battery/percent·battery/voltage(Float32)에 있다. 충전 여부(is_charging)는 핑키가
   어느 토픽으로도 제공하지 않아 항상 False 로 발행한다(하드웨어 확인 시 후속 연동).

※ 초음파(us_range_m): 예전 네임스페이스 기동에서 에러가 나 구독을 뺐었으나, 네임스페이스를
   없애고 bare 로 전환하면서 us_sensor/range 구독을 되살렸다. ADC 노드(pinky_sensor_adc)가
   발행하면 실제 거리(m)가 채워지고, 아직 발행 전이면 0.0(_safe)으로 나간다.

※ task_id: 같은 로봇의 navigate 서버가 goal 을 받을 때마다 /ddago/current_task 로
   알려준다(latched). goal 이 끝나도 0 으로 되돌아가지 않는다 — ACS 는 한 task 를 예약 구간
   단위로 쪼개 여러 goal 로 하달하므로, goal 사이의 틈마다 0 이 되면 QT 화면에서 깜빡이고
   복귀 주행(22-1·E4, 같은 task_id 재사용) 추적도 끊긴다. 부팅 후 goal 을 한 번도 받지
   않았으면 0. "작업이 진행 중인가"의 판단 근거는 이 값이 아니라 DB 의 tasks 상태다.

※ nav_status: IDLE / NAVIGATING 두 값뿐이다. 아래 _STATUS_MAP 주석 참고.

발행:
  /ddago/telemetry                 automato_interfaces/DdagoTelemetry       1Hz
  (ddago 는 자기 정체를 모른다 — msg.robot_id 는 비우고, 어느 로봇인지는 dcs 가 채운다.
   ddago/ddagi 가 같은 망을 공유하므로 /ddago 접두어만 붙여 타입 충돌을 피한다.
   구독 소스 토픽(odom, amcl_pose ...)은 bare 드라이버/Nav2 에 맞춰 상대명 그대로.)

파라미터:
  publish_rate_hz        (float) 발행 주기(Hz)                     기본 1.0
  amcl_stale_sec         (float) amcl 를 신선하다고 볼 최대 나이   기본 3.0
  battery_percent_scale  (float) battery/percent 에 곱할 값          기본 1.0
                                 (핑키 battery_percentage()는 0~100 관례 → 1.0.
                                  혹시 0~1 로 오면 100.0 으로 조정)
"""
import math

from action_msgs.msg import GoalStatus, GoalStatusArray
from automato_interfaces.msg import DdagoTelemetry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Int64
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw_from_quaternion(q):
    """쿼터니언(x,y,z,w) → yaw(rad). tf 의존 없이 Z축 회전만 뽑는다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _safe(value):
    """None/NaN 을 0.0 으로 정리 (아직 값을 못 받았거나 미측정 필드 보호)."""
    if value is None or math.isnan(value):
        return 0.0
    return float(value)


class TelemetryPublisher(Node):
    # Nav2 goal 상태(action_msgs/GoalStatus) → nav_status 문자열 매핑.
    #
    # 값은 IDLE / NAVIGATING 둘뿐이다. nav_status 는 "지금 움직이는 중인가"만 답하는
    # 일시적 상태이고 저절로 IDLE 로 돌아와야 한다(문서 E1 가용 조건 설명).
    #
    #   ABORTED(주행 실패) → IDLE : 실패 사실은 Navigate Result 의 result_code 로 이미
    #     ACS 에 전달된다. 여기에 'FAILED' 같은 값을 따로 두면, 액션 status 토픽이
    #     latched 라 다음 goal 이 올 때까지 그 값이 유지된다. 그러면 E1 가용 조건
    #     "nav_status = IDLE" 을 영영 통과 못 하는데, 그 조건을 통과해야 goal 을 받고
    #     goal 을 받아야 값이 풀리는 교착에 빠진다(로봇은 멀쩡한데 아무도 안 시킴).
    #     배정을 진짜로 막아야 하는 경우는 robots.operational_status(IMMOBILIZED)가 맡는다.
    #   CANCELING → NAVIGATING : 취소를 받았어도 아직 감속 중이라 새 작업을 주면 위험하다.
    #     곧 CANCELED 로 넘어가 자연히 IDLE 이 되므로 교착도 없다.
    _STATUS_MAP = {
        GoalStatus.STATUS_ACCEPTED: 'NAVIGATING',
        GoalStatus.STATUS_EXECUTING: 'NAVIGATING',
        GoalStatus.STATUS_CANCELING: 'NAVIGATING',
        GoalStatus.STATUS_SUCCEEDED: 'IDLE',
        GoalStatus.STATUS_CANCELED: 'IDLE',
        GoalStatus.STATUS_ABORTED: 'IDLE',
        GoalStatus.STATUS_UNKNOWN: 'IDLE',
    }

    def __init__(self, **kwargs):
        # **kwargs 는 테스트에서 parameter_overrides 등을 주입하기 위한 통로.
        super().__init__('telemetry_publisher', **kwargs)

        # --- 파라미터 ---
        # robot_id 는 두지 않는다: 물리망 분리로 이 로봇은 자기 망에 혼자이므로 자기
        # 정체를 알 필요가 없고, 로봇 구분은 dcs(dg control service)가 담당한다.
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('amcl_stale_sec', 3.0)
        self.declare_parameter('battery_percent_scale', 1.0)
        # 위치를 tf 에서 읽을 때 쓰는 프레임 이름. map→base_footprint 변환의
        # 양 끝 프레임이다. 로봇 URDF/nav2 설정에 따라 base_link 로 바꿀 수도 있다.
        self.declare_parameter('global_frame_id', 'map')
        self.declare_parameter('base_frame_id', 'base_footprint')
        rate = self.get_parameter('publish_rate_hz').value
        self._amcl_stale_sec = self.get_parameter('amcl_stale_sec').value
        self._battery_percent_scale = \
            self.get_parameter('battery_percent_scale').value
        self._global_frame = self.get_parameter('global_frame_id').value
        self._base_frame = self.get_parameter('base_frame_id').value

        # --- tf 리스너 ---
        # amcl 이 발행하는 map→base_footprint 변환을 백그라운드로 수집한다.
        # Buffer 가 최근 변환들을 캐시하고, TransformListener 가 /tf·/tf_static 을
        # 구독해 Buffer 를 채운다. (드라이버/Nav2 가 bare 로 /tf 를 발행하므로 이 노드도
        # 리맵 없이 bare /tf 를 그대로 구독한다.)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- 캐시 상태 (콜백이 최신값으로 갱신, 타이머가 읽어감) ---
        self._odom_pose = None          # (x, y, yaw)
        self._amcl_pose = None          # (x, y, yaw)
        self._amcl_stamp = None         # amcl 마지막 수신 시각(rclpy Time)
        self._battery_percent = None    # battery/percent 마지막값(0~100)
        self._battery_voltage = None    # battery/voltage 마지막값(V)
        self._us_range = None           # us_sensor/range 마지막 초음파 거리(m)
        self._nav_status = 'IDLE'       # Nav2 상태 파생 문자열 (기본 대기)
        self._task_id = 0               # /ddago/current_task 로 갱신. goal 전엔 0.

        # --- QoS 프로파일 (소스별로 맞춰야 데이터가 들어온다) ---
        # (배터리 Float32 는 5초 저주기 reliable 발행이라 기본 QoS(reliable, depth 10)로
        #  받아 업데이트를 놓치지 않는다.)
        # latched QoS (RELIABLE + TRANSIENT_LOCAL, depth 1): 발행자가 마지막 값을
        # 붙들고 있다가 늦게 붙은 구독자에게도 즉시 한 번 보내주는 방식. 이 노드가
        # 언제 뜨든 최신 상태를 받으려면 발행자와 같은 프로파일로 맞춰야 한다.
        #   · Nav2 액션 status : Nav2 기본값이 이 조합이다.
        #   · /ddago/current_task : navigate_server 가 같은 조합으로 발행한다.
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- 구독 (상대 토픽명) ---
        # odom/amcl 은 발행자가 reliable 이라 기본 QoS(reliable, depth 10)로 충분.
        self.create_subscription(Odometry, 'odom', self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._amcl_cb, 10)
        # 배터리는 pinky_bringup/battery_publisher 가 Float32 두 토픽으로 발행한다
        # (batt_state.percentage 는 NaN 이라 못 씀 → battery/percent 사용).
        self.create_subscription(
            Float32, 'battery/percent', self._battery_percent_cb, 10)
        self.create_subscription(
            Float32, 'battery/voltage', self._battery_voltage_cb, 10)
        # 초음파: 실사 센서가 best_effort 로 쏠 수 있어 sensor QoS(best_effort)로 받는다
        # (reliable 발행자와도 호환). bare /us_sensor/range 로 매칭.
        self.create_subscription(
            Range, 'us_sensor/range', self._range_cb, qos_profile_sensor_data)
        self.create_subscription(
            GoalStatusArray, 'navigate_to_pose/_action/status',
            self._nav_status_cb, latched_qos)
        # 현재 task: 같은 로봇의 navigate 서버가 goal 을 받을 때마다 알려준다.
        # 로봇 내부 신호라 절대명으로 고정한다(구독 소스처럼 상대명일 필요가 없다).
        self.create_subscription(
            Int64, '/ddago/current_task', self._current_task_cb, latched_qos)

        # --- 발행 + 1Hz 타이머 ---
        # 구독(odom·amcl 등)은 상대명이라 네임스페이스 없이 bare 로 뜨는 드라이버/Nav2
        # 토픽(/odom, /amcl_pose ...)과 그대로 매칭된다. telemetry 발행은 ddago/ddagi 가
        # 같은 망을 공유하므로 /ddago 접두어로 타입 충돌만 피한다. 로봇 구분은 ddago 가
        # 아니라 dcs 가 담당하며, 소비자 dcs 도 /ddago/telemetry 를 구독(팀 협의 완료).
        self._pub = self.create_publisher(DdagoTelemetry, '/ddago/telemetry', 10)
        period = 1.0 / rate if rate > 0.0 else 1.0
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'텔레메트리 Publisher 준비됨: {rate:.1f}Hz → /ddago/telemetry'
        )

    # ------------------------------------------------------------------ #
    # 구독 콜백: 최신값만 캐시 (계산은 발행 타이머에서)
    # ------------------------------------------------------------------ #
    def _odom_cb(self, msg):
        p = msg.pose.pose
        self._odom_pose = (p.position.x, p.position.y,
                           _yaw_from_quaternion(p.orientation))

    def _amcl_cb(self, msg):
        p = msg.pose.pose
        self._amcl_pose = (p.position.x, p.position.y,
                           _yaw_from_quaternion(p.orientation))
        self._amcl_stamp = self.get_clock().now()

    def _battery_percent_cb(self, msg):
        self._battery_percent = msg.data

    def _battery_voltage_cb(self, msg):
        self._battery_voltage = msg.data

    def _range_cb(self, msg):
        self._us_range = msg.range

    def _current_task_cb(self, msg):
        # 값이 바뀔 때만 로그 (latched 재전송·중복 발행으로 같은 값이 또 와도 조용히).
        if int(msg.data) != self._task_id:
            self.get_logger().info(
                f'현재 task 갱신: {self._task_id} → {msg.data}')
        self._task_id = int(msg.data)

    def _nav_status_cb(self, msg):
        if not msg.status_list:
            self._nav_status = 'IDLE'
            return
        # status_list 에 여러 goal 이 쌓일 수 있으므로 stamp 가 가장 최신인 것 선택.
        latest = max(
            msg.status_list,
            key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec),
        )
        self._nav_status = self._STATUS_MAP.get(latest.status, 'IDLE')

    # ------------------------------------------------------------------ #
    # 위치: map→base_footprint tf 우선 → amcl_pose 토픽 → odom fallback
    # ------------------------------------------------------------------ #
    def _map_pose_from_tf(self):
        """map→base_footprint tf 에서 (x, y, yaw) 를 읽는다.

        tf 가 아직 없거나(두 프레임이 연결 안 됨) 너무 오래됐으면(stale) None 을
        돌려 다음 위치 소스로 넘긴다. lookup_transform 의 시각 인자로 Time()(=0)
        을 주면 tf2 가 '지금 캐시에 있는 가장 최근 변환'을 반환한다.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame,   # target 프레임 (map)
                self._base_frame,     # source 프레임 (base_footprint)
                Time(),               # 가장 최근에 확보된 변환
            )
        except TransformException:
            # 아직 tf 미수신이거나 map↔base_footprint 경로 없음 → 이 소스는 스킵.
            return None

        # 신선도 확인: 변환 stamp 나이가 amcl_stale_sec 을 넘으면 버린다
        # (로봇이 멈춰도 오래된 위치를 최신인 척 발행하지 않도록).
        stamp = Time.from_msg(tf.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if age > self._amcl_stale_sec:
            return None

        t = tf.transform.translation
        yaw = _yaw_from_quaternion(tf.transform.rotation)
        return (t.x, t.y, yaw)

    def _select_position(self):
        # 1순위: map→base_footprint tf (가장 신선한 맵 절대좌표).
        map_pose = self._map_pose_from_tf()
        if map_pose is not None:
            return map_pose, 'map'
        # 2순위: amcl_pose 토픽 (tf 를 못 읽을 때, 신선한 경우만).
        if self._amcl_pose is not None and self._amcl_stamp is not None:
            age = (self.get_clock().now() - self._amcl_stamp).nanoseconds * 1e-9
            if age <= self._amcl_stale_sec:
                return self._amcl_pose, 'map'
        # 3순위: odom (맵 정합 전이라 상대좌표지만 위치는 나온다).
        if self._odom_pose is not None:
            return self._odom_pose, 'odom'
        # 최후: 오래됐어도 amcl 값이라도 있으면 사용.
        if self._amcl_pose is not None:
            return self._amcl_pose, 'map'
        return (0.0, 0.0, 0.0), ''

    # ------------------------------------------------------------------ #
    # 1Hz 발행: 캐시값을 DdagoTelemetry 로 취합
    # ------------------------------------------------------------------ #
    def _publish(self):
        msg = DdagoTelemetry()
        (x, y, yaw), frame = self._select_position()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        # msg.robot_id 는 비워 둔다(기본 ''): 어느 로봇인지는 수신하는 dcs 가 채운다.
        msg.task_id = self._task_id
        msg.nav_status = self._nav_status

        msg.x = float(x)
        msg.y = float(y)
        msg.yaw = float(yaw)

        if self._battery_percent is None or math.isnan(self._battery_percent):
            msg.battery_percent = 0.0
        else:
            msg.battery_percent = float(
                self._battery_percent * self._battery_percent_scale)
        msg.battery_voltage = _safe(self._battery_voltage)
        # 충전 여부: pinky_pro 는 충전 상태를 어느 토픽으로도 제공하지 않는다
        #   (sensor_adc 가 power_supply_status 를 UNKNOWN 으로 박아둠).
        #   → E0 에선 항상 False. 하드웨어 충전감지선 확인되면 별도 소스로 연동.
        msg.is_charging = False

        # 초음파: 값을 못 받았으면 _safe 가 0.0 으로 보정.
        msg.us_range_m = _safe(self._us_range)

        self._pub.publish(msg)
        self._warn_if_missing()

    def _warn_if_missing(self):
        """아직 값을 못 받은 소스가 있으면 5초마다 한 번 경고 (0 발행 원인 안내)."""
        missing = []
        if self._odom_pose is None and self._amcl_pose is None:
            missing.append('위치(odom/amcl_pose)')
        if self._battery_percent is None and self._battery_voltage is None:
            missing.append('battery/percent·voltage')
        if self._us_range is None:
            missing.append('us_sensor/range')
        if missing:
            self.get_logger().warn(
                '아직 수신되지 않은 소스: ' + ', '.join(missing)
                + ' (해당 필드는 0으로 발행 중)',
                throttle_duration_sec=5.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

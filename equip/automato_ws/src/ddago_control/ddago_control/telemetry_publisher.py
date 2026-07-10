#!/usr/bin/env python3
"""RP-75  E0: DdaGo(주행 로봇) 상시 텔레메트리 스트리밍 Publisher.

순찰 요청 여부와 무관하게 상시 도는 텔레메트리 루프. DdaGo가 자신의
위치/배터리/내비 상태를 1Hz로 HQ에 publish 한다.
DB/메모리/rosbag2 저장 없이 실시간 스트리밍만 수행한다.

여러 소스 토픽을 구독해 콜백마다 "마지막 값"을 캐시하고, 1Hz 타이머에서
캐시값을 DdagoTelemetry 하나로 취합해 발행한다.
(배터리처럼 저주기로 오는 값은 새 값이 올 때까지 마지막 수신값을 유지한다.)

구독 소스 (모두 상대 토픽명 — 실행 시 네임스페이스 주입):
  amcl_pose                        geometry_msgs/PoseWithCovarianceStamped  위치(맵 절대좌표, 우선)
  odom                             nav_msgs/Odometry                        위치(amcl 없을 때 fallback)
  battery/percent                  std_msgs/Float32                         배터리 퍼센트(0~100)
  battery/voltage                  std_msgs/Float32                         배터리 전압(V)
  us_sensor/range                  sensor_msgs/Range                        전방 초음파 거리
  navigate_to_pose/_action/status  action_msgs/GoalStatusArray              Nav2 주행 상태

※ 배터리: 핑키의 batt_state(BatteryState).percentage 는 NaN, power_supply_status 는
   항상 UNKNOWN 이라 못 쓴다. 실제 값은 pinky_bringup/battery_publisher 가 발행하는
   battery/percent·battery/voltage(Float32)에 있다. 충전 여부(is_charging)는 핑키가
   어느 토픽으로도 제공하지 않아 항상 False 로 발행한다(하드웨어 확인 시 후속 연동).

발행:
  ddago/telemetry                  automato_interfaces/DdagoTelemetry       1Hz

파라미터:
  robot_id               (str)   보고서에 적을 로봇 식별자        기본 'dg_01'
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
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Range
from std_msgs.msg import Float32


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
    _STATUS_MAP = {
        GoalStatus.STATUS_ACCEPTED: 'NAVIGATING',
        GoalStatus.STATUS_EXECUTING: 'NAVIGATING',
        GoalStatus.STATUS_CANCELING: 'CANCELING',
        GoalStatus.STATUS_SUCCEEDED: 'IDLE',
        GoalStatus.STATUS_CANCELED: 'IDLE',
        GoalStatus.STATUS_ABORTED: 'FAILED',
        GoalStatus.STATUS_UNKNOWN: 'IDLE',
    }

    def __init__(self, **kwargs):
        # **kwargs 는 테스트에서 parameter_overrides 등을 주입하기 위한 통로.
        super().__init__('telemetry_publisher', **kwargs)

        # --- 파라미터 ---
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('amcl_stale_sec', 3.0)
        self.declare_parameter('battery_percent_scale', 1.0)
        self._robot_id = self.get_parameter('robot_id').value
        rate = self.get_parameter('publish_rate_hz').value
        self._amcl_stale_sec = self.get_parameter('amcl_stale_sec').value
        self._battery_percent_scale = \
            self.get_parameter('battery_percent_scale').value

        # --- 캐시 상태 (콜백이 최신값으로 갱신, 타이머가 읽어감) ---
        self._odom_pose = None          # (x, y, yaw)
        self._amcl_pose = None          # (x, y, yaw)
        self._amcl_stamp = None         # amcl 마지막 수신 시각(rclpy Time)
        self._battery_percent = None    # battery/percent 마지막값(0~100)
        self._battery_voltage = None    # battery/voltage 마지막값(V)
        self._us_range = None           # 마지막 초음파 거리(m)
        self._nav_status = 'IDLE'       # Nav2 상태 파생 문자열 (기본 대기)
        self._task_id = 0               # E1(순찰 명령) 연동 시 갱신. E0에선 0.

        # --- QoS 프로파일 (소스별로 맞춰야 데이터가 들어온다) ---
        # 초음파: best_effort 로 구독하면 발행자가 reliable/best_effort 어느 쪽이든
        #   호환된다(구독자 best_effort 는 모든 발행자와 매칭).
        # (배터리 Float32 는 5초 저주기 reliable 발행이라 기본 QoS(reliable, depth 10)로
        #  받아 업데이트를 놓치지 않는다.)
        sensor_qos = qos_profile_sensor_data
        # Nav2 액션 status: 기본 QoS 가 RELIABLE + TRANSIENT_LOCAL(depth 1).
        #   마지막 상태를 latched 로 받으려면 동일하게 맞춘다.
        status_qos = QoSProfile(
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
        self.create_subscription(
            Range, 'us_sensor/range', self._range_cb, sensor_qos)
        self.create_subscription(
            GoalStatusArray, 'navigate_to_pose/_action/status',
            self._nav_status_cb, status_qos)

        # --- 발행 + 1Hz 타이머 ---
        self._pub = self.create_publisher(DdagoTelemetry, 'ddago/telemetry', 10)
        period = 1.0 / rate if rate > 0.0 else 1.0
        self._timer = self.create_timer(period, self._publish)

        self.get_logger().info(
            f'텔레메트리 Publisher 준비됨: robot_id={self._robot_id}, '
            f'{rate:.1f}Hz → ddago/telemetry'
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
    # 위치 선택: amcl 우선(신선할 때), 아니면 odom fallback
    # ------------------------------------------------------------------ #
    def _select_position(self):
        if self._amcl_pose is not None and self._amcl_stamp is not None:
            age = (self.get_clock().now() - self._amcl_stamp).nanoseconds * 1e-9
            if age <= self._amcl_stale_sec:
                return self._amcl_pose, 'map'
        if self._odom_pose is not None:
            return self._odom_pose, 'odom'
        if self._amcl_pose is not None:
            # amcl 이 오래됐지만 odom 도 없으면 그래도 amcl 사용.
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
        msg.robot_id = self._robot_id
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

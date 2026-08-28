#!/usr/bin/env python3
"""RP-76  E2: DdaGo 카메라 노드 — CaptureFrame 서비스 서버.

시나리오1 E2 에서 순찰 로봇이 capture==true 노드에 도착·정지하면, 주행 노드
(navigate_server)가 이 노드에 "지금 한 장 찍어 달라"고 요청한다. 이 노드는 그
순간의 RGB 프레임 1장을 sensor_msgs/Image 로 담아 응답한다.

왜 토픽이 아니라 서비스인가:
  순찰 한 바퀴에 실제로 찍는 지점은 십수 곳뿐이다. 30fps 로 이미지를 상시
  흘리면(토픽) 제약된 RPi 의 CPU·메모리 대역과 공유메모리(SHM)를 계속 갉아먹는다.
  요청이 올 때만 프레임을 만들어 응답하면(서비스) 그 비용을 촬영 순간에만 낸다.
  또 카메라 접근을 이 노드 하나로 몰아, 소비자(주행 노드)는 카메라 내부를 몰라도
  되는 깔끔한 계약(CaptureFrame)만 본다.

프레임 소스 두 가지 (source 파라미터):
  * device : 실물 USB 웹캠. 요청이 오면 열어(cv2.VideoCapture) 재사용하고,
             마지막 사용 후 idle_release_sec 이 지나면 release 한다(유휴 절전).
             UVC 웹캠은 열려 있는 동안 read 를 안 해도 상시 스트리밍(약 1~2W)
             상태라 배터리를 계속 소모한다 — release 하면 USB autosuspend 로
             들어가 사실상 0W 가 된다(pinky3 실측: 열림 342 IRQ/s·active,
             닫힘 0 IRQ/s·suspended). 재오픈 비용은 실측 ~1.5초로, 촬영 시점엔
             로봇이 정지해 있고 호출 타임아웃(capture_timeout_sec=5초) 안이라
             수용한다. 요청이 오면 flush 후 최신 1장을 read 하는 건 동일하다.
  * file   : 웹캠 없이 테스트. 지정한 JPEG 1장을 매 요청마다 그대로 반환한다.
             하드웨어·배터리 없이 CaptureFrame→AnalyzeFrame 사슬을 검증할 때 쓴다.
  두 모드 모두 응답(CaptureFrame.Response) 형태가 같아, 소비자는 실물/더미를
  구분하지 못한다.

서비스 이름을 절대이름 /ddago/capture_frame 로 두는 이유:
  로봇이 물리적으로 분리되어(1대=1망) 로봇 내부 이름에는 robot_id 네임스페이스를
  붙이지 않는다. /ddago/navigate·/dg/analyze_frame 과 같은 규칙이다.

파라미터:
  robot_id        (str)  로그 표기용 로봇 식별자             기본 'dg_01'
  capture_service (str)  CaptureFrame 서비스 이름(절대)       기본 '/ddago/capture_frame'
  source          (str)  'device' | 'file'                   기본 'device'
  device_index    (int)  V4L2 장치 인덱스. -1=자동탐색(by-id) / 0이상=강제   기본 -1
  image_path      (str)  file 모드에서 반환할 JPEG 경로        기본 ''
  frame_width     (int)  요청 해상도(가로). device 모드        기본 1280 (16:9=최대 화각)
  frame_height    (int)  요청 해상도(세로). device 모드        기본 720  (16:9=최대 화각)
  flush_frames    (int)  read 전에 버릴 오래된 프레임 수       기본 3
  idle_release_sec(float) 마지막 사용 후 웹캠 release 까지 유휴 시간(초).
                          0 이하 = 안 놓음(상시 열림, 예전 동작)   기본 10.0
  warmup_frames   (int)  새로 연 직후 자동노출(AE) 수렴용으로 버리는
                          프레임 수(30fps 기준 15장≈0.5초)        기본 15
"""
import glob
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node

from automato_interfaces.srv import CaptureFrame


class CameraNode(Node):
    def __init__(self, **kwargs):
        # **kwargs 는 테스트에서 parameter_overrides 등을 주입하기 위한 통로.
        super().__init__('camera_node', **kwargs)

        # --- 파라미터 ---
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('capture_service', '/ddago/capture_frame')
        self.declare_parameter('source', 'device')
        self.declare_parameter('device_index', -1)
        self.declare_parameter('image_path', '')
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('flush_frames', 3)
        self.declare_parameter('idle_release_sec', 10.0)
        self.declare_parameter('warmup_frames', 15)

        self._robot_id = self.get_parameter('robot_id').value
        service_name = self.get_parameter('capture_service').value
        self._source = self.get_parameter('source').value
        self._device_index = int(self.get_parameter('device_index').value)
        self._image_path = self.get_parameter('image_path').value
        self._width = int(self.get_parameter('frame_width').value)
        self._height = int(self.get_parameter('frame_height').value)
        self._flush = int(self.get_parameter('flush_frames').value)
        self._idle_release = float(self.get_parameter('idle_release_sec').value)
        self._warmup = int(self.get_parameter('warmup_frames').value)

        self._bridge = CvBridge()
        self._cap = None          # device 모드: 열어 둔 VideoCapture (지연 오픈)
        self._file_frame = None   # file 모드: 시작 시 읽어 둔 정지 이미지
        # 유휴 release 판단 기준 시각. ROS 클록이 아니라 monotonic 을 쓰는 이유:
        # USB 하드웨어 절전은 실제 경과 시간의 문제라 sim time 과 무관하고,
        # monotonic 은 시스템 시계 조정(NTP 등)에도 뒤로 가지 않는다.
        self._last_use = time.monotonic()

        if self._source == 'file':
            # file 모드는 시작할 때 이미지를 한 번만 읽어 둔다. 매 요청 디스크를
            # 다시 읽을 이유가 없다(같은 사진).
            self._file_frame = cv2.imread(self._image_path)
            if self._file_frame is None:
                self.get_logger().error(
                    f'file 모드인데 이미지를 못 읽음: {self._image_path!r} '
                    f'→ 매 요청 success=false 로 응답. 경로 확인 필요')
            else:
                h, w = self._file_frame.shape[:2]
                self.get_logger().info(
                    f'file 모드 준비: {self._image_path} ({w}x{h})')
        else:
            # device 모드는 여기서 한 번 열어 본다. 실패해도 노드는 살려 두고
            # (카메라를 나중에 꽂을 수 있으니) 요청 때 다시 시도한다.
            self._ensure_device()

        # CaptureFrame 서비스 서버 (절대이름 → 네임스페이스 영향 없음)
        self._srv = self.create_service(
            CaptureFrame, service_name, self._on_capture)

        # 유휴 감시 타이머(device 모드 전용): 마지막 사용 후 idle_release_sec
        # 지나면 release → 웹캠이 USB autosuspend 로 들어가 배터리 소모를 멈춘다.
        # 주기 2초인 이유: idle_release_sec(기본 10초)보다 충분히 짧아 release
        # 지연이 최대 +2초에 그치고, 콜백은 시각 비교뿐이라 비용이 사실상 0이다.
        # 타이머·서비스 콜백은 같은(기본) 콜백 그룹이라 동시에 돌지 않는다
        # → _cap 을 두 콜백이 같이 만져도 잠금이 필요 없다.
        self._idle_timer = None
        if self._source == 'device' and self._idle_release > 0:
            self._idle_timer = self.create_timer(2.0, self._on_idle_check)

        self.get_logger().info(
            f'카메라 노드 준비됨: robot_id={self._robot_id} '
            f'source={self._source} → 서비스 {service_name}')

    # ------------------------------------------------------------------ #
    # device 모드: 장치를 열어 두는(없으면 여는) 헬퍼. 성공 시 True.
    # ------------------------------------------------------------------ #
    def _ensure_device(self):
        if self._cap is not None and self._cap.isOpened():
            return True
        cap, src = self._open_webcam()
        if cap is None:
            self.get_logger().warn(
                '웹캠 열기 실패 — 연결/점유(다른 프로세스)·권한 확인. 요청 시 재시도한다')
            return False
        # (해상도·버퍼 설정은 _try_open 에서 검증 read 전에 이미 끝났다)
        # AE 워밍업: 갓 깨어난 카메라는 자동노출이 수렴 전이라 첫 프레임들이
        # 어둡다. 프레임을 몇 장 흘려보내 수렴 시간을 준다(grab 은 디코드 없이
        # 버려서 싸다). 상시 열림이던 예전엔 불필요했지만, 유휴 release 후
        # 재오픈이 일상이 된 지금은 이게 없으면 매 촬영이 어두운 사진이 된다.
        for _ in range(self._warmup):
            cap.grab()
        self._cap = cap
        self.get_logger().info(
            f'웹캠 열림: {src} 요청 {self._width}x{self._height} '
            f'(AE 워밍업 {self._warmup}프레임)')
        return True

    def _try_open(self, source):
        """source(경로 또는 인덱스)를 열어 설정·검증까지 통과한 cap / 실패 시 None.

        해상도·버퍼 설정을 검증 read '전에' 하는 이유: 기본 해상도로 스트림을
        시작한 뒤 해상도를 바꾸면 UVC 스트림이 정지·재협상되어(실측 0.5~1초)
        콜드 촬영이 그만큼 느려진다. 먼저 설정하면 스트림 시작이 한 번으로
        끝나고, 검증 read 도 실제 촬영 해상도로 이뤄진다.
        버퍼 1 인 이유(드라이버가 지원하면): 버퍼가 크면 오래된 프레임이 쌓여
        read 가 과거 장면을 줄 수 있다 → _grab_frame 의 flush 와 함께 최신성 확보.
        """
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.read()[0]:
            cap.release()
            return None
        return cap

    def _open_webcam(self):
        """열린 (VideoCapture, 소스표기) 반환 / 실패 시 (None, None).

        device_index >= 0 이면 그 인덱스를 강제 사용(수동 오버라이드). -1(기본)이면
        자동 탐색한다: USB 재연결에도 이름이 안 바뀌는 /dev/v4l/by-id 경로를 우선
        쓰고, 없으면 인덱스를 훑는다. RPi 의 CSI(/dev/video0)는 USB 웹캠이 아니므로
        자동 탐색에서 건너뛴다(인덱스 1부터).
        """
        if self._device_index >= 0:
            cap = self._try_open(self._device_index)
            if cap is not None:
                return cap, f'/dev/video{self._device_index}'
            return None, None
        for path in sorted(glob.glob('/dev/v4l/by-id/*-video-index0')):
            cap = self._try_open(path)
            if cap is not None:
                return cap, path
        for idx in (1, 2, 3, 4, 5):
            cap = self._try_open(idx)
            if cap is not None:
                return cap, f'/dev/video{idx}'
        return None, None

    # ------------------------------------------------------------------ #
    # CaptureFrame 서비스 콜백: 최신 프레임 1장 → 응답
    # ------------------------------------------------------------------ #
    def _on_capture(self, request, response):
        self.get_logger().info(
            f'촬영 요청 수신 task={request.task_id} '
            f'waypoint={request.waypoint_id} (source={self._source})')

        frame, err = self._grab_frame()
        if frame is None:
            response.success = False
            response.message = err
            self.get_logger().warn(
                f'촬영 실패 task={request.task_id} '
                f'waypoint={request.waypoint_id}: {err}')
            return response

        # OpenCV 원본은 BGR8. 인코딩을 메시지에 실어 그대로 하류로 전달한다
        # (무엇으로 찍혔는지는 메시지가 스스로 알린다 → 소비자 하드코딩 불필요).
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        response.success = True
        response.image = msg
        response.message = ''
        h, w = frame.shape[:2]
        self.get_logger().info(
            f'촬영 성공 task={request.task_id} '
            f'waypoint={request.waypoint_id} → {w}x{h} bgr8 응답')
        return response

    # ------------------------------------------------------------------ #
    # 소스별 프레임 획득. (frame, None) 성공 / (None, 사유) 실패.
    # ------------------------------------------------------------------ #
    def _grab_frame(self):
        if self._source == 'file':
            if self._file_frame is None:
                return None, f'file 이미지 미로딩: {self._image_path!r}'
            # 매 요청 같은 이미지지만 복사해 돌려준다(하류에서 손대도 원본 보존).
            return self._file_frame.copy(), None

        # device 모드
        if not self._ensure_device():
            return None, f'웹캠 미개방 /dev/video{self._device_index}'
        self._last_use = time.monotonic()   # 유휴 release 카운트다운 리셋
        # 오래된 버퍼 프레임을 버려 '지금' 장면을 얻는다. grab()은 디코드 없이
        # 버퍼만 넘겨 read()보다 싸다(그래서 flush 용으로 grab, 최종만 read).
        for _ in range(self._flush):
            self._cap.grab()
        ok, frame = self._cap.read()
        if not ok or frame is None:
            return None, 'read 실패(프레임 없음) — 카메라 연결/드라이버 확인'
        return frame, None

    # ------------------------------------------------------------------ #
    # 유휴 감시: 마지막 사용 후 idle_release_sec 지나면 웹캠을 놓아 준다.
    # 놓으면 커널 USB autosuspend(control=auto)가 웹캠을 절전으로 내려
    # 스트리밍 전력(~1-2W)이 0 이 된다. 다음 요청 때 _ensure_device 가 다시 연다.
    # ------------------------------------------------------------------ #
    def _on_idle_check(self):
        if self._cap is None:
            return
        idle = time.monotonic() - self._last_use
        if idle < self._idle_release:
            return
        self._cap.release()
        self._cap = None
        self.get_logger().info(
            f'유휴 {idle:.0f}초 → 웹캠 release (USB 절전 진입, 다음 요청 때 재오픈)')

    def destroy_node(self):
        # 노드 종료 시 장치를 반드시 놓아 준다(안 놓으면 다음 기동에서 못 열 수 있다).
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

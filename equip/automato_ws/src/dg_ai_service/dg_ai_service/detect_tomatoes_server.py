#!/usr/bin/env python3
"""RP-127  시나리오2 E3 — Ddagi -> DG AI Service 개별 토마토 검출 서비스.

Ddagi가 관측 자세로 복귀한 뒤 이 서비스(/ai/detect_tomatoes)를 호출하면,
AI Service가 USB로 직결된 D435에서 프레임을 직접 획득해(analysis_server.py의
analyze_frame과 달리 이미지 자체는 오가지 않는다) YOLO로 개별 토마토를
검출하고 등급(NORMAL/DISCARD)·3D 좌표(camera_link)를 묶어 반환한다.
안익은(unripe) 열매는 애초에 반환하지 않는다(따지 않으므로).

서비스 이름이 절대경로 /ai/detect_tomatoes 인 이유: Ddagi ↔ AI Service는
같은 로봇 세트 내부 통신이라 robot_id 네임스페이스를 붙이지 않는다
(ddago_control/camera_node.py의 /ddago/capture_frame과 같은 규칙).
"""
import os

import numpy as np
import rclpy
from automato_interfaces.msg import Tomato
from automato_interfaces.srv import DetectTomatoes
from rclpy.node import Node

from dg_ai_service.analysis_server import DEFAULT_CONF, build_classifier, models_dir
from dg_ai_service.camera_stream import CameraStream
from dg_ai_service.tomato_grading import DEFAULT_MASK_PADDING_PX, build_tomatoes

DEFAULT_SERVICE_NAME = '/ai/detect_tomatoes'

# Ddagi(DetectTomatoes)는 DG Control Service(analyze_frame, v6)와 다른 버전(v8)을
# 쓴다 — analysis_server.DEFAULT_MODEL_PATH를 그대로 재사용하지 않는 이유.
DEFAULT_MODEL_PATH = os.environ.get('DG_AI_MODEL_PATH') or os.path.join(models_dir(), 'tomato_4cls_v8.pt')


class DetectTomatoesServer(Node):
    def __init__(self, **kwargs):
        # **kwargs는 테스트에서 parameter_overrides 등을 주입하기 위한 통로.
        super().__init__('detect_tomatoes_server', **kwargs)

        self.declare_parameter('model_path', DEFAULT_MODEL_PATH or '')
        self.declare_parameter('conf', DEFAULT_CONF)
        self.declare_parameter('service_name', DEFAULT_SERVICE_NAME)
        self.declare_parameter('mask_padding_px', DEFAULT_MASK_PADDING_PX)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value or None
        conf = self.get_parameter('conf').get_parameter_value().double_value
        service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self._padding_px = self.get_parameter('mask_padding_px').get_parameter_value().double_value

        self._detector = build_classifier(model_path, conf)
        self._detector.warmup()

        # 카메라는 지연 오픈한다(ddago_control/camera_node.py와 같은 이유): D435가
        # 아직 안 꽂혔거나 다른 프로세스(Octomap용 realsense2_camera_node 등)가
        # 점유 중이어도 노드 자체는 살아 있어야 요청이 올 때마다 재시도할 수 있다.
        self._camera = None

        self.create_service(DetectTomatoes, service_name, self._on_detect_tomatoes)

        self.get_logger().info(f'detect_tomatoes_server 준비 완료 -> {service_name}')

    def _on_detect_tomatoes(self, request, response):
        self.get_logger().info(
            f'[detect_tomatoes] 요청 수신 task={request.task_id} round={request.round}'
        )

        if not self._ensure_camera():
            return self._error_response(
                response, 'CAMERA_NOT_AVAILABLE',
                '카메라가 열려 있지 않음(미연결 또는 다른 프로세스가 점유 중) — 다음 요청 때 재시도'
            )

        try:
            color_frame, depth_frame = self._camera.get_frames()
            color_img = np.asanyarray(color_frame.get_data())
            depth_img = np.asanyarray(depth_frame.get_data())
        except Exception as exc:
            # 열려 있던 파이프라인이 도중에 끊겼을 수 있으니 다음 요청이 재오픈을
            # 시도하도록 리셋한다.
            self._camera = None
            return self._error_response(
                response, 'CAMERA_NOT_AVAILABLE', f'카메라 프레임 획득 실패: {exc}'
            )

        try:
            tomatoes = build_tomatoes(
                self._detector, color_img, depth_img, self._camera.get_intrinsics(),
                padding_px=self._padding_px,
            )
        except Exception as exc:
            return self._error_response(response, 'INFERENCE_FAILED', f'추론 실패: {exc}')

        response.success = True
        response.frame_id = 'camera_link'
        response.tomatoes = [Tomato(**t) for t in tomatoes]
        response.error_code = ''
        response.message = f'{len(tomatoes)}개 검출'
        self.get_logger().info(
            f'[detect_tomatoes] task={request.task_id} round={request.round}: {response.message}'
        )
        return response

    def _ensure_camera(self) -> bool:
        if self._camera is not None:
            return True
        try:
            self._camera = CameraStream()
        except Exception as exc:
            self.get_logger().warn(f'카메라 열기 실패(다음 요청 때 재시도): {exc}')
            return False
        return True

    def _error_response(self, response, error_code: str, message: str):
        response.success = False
        response.frame_id = ''
        response.tomatoes = []
        response.error_code = error_code
        response.message = message
        self.get_logger().error(f'[detect_tomatoes] {message}')
        return response

    def destroy_node(self):
        # 노드 종료 시 장치를 반드시 놓아 준다(연 적이 있을 때만).
        if self._camera is not None:
            self._camera.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectTomatoesServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

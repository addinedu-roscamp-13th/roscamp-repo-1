#!/usr/bin/env python3
"""RP-50  DG AI Service — 정밀 분석 TCP 서버.

Sprint 3 Scenario 1에 따라 DG Control Service(HQ)로부터 JPEG 이미지 기반
분석 요청을 받는 TCP/JSON 서버를 구현합니다.

서버는 아래 두 프로토콜을 모두 지원합니다.
- 기존 Legacy: {"op":"start"|"stop"}
- Scenario1: {
      "message_type":"analyze_frame_request",
      "request_id":"...",
      "task_id":..., "waypoint_id":..., "image_encoding":"jpeg",
      "image_data":"<base64>"
  }

응답은 길이 접두어 프레이밍([4B length][UTF-8 JSON])으로 전송합니다.

연결/요청/응답은 모두 logging 으로 기록되며, 기본적으로 콘솔과
logs/dg_ai_service.log 파일에 동시에 남는다 (--log-file 로 변경 가능).
"""
import argparse
import logging
import os
import socket
from typing import Any, Dict, Optional

from dg_ai_service.framing import recv_msg, send_msg
from dg_ai_service.yolo_detector import ModelNotReadyError, TomatoDetector

DEFAULT_HOST = '0.0.0.0'
DEFAULT_PORT = 9100
DEFAULT_CONF = 0.4
DEFAULT_MODEL_PATH = os.environ.get('DG_AI_MODEL_PATH', None)
DEFAULT_LOG_FILE = os.path.join('logs', 'dg_ai_service.log')

SUPPORTED_ENCODINGS = {'jpeg', 'jpg', 'png'}

LOG = logging.getLogger('dg_ai_service')


def configure_logging(log_file: Optional[str] = DEFAULT_LOG_FILE, level: int = logging.INFO) -> None:
    """콘솔 + (선택) 파일로 동시에 로그를 남기도록 설정.

    반복 호출(테스트 등)에서 핸들러가 중복 추가되지 않도록 기존 핸들러를
    먼저 정리한다.
    """
    LOG.setLevel(level)
    for handler in list(LOG.handlers):
        LOG.removeHandler(handler)

    fmt = logging.Formatter('%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s', datefmt='%H:%M:%S')

    stream_handler = logging.StreamHandler()
    stream_handler.setFormatter(fmt)
    LOG.addHandler(stream_handler)

    if log_file:
        log_dir = os.path.dirname(os.path.abspath(log_file))
        if log_dir:
            os.makedirs(log_dir, exist_ok=True)
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_handler.setFormatter(fmt)
        LOG.addHandler(file_handler)
        LOG.info(f'로그 파일: {os.path.abspath(log_file)}')

    LOG.propagate = False


def _summarize(payload: Any) -> Any:
    """로그 출력용: base64 이미지 블롭을 <base64 N bytes> 로 축약."""
    if isinstance(payload, dict):
        summary = {}
        for key, value in payload.items():
            if key in ('image_data', 'labeled_image') and isinstance(value, str):
                summary[key] = f'<base64 {len(value)} bytes>'
            else:
                summary[key] = _summarize(value)
        return summary
    if isinstance(payload, list):
        return [_summarize(v) for v in payload]
    return payload


class LegacyAnalyzer:
    @staticmethod
    def infer() -> Dict[str, Any]:
        return {'status': 'ripe', 'coord': [12.3, 4.5, 6.7], 'confidence': 0.94}


def build_classifier(model_path: Optional[str] = None, conf: float = DEFAULT_CONF) -> TomatoDetector:
    if model_path is None:
        model_path = DEFAULT_MODEL_PATH
    if model_path is None:
        raise ModelNotReadyError(
            'Model path is not configured. Set DG_AI_MODEL_PATH or pass --model-path.'
        )
    return TomatoDetector(model_path, conf)


def build_error_response(request_id: Optional[str], error_code: str, error_message: str) -> Dict[str, Any]:
    return {
        'message_type': 'analyze_frame_response',
        'request_id': request_id or 'unknown',
        'status': 'ERROR',
        'error_code': error_code,
        'error_message': error_message,
    }


def handle_analyze_frame_request(request: Dict[str, Any], detector: TomatoDetector) -> Dict[str, Any]:
    request_id = request.get('request_id')
    LOG.info(f'[analyze_frame] 요청 수신: {_summarize(request)}')

    if request.get('message_type') != 'analyze_frame_request':
        LOG.warning(f'[analyze_frame] {request_id}: message_type 오류')
        return build_error_response(request_id, 'INVALID_MESSAGE_TYPE', 'message_type must be analyze_frame_request')

    if not request_id:
        LOG.warning('[analyze_frame] request_id 누락')
        return build_error_response(None, 'MISSING_REQUEST_ID', 'request_id is required')

    image_encoding = request.get('image_encoding', '').lower()
    if image_encoding not in SUPPORTED_ENCODINGS:
        LOG.warning(f'[analyze_frame] {request_id}: 지원하지 않는 image_encoding={image_encoding!r}')
        return build_error_response(request_id, 'UNSUPPORTED_IMAGE_ENCODING', 'image_encoding must be jpeg/png')

    image_data = request.get('image_data')
    if not isinstance(image_data, str):
        LOG.warning(f'[analyze_frame] {request_id}: image_data 누락/타입 오류')
        return build_error_response(request_id, 'MISSING_IMAGE_DATA', 'image_data must be base64-encoded string')

    try:
        counts, result = detector.analyze(image_data)
    except ValueError as exc:
        LOG.error(f'[analyze_frame] {request_id}: 디코딩 실패 - {exc}')
        return build_error_response(request_id, 'IMAGE_DECODE_FAILED', str(exc))
    except ModelNotReadyError as exc:
        LOG.error(f'[analyze_frame] {request_id}: 모델 미준비 - {exc}')
        return build_error_response(request_id, 'MODEL_NOT_READY', str(exc))
    except Exception as exc:
        LOG.exception(f'[analyze_frame] {request_id}: 추론 실패')
        return build_error_response(request_id, 'INFERENCE_FAILED', str(exc))

    analysis_result = TomatoDetector.percentages(counts)
    # rotten/disease 가 감지된 이미지는 확인·알림용으로 레이블링된 이미지를 함께 반환한다.
    if TomatoDetector.needs_labeled_image(counts):
        try:
            analysis_result['labeled_image'] = TomatoDetector.encode_labeled_image(result)
            analysis_result['labeled_image_encoding'] = 'jpeg'
        except ValueError as exc:
            LOG.error(f'[analyze_frame] {request_id}: 레이블링 이미지 인코딩 실패 - {exc}')
            return build_error_response(request_id, 'LABELED_IMAGE_ENCODE_FAILED', str(exc))

    LOG.info(f'[analyze_frame] {request_id}: 분석 결과 {dict(counts)} -> {TomatoDetector.percentages(counts)}'
              f'{" (labeled_image 포함)" if TomatoDetector.needs_labeled_image(counts) else ""}')

    response = {
        'message_type': 'analyze_frame_response',
        'request_id': request_id,
        'status': 'OK',
        'result': analysis_result,
    }
    LOG.info(f'[analyze_frame] 응답 전송: {_summarize(response)}')
    return response


def handle_connection(
    conn: socket.socket,
    detector: TomatoDetector,
    legacy: Optional[LegacyAnalyzer] = None,
    addr: Optional[Any] = None,
) -> None:
    if legacy is None:
        legacy = LegacyAnalyzer()

    peer = addr or '(unknown)'
    while True:
        try:
            req = recv_msg(conn)
        except ConnectionError:
            LOG.info(f'[conn {peer}] 연결 종료(상대방)')
            break

        if not isinstance(req, dict):
            LOG.warning(f'[conn {peer}] JSON 객체가 아닌 요청 수신: {req!r}')
            send_msg(conn, build_error_response(None, 'INVALID_REQUEST', 'Request payload must be JSON object'))
            continue

        if req.get('message_type') == 'analyze_frame_request':
            send_msg(conn, handle_analyze_frame_request(req, detector))
            continue

        op = req.get('op')
        if op == 'stop':
            LOG.info(f'[conn {peer}] stop 요청 수신, 연결 종료')
            break
        if op == 'start':
            LOG.info(f'[conn {peer}] legacy start 요청 수신')
            send_msg(conn, legacy.infer())
            continue

        LOG.warning(f'[conn {peer}] 알 수 없는 요청 형식: {_summarize(req)}')
        send_msg(conn, build_error_response(req.get('request_id'), 'INVALID_REQUEST', 'Unknown request format'))


class LazyDetector:
    """analyze_frame_request 가 실제로 들어올 때만 YOLO 모델을 로드.

    레거시 {"op":"start"} 스텁 경로는 모델이 필요 없으므로, 서버 기동
    시점에 DG_AI_MODEL_PATH/모델 파일이 없어도 죽지 않게 하기 위함.
    """

    def __init__(self, model_path: Optional[str] = None, conf: float = DEFAULT_CONF):
        self._model_path = model_path
        self._conf = conf
        self._detector: Optional[TomatoDetector] = None

    def _ensure(self) -> TomatoDetector:
        if self._detector is None:
            LOG.info(f'[model] 최초 analyze_frame_request 수신, 모델 로딩 시작 '
                      f'(model_path={self._model_path or DEFAULT_MODEL_PATH})')
            self._detector = build_classifier(self._model_path, self._conf)
            LOG.info('[model] 모델 로딩 완료')
        return self._detector

    def analyze(self, image_data: str):
        return self._ensure().analyze(image_data)


def run_server(
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    once: bool = False,
    ready: Optional[Any] = None,
    detector: Optional[Any] = None,
) -> None:
    if detector is None:
        detector = LazyDetector()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen()
        LOG.info(f'[server] 대기 시작: {host}:{port}')
        if ready is not None:
            ready.set()
        while True:
            conn, addr = srv.accept()
            LOG.info(f'[conn {addr}] 연결 수립')
            with conn:
                try:
                    handle_connection(conn, detector, addr=addr)
                except ConnectionError:
                    LOG.info(f'[conn {addr}] 연결 비정상 종료')
            if once:
                break


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='DG AI Service TCP analysis server')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--port', type=int, default=DEFAULT_PORT)
    parser.add_argument('--model-path', default=None)
    parser.add_argument('--conf', type=float, default=DEFAULT_CONF)
    parser.add_argument('--once', action='store_true')
    parser.add_argument('--log-file', default=DEFAULT_LOG_FILE,
                         help=f'로그 파일 경로 (기본: {DEFAULT_LOG_FILE}). 빈 문자열이면 파일 로깅 비활성화')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    configure_logging(args.log_file or None)
    LOG.info(f'[dg_ai_service] 분석 서버 시작: {args.host}:{args.port}')
    # 모델은 첫 analyze_frame_request 수신 시 지연 로드된다(LazyDetector).
    # 레거시 {"op":"start"} 스텁 경로는 모델이 없어도 서버가 정상 기동한다.
    detector = LazyDetector(args.model_path, args.conf)

    try:
        run_server(host=args.host, port=args.port, once=args.once, ready=None, detector=detector)
    except KeyboardInterrupt:
        LOG.info('[dg_ai_service] 종료')


if __name__ == '__main__':
    main()

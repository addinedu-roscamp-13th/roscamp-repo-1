# DG AI Service TCP Interface Specification

DG AI Service 는 DG Control Service 와 TCP 기반 요청/응답 통신을 수행한다.
프로토콜은 두 가지가 함께 존재한다.

- **레거시 (`op:start/stop`)**: 집기 직전 1회 정밀 분석(RP-50). 실제
  추론 없이 고정된 stub 응답을 반환하는 단계.
- **`analyze_frame` (시나리오1 E2)**: 순찰 웨이포인트 RGB 프레임 분석.
  실제 YOLO 모델(`tomato_4cls_model.pt`, 4클래스: ripe/unripe/rotten/
  disease)로 추론하며, rotten 또는 disease 가 감지되면 레이블링된
  이미지를 함께 반환한다.

## 1. 통신 방식
- Protocol: TCP, 지속 연결
- Direction: DG Control Service → DG AI Service
- Purpose: 분석 요청 전달 및 결과 수신

## 2. 프레이밍 규격
메시지는 길이 프리픽스 방식으로 전송된다.

- Format: [4B payload_size][UTF-8 JSON]
- payload_size: 뒤따르는 JSON 데이터의 길이 (Big-Endian unsigned int)
- JSON payload: UTF-8 인코딩된 JSON 객체

## 3. 레거시 프로토콜 (`op:start/stop`)

**요청**
```json
{"op":"start"}
```
- `start`: 분석 요청
- `stop`: 연결 종료

**응답**
```json
{
  "status": "ripe",
  "coord": [x, y, z],
  "confidence": 0.9
}
```
- `status`: 분류 결과 (`ripe`, `unripe`, `overripe`, `damaged`)
- `coord`: 3차원 좌표 (grasp 목표)
- `confidence`: 신뢰도 값 (0.0 ~ 1.0)

**동작 흐름**
1. DG Control Service가 AI Service에 TCP 접속
2. `start` 요청 전송
3. AI Service가 분석 결과 응답 (현재는 고정 stub)
4. DG Control Service가 `stop` 전송
5. 연결 종료

구현: `dg_control/ai_client.analyze()`, `dg_ai_service/analysis_server.LegacyAnalyzer`

## 4. `analyze_frame` 프로토콜 (시나리오1 E2)

**요청**
```json
{
  "message_type": "analyze_frame_request",
  "request_id": "req_20260706_001",
  "task_id": 1024,
  "waypoint_id": 3,
  "image_encoding": "jpeg",
  "image_data": "<base64 encoded bytes>"
}
```
- `request_id`: DG Control Service 가 생성. 응답에서 그대로 되돌아온다.
- `image_encoding`: `jpeg` / `jpg` / `png` 만 지원.

**응답 (성공)**
```json
{
  "message_type": "analyze_frame_response",
  "request_id": "req_20260706_001",
  "status": "OK",
  "result": {
    "ripe_percent": 0,
    "unripe_percent": 0,
    "rotten_percent": 93,
    "disease_percent": 7,
    "labeled_image": "<base64 encoded JPEG>",
    "labeled_image_encoding": "jpeg"
  }
}
```
- `*_percent`: 검출된 클래스별 개수를 백분율로 환산 (검출 0개면 전부 0).
- `labeled_image` / `labeled_image_encoding`: **rotten 또는 disease 가
  하나라도 검출된 경우에만** 포함된다. 검출 박스가 그려진 JPEG 를
  base64 로 인코딩한 값. 정상(ripe/unripe 만 검출)인 경우 이 두 필드는
  응답에 아예 없다.

**응답 (에러)**
```json
{
  "message_type": "analyze_frame_response",
  "request_id": "req_20260706_001",
  "status": "ERROR",
  "error_code": "IMAGE_DECODE_FAILED",
  "error_message": "이미지 디코딩 실패"
}
```
- 주요 `error_code`: `INVALID_MESSAGE_TYPE`, `MISSING_REQUEST_ID`,
  `UNSUPPORTED_IMAGE_ENCODING`, `MISSING_IMAGE_DATA`,
  `IMAGE_DECODE_FAILED`, `MODEL_NOT_READY`, `INFERENCE_FAILED`,
  `LABELED_IMAGE_ENCODE_FAILED`.

구현: `dg_control/ai_client.analyze_frame()` / `decode_labeled_image()`,
`dg_ai_service/analysis_server.handle_analyze_frame_request()`,
`dg_ai_service/yolo_detector.TomatoDetector`

모델 로드는 지연 로딩된다(`analysis_server.LazyDetector`) — 첫
`analyze_frame_request` 가 들어올 때 `DG_AI_MODEL_PATH` 로 지정된
로컬 모델 파일을 로드하며, 레거시 프로토콜만 쓰는 경우 모델이 없어도
서버는 정상 기동한다.

## 5. 개발/테스트 환경
venv 설치, 서버 기동, 수동 테스트 방법은
[ai_service_dev_env_setup.md](ai_service_dev_env_setup.md) 참고.

## 6. 참고 파일
- equip/automato_ws/src/dg_control/dg_control/ai_client.py
- equip/automato_ws/src/dg_control/dg_control/send_test_frame.py
- equip/automato_ws/src/dg_ai_service/dg_ai_service/analysis_server.py
- equip/automato_ws/src/dg_ai_service/dg_ai_service/yolo_detector.py
- equip/automato_ws/src/dg_ai_service/dg_ai_service/framing.py

# [RP-69] DG AI Service ↔ DG Control 순찰(E2) 분석 연동 점검 및 레이블링 이미지 반환 기능 추가

- 작성자: minho
- 작성일: 2026-07-08
- 브랜치: `sprint4-minho`
- 관련 티켓: RP-69
- 관련 문서: [ai_service_interface_spec.md](ai_service_interface_spec.md), [ai_service_dev_env_setup.md](ai_service_dev_env_setup.md), [Scenario1_Sequence Diagram.md](Scenario1_Sequence%20Diagram.md)

## 1. 개요

다른 팀원이 시나리오1 Sequence Diagram과 Eval_Yolo 코드를 참고해 구현한
`dg_ai_service`(RGB 프레임 YOLO 분석)와, 동작 검증용으로 함께 수정된
`dg_control` 코드를 검토했다. 검토 중 발견한 버그 2건을 수정하고,
요청 사항이던 "rotten/disease 검출 시 레이블링 이미지 반환" 기능을
추가했다. 이어서 팀 공용 로컬 개발환경(venv)을 정비하고, 수동
테스트 도구를 만들어 실제 모델로 end-to-end 동작을 확인했다.

## 2. 작업 배경

- 시나리오1 E2(웨이포인트별 순찰 분석)는 `DdaGo → HQ → DG AI Service`
  구간이 TCP/JSON 프로토콜(`analyze_frame_request/response`)로
  정의되어 있다 (ROS2 아님).
- `dg_ai_service`는 4클래스(ripe/unripe/rotten/disease) YOLO 모델로
  실제 추론하도록 구현되어 있었으나, `dg_control` 쪽에는 이 프로토콜을
  호출하는 코드가 전혀 없어 검증이 불가능한 상태였다.
- 학습 가중치(`tomato_4cls_model.pt`)는 별도 프로젝트(Eval_Yolo)에만
  있고 git에는 올리지 않는 것이 전제 조건이었다.

## 3. 발견된 문제 및 수정

| # | 문제 | 원인 | 조치 |
|---|------|------|------|
| 1 | `dg_control/test/test_orchestration.py` 통합 테스트가 `TypeError`로 실패 | 테스트 fixture가 `analyze` 함수를 인자 없는 람다로 monkeypatch했으나, 실제 `op_harvest_server.py`는 `analyze(ai_host, ai_port)`로 2개 인자를 넘겨 호출 | monkeypatch 제거, 이미 노드에 존재하던 `ai_host`/`ai_port` 파라미터를 `set_parameters()`로 지정하도록 수정 |
| 2 | `DG_AI_MODEL_PATH`가 없으면 AI 서버 자체가 기동 실패 | `analysis_server.run_server()`가 기동 시점에 무조건 실제 YOLO 모델을 로드. 레거시 `{"op":"start"}` 스텁 경로는 원래 모델이 필요 없었는데 결합되어 있었음 | `LazyDetector`를 도입해 `analyze_frame_request`가 실제로 들어올 때만 모델을 지연 로드하도록 분리 |

## 4. 신규 기능: rotten/disease 검출 시 레이블링 이미지 반환

`analyze_frame_response`의 `result`에 다음 필드를 조건부로 추가했다.

```json
{
  "ripe_percent": 0, "unripe_percent": 0,
  "rotten_percent": 93, "disease_percent": 7,
  "labeled_image": "<base64 JPEG>",
  "labeled_image_encoding": "jpeg"
}
```

- rotten 또는 disease가 하나라도 검출된 경우에만 포함(정상 이미지는
  기존과 동일하게 퍼센트만 반환 → 하위 호환).
- 구현: `yolo_detector.TomatoDetector.encode_labeled_image()`(YOLO
  결과의 바운딩 박스를 그려 JPEG로 인코딩) +
  `analysis_server.handle_analyze_frame_request()`에서 조건부 첨부.
- `dg_control` 쪽에 이 프로토콜(`analyze_frame`)을 호출하는 코드가
  아예 없었기 때문에, `ai_client.py`에 `analyze_frame()` /
  `decode_labeled_image()` 클라이언트 함수를 신규 작성했다.

## 5. 개발 환경 정비

- `equip/automato_ws` 전용 Python venv를 `uv`로 구성 (Eval_Yolo venv와
  독립). `requirements.txt`(ultralytics, opencv-python, numpy,
  pytest) + 설치 가이드 `ai_service_dev_env_setup.md` 작성.
- `dg_control/test_images/` 폴더 신설(이미지 파일 자체는
  `.gitignore`로 제외) + 수동 테스트 CLI `dg_control/send_test_frame.py`
  작성: 폴더의 이미지를 순서대로 AI 서비스에 전송하고, rotten/disease
  검출 시 `<파일명>_labeled.jpg`로 결과를 저장.
- 학습 가중치가 실수로 커밋되지 않도록 루트 `.gitignore`에 `*.pt`,
  `*.onnx` 추가.

## 6. 문서/코드 정리 (리팩토링)

- `yolo_detector.py`에 있던 미사용 중복 상수(`SUPPORTED_ENCODINGS`,
  `analysis_server.py`에도 동일하게 정의되어 있었으나 실제로는
  `analysis_server.py` 쪽만 사용됨) 제거.
- `ai_service_interface_spec.md`가 레거시 `{"op":"start"}` 스텁
  프로토콜만 설명하고 실제 사용 중인 `analyze_frame` 프로토콜(퍼센트
  결과 + labeled_image)은 문서화되어 있지 않아 최신화.

## 7. 검증 결과

- 신규/기존 pytest 12건 전부 통과 (`test_ai_analysis.py`,
  `test_labeled_image.py`, `test_analyze_frame_client.py`) — fake
  detector 기반이라 실제 모델 없이도 CI에서 검증 가능.
- `test_orchestration.py`(rclpy 필요)는 ROS2 환경에서 별도 확인 필요
  (본 세션 환경에는 ROS2 미설치로 직접 실행은 못 했으나 로직 검토 완료).
- Eval_Yolo의 실제 모델(`tomato_4cls_model.pt`)과 `disease_n01.jpg`로
  end-to-end 수동 검증: rotten 93% / disease 7% 정상 검출 및
  바운딩 박스가 그려진 레이블링 이미지 정상 생성 확인.

## 8. 향후 과제

- `DdaGo Control Service`가 실제로 `/dg/analyze_frame` ROS2 서비스를
  통해 `dg_control`을 호출하고, `dg_control`이 이를 받아
  `analyze_frame()`으로 중계하는 상위 오케스트레이션 로직은 아직
  미구현 (현재는 수동 테스트 스크립트로만 검증 가능한 상태).
- `Automato Control Service`로의 `SaveDetection` 저장 연동(E2 5~8번)
  은 이번 작업 범위 밖.

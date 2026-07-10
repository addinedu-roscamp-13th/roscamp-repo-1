# RP-69: DG AI Service 순찰 분석 결과에 rotten/disease 레이블링 이미지 반환

- 유형: Task
- 브랜치: `sprint4-minho`
- 담당: minho
- 완료일: 2026-07-08

## 작업 설명

시나리오1 E2(웨이포인트별 순찰 분석) 흐름에서, `DG AI Service`가
`DG Control Service`에 분석 결과(`analyze_frame_response`)를 반환할 때
rotten 또는 disease가 검출된 이미지에 한해 바운딩 박스가 그려진
레이블링 이미지를 함께 반환하도록 한다. 착수 전, 기존 코딩 에이전트가
작업한 `dg_ai_service`/`dg_control` 코드에 문제가 없는지 검토한다.

## 상세 작업 내용

1. **기존 코드 검토 및 버그 수정**
   - `dg_control/test/test_orchestration.py`의 `analyze()` monkeypatch
     인자 불일치로 인한 `TypeError` 수정 (→ 노드 파라미터
     `ai_host`/`ai_port`를 통해 테스트 포트를 주입하는 방식으로 변경)
   - `dg_ai_service/analysis_server.py`가 서버 기동 시점에 무조건 YOLO
     모델을 로드해, `DG_AI_MODEL_PATH` 미설정 시 레거시 프로토콜조차
     쓸 수 없던 문제 수정 (→ `LazyDetector`로 지연 로딩)

2. **레이블링 이미지 반환 기능 구현**
   - `yolo_detector.TomatoDetector`: `analyze()`가 (검출 개수, YOLO
     결과 객체)를 반환하도록 변경, `needs_labeled_image()` /
     `encode_labeled_image()` 추가
   - `analysis_server.handle_analyze_frame_request()`: rotten 또는
     disease 검출 시 `result.labeled_image`(base64 JPEG) /
     `result.labeled_image_encoding` 필드를 조건부로 응답에 포함
   - `dg_control/ai_client.py`: 기존에 없던 `analyze_frame()` /
     `decode_labeled_image()` 클라이언트 함수 신규 작성 (E2
     `analyze_frame` 프로토콜을 실제로 호출하는 첫 코드)

3. **테스트 작성**
   - `dg_ai_service/test/test_labeled_image.py`: fake detector로
     rotten/disease 유무에 따른 `labeled_image` 포함 여부 단위 테스트
     (4건)
   - `dg_control/test/test_analyze_frame_client.py`: TCP 왕복까지
     포함한 클라이언트 통합 테스트 (4건)
   - 실제 모델(`tomato_4cls_model.pt`)로 `disease_n01.jpg` 등에 대해
     수동 end-to-end 검증 (rotten 93%/disease 7% 검출, 레이블링 이미지
     정상 생성)

4. **개발 환경/도구 정비**
   - `equip/automato_ws` 전용 Python venv 구성 가이드 작성
     (`uv` 사용, Eval_Yolo venv와 독립) — `requirements.txt`,
     `docs/ai_service_dev_env_setup.md`
   - 수동 테스트 CLI `dg_control/send_test_frame.py` +
     `dg_control/test_images/` 폴더 신설 (테스트 이미지 자체는
     `.gitignore`로 git 제외)
   - 학습 가중치(`*.pt`, `*.onnx`) git 제외 규칙을 루트 `.gitignore`에
     추가

5. **문서 정리**
   - `ai_service_interface_spec.md`를 레거시 프로토콜뿐 아니라 실제
     사용 중인 `analyze_frame` 프로토콜(요청/응답 포맷, 에러 코드,
     `labeled_image` 조건)까지 포함하도록 갱신
   - 미사용 중복 상수(`yolo_detector.py`의 `SUPPORTED_ENCODINGS`) 제거

## 작업 완료 조건 (Definition of Done)

- [x] 기존 `dg_ai_service`/`dg_control` 코드 검토 완료, 발견된 버그
      (테스트 monkeypatch 인자 불일치, 서버 기동 시 모델 강제 로딩)
      수정
- [x] rotten 또는 disease가 검출된 이미지에 대해서만
      `analyze_frame_response.result.labeled_image`(base64 JPEG)가
      포함되고, 정상 이미지는 기존과 동일한 응답 형식을 유지함
      (하위 호환)
- [x] 학습 가중치 파일(`tomato_4cls_model.pt`)이 git에 포함되지 않고
      `DG_AI_MODEL_PATH` 환경변수로 로컬 경로만 참조함
- [x] 신규 기능에 대한 자동화 테스트 8건 작성 및 통과 (기존 4건 포함
      총 12건 전부 통과, fake detector 기반이라 모델 없이도 CI 가능)
- [x] 실제 YOLO 모델로 최소 1개 이상의 rotten/disease 포함 이미지에
      대해 레이블링 이미지가 정상 생성됨을 수동 검증
- [x] `equip/automato_ws` 전용 venv 설치 가이드 문서화, 팀원이 별도
      셋업 없이 재현 가능
- [x] 관련 인터페이스 문서(`ai_service_interface_spec.md`)가 실제
      구현과 일치하도록 최신화됨

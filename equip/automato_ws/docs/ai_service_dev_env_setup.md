# DG AI Service 개발 환경 설정 (Python venv, uv)

## 배경

`dg_control` ↔ `dg_ai_service` 의 `analyze_frame` 프로토콜은 TCP 위에서
동작하며(ROS2 아님, `automato_interfaces/tcp/ai_analysis.md` 참고), AI
서비스 쪽은 실제 YOLO 추론을 위해 `ultralytics` / `opencv-python` /
`numpy` 가 필요합니다.

- 시스템 python3 에는 이 패키지들이 없고, Ubuntu 는 PEP 668
  (externally-managed-environment) 때문에 시스템 python 에 바로
  `pip install` 이 되지 않습니다.
- 그래서 `equip/automato_ws` 전용 venv 를 따로 둡니다. **Eval_Yolo
  프로젝트의 venv 와는 완전히 독립적**이며, 서로 참조하지 않습니다.

## 1. uv 설치 (최초 1회, 이미 있으면 스킵)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

## 2. venv 생성 + 의존성 설치

```bash
cd equip/automato_ws
uv venv .venv
uv pip install --python .venv/bin/python -r requirements.txt
```

`.venv/` 는 루트 `.gitignore` 에 이미 등록되어 있어 git push 되지
않습니다. 팀원은 저장소를 새로 받은 뒤 이 2줄만 실행하면 동일한 환경을
재현할 수 있습니다.

## 3. 모델 파일 경로 지정 (로컬 전용, git push 금지)

학습된 가중치(`tomato_4cls_model.pt`)는 저장소에 올리지 않고, 각자
로컬 경로를 환경변수로만 지정합니다.

```bash
export DG_AI_MODEL_PATH=~/Projects/Eval_Yolo/tomato_4cls_model.pt   # 본인 로컬 경로로 수정
```

세션마다 다시 치기 번거로우면 `~/.bashrc` 에 추가해도 됩니다.

## 4. AI 서비스 실행

```bash
cd equip/automato_ws
PYTHONPATH=src/dg_ai_service:src/dg_control \
  .venv/bin/python -m dg_ai_service.analysis_server
```

서버는 소켓 바인딩 자체는 즉시 하지만(모델은 지연 로드), 최초
`analyze_frame` 요청이 들어올 때 torch/ultralytics 를 그 자리에서
초기화하느라 몇 초 걸릴 수 있습니다. 그 순간 요청한 클라이언트만
잠깐 대기하며, 서버 자체가 죽은 건 아닙니다.

기본 host 는 `0.0.0.0` 이라 별도 옵션 없이도 같은 네트워크의 다른 PC
접속을 받습니다. `--host`/`--port`/`--model-path`/`--conf` 로 재정의
가능합니다.

### 로그

연결/요청/응답이 콘솔과 `equip/automato_ws/logs/dg_ai_service.log`
파일에 동시에 기록됩니다 (`*.log` 는 `.gitignore` 에 이미 등록되어
있어 git에는 안 올라갑니다). 이미지 바이트(`image_data`,
`labeled_image`)는 `<base64 N bytes>` 로 축약해서 찍히므로 로그가
거대해지지 않습니다.

```bash
tail -f equip/automato_ws/logs/dg_ai_service.log
```

`--log-file <경로>` 로 저장 위치를 바꾸거나, `--log-file ""` 로 파일
로깅을 끄고 콘솔만 쓸 수 있습니다.

## 5. 원격 PC에서 테스트 (다른 PC의 dg_control → 이 PC의 AI 서비스)

같은 네트워크에 있다면 코드 수정 없이 됩니다. AI 서비스 PC의 IP를
클라이언트 쪽 `--host` 로 넘기기만 하면 됩니다.

```bash
# dg_control PC 에서
PYTHONPATH=src/dg_ai_service:src/dg_control \
  .venv/bin/python -m dg_control.send_test_frame --host <AI서비스PC IP> --port 9100
```

접속이 안 되면 (연결 자체가 안 되는 경우) 아래로 네트워크부터 확인:

```bash
# dg_control PC 에서, AI 서비스 PC 포트가 열려있는지 확인
nc -zv <AI서비스PC IP> 9100
```
- `Connection refused` → AI 서비스가 그 PC에서 안 떠 있거나 다른 host/port 에 bind됨
- 응답 없이 멈춤 → AI 서비스 PC 방화벽이 9100 을 막고 있음 → `sudo ufw allow 9100/tcp`
- `Connected` → 네트워크는 정상, `--host` 를 빠뜨렸는지(기본값 `127.0.0.1`) 확인

## 6. 테스트 이미지 전송 (dg_control)

`src/dg_control/test_images/` 폴더에 jpg/jpeg/png 를 넣고:

```bash
PYTHONPATH=src/dg_ai_service:src/dg_control \
  .venv/bin/python -m dg_control.send_test_frame
```

rotten/disease 가 감지된 이미지는 같은 폴더에 `<파일명>_labeled.jpg` 로
저장됩니다. 이 `_labeled.jpg` 결과물은 다음 실행 때 다시 입력으로
집히지 않도록 자동으로 제외됩니다. (`send_test_frame.py` 자체는
cv2/numpy 가 필요 없는 순수 소켓 스크립트라 시스템 python3 로도
동작하지만, 통일해서 venv 사용을 권장합니다.)

## 7. 문제 진단 체크리스트

`analyze_frame` 요청이 `status: "ERROR"` 로 오거나 응답이 아예 없을 때,
먼저 [4단계 로그](#4-ai-서비스-실행)의 `error_code`/로그 메시지를 확인하세요.

| 증상 | 원인 | 확인 방법 |
|---|---|---|
| 클라이언트가 응답 없이 멈춤(타임아웃) | `--host` 를 안 줘서 자기 자신에 접속 시도, 또는 AI 서비스 PC 방화벽이 포트를 막음 | `nc -zv <AI서비스PC IP> 9100` (5단계 참고) |
| `MODEL_NOT_READY` | ①`DG_AI_MODEL_PATH` 미설정 ②경로는 있는데 그 PC엔 파일이 없음 ③`analysis_server` 를 시스템 `python3` 로 띄워서 ultralytics/opencv/numpy 없음 | AI 서비스 PC에서 `echo $DG_AI_MODEL_PATH`, `ls -la "$DG_AI_MODEL_PATH"`, `which python3`(`.venv/bin/python` 이어야 함) |
| `IMAGE_DECODE_FAILED` | 보낸 `image_data` 가 실제 JPEG/PNG로 인코딩되지 않음(원본 픽셀을 그대로 base64 했거나, `data:image/jpeg;base64,` 접두어가 안 지워졌거나, URL-safe base64 사용) | 서버 로그의 에러 메시지에 수신 바이트 수 + 첫 8바이트 hex 가 찍힘. 보내는 쪽에서 `cv2.imencode()`(또는 동급) 를 거쳤는지, 표준 base64(`+`/`/`) 인지 확인 |
| 같은 waypoint 마다 검출 결과가 계속 동일 | 보내는 쪽이 waypoint 마다 실제 카메라 프레임을 안 갈아끼우고 같은 이미지 파일을 반복 전송 | 로그의 `image_data` 바이트 길이가 요청마다 동일하면 거의 확실. AI 서비스 문제 아님 |

## 8. pytest

```bash
PYTHONPATH=src/dg_ai_service:src/dg_control \
  .venv/bin/python -m pytest src/dg_ai_service/test src/dg_control/test/test_analyze_frame_client.py -v
```

## 참고

- 이 venv 는 `dg_ai_service` 의 YOLO 추론 전용입니다. `analysis_server`/
  `send_test_frame` 은 `rclpy` 가 필요 없는 순수 TCP 코드라 이 venv
  만으로 충분합니다. ROS2 액션/토픽을 쓰는 다른 노드를 띄우려면
  기존대로 ROS2 + `colcon build` 환경이 별도로 필요합니다.
- `ros2 run` 으로 `analysis_server`/`send_test_frame` 을 돌리는 것도
  가능은 하지만, 그 경우 ROS2 시스템 python 에도 `requirements.txt` 의
  패키지가 설치되어 있어야 합니다. 별도 venv 를 쓰는 이 가이드가 더
  간단합니다.

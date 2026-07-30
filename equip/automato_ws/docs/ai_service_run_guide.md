# DG AI Service 기동 가이드 (analysis_server / detect_tomatoes_server)

이 문서는 **띄우는 방법**만 다룬다. venv 생성·의존성 설치는
[ai_service_dev_env_setup.md](ai_service_dev_env_setup.md) 참고.

`dg_ai_service` 에는 **독립적으로 뜨는 서버가 2개** 있고, 서로 다른 모델을
쓴다. 프로토콜도 달라서(TCP vs ROS 서비스) **동시에 띄워도 충돌하지 않는다.**

| 서버 | 시나리오 | 모델 (기본값) | 엔드포인트 | D435 사용 |
|---|---|---|---|---|
| `analysis_server` | S1 순찰 `analyze_frame` (DCS↔TCP) | `tomato_4cls_v6.pt` | TCP `0.0.0.0:9100` | **안 씀** (DCS가 이미지를 보내줌) |
| `detect_tomatoes_server` | S2 수확 `DetectTomatoes` (Ddagi↔ROS) | `tomato_4cls_v8.pt` | ROS 서비스 `/ai/detect_tomatoes` | **씀** (직접 카메라 읽음) |

## 사전 조건

```bash
cd ~/Projects/roscamp-repo-1/equip/automato_ws
ls .venv/bin/python                       # venv (ultralytics/opencv/pyrealsense2)
ls src/dg_ai_service/models/              # tomato_4cls_v6.pt, tomato_4cls_v8.pt 둘 다 있어야 함
colcon build --symlink-install            # ★ --symlink-install 필수 (아래 주의사항 참고)
```

모델 파일은 `.gitignore` 로 커밋에서 제외되므로 **저장소를 새로 받으면 수동으로
복사해 넣어야 한다.** 파일명이 위와 정확히 같아야 환경변수 없이 자동으로 잡힌다.

## 기동

### 공통 준비 (매 터미널)

```bash
cd ~/Projects/roscamp-repo-1/equip/automato_ws   # ★ 반드시 이 디렉토리에서 (로그 경로가 상대경로)
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source .venv/bin/activate
```

### 터미널 1 — analysis_server (v6, 시나리오1)

```bash
python -m dg_ai_service.analysis_server
```

옵션: `--host` `--port` `--model-path` `--conf` `--log-file` `--once`

정상 기동 로그:

```
[INFO] [dg_ai_service] 분석 서버 시작: 0.0.0.0:9100
[INFO] [server] 대기 시작: 0.0.0.0:9100
[INFO] [model] 모델 로딩 시작 (model_path=.../models/tomato_4cls_v6.pt)
[INFO] [model] 모델 로딩 + 워밍업 완료
```

실모델 로딩·워밍업에 **약 7초** 걸린다. 로그는 콘솔 + `logs/dg_ai_service.log` 동시 기록.

### 터미널 2 — detect_tomatoes_server (v8, 시나리오2)

```bash
python -m dg_ai_service.detect_tomatoes_server
```

파라미터: `--ros-args -p model_path:=... -p conf:=0.4 -p mask_padding_px:=... -p service_name:=...`

정상 기동 로그:

```
[INFO] [detect_tomatoes_server]: detect_tomatoes_server 준비 완료 -> /ai/detect_tomatoes
```

기동 시 `torch` 의 `Can't initialize NVML` / `CUDA unknown error` 경고가 뜨는 것은
정상이다(GPU 없이 CPU 추론으로 넘어감).

## 주의사항

### 1. `DG_AI_MODEL_PATH` 를 export 하지 마라

두 서버가 **같은** 환경변수를 읽는다
([analysis_server.py:49](../src/dg_ai_service/dg_ai_service/analysis_server.py#L49),
[detect_tomatoes_server.py:30](../src/dg_ai_service/dg_ai_service/detect_tomatoes_server.py#L30)).
한 셸에서 export 하고 둘을 띄우면 v6/v8 분리가 무너져 **양쪽이 같은 모델을 쓴다.**
한쪽만 바꾸려면 그 프로세스에만 옵션으로 준다:

```bash
python -m dg_ai_service.analysis_server --model-path /path/to/other.pt
python -m dg_ai_service.detect_tomatoes_server --ros-args -p model_path:=/path/to/other.pt
```

### 2. `ros2 run` 으로 띄우지 마라

`ros2 run dg_ai_service analysis_server` 는 뜨긴 하지만, install 스크립트의
shebang 이 `#!/usr/bin/python3` 로 고정돼 **venv 를 무시한다.** 시스템 python 에는
ultralytics 가 없어서 이렇게 된다:

```
[WARNING] [model] 사전 워밍업 생략(요청 시 재시도): Missing dependency: install ultralytics, opencv-python, numpy ...
```

서버는 살아 있지만 요청마다 추론이 실패한다. **반드시 venv 의 `python -m`** 으로 띄운다.

### 3. `--symlink-install` 로 빌드해야 모델 경로가 맞는다

`models_dir()` 이 `realpath(__file__)` 기준으로 `../models` 를 찾는다
([analysis_server.py:40-42](../src/dg_ai_service/dg_ai_service/analysis_server.py#L40-L42)).
`--symlink-install` 로 빌드하면 `build/dg_ai_service/dg_ai_service` 가 `src/` 로
걸린 심링크라 `src/dg_ai_service/models` 로 풀린다. 심링크 없이 그냥
`colcon build` 하면 존재하지 않는 `build/dg_ai_service/models` 를 찾게 되므로,
그때는 모델 경로를 명시해야 한다.

### 4. D435 는 한 프로세스만 점유할 수 있다

`detect_tomatoes_server` 는 카메라를 **첫 서비스 요청 때 지연 오픈**한다
([detect_tomatoes_server.py:51-54](../src/dg_ai_service/dg_ai_service/detect_tomatoes_server.py#L51-L54)).
노드는 살아 있고, 열기 실패하면 다음 요청에서 재시도한다.

아래가 카메라를 먼저 잡고 있으면 `CAMERA_NOT_AVAILABLE` 이 돌아온다:

- `realsense2_camera_node` (Octomap용)
- `ros2 run dg_ai_service camera_viewer`
- `ddagi_harvest` 의 `camera_probe` / `camera_view`
- 이전에 띄웠다가 안 죽은 `detect_tomatoes_server`

```bash
ps -eo pid,cmd | grep -E "realsense|camera_view|camera_probe|detect_tomatoes" | grep -v grep
```

`Ctrl+C` 로 정상 종료하면 `destroy_node()` 가 파이프라인을 닫아 장치를 놓아 준다
([detect_tomatoes_server.py:120-124](../src/dg_ai_service/dg_ai_service/detect_tomatoes_server.py#L120-L124)).

### 5. 특정 네트워크에서 기동이 ~10초 멎어 있는 것처럼 보임

`yolo_detector.py`가 import하는 `ultralytics`가 로드 시점에 `is_online()`으로
Cloudflare/Google DNS(`one.one.one.one`, `dns.google`)를 조회한다(`analysis_server.py`/
`detect_tomatoes_server.py` 둘 다 `yolo_detector`를 통해 이 import를 거치므로
두 서버 모두 해당). 인터넷이 안 되는 네트워크라도:

- 라우터가 요청을 **빠르게 거부**(RST 등)하면 바로 넘어가서 티가 안 남
- 라우터가 요청을 **그냥 버리면**(silent drop) OS 리졸버 타임아웃까지 그대로
  멎어 있다 — 로그 한 줄도 안 찍히는 구간이라(`configure_logging()`/노드 생성
  **전**, 파일 최상단 import 단계) "실행이 안 된다"처럼 보이지만 사실은 느리게
  기동 중인 것뿐이다. 실측 ~10초.

조치 — 서버 실행 **전에** export(반드시 문자열 `"true"`, `"1"`은 인식 안 됨):

```bash
export YOLO_OFFLINE=true
```

`ultralytics/utils/__init__.py`의 `is_online()`이 `os.getenv("YOLO_OFFLINE", "")`가
`"true"`(대소문자 무관)일 때만 DNS 조회 자체를 건너뛴다. 매번 export하기
번거로우면 `~/.bashrc`(`ROS_DOMAIN_ID` 옆)에 넣어둔다.

## 동작 확인

### D435 단독 확인

```bash
.venv/bin/python -c "
import pyrealsense2 as rs
for d in rs.context().query_devices():
    print(d.get_info(rs.camera_info.name),
          d.get_info(rs.camera_info.serial_number),
          'USB', d.get_info(rs.camera_info.usb_type_descriptor))
"
```

기대: `RealSense D435 <시리얼> USB 3.2` — USB 2.x 로 잡히면 케이블/포트를 바꾼다.

스트리밍까지 확인:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
.venv/bin/python -m dg_ai_service.camera_stream
```

### detect_tomatoes 왕복 호출

서버를 띄운 상태에서 다른 터미널:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 service call /ai/detect_tomatoes automato_interfaces/srv/DetectTomatoes "{task_id: 9001, round: 1}"
```

기대 응답:

```
success=True, frame_id='camera_link', tomatoes=[], error_code='', message='0개 검출'
```

카메라 앞에 토마토가 없으면 `0개 검출` 이 정상이다(추론은 실제로 돌았다는 뜻 —
실패라면 `error_code` 가 채워진다). 첫 호출은 카메라 오픈 때문에 ~0.9초, 이후
호출은 카메라를 재사용해 ~0.25초.

### analysis_server 왕복 호출

```bash
.venv/bin/python -m dg_control.send_test_frame          # 로컬
.venv/bin/python -m dg_control.send_test_frame --host <AI서비스PC IP> --port 9100
```

## 문제 진단

| 증상 | 원인 | 조치 |
|---|---|---|
| `Missing dependency: install ultralytics...` | 시스템 python 으로 띄움 (`ros2 run` 포함) | venv 활성화 후 `python -m` |
| 모델 경로가 `build/dg_ai_service/models/...` 로 찍힘 | `--symlink-install` 없이 빌드 | 재빌드 또는 `--model-path` 명시 |
| 두 서버가 같은 모델을 씀 | `DG_AI_MODEL_PATH` export 됨 | `unset DG_AI_MODEL_PATH` |
| `CAMERA_NOT_AVAILABLE` | 다른 프로세스가 D435 점유, 또는 미연결 | 위 `ps` 명령으로 점유 프로세스 확인 후 종료 |
| `MODEL_NOT_READY` | 모델 파일 없음 (`models/` 는 커밋 제외) | `ls src/dg_ai_service/models/` 로 v6/v8 확인 |
| `logs/dg_ai_service.log` 가 엉뚱한 곳에 생김 | `automato_ws` 밖에서 실행 | `cd equip/automato_ws` 후 실행 |
| 로그 한 줄도 없이 ~10초 멎어 있다가 기동됨 (특정 공유기에서만) | `ultralytics` import 시 `is_online()` DNS 조회가 응답 없는 네트워크에서 타임아웃 | `export YOLO_OFFLINE=true` (문자열 `true`만 인식) 후 재실행 |

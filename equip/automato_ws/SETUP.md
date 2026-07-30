# automato_ws 개발 환경 설정 기록

`equip/automato_ws` (ROS2 Jazzy) 를 빌드·실행·테스트하기 위해 설치한 항목을 기록한다.
**새로 설치할 때마다 이 문서에 추가한다.**

## 기본 환경

| 항목 | 값 |
|---|---|
| OS | Ubuntu (Linux) |
| ROS2 | Jazzy (`/opt/ros/jazzy`) |
| Python | 3.12.3 |
| 가상환경 | `/home/ane/dev_ws/.venv` (`include-system-site-packages = false`) |

> ⚠️ **venv 격리 주의:** 이 venv 는 시스템 site-packages 를 안 본다. ROS2 빌드/실행에 필요한
> 파이썬 모듈(`empy`, `catkin_pkg`, `yaml` 등)이 시스템에만 있으면 venv 안에서는 `ModuleNotFoundError`
> 가 난다. 그래서 아래 패키지들을 **venv 안에** 별도 설치했다.
> (대안: venv 밖 시스템 python 으로 빌드하면 대부분 이미 있음.)

## venv 에 설치한 pip 패키지

설치 명령:
```bash
source /home/ane/dev_ws/.venv/bin/activate
pip install "empy==3.3.4" catkin_pkg lark numpy pyyaml
```

| 패키지 | 버전 | 용도 | 설치 계기 |
|---|---|---|---|
| `empy` | 3.3.4 | rosidl 인터페이스 코드 생성(템플릿) | `automato_interfaces` colcon build 시 `No module named 'em'` |
| `catkin_pkg` | 1.1.0 | `package.xml` 파싱 | build 시 `No module named 'catkin_pkg'` |
| `lark` | 1.3.1 | .msg/.action 파서 | rosidl build 의존성 |
| `numpy` | 2.4.6 | 메시지 타입 지원 | rosidl build 의존성 (이미 있었을 수 있음) |
| `PyYAML` | 6.0.3 | `rclpy` import, `launch` | pytest 에서 `rclpy` import 시 `No module named 'yaml'` |
| `PyQt5` | 5.15.11 | `rqt_graph` 등 rqt GUI (Qt 바인딩) | `ros2 run rqt_graph rqt_graph` 시 `Could not find Qt binding` (venv 격리로 시스템 PyQt5 안 보임) |
| `pydot` | 4.0.1 | `rqt_graph` 그래프 렌더링 | PyQt5 설치 후 `No module named 'pydot'` (venv 격리). 런타임에 시스템 `graphviz`(dot, `/usr/bin/dot`) 필요 — 이미 설치돼 있음 |
| `tornado` | 6.5.7 | `rosbridge_websocket` 웹서버 | venv 에서 rosbridge 실행 시 `No module named 'tornado'` |
| `pymongo` | 4.17.0 | rosbridge 의 `bson` (BSON) | `module 'bson' has no attribute 'BSON'` — standalone `bson` 말고 pymongo 의 bson 필요. (standalone `bson` 설치했다면 `pip uninstall bson` 후 pymongo) |
| `Pillow` | 12.2.0 | rosbridge 이미지 처리(PIL) | rosbridge 실행 시 `No module named 'PIL'` |
| `cbor2` | 6.1.2 | rosbridge CBOR 직렬화 | rosbridge 실행 시 `No module named 'cbor2'` |
| `websockets` | 16.0 | (검증 전용) 웹소켓으로 rosbridge 수신 테스트 | 브라우저 없이 대시보드 데이터 흐름 검증용. 앱 실행엔 불필요 |
| `opencv-python-headless` | 5.0.0 | `dg_ddago` 실제 주행 노드의 도착 후 USB 웹캠 1프레임 캡처(`cv2.VideoCapture`) | E2 촬영용. **headless** 선택(로봇/CI 무헤드, Qt·GUI 의존 없음, Pi4에도 가벼움). **없어도 됨** — `ddago_node`가 `cv2` import 실패 시 `sample_frames` 이미지로 자동 폴백 |

### 함께 딸려 설치된 의존성
`docutils 0.23`, `pyparsing 3.3.2`, `packaging 26.2`, `python-dateutil 2.9.0.post0`,
`six 1.17.0`, `setuptools 82.0.1`, `PyQt5-Qt5 5.15.19`, `PyQt5-sip 12.18.0`

### 아직 미설치 (경고만, 현재 불필요)
`generate-parameter-library-py` 가 요구하는 `jinja2`, `typeguard` — 파라미터 라이브러리를
쓰는 패키지를 빌드할 때 필요해지면 설치한다.

## 시스템(apt) 패키지 — 웹 대시보드용

브라우저 시각화(`web/index.html`)는 rosbridge 로 ROS↔WebSocket 을 중계한다. **apt 설치(sudo 필요)**:
```bash
sudo apt install -y ros-jazzy-rosbridge-suite
```
- 실행: `ros2 run rosbridge_server rosbridge_websocket` (ws://localhost:9090)
- graphviz(dot) 는 pydot 런타임용으로 이미 설치돼 있음.
- **상태(2026-07-01):** rosbridge 설치·검증 완료. 단 **venv 안에서 실행하려면** 위 표의 `tornado`·`pymongo`·`Pillow`·`cbor2` pip 설치가 필요했다(venv 격리). venv 밖(시스템 python)에서 실행하면 불필요.

## 서드파티 로봇팔 의존성 (`mycobot.repos`)

`mycobot_280_pick`(coord_to_goal_node, MoveIt2 목표 플래닝)이 필요로 하는
`easy_handeye2`/`mycobot_ros2`/`pymoveit2`는 각자 자체 git 히스토리를 가진
서드파티 패키지라 이 저장소에 벤더링하지 않는다. `moveit_calibration`도
동일하게 업스트림만 참조한다. `vcstool`로 별도 clone:

```bash
pip install vcstool  # 또는 apt install python3-vcstool
cd equip/automato_ws
vcs import src < mycobot.repos
```

## 빌드 방법

```bash
source /home/ane/dev_ws/.venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd /home/ane/dev_ws/roscamp-repo-1/equip/automato_ws
colcon build
source install/setup.bash
```

## 테스트 방법 (pytest)

```bash
source /home/ane/dev_ws/.venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd /home/ane/dev_ws/roscamp-repo-1/equip/automato_ws
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/<pkg>/test/ -v
```

- **`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 필수** — 없으면 venv 에서 `launch_testing` 플러그인
  자동로드 중 `yaml` 관련 import 로 수집 단계부터 실패.
- 액션 클라이언트 테스트는 **`SingleThreadedExecutor`** 사용 — `MultiThreadedExecutor` 는
  rclpy 액션 클라이언트에서 "Two goals accepted with same ID" 충돌 발생.

## 변경 이력

| 날짜 | 내용 |
|---|---|
| 2026-07-01 | 최초 작성. empy·catkin_pkg·lark·numpy·pyyaml 설치 기록 (automato_interfaces 빌드 + ddago_control pytest 를 위한 세팅) |
| 2026-07-01 | PyQt5 5.15.11 설치 — `rqt_graph` GUI 실행용 (venv 격리로 시스템 PyQt5 미인식) |
| 2026-07-01 | pydot 4.0.1 설치 — `rqt_graph` 그래프 렌더링 (`No module named 'pydot'`). 시스템 graphviz(dot)는 기설치 |
| 2026-07-01 | 웹 대시보드용: rosbridge(apt) 설치 + venv 실행 의존성 tornado 6.5.7·pymongo 4.17.0·Pillow 12.2.0·cbor2 6.1.2 설치. 검증용 websockets 16.0. rosbridge 웹소켓으로 `/dg1/op/status` 수신 e2e 검증 완료 |
| 2026-07-08 | `dg_control`(HQ 본체)+`dg_sim`(시뮬 4종) 신규 패키지 추가, `web/`→`dg_web/` 재작성. **새 pip/apt 설치 없음**(기존 rclpy·automato_interfaces만 사용). 빌드: `colcon build --packages-select automato_interfaces dg_control dg_sim`. 테스트: `src/dg_sim/test/test_e2e.py`, `src/dg_control/test/test_ai_switch.py` (4건 통과). 상세: `docs/dg_control_dev_2026-07-08.md` |
| 2026-07-09 | `dg_ddago`(실제 DdaGo 주행 노드) 신규 패키지 추가 — `DdagoPatrol` 액션 서버 → Nav2 `NavigateToPose`. **pip: `opencv-python-headless` 5.0.0** 설치(도착 후 USB 웹캠 캡처용, 없으면 sample_frames 폴백). 빌드: `colcon build --packages-select dg_ddago`. 테스트: `src/dg_ddago/test/test_ddago_drive.py`(2건 통과, 가짜 Nav2 서버). 통합: 실 ddago_node+HQ+시뮬로 4 waypoint 순찰 완주 확인(result_code=0 visited=4) |
| 2026-07-10 | `dg_ddago`에 **`webcam_node`** 추가(E2 웹캠 분리, 온디맨드). 분산 구성용: 노트북 `ddago_node`(E0+E1) + Pi4 `webcam_node`. `webcam_node`는 `std_srvs/Trigger` 서비스 호출 시 웹캠 1프레임 캡처→`sensor_msgs/Image` 발행(상시 스트리밍 X). `ddago_node`에 `capture_service` 파라미터 추가(설정 시 도착마다 서비스 호출→새 프레임). `automato_interfaces` 미수정(표준 Trigger+Image만). package.xml에 `std_srvs` 추가. **Pi4에서 `webcam_node` 실행하려면 로봇에 opencv 설치 필요**(`pip3 install --user opencv-python-headless` 또는 apt `python3-opencv`) — 배포·실기 테스트는 다음 세션. 노트북 빌드+오프라인 테스트 2건 통과(회귀 없음) |
| 2026-07-14 | **로봇 ID 단일 출처화**: `~/.bashrc` 에 `export ROBOT_ID=dg_01` 추가(노트북 + 로봇 pinky7/192.168.100.7, `ROS_DOMAIN_ID` 다음 줄). `dashboard.sh` 가 DCS·시뮬에 `-p robot_id:=$ROBOT_ID` 주입, `dg_sim.launch.py` 는 `EnvironmentVariable('ROBOT_ID')` 기본값, 대시보드 `/api/status` 에 robot_id 표시, DCS 는 robot_id 불일치 텔레메트리를 경고 후 무시. 검증: `ROBOT_ID=dg_02 ./dashboard.sh up` → 토픽/액션이 `/dg_02/...` 로 생성되고 순찰 완주 |
| 2026-07-14 | 시퀀스 다이어그램 개정(**Patrol→Navigate**) 반영. `dg_control` 노드명 `hq_node`→**`dcs_node`**(HQ 표기를 전부 DCS 로 통일), `dg_sim`(acs/ddago/dg_ai) 및 `dg_web`/`dashboard.sh` 동반 수정 — 대시보드 노드 키도 `hq`→`dcs`. 인터페이스는 팀원이 커밋한 `Navigate.action`/`Waypoint.msg`/`SaveDetection.disease_image` 를 그대로 사용(수정 없음). ddago 시뮬 이미지 경로 → `~/dev_ws/test_data/sample_frames`. **새 설치 없음.** 빌드: `colcon build --packages-select automato_interfaces dg_control dg_sim`. 테스트: `src/dg_sim/test/test_e2e.py`, `src/dg_control/test/test_ai_switch.py` (4건 통과) |
| 2026-07-08 | 참고: 위 통합 테스트는 **`MultiThreadedExecutor`** 사용 가능 — 액션 goal을 **엄격히 1개씩 순차** 발행하고 액션 클라이언트를 전용 콜백그룹에 두면 "Two goals accepted with same ID" 미발생(HQ 오케스트레이션 방식). |
| 2026-07-18 | `mycobot_280_pick`(coord_to_goal_node, RP-107) 신규 패키지 추가 — `mycobot` 프로젝트에서 검증된 MoveIt2 목표 플래닝 노드를 이관. 원본에 있던 `yolo_d435_detector_node`는 제외(같은 역할을 `dg_ai_service`가 대체). 서드파티 의존성(`easy_handeye2`/`mycobot_ros2`/`pymoveit2`/`moveit_calibration`)은 벤더링 대신 `mycobot.repos`(vcstool)로 관리 |
| 2026-07-27 | **RP-101 [DCS] S2 E6 예냉실 하역(Unload 중계)**. `Unload.action` 신규(`/{robot_id}/unload`→`/ddagi/unload`, Goal `task_id`/`shake_delay_sec`, Result `result_code`(0성공/1파지실패/2중단), Feedback `phase`). DCS(`dcs_node.py`)에 Unload 중계 서버/클라이언트 추가 — RP-99 Harvest 중계와 동판: 예냉실 도킹 성공(`is_docked`) task 만 accept(게이트 소비), 취소 전파, Feedback 무수신 워치독(`unload_feedback_timeout_sec`=15s). 하역 실패도 result_code 가공 없이 중계(FAILED 판정은 ACS 몫). 시뮬: `ddagi_sim` Unload 서버(phase 5단계+`unload_mode` success/grip_fail/hang), `acs_sim` Unload 클라·`/acs_sim/start_unload` 트리거. **새 설치 없음.** 빌드: `colcon build --packages-select automato_interfaces dg_control dg_sim`. 테스트: `src/dg_sim/test/test_e2e.py` **18건 통과**(신규 Unload 5건: gate reject/success/grip_fail/cancel/watchdog) |
| 2026-07-30 | **RP-131 [DCS] H마커·반사테이프 정밀 도킹 중계**. 도킹 방식이 셋으로 늘어(charuco 휴면 / 바닥 H 마커=수확지·예냉실 / 반사테이프=충전소) DCS 에 `FloorDock`(`/{robot_id}/floor_dock`→`/ddago/floor_dock`)·`ReflectiveDock`(`/{robot_id}/reflective_dock`→`/ddago/reflective_dock`) 중계 추가. 액션 3종을 `DOCK_KINDS` 표 하나로 돌려 서버·클라이언트를 만들고, 중계는 같은 타입끼리 **필드 통째 복사**(`_copy_fields`)로 처리 — 방식마다 이름이 다른 앞뒤 오차(`final_error_m`/`final_wall_gap_m`/`final_gap_m`)와 feedback 거리(`distance_to_marker_m`/`distance_to_wall_m`)를 손으로 옮기지 않는다. 도킹 3종이 주행과 `_ddago_lock` **하나를 공유**(방식이 달라도 로봇에 동시 하달 안 됨), 성공 시 방식 무관하게 `is_docked` 게이트 오픈, `dock_result_timeout_sec`(180s) 공유. 시뮬: `ddago_sim` 에 두 서버 추가(방식별 phase 표 + 동시실행 최대치 `dock_concurrent_max` 기록), `acs_sim` 에 도킹 클라 3종 + `dock_method`(charuco/floor/reflective/auto — auto 는 실제 ACS `docking.method_for` 규칙). 대시보드: `dashboard.sh floor-dock`/`reflective-dock` 추가, **`E4` 버튼을 '순찰 종료 후 복귀 및 충전'으로 개칭 + 도킹 단발 하달 → 2단계 시나리오로 변경**(① 충전소까지 복귀 주행 `capture=false` → ② 도착 후 `ReflectiveDock`. `acs_sim` 에 `send_return_to_charger()`·`/acs_sim/start_return` 트리거·`return_waypoints`/`return_seg`/`charger_point` 파라미터 신설, `dashboard.sh return-dock`. 수확지 이동(S2 E2)과 같은 '이동→도킹' 구현을 label 로만 구분해 공유. charuco 는 휴면이라 버튼에서 빼고 `dashboard.sh dock` 수동 명령만 남김), `H마커 도킹 중계`(`e4floor`) 버튼 신설, 메시지 목록에 도킹 3종 18줄. **새 설치 없음**(`ReflectiveDock.action`·로봇 쪽 서버는 dev 에 이미 있음). 빌드: `colcon build --packages-select automato_interfaces dg_control dg_sim`. 테스트: `test_e2e.py` **신규 8건 통과**(방식별 성공 3(charuco 포함)/취소 2, auto 라우팅, 3종 직렬화, E4 복귀 2단계) + CLI 왕복 실측(`floor-dock`→`final_wall_gap_m=0.025`, `reflective-dock`→`final_gap_m=0.020`). **`acs_sim` 기본 `dock_method='auto'`** — 실제 ACS 규칙대로 지점 id 로 고르므로 S2 E2(수확지)·E6(예냉실) 시뮬은 H마커, E4 복귀는 반사테이프로 간다(charuco 는 명시 지정할 때만) |
| 2026-07-30 | **시뮬 수신 이미지 저장 위치 수정**. `dg_ai_sim.SAVE_DIR`·`acs_sim.ACS_SAVE_DIR` 기본값이 사라진 옛 클론 경로(`~/dev_ws/roscamp-rp108-navigate/equip/automato_ws/{dg_ai_recv,acs_recv}`)를 가리켜, 저장할 때마다 `os.makedirs` 가 그 빈 트리를 되살렸다(순찰 시나리오에서 `disease_percent>=5` 프레임이 나올 때마다 재생성). 기본값을 **`~/dg_sim_data/{dg_ai_recv,acs_recv}`** 로 변경 — 홈 한 곳에 모아 레포·워크스페이스를 더럽히지 않고 클론을 옮겨도 안 깨진다. 환경변수 `DG_AI_SIM_SAVE_DIR`·`ACS_SIM_SAVE_DIR` 로 덮어쓰는 동작은 그대로. **새 설치 없음.** 확인: `test_full_patrol_loop` 실행 후 `~/dg_sim_data` 에만 생성, 옛 경로 파일 수 변화 없음 |

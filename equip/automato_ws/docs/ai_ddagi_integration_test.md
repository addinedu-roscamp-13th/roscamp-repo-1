# AI Service + Ddagi 연동 테스트 실행법

DCS(dg_control)가 AI Service에 분석을 요청하면 토마토 성숙도/병해충 정보를
리턴받고, Ddagi(로봇팔)는 실물 헬스 상태를 텔레메트리로 발행하는지 확인하는
절차. **로봇팔 제어(pick 등)는 포함하지 않음** — telemetry_publisher는
서보 상태를 읽기만 하고 움직이지 않는다.

## 실행 위치 표기 규칙

| 라벨 | 실제 위치 |
| --- | --- |
| 🖥️ **[노트북]** | roscamp_ws 있는 개발 노트북 |
| 🤖 **[RPi]** | mycobot이 물려있는 원격 라즈베리파이 (`ssh <user>@<RPi IP>`) |

## 물리 구성

- **노트북**: `dg_ai_service`(AI Service), `dg_control`(DCS) 실행
- **RPi**: mycobot이 USB(`/dev/ttyUSB0`)로 직결돼 있어 `ddagi_control`의
  `telemetry_publisher`는 **반드시 RPi에서** 실행 (노트북에서는 시리얼
  포트 자체가 없어서 실행 불가)
- 노트북 ↔ RPi는 같은 `ROS_DOMAIN_ID`, 같은 LAN/WiFi로 토픽 공유
- 텔레메트리 토픽은 `/ddagi/telemetry` (robot_id 없는 bare 토픽 — RP-111.
  robot_id는 메시지 필드로 전달되고, DCS가 자기 robot_id로 매칭/필터링)

## 0. 사전 조건 체크리스트

- [ ] 🖥️🤖 양쪽 `echo $ROS_DOMAIN_ID` 값이 같음
- [ ] 🖥️🤖 같은 네트워크(LAN/WiFi)
- [ ] 🤖 `python3 -c "import pymycobot"` 이 에러 없이 통과
- [ ] 🤖 mycobot USB 연결 확인 (`ls /dev/ttyUSB0`)
- [ ] 🖥️ `DG_AI_MODEL_PATH` 환경변수에 YOLO 가중치(`.pt`) 경로 지정돼 있음

## 1. 코드 동기화 (RPi가 최신이 아닐 때)

```bash
# 🖥️ [노트북] — 브랜치 push (원격 브랜치가 아직 없으면)
git push -u origin <브랜치명>

# 🤖 [RPi]
cd ~/automato_ws
git fetch origin
git checkout <브랜치명>   # 처음이면: git checkout -b <브랜치명> origin/<브랜치명>
git pull
```

## 2. Ddagi 텔레메트리 기동 (🤖 RPi)

```bash
# 🤖 [RPi]
source /opt/ros/jazzy/setup.bash
cd ~/automato_ws
rm -rf build install log   # 이전 빌드 산출물과 아키텍처/버전 충돌 방지
colcon build --packages-select automato_interfaces ddagi_control
source install/setup.bash

ros2 run ddagi_control telemetry_publisher --ros-args -p robot_id:=dg_01
```

기동 로그에 `Ddagi 텔레메트리 퍼블리셔 시작: /ddagi/telemetry (1.0Hz)` 가
찍히면 정상.

## 3. AI Service 기동 (🖥️ 노트북)

```bash
# 🖥️ [노트북]
cd equip/automato_ws
export DG_AI_MODEL_PATH=~/Projects/Eval_Yolo/tomato_4cls_model.pt   # 본인 경로로 수정
PYTHONPATH=src/dg_ai_service:src/dg_control \
  .venv/bin/python -m dg_ai_service.analysis_server
```

(venv 미설치 시 [ai_service_dev_env_setup.md](ai_service_dev_env_setup.md) 1~2단계 먼저 진행)

## 4. DCS 기동 (🖥️ 노트북)

```bash
# 🖥️ [노트북]
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run dg_control dcs_node --ros-args \
  -p robot_id:=dg_01 \
  -p ai_target_file:="$PWD/dg_web/dg_ai_target.json"
```

`ai_target_file`을 꼭 넘길 것 — 코드 기본값이 다른 팀원 PC의 절대경로로
하드코딩돼 있어(`dcs_node.py`의 `DEFAULT_AI_TARGET_FILE`), 안 넘기면 그 파일을
못 찾고 조용히 `127.0.0.1:9100` 폴백으로 동작한다(로컬 테스트엔 결과적으로
문제없지만, 대시보드 real/sim 토글이 이 인스턴스엔 안 먹힌다).

## 5. 검증 (🖥️ 노트북)

**Ddagi 텔레메트리 수신 확인:**
```bash
ros2 topic echo /ddagi/telemetry
```
`servo_health` 7개(관절 6 + 그리퍼), `robot_id: dg_01` 이 1Hz로 찍히면 정상.
DCS를 거친 취합 결과는:
```bash
ros2 topic echo /automato/telemetry/fleet
```

**AI Service 응답 확인** (DCS 경유 없이 AI Service 자체를 바로 테스트):
```bash
# src/dg_control/test_images/ 에 jpg/png 넣어둔 뒤
PYTHONPATH=src/dg_ai_service:src/dg_control \
  .venv/bin/python -m dg_control.send_test_frame
```
rotten/disease 감지 시 같은 폴더에 `<파일명>_labeled.jpg` 생성됨.

## 트러블슈팅

| 증상 | 원인 | 확인 |
| --- | --- | --- |
| 노트북에서 `/ddagi/telemetry` 안 보임 | `ROS_DOMAIN_ID` 불일치 또는 다른 네트워크 | 🖥️🤖 양쪽 `echo $ROS_DOMAIN_ID` 비교 |
| RPi에서 `telemetry_publisher` 기동 시 시리얼 에러 | mycobot USB 연결 끊김/다른 프로세스가 포트 점유 중 | `ls /dev/ttyUSB0`, `harvest_server` 등 다른 노드가 동시에 안 떠 있는지 확인 |
| AI Service `MODEL_NOT_READY` | `DG_AI_MODEL_PATH` 미설정/경로 오류 | [ai_service_dev_env_setup.md](ai_service_dev_env_setup.md) 7단계 표 참고 |
| DCS 뜨는데 AI 응답 없음 | AI Service가 그 사이 안 떠 있거나 다른 포트 | `nc -zv 127.0.0.1 9100` |

# 260710 E0~E3 통합 테스트 준비 런북 (카메라 없이 더미 사진)

> 시나리오 1(주간 순찰) **E0~E3**를 실물 로봇 **1대(dg_01)** 로 통합 검증하기 전,
> 내 담당인 **Automato Control Service(ACS)** · **DdaGo Control Service** · **DB**를
> 어떻게 띄우고, **카메라가 없는 상황에서 병해충 더미 사진을 어떻게 흘려보내는지**
> 정리한 준비 절차서. 로봇 앞에서 **위에서 아래로** 따라 하면 된다.
>
> **이번 테스트의 대체물 (실물이 없는 부분).**
> - **카메라 → 더미 사진**: 실카메라(RP-85)가 없으니, 병해충 토마토 사진 1장을
>   `/dg_01/image_raw` 로 반복 발행하는 **더미 카메라 노드**를 로봇에서 띄운다.
>   `patrol_server`(RP-76)는 이 토픽을 **구독만** 하므로 코드 수정이 없다.
> - **병해충 사진이라 E3까지 실제로 탄다**: 이번엔 AI가 이 사진을 `disease_percent>=5`
>   로 판정할 수 있어, ACS의 **이미지 파일 저장 + 병해충 알림(E3)** 경로가 진짜로 작동한다.
>   (지난 mock 테스트는 `disease_percent=0` 이라 이 경로를 안 탔다.)
>
> **개념(초보자용).**
> - *Topic(토픽)* = 발행자가 보낼 때마다 구독자 콜백이 자동 실행되는 방송(스트리밍). 텔레메트리·카메라가 토픽.
> - *Service(서비스)* = 요청 1건 → 응답 1건인 함수 호출식 통신. `analyze_frame`(로봇→HQ), `save_detection`(HQ→ACS)이 서비스.
> - *Action(액션)* = 주행처럼 오래 걸리는 작업을 Goal로 보내고 Feedback/Result를 받는 통신. 순찰 하달이 액션.
> - *Namespace(네임스페이스)* = 토픽/노드 이름 앞의 접두어(`/dg_01/...`). 로봇마다 이름이 안 겹치게 한다.
> - *QoS* = 발행/구독의 신뢰성 정책. 안 맞으면 데이터가 아예 안 흐른다(아래 각 항목의 QoS를 맞출 것).

---

## 0. 무엇을 준비하나 (E0~E3 매핑)

| 단계 | 흐름 | 내 담당이 띄우는 것 | 팀원 의존 |
| --- | --- | --- | --- |
| **E0** | 로봇 상태 상시 스트리밍 → 취합 → 대시보드 | `telemetry_publisher`(로봇), `fleet_telemetry_aggregator`(ACS) | ACS가 `/{robot_id}/telemetry`를 로봇별 구독해 취합(RP-114) |
| **E1** | 순찰 접수 → 로봇 선정 → 하달 → 주행 | ACS API + `patrol_server`(로봇) | HQ가 `/dg_01/patrol` 받아 로봇에 중계 |
| **E2** | 도착 촬영 → 분석요청 → 탐지 저장 | `patrol_server`(로봇) + ACS `save_detection` + DB | HQ/AI가 `analyze_frame` 서버 |
| **E3** | 순찰현황 중계 + 병해충 알림 | ACS notify/alert 발신 | Web Service가 수신 |

> **전제.** 상세 인터페이스 계약(토픽/서비스/액션 이름·타입·QoS·API)은 Confluence
> **"260710 통합 테스트 전 인터페이스 합의"** 문서를 먼저 팀원과 맞춘 뒤 이 런북을 실행한다.

---

## 1. 실행 위치 표기

| 라벨 | 실제 위치 | 이번에 띄우는 것 |
| --- | --- | --- |
| 🤖 **[로봇 PC]** | dg_01 로봇의 RPi5 (`ssh pinky@<로봇IP>`) | 로봇 드라이버, Nav2, `telemetry_publisher`, `patrol_server`, 더미 카메라 |
| 🖥️ **[로컬 PC]** | 관제/개발 노트북 | DB, ACS(`patrol_node`), `fleet_telemetry_aggregator`, 순찰 트리거 |

> **네트워크 전제.** 두 기기가 **같은 무선망 + 같은 `ROS_DOMAIN_ID`** 여야 서로 보인다.
> 확인: 양쪽에서 `echo $ROS_DOMAIN_ID` 값이 같은지(비어 있으면 둘 다 0). 다르면
> `export ROS_DOMAIN_ID=<같은값>` 후 그 터미널에서 여는 모든 노드를 그 값으로 통일.

---

## 2. 사전 준비

### 2-1. 인터페이스 + 패키지 빌드 (🤖 로봇 / 🖥️ 로컬 — 양쪽 모두)

`automato_interfaces`(메시지 타입)와 `ddago_control`(로봇 노드)이 양쪽에 빌드돼 있어야 한다.
특히 `SaveDetection.srv`의 신규 필드(`disease_image` / `disease_image_encoding`)가 포함된
**최신 커밋**으로 팀 전원이 재빌드해야 서비스가 연결된다.

```bash
# 🤖 [로봇 PC] / 🖥️ [로컬 PC] — 양쪽 모두
cd ~/roscamp-repo-1/equip/automato_ws
colcon build --packages-select automato_interfaces ddago_control
source install/setup.bash
# 확인 (신규 필드가 보이는지)
ros2 interface show automato_interfaces/srv/SaveDetection
ros2 interface show automato_interfaces/action/Patrol
```

> **왜 양쪽 다?** ROS2 통신은 메시지 "타입"이 양쪽에 동일하게 빌드돼 있어야 잡힌다.
> 한쪽만 새 필드로 빌드하면 타입 불일치로 연결이 안 된다.

### 2-2. DB 기동 + 마이그레이션 (🖥️ 로컬 PC)

```bash
# 🖥️ [로컬 PC]
cd ~/roscamp-repo-1/services/database
docker compose up -d && docker compose ps        # STATUS 가 healthy 인지 확인
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
alembic upgrade head                             # 스키마(0001) + 시드(robots/waypoints, corridors) 적용
python smoke_check.py                            # 연결 OK 확인
```

> 시드로 **`robots`(dg_01~03)** 와 **`waypoints`(id 1~19, 실제 맵 좌표)**, **`corridors`(통로)**
> 가 들어간다. `corridors`가 비어 있으면 순찰 이동이 전부 skip될 수 있으니 꼭 적용.

> **task 는 수동으로 만들지 않는다.** 순찰을 ACS 접수 API로 트리거하면 ACS가 `task`와
> `task_paths`를 **자동 생성**한다([`accept_patrol_task`](../automato_control_service/automato_db.py):
> tasks INSERT → is_patrol_point 웨이포인트로 task_paths 복사 → snapshot → IN_PROGRESS).
> task_id는 4절 API 응답에서 받아 DB 확인에 쓴다.

### 2-3. 더미 병해충 사진 준비 (🖥️ 로컬 PC → 🤖 로봇 PC)

무거운 raw 이미지가 네트워크로 새지 않게, 더미 카메라는 **로봇 로컬에서** 돌린다(팀 원칙).
그래서 사진을 로봇으로 먼저 복사한다.

```bash
# 🖥️ [로컬 PC] → 🤖 [로봇] 으로 사진 복사
scp "/home/ane/Documents/1팀자료/disease_tomato_example.jpg" \
    pinky@<로봇IP>:/home/pinky/disease_tomato_example.jpg
```

### 2-4. 더미 카메라 스크립트 (🤖 로봇 PC)

로봇의 `~/dummy_camera.py` 로 저장. 사진 1장을 `sensor_msgs/Image`(bgr8)로 반복 발행한다.
`patrol_server`가 sensor QoS(best_effort)로 구독하므로 **같은 프로파일**로 발행해 매칭을 확실히 한다.

```python
#!/usr/bin/env python3
"""더미 카메라 — 실물 카메라 없이 JPEG 1장을 sensor_msgs/Image 로 반복 발행."""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class DummyCamera(Node):
    def __init__(self):
        super().__init__('dummy_camera')
        self.declare_parameter('topic', '/dg_01/image_raw')   # patrol 구독 토픽과 동일해야 함
        self.declare_parameter('image_path', '/home/pinky/disease_tomato_example.jpg')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('width', 640)                  # 실카메라급 해상도(대역폭/CPU)
        self.declare_parameter('height', 480)

        topic = self.get_parameter('topic').value
        path = self.get_parameter('image_path').value
        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)

        img = cv2.imread(path)                                # BGR8 로 로드
        if img is None:
            self.get_logger().error(f'이미지를 못 읽음: {path} — 경로 확인!')
            sys.exit(1)
        self.frame = cv2.resize(img, (w, h))

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
        self.get_logger().info(f'더미 카메라 시작 → {topic}, {w}x{h}, src={path}')

    def tick(self):
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        rclpy.spin(DummyCamera())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

의존성(로봇에 없으면): `sudo apt install ros-jazzy-cv-bridge python3-opencv`

### 2-5. ACS 환경변수 확인 (🖥️ 로컬 PC) — E3(병해충) 대비

병해충 사진이라 `disease_percent>=5`면 ACS가 **이미지 파일 저장 + 알림 발송**까지 실제로 한다.

| 변수 | 기본값 | 이번에 신경 쓸 점 |
| --- | --- | --- |
| `DETECTION_IMAGE_ROOT` | `~/automato_detections` | 이 경로가 **쓰기 가능**해야 이미지 저장됨 |
| `AUTOMATO_WEB_SERVICE_URL` | `http://localhost:8100` | Web Service를 붙이면 실제 주소로. 안 띄우면 alert는 3회 재시도 후 실패 로그(순찰엔 무해) |

---

## 3. 기동 순서

각각 **별도 터미널**. 매 터미널에서 먼저 워크스페이스를 소싱한다.

```bash
# (🤖 로봇 / 🖥️ 로컬 공통 준비 — 각 터미널)
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
```

### ① DB — 🖥️ 로컬 (2-2에서 기동됨, 확인만)

```bash
cd ~/roscamp-repo-1/services/database && docker compose ps    # healthy
```

### ② ACS(patrol_node) — 🖥️ 로컬

`patrol_node` 한 프로세스에 **순찰 API(:8200) + 텔레메트리 구독 + `save_detection` 서비스**가 함께 뜬다.

```bash
cd ~/roscamp-repo-1/services/automato_control_service
source .venv-acs/bin/activate
python3 -m automato_control_service.patrol_node
```

기대 로그:
```
순찰 제어 노드 준비: 구독 /automato/telemetry/fleet, 하달 /<robot_id>/patrol ...
탐지 저장 서비스 준비: /automato/save_detection
Automato Control Service (순찰) HTTP API → http://0.0.0.0:8200
```

확인:
```bash
ros2 service list | grep save_detection      # /automato/save_detection
curl -s http://localhost:8200/health         # {"ok":true,...}
```

### ③ Fleet 취합(E0 대시보드 발행) — 🖥️ 로컬 (선택)

```bash
ros2 run automato_control_service fleet_telemetry_aggregator
#  구독 /{robot_id}/telemetry (로봇 수만큼) → 발행 /automato/dashboard/fleet_telemetry (1Hz)
#  DG(dg_control) 이전이 끝나기 전까지는 옛 /automato/telemetry/fleet 도 함께 구독한다.
#  이전 완료 후: --ros-args -p legacy_input:=false
```

### ④ 로봇 드라이버 + Nav2 — 🤖 로봇

`telemetry_publisher`가 읽는 소스(odom/amcl_pose/battery/us_sensor/nav 상태)와 `patrol_server`가
쓰는 Nav2(`navigate_to_pose`)가 **`/dg_01` 네임스페이스**로 올라와야 한다(런처는 로봇 셋업에 맞게).

```bash
# 예시 — 실제 런처/인자는 로봇 환경에 맞춰 조정
ros2 launch pinky_bringup bringup.launch.py namespace:=/dg_01
ros2 launch pinky_navigation navigation.launch.py namespace:=/dg_01
```

확인:
```bash
ros2 action list | grep navigate_to_pose     # /dg_01/navigate_to_pose
ros2 topic list | grep -E "/dg_01/(odom|amcl_pose|battery|us_sensor)"
```

### ⑤ 텔레메트리(E0) — 🤖 로봇

```bash
ros2 launch ddago_control ddago_telemetry.launch.py robot_id:=dg_01
```

확인(🖥️ 로컬에서):
```bash
ros2 topic echo /dg_01/ddago/telemetry --once   # robot_id/nav_status/battery/x,y 등이 채워지는지
```

> ⑤ 로그에 `아직 수신되지 않은 소스: ...` 경고가 계속 뜨면 ④ 드라이버/Nav2의 해당 토픽이
> `/dg_01` 네임스페이스로 안 올라온 것 — 그 필드는 0으로 발행된다.

### ⑥ Patrol 서버(E1 받는 쪽/E2) — 🤖 로봇

```bash
ros2 launch ddago_control ddago_patrol.launch.py \
    robot_id:=dg_01 \
    camera_topic:=image_raw
```

기대 로그:
```
Patrol 서버 준비됨: robot_id=dg_01 → 서버 ddago/patrol, Nav2=navigate_to_pose,
                     카메라=image_raw, 분석=/dg/analyze_frame
```

확인(🖥️ 로컬에서):
```bash
ros2 action list | grep patrol               # /dg_01/ddago/patrol
```

### ⑦ 더미 카메라 — 🤖 로봇

```bash
python3 ~/dummy_camera.py --ros-args \
    -p topic:=/dg_01/image_raw \
    -p image_path:=/home/pinky/disease_tomato_example.jpg
```

확인(프레임이 흐르는지):
```bash
ros2 topic hz /dg_01/image_raw                        # ~5Hz
ros2 topic echo /dg_01/image_raw --field encoding --once   # bgr8
```

### ⑧ 팀원 컴포넌트(HQ/AI) — 🖥️ 로컬/팀원

E0 취합(`/automato/telemetry/fleet`), E1 순찰 중계(`/dg_01/patrol`→로봇), E2 `analyze_frame` 서버 +
`save_detection` 호출을 **팀원 HQ/AI가 제공**한다. 준비됐는지 확인:
```bash
ros2 topic list | grep /automato/telemetry/fleet     # E0 취합 발행 중?
ros2 action list | grep /dg_01/patrol                # E1 HQ 순찰 서버 있음?
ros2 service list | grep /dg/analyze_frame           # E2 AI 분석 서버 있음?
```

> HQ 취합이 아직 없으면 임시 스탠드인으로 대체 가능:
> `ros2 run automato_control_service fleet_aggregator` (로봇별 telemetry → FleetTelemetry).

---

## 4. 트리거 & E0~E3 확인 포인트

**E1 순찰 시작** — 🖥️ 로컬. ACS 접수 API로 트리거한다. ACS가 `task`·`task_paths`를 **자동 생성**하고
HQ로 하달한다. 로봇이 "가용"으로 판정돼야 접수되므로 **E0 텔레메트리가 흐르고(HQ 취합)**,
**팀원 HQ의 `/dg_01/patrol` 중계**가 있어야 한다.

```bash
curl -s -X POST http://localhost:8200/internal/v1/tasks/patrol \
  -H 'Content-Type: application/json' \
  -d '{"robot_selection":"manual","robot_id":"dg_01"}'
#  → {"task_id": 42, "assigned_robot_id":"dg_01", "status":"ACCEPTED", ...}

# 응답의 task_id 를 아래 DB 확인에 사용
export TASK=<응답 JSON의 task_id>
```

| 단계 | 확인 포인트 |
| --- | --- |
| **E0** | 🖥️ `ros2 topic echo /dg_01/ddago/telemetry` 값 채워짐 / (릴레이 시) `/automato/dashboard/fleet_telemetry` 흐름 |
| **E1** | 🖥️ ACS 200 응답에 `task_id` / 🤖 Patrol 주행 Feedback → 도착 → `result_code=0` |
| **E2** | 🤖 `analyze_frame 요청 전송...수락됨` / 🖥️ ACS `detection <id> 저장 완료` / DB `detection_logs` 1건 + `task_paths` 해당 wp `is_visited=TRUE` |
| **E3** | disease_percent>=5면 🖥️ ACS `병해충 이미지 저장: ...` + `disease alert 발송`(수신 서버 있으면 OK) |

**DB 확인** — 🖥️ 로컬:
```bash
cd ~/roscamp-repo-1/services/database
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT detection_id, task_id, robot_id, waypoint_id, disease_percent, disease_image_path, detected_at
    FROM detection_logs ORDER BY detection_id DESC LIMIT 3;"
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT task_id, waypoint_id, is_visited FROM task_paths WHERE task_id=$TASK ORDER BY point_index;"
```

---

## 5. 트러블슈팅

| 증상 | 원인 후보 | 조치 |
| --- | --- | --- |
| `ros2 action list`에 `/dg_01/ddago/patrol` 없음 | ⑥ Patrol 서버 미기동 / 도메인 불일치 | ⑥ 로그 확인, 양쪽 `ROS_DOMAIN_ID` 일치 |
| Patrol 로그 `Nav2 navigate_to_pose 서버 없음 → 실패(1)` | ④ Nav2가 `/dg_01`로 안 뜸 | ④를 `namespace:=/dg_01`로 기동 |
| Patrol 로그 `카메라 프레임 미수신 → 분석요청 스킵` | ⑦ 토픽 ≠ `camera_topic` | ⑥ `camera_topic:=image_raw`, ⑦ `topic:=/dg_01/image_raw` 일치 |
| `analyze_frame 서비스 미준비 → 스킵` | ⑧ AI 서버 미기동 | 팀원 AI 확인, `ros2 service list \| grep analyze_frame` |
| `save_detection` 응답 `success=false` | `task_id`/`waypoint_id`가 DB에 없음(FK 위반) | API 응답의 task_id가 유효한지, AI가 보낸 waypoint_id가 접수된 task_paths(is_patrol_point)에 있는지 확인 |
| `detection_logs`는 늘었는데 `is_visited` 그대로 | 해당 `task_paths` 행 없음(UPDATE 0행, 에러 아님) | AI가 보낸 waypoint_id가 그 task의 task_paths에 포함되는지(is_patrol_point=TRUE) 확인 |
| E0 텔레메트리 값이 전부 0 | ④ 로봇 드라이버/Nav2 소스 토픽 미발행 | ⑤ 경고 로그의 누락 소스 확인 |
| ACS `notify 실패(재시도 안 함)` 경고 | 수신 서버(Web) 미기동 | 순찰 비블로킹 확인용 — 무시 가능 |
| `병해충 이미지 저장 실패(...)` | `DETECTION_IMAGE_ROOT` 쓰기 불가 | 2-5 경로 권한 확인 |

---

## 부록. 참고 — 로봇 없이 로직만

순서·게이트·트랜잭션 등은 단위테스트로 이미 검증된다.
```bash
# ACS(RP-79)
cd ~/roscamp-repo-1/services/automato_control_service
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_detection_service.py -v
# DdaGo(RP-76)
cd ~/roscamp-repo-1/equip/automato_ws/src/ddago_control
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_patrol.py -v
```

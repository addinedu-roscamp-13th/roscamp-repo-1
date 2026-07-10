# RP-76 × RP-79 통합 — 실물 로봇 순찰→탐지저장 검증 런북

> 실물 로봇 **1대(dg_01)** 를 붙여, 순찰 핵심 루프의 로봇측(RP-76 DdaGo Patrol 서버)과
> 관제측(RP-79 ACS 탐지 저장)이 **한 줄로 이어져 동작**하는지 검증한다.
> 로봇이 waypoint에 도착 → 프레임 촬영 → HQ 분석요청 → ACS가 DB에 탐지 저장까지.
>
> **개념(초보자용).**
> - *Action(액션)* = 시간이 걸리는 작업(주행)을 Goal로 보내고 Feedback/Result를 받는 통신. 여기선 ACS/사람이 로봇에 "이 지점 가라"를 액션으로 하달한다.
> - *Service(서비스)* = 요청 1건 → 응답 1건인 함수 호출식 통신. 로봇→HQ 분석요청(`analyze_frame`), HQ→ACS 저장요청(`save_detection`)이 서비스다.
> - *Namespace(네임스페이스)* = 토픽/노드 이름 앞에 붙는 접두어(`/dg_01/...`). 로봇마다 이름이 안 겹치게 한다.
>
> **이번 테스트의 대체물 2가지 (실물이 아직 없는 부분).**
> - **카메라 → 더미**: RP-85(USB 카메라 bringup)가 아직이라, 카메라 대신 토마토 사진 1장을 `/dg_01/image_raw` 로 발행하는 **더미 카메라**를 로봇에서 띄운다. RP-76 코드는 토픽을 구독만 하므로 **코드 수정 없음**.
> - **HQ AI → 목(mock)**: 실제 AI/HQ 대신, `analyze_frame` 을 받아 `save_detection` 을 호출해 주는 **HQ AI 목**을 관제 PC에서 띄운다. RP-76과 RP-79는 서로 직접 통신하지 않고 이 목이 다리 역할을 한다.
>
> **범위 밖(이번 미검증).** 병해충 알림(alert)·이미지 저장은 뺀다. HQ 목이 `disease_percent=0`(임계값 5 미만)으로 보내 ACS가 알림/이미지 경로를 아예 타지 않게 한다. 순찰 현황 중계(notify)는 ACS가 발송을 시도하지만 수신 서버를 띄우지 않으므로 "실패(재시도 안 함)" 경고만 남는다 — **정상이며 판정과 무관**(순찰 비블로킹 확인용).

---

## 0. 무엇을 검증하나 (DoD 매핑)

### 데이터 흐름

```
🤖 로봇 dg_01                                    🖥️ 관제 PC
[더미 카메라] ──/dg_01/image_raw──▶ [Patrol 서버(RP-76)]
                                        │ ① 도착·정지·프레임 grab
                                        ▼ ② /dg/analyze_frame (Service)
                                   ─────────────────────▶ [HQ AI 목]
                                   accepted+request_id ◀──    │ ③ 가짜 분석값
                                                              ▼ ④ /automato/save_detection
                                                         [ACS(RP-79)]
                                                          ⑤ detection_logs INSERT
                                                             + task_paths.is_visited=TRUE
▶ 트리거(관제 PC): ros2 action send_goal /dg_01/ddago/patrol ...
```

### 검증 항목

| # | 검증 항목 | 기대 | 확인 방법 |
| --- | --- | --- | --- |
| 76-1 | 단일 waypoint 주행 | (x,y) 인접 노드까지 Nav2 주행 → `result_code=0` | `send_goal` 결과 |
| 76-2 | Feedback 발행 | 주행 중 `current_waypoint_id`,`current_x/y/yaw` | `send_goal --feedback` |
| 76-3 | 도착 촬영·분석요청 | 정지 후 프레임 grab → `analyze_frame` 호출 | 🤖 로그 `analyze_frame 수락됨 ... request_id=` |
| 76-4 | 연속 순찰 | 도착→촬영→반환→다음 goal 끊김 없음 | S2 연속 goal |
| 76-5 | 카메라 미수신 비블로킹 | 프레임 없으면 분석 스킵, **주행은 성공** | S3(더미 끔) |
| 다리 | 프레임이 HQ까지 전달 | 목이 실제 이미지 크기 수신 | 목 로그 `[AnalyzeFrame] 수신 ... image WxH` |
| 79-1 | DB 저장(트랜잭션) | `detection_logs` INSERT + `task_paths.is_visited=TRUE`가 **같은 트랜잭션** | `psql` 조회 |
| 79-2 | 저장 성공 응답 | `save_detection` 응답 `success=true` + message | 목 로그 `ACS 응답 success=True` |
| 79-3 | 저장 실패 응답 | 없는 task_id면 `success=false`+사유(FK 위반) | S4(선택) |

> 참고: 로봇 없이 로직만 보는 단위테스트는 이미 통과 상태다(RP-76 `test_patrol.py`, RP-79 `test_detection_service.py`). 이 문서는 **실물 통합**만 다룬다.

---

## 1. 실행 위치 표기

| 라벨 | 실제 위치 | 이번에 띄우는 것 |
| --- | --- | --- |
| 🤖 **[로봇 dg_01]** | dg_01 로봇의 RPi5 (`ssh pinky@<로봇IP>`) | Nav2, Patrol 서버(RP-76), 더미 카메라 |
| 🖥️ **[관제 PC]** | 관제/개발 노트북 | DB, ACS(RP-79), HQ AI 목, `send_goal` |

> **네트워크 전제.** 두 기기가 **같은 무선망 + 같은 `ROS_DOMAIN_ID`** 여야 액션/서비스가 서로 보인다.
> 확인: 양쪽에서 `echo $ROS_DOMAIN_ID` 값이 같은지(비어 있으면 둘 다 0). 다르면
> `export ROS_DOMAIN_ID=<같은값>` 후 모든 터미널을 그 값으로 통일.

---

## 2. 사전 준비

### 2-1. 인터페이스 빌드 (양쪽 기기)

`Patrol`/`WaypointGoal`/`AnalyzeFrame`/`SaveDetection` 타입이 로봇·관제 PC 양쪽에 빌드돼 있어야 한다.

```bash
# 🤖 [로봇 dg_01]  /  🖥️ [관제 PC]  — 양쪽 모두
cd ~/roscamp-repo-1/equip/automato_ws
colcon build --packages-select automato_interfaces ddago_control
source install/setup.bash
# 확인
ros2 interface show automato_interfaces/action/Patrol
ros2 interface show automato_interfaces/srv/AnalyzeFrame
ros2 interface show automato_interfaces/srv/SaveDetection
```

### 2-2. DB 기동 + 마이그레이션 (관제 PC)

```bash
# 🖥️ [관제 PC]
cd ~/roscamp-repo-1/services/database
docker compose up -d && docker compose ps        # STATUS 가 healthy 인지 확인
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
alembic upgrade head                             # 스키마(0001) + 시드(0002,0003) 적용
python smoke_check.py                            # 연결 OK 확인
```

> 시드(0002)로 **`robots`(dg_01~03)** 와 **`waypoints`(id 1~19, 실제 맵 좌표)** 는 이미 들어간다.
> 우리가 추가로 넣을 것은 `tasks` 1개와 그 `task_paths` 뿐(아래 2-3).

### 2-3. 테스트용 시드 — task + task_paths (관제 PC)

`detection_logs` INSERT는 `task_id`(→`tasks`)·`waypoint_id`(→`waypoints`)·`robot_id`(→`robots`)를 FK로
참조한다. `waypoints`/`robots`는 시드에 있으니 **`tasks` 1개와 방문할 `task_paths` 행만** 만든다.
(`task_paths` 행이 있어야 `is_visited=TRUE` 로 바뀌는 걸 눈으로 확인할 수 있다.)

```bash
# 🖥️ [관제 PC]  (services/database 에서)
docker compose exec postgres psql -U robot8 -d automatodb
```

```sql
-- PATROL task 1개 생성 + 방문 대상 순찰점(5,6,10) 의 task_paths 를 한 번에 만든다.
-- 반환된 task_id 를 이후 명령의 $TASK 로 쓴다.  (waypoint_id 5,6,10 = 0002 시드의 순찰점)
WITH t AS (
  INSERT INTO tasks (task_type, status) VALUES ('PATROL','IN_PROGRESS')
  RETURNING task_id
)
INSERT INTO task_paths (task_id, waypoint_id, point_index, is_visited)
SELECT t.task_id, v.wp, v.idx, FALSE
FROM t, (VALUES (5,1),(6,2),(10,3)) AS v(wp, idx)
RETURNING task_id, waypoint_id, point_index;
```

출력에 나온 `task_id` 를 기억한다(예: `3`). 아래에서 `TASK=3` 로 export 해서 쓴다.

```bash
# 🖥️ [관제 PC]  — 위에서 확인한 값으로
export TASK=3      # ← psql 이 반환한 task_id
```

> 각 순찰점의 실제 맵 좌표(0002 시드): **wp5=(0.66, -0.018)**, **wp6=(0.389, -0.008)**, **wp10=(0.314, 0.261)**.
> `send_goal` 의 `x,y` 는 이 값을 쓴다(로봇이 실제로 갈 수 있는 지점이어야 하므로).

### 2-4. 헬퍼 스크립트 2개

#### (a) 더미 카메라 — 🤖 로봇에서 실행

토마토 사진을 `/dg_01/image_raw` 로 발행한다. **실카메라처럼 로봇 로컬에서** 돌려
raw 이미지가 네트워크로 새지 않게 한다(팀 원칙: 무거운 영상은 토픽으로 안 흘림).
로봇의 `~/dummy_camera.py` 로 저장:

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
        self.declare_parameter('image_path', '/home/pinky/dummy_tomato.jpg')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('width', 640)                  # 실카메라급 해상도로 축소(대역폭/CPU)
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
        # patrol 이 sensor QoS(best_effort)로 구독 → 같은 프로파일로 발행해 매칭을 확실히 한다.
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

#### (b) HQ AI 목 — 🖥️ 관제 PC에서 실행

`analyze_frame` 을 받아 `accepted` 를 돌려주고, 가짜 분석값으로 `save_detection` 을 호출한다.
관제 PC의 `~/hq_ai_mock.py` 로 저장:

```python
#!/usr/bin/env python3
"""HQ AI 서비스 목 — RP-76(DdaGo) 과 RP-79(ACS) 사이를 잇는 가짜 노드.
 (1) AnalyzeFrame 서버  /dg/analyze_frame        : 프레임 수신 → accepted+request_id 즉시 반환
 (2) SaveDetection 클라이언트 /automato/save_detection : 가짜 분석값을 ACS 로 전달
disease_percent 는 파라미터(기본 0 = 병해충 미발동). 5 이상으로 올리면 이미지/알림 경로도 탄다."""
import rclpy
from rclpy.node import Node
from automato_interfaces.srv import AnalyzeFrame, SaveDetection


class HqAiMock(Node):
    def __init__(self):
        super().__init__('hq_ai_mock')
        self.declare_parameter('analyze_frame_service', '/dg/analyze_frame')
        self.declare_parameter('save_detection_service', '/automato/save_detection')
        self.declare_parameter('robot_id', 'dg_01')
        self.declare_parameter('ripe_percent', 60)
        self.declare_parameter('unripe_percent', 5)
        self.declare_parameter('rotten_percent', 10)
        self.declare_parameter('disease_percent', 0)          # 0=병해충 미발동(이번 범위)

        af = self.get_parameter('analyze_frame_service').value
        self.sd = self.get_parameter('save_detection_service').value
        self.counter = 0

        self.srv = self.create_service(AnalyzeFrame, af, self.on_analyze)
        self.cli = self.create_client(SaveDetection, self.sd)
        self.get_logger().info(f'HQ AI mock 시작 → 서버 {af}, 클라이언트 {self.sd}')

    def on_analyze(self, request, response):
        self.counter += 1
        rid = f'req-{request.waypoint_id}-{self.counter}'
        self.get_logger().info(
            f'[AnalyzeFrame] 수신 task={request.task_id} wp={request.waypoint_id} '
            f'image={request.image.width}x{request.image.height}'
            f'({request.image.encoding}) → {rid}')
        response.accepted = True                              # 즉시 응답(fire-and-forget)
        response.request_id = rid
        self.forward(request.image, request.task_id, request.waypoint_id)
        return response

    def forward(self, image_msg, task_id, waypoint_id):
        if not self.cli.service_is_ready():
            self.get_logger().warn(f'[SaveDetection] {self.sd} 미준비 → 스킵 (ACS 확인)')
            return
        req = SaveDetection.Request()
        req.task_id = task_id
        req.waypoint_id = waypoint_id
        req.robot_id = self.get_parameter('robot_id').value
        req.ripe_percent = self.get_parameter('ripe_percent').value
        req.unripe_percent = self.get_parameter('unripe_percent').value
        req.rotten_percent = self.get_parameter('rotten_percent').value
        disease = self.get_parameter('disease_percent').value
        req.disease_percent = disease
        req.disease_image_encoding = ''
        if disease >= 5:                       # 병해충 시나리오일 때만 이미지 첨부(지연 임포트)
            from cv_bridge import CvBridge
            import cv2
            cv = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', cv)
            if ok:
                req.disease_image = buf.tobytes()
                req.disease_image_encoding = 'jpeg'
        self.get_logger().info(f'[SaveDetection] 호출 disease={disease} wp={waypoint_id}')
        self.cli.call_async(req).add_done_callback(self.on_done)

    def on_done(self, future):
        try:
            r = future.result()
            self.get_logger().info(
                f'[SaveDetection] ACS 응답 success={r.success} msg="{r.message}"')
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'[SaveDetection] 호출 실패: {e}')


def main():
    rclpy.init()
    try:
        rclpy.spin(HqAiMock())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2-5. 더미 토마토 이미지 준비

병해충/부패가 보이는 토마토 사진(또는 아무 토마토 사진)을 준비해 **로봇**에 둔다.
(이번 범위에선 `disease_percent=0` 이라 사진 내용은 판정에 영향 없음 — 흐름 확인용.)

```bash
# 🖥️ [관제 PC] → 🤖 [로봇] 로 사진 복사 (관제 PC에 /home/ane/dummy_tomato.jpg 가 있다고 가정)
scp /home/ane/dummy_tomato.jpg pinky@<로봇IP>:/home/pinky/dummy_tomato.jpg
```

---

## 3. 기동 순서

아래 순서대로 **각각 별도 터미널**에서 띄운다. 매 터미널에서 먼저 워크스페이스를 소싱한다.

```bash
# (모든 터미널 공통 준비)
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
```

### ① DB — 🖥️ 관제 PC (2-2에서 이미 기동됨, 확인만)

```bash
cd ~/roscamp-repo-1/services/database && docker compose ps    # healthy
```

### ② ACS(RP-79) — 🖥️ 관제 PC

```bash
cd ~/roscamp-repo-1/services/automato_control_service
source .venv-acs/bin/activate          # RP-78/79 문서에서 만든 ACS 전용 venv
python3 -m automato_control_service.patrol_node
```

기대 로그:
```
순찰 제어 노드 준비: 구독 /automato/telemetry/fleet, 하달 /<robot_id>/patrol ...
탐지 저장 서비스 준비: /automato/save_detection
Automato Control Service (순찰) HTTP API → http://0.0.0.0:8200
```

🖥️ 다른 터미널에서 서비스가 보이는지 확인:
```bash
ros2 service list | grep save_detection      # /automato/save_detection
ros2 service type /automato/save_detection   # automato_interfaces/srv/SaveDetection
```

### ③ HQ AI 목 — 🖥️ 관제 PC

```bash
python3 ~/hq_ai_mock.py
#  → "HQ AI mock 시작 → 서버 /dg/analyze_frame, 클라이언트 /automato/save_detection"
```

확인:
```bash
ros2 service list | grep analyze_frame       # /dg/analyze_frame
```

### ④ Nav2 — 🤖 로봇 dg_01

`pinky_navigation` 의 localization+navigation 을 **`/dg_01` 네임스페이스**로 띄워 `navigate_to_pose`
액션과 map 이 살아 있게 한다(런처 이름은 로봇 셋업에 맞게).

```bash
# 예시 — 실제 런처/인자는 로봇 환경에 맞춰 조정
ros2 launch pinky_navigation navigation.launch.py namespace:=/dg_01
```

확인:
```bash
ros2 action list | grep navigate_to_pose     # /dg_01/navigate_to_pose
```

### ⑤ Patrol 서버(RP-76) — 🤖 로봇 dg_01

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

🖥️ 관제 PC에서 확인:
```bash
ros2 action list | grep patrol               # /dg_01/ddago/patrol
ros2 action info /dg_01/ddago/patrol -t      # automato_interfaces/action/Patrol
```

### ⑥ 더미 카메라 — 🤖 로봇 dg_01

```bash
python3 ~/dummy_camera.py --ros-args \
    -p topic:=/dg_01/image_raw \
    -p image_path:=/home/pinky/dummy_tomato.jpg
```

확인(프레임이 실제로 흐르는지):
```bash
ros2 topic hz /dg_01/image_raw               # ~5Hz
ros2 topic echo /dg_01/image_raw --field encoding --once   # bgr8
```

> ⑤ Patrol 로그에 `카메라 프레임 미수신` 경고가 계속 뜨면 ⑥ 토픽 이름이 `camera_topic` 과
> 어긋난 것이다. 둘 다 `/dg_01/image_raw` 인지 확인.

---

## 4. 시나리오

각 시나리오: **목적 / 실행 / 관찰(기대) / 판정 [ ]**. `$TASK` 는 2-3에서 만든 task_id.

### S1. 단일 waypoint — 전 체인 스모크 테스트 [76-1~3, 다리, 79-1·2]

**목적:** goal 1개로 주행→촬영→분석요청→ACS 저장까지 한 줄로 흐르는지 본다.

**실행** — 🖥️ 관제 PC (wp5 좌표):
```bash
ros2 action send_goal --feedback /dg_01/ddago/patrol \
    automato_interfaces/action/Patrol \
    "{task_id: $TASK, waypoint: {waypoint_id: 5, x: 0.66, y: -0.018}}"
```

**관찰(기대):**
- 🤖 Patrol: 주행 중 Feedback(`current_waypoint_id/x/y/yaw`) → 도착 → `analyze_frame 요청 전송 task=$TASK waypoint=5` → `analyze_frame 수락됨 waypoint=5 request_id=req-5-1`
- 🖥️ HQ 목: `[AnalyzeFrame] 수신 task=$TASK wp=5 image=640x480(bgr8) → req-5-1` → `[SaveDetection] 호출 disease=0 wp=5` → `[SaveDetection] ACS 응답 success=True msg="detection <id> 저장 완료"`
- 🖥️ ACS: (notify 실패 경고 1줄 — 정상)
- `send_goal` 최종: `result_code: 0`, `message: arrived`

**DB 확인** — 🖥️ 관제 PC:
```bash
cd ~/roscamp-repo-1/services/database
# 탐지 1건 저장됐는지 (disease_image_path 는 NULL — 이번 범위)
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT detection_id, task_id, robot_id, waypoint_id, ripe_percent, disease_percent,
         disease_image_path, detected_at
    FROM detection_logs ORDER BY detection_id DESC LIMIT 3;"
# 같은 트랜잭션으로 방문표시됐는지 (wp5 만 is_visited=TRUE)
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT task_id, waypoint_id, point_index, is_visited
    FROM task_paths WHERE task_id=$TASK ORDER BY point_index;"
```

**판정:**
- [ ] `result_code=0` + Feedback 정상
- [ ] 🤖 로그에 `analyze_frame 수락됨 ... request_id=`
- [ ] 🖥️ 목 로그에 `image=640x480` (프레임이 실제로 HQ까지 감)
- [ ] 목 로그 `ACS 응답 success=True`
- [ ] `detection_logs` 에 wp5 행 1건 추가(robot_id=dg_01, task_id=$TASK)
- [ ] `task_paths` 에서 **wp5 만 `is_visited=TRUE`**, wp6·wp10 은 아직 FALSE

### S2. 연속 순찰 — 끊김 없음 [76-4, 79-1]

**목적:** 도착→촬영→반환→다음 goal 루프가 끊기지 않고, 탐지가 waypoint마다 쌓이는지.

**실행** — 🖥️ 관제 PC (wp6 → wp10 연달아):
```bash
ros2 action send_goal /dg_01/ddago/patrol automato_interfaces/action/Patrol \
    "{task_id: $TASK, waypoint: {waypoint_id: 6, x: 0.389, y: -0.008}}"
ros2 action send_goal /dg_01/ddago/patrol automato_interfaces/action/Patrol \
    "{task_id: $TASK, waypoint: {waypoint_id: 10, x: 0.314, y: 0.261}}"
```

**관찰(기대):** 각 goal마다 도착 촬영 1회 + 목 로그 `ACS 응답 success=True` 1회.

**DB 확인:**
```bash
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT waypoint_id, count(*) FROM detection_logs
   WHERE task_id=$TASK GROUP BY waypoint_id ORDER BY waypoint_id;"
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT task_id, waypoint_id, is_visited
    FROM task_paths WHERE task_id=$TASK ORDER BY point_index;"
```

**판정:**
- [ ] 각 goal 도착 후 다음 goal 이 즉시 처리됨(반환 전 grab이라 사진 흔들림 없음)
- [ ] `detection_logs` 에 wp6·wp10 행이 각각 추가
- [ ] `task_paths` 에서 wp5·wp6·wp10 **모두 `is_visited=TRUE`**

### S3. 카메라 프레임 미수신 — 비블로킹 확인 [76-5]

**목적:** 카메라(더미)가 없어도 주행 자체는 성공하고 분석요청만 스킵되는지(순찰 루프 안 멈춤).

**실행:**
1. 🤖 ⑥ 더미 카메라 터미널을 `Ctrl+C` 로 **끈다**.
2. 🖥️ 관제 PC에서 아직 안 간 지점으로 goal 하달(예: wp5 재방문):
   ```bash
   ros2 action send_goal /dg_01/ddago/patrol automato_interfaces/action/Patrol \
       "{task_id: $TASK, waypoint: {waypoint_id: 5, x: 0.66, y: -0.018}}"
   ```

**관찰(기대):**
- 🤖 Patrol: `waypoint=5 도착했으나 카메라 프레임 미수신 → 분석요청 스킵(주행은 성공 처리)...`
- `send_goal` 최종: `result_code: 0` (주행은 성공)
- 🖥️ 목/ACS: `analyze_frame` 호출 자체가 없으므로 이번엔 새 저장 없음

**판정:**
- [ ] 프레임 없어도 `result_code=0` (주행 성공, 비블로킹)
- [ ] `analyze_frame` 호출 안 됨(목 로그에 새 `[AnalyzeFrame] 수신` 없음)

> 확인 후 ⑥ 더미 카메라를 다시 켠다.

### S4. (선택) 저장 실패 — success=false [79-3]

**목적:** 없는 task_id면 FK 위반으로 DB INSERT 실패 → 응답 `success=false`.
로봇 없이 `save_detection` 을 직접 불러 빠르게 본다.

**실행** — 🖥️ 관제 PC:
```bash
ros2 service call /automato/save_detection automato_interfaces/srv/SaveDetection \
 "{task_id: 999999, waypoint_id: 5, robot_id: 'dg_01',
   ripe_percent: 70, unripe_percent: 10, rotten_percent: 5, disease_percent: 0,
   disease_image: [], disease_image_encoding: ''}"
```

**판정:**
- [ ] 응답 `success: false`, `message` 에 DB 오류(FK 위반) 사유

---

## 5. 트러블슈팅

| 증상 | 원인 후보 | 조치 |
| --- | --- | --- |
| `ros2 action list` 에 `/dg_01/ddago/patrol` 없음 | Patrol 서버 미기동 / 도메인 불일치 | ⑤ 로그 확인, 양쪽 `ROS_DOMAIN_ID` 일치 |
| `Nav2 navigate_to_pose 서버 없음 → 실패(1)` | Nav2가 `/dg_01` 네임스페이스로 안 뜸 | ④ 를 `namespace:=/dg_01` 로 기동 |
| Patrol 로그 `카메라 프레임 미수신 → 분석요청 스킵` (원치 않을 때) | `camera_topic` ≠ 더미 토픽 | ⑤ `camera_topic:=image_raw`, ⑥ `topic:=/dg_01/image_raw` 일치 확인 |
| `analyze_frame 서비스 미준비 → 스킵` | ③ HQ 목 미기동 | ③ 기동, `ros2 service list \| grep analyze_frame` |
| 목 로그 `[SaveDetection] ... 미준비 → 스킵` | ② ACS 미기동 / 서비스 이름 불일치 | ② 로그 `탐지 저장 서비스 준비` 확인, 목의 `save_detection_service=/automato/save_detection` |
| 응답 `success=false` (정상 데이터인데) | `task_id`/`waypoint_id` 가 DB에 없음(FK 위반) | 2-3 시드의 `$TASK`·시드 waypoint(5/6/10) 사용 |
| `detection_logs` 는 늘었는데 `is_visited` 그대로 | 해당 `task_paths` 행이 없음(UPDATE 0행, 에러 아님) | 2-3에서 그 waypoint의 task_paths 행을 만들었는지 확인 |
| ACS `notify 실패(재시도 안 함)` 경고 | 수신 서버 미기동(의도됨) | **정상** — 이번 범위에서 무시(순찰 비블로킹 확인) |
| 목 로그 `image=0x0` 또는 프레임 큼 | 더미 해상도/토픽 문제 | ⑥ `width/height`(640x480) 확인, `ros2 topic hz` 로 흐름 확인 |

---

## 부록 A. 도착 시 순서(참고)

```
Nav2 도착 → 정지 → settle(~0.3s) → 최신 프레임 grab
          → (병렬) result_code=0 반환  &  /dg/analyze_frame 호출(응답 안 기다림)
```
프레임 grab 은 멤버변수 스냅샷이라 즉시 끝나므로 "프레임 확보 후 반환"이 보장된다
(반환 후 다음 goal 이 즉시 와도 촬영 흔들림 없음). — `patrol_server.py` 4절.

## 부록 B. 테스트 정리(cleanup)

테스트로 생성한 행을 지우려면(FK 순서: 자식 → 부모):
```sql
DELETE FROM detection_logs WHERE task_id = :TASK;
DELETE FROM task_paths     WHERE task_id = :TASK;
DELETE FROM tasks          WHERE task_id = :TASK;
```

## 부록 C. 로봇 없이 로직만 (참고)

순서·게이트·트랜잭션 등 로직은 단위테스트로 이미 검증된다.
```bash
# RP-79
cd ~/roscamp-repo-1/services/automato_control_service
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_detection_service.py -v
# RP-76
cd ~/roscamp-repo-1/equip/automato_ws/src/ddago_control
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_patrol.py -v
```

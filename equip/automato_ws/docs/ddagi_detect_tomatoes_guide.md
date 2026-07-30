# Ddagi ↔ DG AI Service: `/ai/detect_tomatoes` 연동 가이드

Ddagi Control Service(수확 루프 주관)가 라운드마다 AI Service에 토마토 검출을
요청하는 구간(시나리오2 E3)의 ROS2 서비스 스펙과, 호출 시 지켜야 할
규칙을 정리한 문서. 배경/전체 흐름은
[`시나리오 2 수확 및 예냉실 이동.md`](시나리오%202%20수확%20및%20예냉실%20이동.md)의
E3 참고. AI Service는 이 서비스의 **서버**를 이미 구현·검증했고
(`detect_tomatoes_server.py`, RP-127), Ddagi 쪽 **클라이언트**(harvest 루프
안에서 이 서비스를 호출하는 부분)는 아직 없다 — 이 문서는 그 클라이언트를
구현할 때 참고용.

시나리오1 순찰용 `analyze_frame` TCP 프로토콜과는 완전히 별개다
([`ai_service_interface_spec.md`](ai_service_interface_spec.md) 참고). 이쪽은
DG Control Service가 아니라 **Ddagi**가 직접 호출하는 ROS2 서비스다.

**모델 버전도 서비스별로 다르다** — DG Control Service(`analyze_frame`)는
`tomato_4cls_v6.pt`, Ddagi(`DetectTomatoes`)는 `tomato_4cls_v8.pt`를 기본으로
쓴다(각각 `analysis_server.DEFAULT_MODEL_PATH` / `detect_tomatoes_server.DEFAULT_MODEL_PATH`).
두 서버가 별도 프로세스라 `DG_AI_MODEL_PATH` 환경변수로 오버라이드해도
서로 영향을 주지 않는다.

## 1. 통신 방식

- **Protocol**: ROS2 Service (동기 요청/응답, 커넥션 없음)
- **Direction**: Ddagi Control Service → DG AI Service
- **Service Name**: `/ai/detect_tomatoes` (**절대경로**, robot_id 네임스페이스
  없음 — DG·DdaGo·Ddagi가 로봇 한 대를 이루는 세트 내부 통신이라 로봇 식별이
  필요 없다. `/ddago/navigate`·`/ddago/dock`과 같은 규칙)
- **Interface**: `automato_interfaces/srv/DetectTomatoes`

## 2. 인터페이스 정의

```
# automato_interfaces/srv/DetectTomatoes
# ---------- Request ----------
int64   task_id
int32   round               # 촬영 라운드 (진단용)
---
# ---------- Response ----------
bool      success
string    frame_id          # 항상 camera_link
Tomato[]  tomatoes          # 익은 대상만 (NORMAL / DISCARD)
string    error_code        # 실패 시 (예: CAMERA_NOT_AVAILABLE)
string    message
```

```
# automato_interfaces/msg/Tomato
int32   tomato_id           # 사진 1장 안에서만 유효
string  grade               # NORMAL(수확품 바구니) / DISCARD(폐기품 바구니)
float64 x
float64 y
float64 z                   # camera_link 좌표
```

## 3. 에러 코드

| `error_code` | 의미 | Ddagi가 할 일 |
| --- | --- | --- |
| `CAMERA_NOT_AVAILABLE` | 카메라가 아직 안 열림(미연결 또는 다른 프로세스 점유) | 영구 실패로 보지 말 것. 다음 라운드에 재시도하면 됨(서버가 매 요청마다 재오픈을 시도한다) |
| `INFERENCE_FAILED` | YOLO 추론 중 예외 | 로그로 남기고 재시도. 반복되면 AI Service 쪽 점검 필요 |

두 경우 다 `success=false`, `tomatoes=[]`로 온다. `MAX_ROUNDS`(5)가 이런
반복 실패 상황의 상한선 역할을 한다 — 무한 재시도하지 말고 그 안에서
`MAX_ROUNDS_EXCEEDED`로 빠져나가야 한다.

## 4. Ddagi가 호출 전에 알아야 할 것

1. **동기 호출이 오래 걸릴 수 있다** — 응답이 오기 전에 서버 안에서 YOLO
   추론 + depth 조회가 다 끝난다. `wait_for_service`만 걸고 응답을
   기다리지 않으면 안 되고, 타임아웃을 넉넉히 잡아야 한다.
2. **`CAMERA_NOT_AVAILABLE`은 재시도 대상이다** — 서버는 카메라를 지연
   오픈한다(생성자에서 안 열고, 요청이 올 때 연다). 이미 실패했어도 다음
   요청에서 자동으로 재오픈을 시도하므로, Ddagi 쪽에서 노드를 재기동할
   필요는 없다.
3. **좌표 변환은 Ddagi 책임** — `x,y,z`는 `camera_link` 좌표 그대로 온다.
   base(팔) 좌표계로의 TF 변환은 이 서비스가 하지 않는다.
4. **`tomato_id`로 라운드 간 추적 금지** — 그 호출(그 사진) 안에서만
   유효한 번호다. 제외 목록은 좌표(x,y,z, `EXCLUSION_RADIUS`=3cm 이내면
   동일 개체로 판정)로 관리해야 한다.
5. **등급 필터링은 이미 끝나 있다** — `tomatoes[]`에 있는 건 전부 NORMAL
   아니면 DISCARD고, 안익은(unripe) 열매는 애초에 안 온다. Ddagi는
   `grade`로 어느 바구니에 넣을지만 결정하면 되고, 추가로 성숙도를 다시
   거를 필요는 없다.

## 5. 호출 예시 (rclpy)

```python
from automato_interfaces.srv import DetectTomatoes

class HarvestLoop(Node):
    def __init__(self):
        super().__init__('harvest_loop')
        self._ai_client = self.create_client(DetectTomatoes, '/ai/detect_tomatoes')

    def detect_once(self, task_id: int, round_: int, timeout: float = 15.0):
        if not self._ai_client.wait_for_service(timeout_sec=5.0):
            return None  # AI Service 노드가 아직 안 떠 있음
        req = DetectTomatoes.Request()
        req.task_id = task_id
        req.round = round_
        future = self._ai_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result() if future.done() else None
```

## 6. 수동 테스트 방법

AI Service 쪽 서버가 실제로 응답하는지 Ddagi 코드 없이도 확인할 수 있다.
D435가 다른 프로세스(Octomap용 `realsense2_camera_node` 등)에 점유돼 있으면
`CAMERA_NOT_AVAILABLE`까지만 확인되고, 실제 검출 결과는 카메라가 빈 뒤에
다시 호출하면 된다(서버 재기동 불필요).

```bash
# 터미널 1 — 서버 (모델 경로 기본값이 이미 tomato_4cls_v8.pt라 DG_AI_MODEL_PATH 지정 불필요.
# 다른 모델로 오버라이드하고 싶을 때만 export DG_AI_MODEL_PATH=... 로 지정)
source install/setup.bash
ros2 run dg_ai_service detect_tomatoes_server

# 터미널 2 — 1회성 확인
ros2 service call /ai/detect_tomatoes automato_interfaces/srv/DetectTomatoes "{task_id: 1, round: 1}"

# 터미널 2 — Ddagi 라운드 루프를 흉내 내는 반복 호출(RP-127 검증용 클라이언트)
ros2 run dg_ai_service detect_tomatoes_test_client --task-id 1 --rounds 3
```

## 7. 참고 파일

- `equip/automato_ws/src/automato_interfaces/srv/DetectTomatoes.srv`
- `equip/automato_ws/src/automato_interfaces/msg/Tomato.msg`
- `equip/automato_ws/src/dg_ai_service/dg_ai_service/detect_tomatoes_server.py`
- `equip/automato_ws/src/dg_ai_service/dg_ai_service/tomato_grading.py` (등급 매핑 + depth 조회 순수 로직)
- `equip/automato_ws/src/dg_ai_service/dg_ai_service/detect_tomatoes_test_client.py` (수동 검증용 테스트 클라이언트)
- `equip/automato_ws/docs/시나리오 2 수확 및 예냉실 이동.md` (E3 — 전체 흐름/설계 배경)

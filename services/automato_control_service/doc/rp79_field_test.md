# RP-79 탐지 저장·중계·알림 — 검증 런북

> ACS(Automato Control Service)가 HQ의 `/automato/save_detection`(ROS2 Service)을 받아
> **① 병해충 이미지 저장 → ② DB 저장 → ③ 순찰 현황 중계(notify) → ④ 병해충 알림(alert)**
> 을 순서대로 처리하는지 검증하는 절차서. (시나리오 1 E2/E3)
>
> **개념(초보자용).** *ROS2 Service* = 토픽(계속 흘러오는 방송)과 달리 **요청 1건 → 응답 1건**인
> 함수 호출식 통신이다. 여기선 HQ가 "이 탐지 저장해줘"라고 부르면(요청) ACS가 "성공/실패"로
> 답한다(응답). ACS가 **서비스 서버**(요청 받는 쪽)다.

---

## 0. 무엇을 검증하나 (DoD 매핑)

| # | 검증 항목 | 기대 |
| --- | --- | --- |
| E2-1 | DB 저장 | `detection_logs` INSERT + `task_paths.is_visited=TRUE`가 **같은 트랜잭션** |
| E2-2 | 저장 실패 | `success=false`+message, 그래도 notify/alert는 발송 |
| E2-3 | notify | waypoint별 현황 전달, 비200이어도 순찰 안 멈춤, `zone_cumulative` 없음 |
| E3-1 | 게이트 | `disease_percent>=5`일 때만 이미지 저장 + alert, 미만이면 둘 다 안 함 |
| E3-2 | 매 순찰 발송 | 중복 억제(dedup) 없음 — 같은 지점 반복해도 매번 alert |
| E3-3 | payload | alert에 `disease_percent`/`image_path`/`detected_at` 포함, 200 OK |
| E3-4 | 이미지 경로 | 바이트 수신 → 파일 저장 → `image_path` 기록 → alert에 경로 전달 |
| E3-5 | alert 재시도 | 비200 시 최대 3회 시도 후 로그 |
| 공통 | 비블로킹 | 이미지/저장/알림 어떤 실패에도 순찰 루프 안 멈춤 |

---

## 1. 사전 준비

### 1-1. ⚠️ 인터페이스 재빌드 (srv가 바뀜)

RP-79에서 `SaveDetection.srv`에 `disease_image`(uint8[]) + `disease_image_encoding` 필드를
추가했다. 이 타입을 쓰는 **모든 기기(관제 PC + HQ 측)에서 재빌드**해야 새 필드가 반영된다.

```bash
# 🖥️ [관제 PC]
cd ~/roscamp-repo-1/equip/automato_ws
colcon build --packages-select automato_interfaces
source install/setup.bash
# 확인: 새 필드가 보여야 함
ros2 interface show automato_interfaces/srv/SaveDetection
```

### 1-2. DB 기동 (RP-82 스키마 적용)

```bash
# 🖥️ [관제 PC]
cd ~/roscamp-repo-1/services/database
docker compose up -d && source .venv/bin/activate && alembic upgrade head
```

> `detection_logs.disease_image_path`, `task_paths.is_visited` 컬럼이 있어야 한다(0001 스키마).

### 1-3. notify/alert를 받을 임시 목(mock) 서버

수신 백엔드는 아직 미정이므로, POST를 받아 로그만 찍는 목 서버를 하나 띄워 관찰한다.
`scratch_mock.py`로 저장:

```python
# 🖥️ [관제 PC] python3 scratch_mock.py  (기본 8100 포트)
import json
from http.server import BaseHTTPRequestHandler, HTTPServer

class H(BaseHTTPRequestHandler):
    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(n).decode() if n else ""
        print(f"\n[RECV] {self.path}")
        try:
            print(json.dumps(json.loads(body), ensure_ascii=False, indent=2))
        except Exception:
            print(body)
        self.send_response(200); self.end_headers(); self.wfile.write(b'{"ok":true}')
    def log_message(self, *a):
        pass  # 기본 액세스 로그 끔

HTTPServer(("0.0.0.0", 8100), H).serve_forever()
```

```bash
python3 scratch_mock.py    # 이 터미널에 notify/alert 수신 내용이 찍힌다
```

### 1-4. ACS 기동

```bash
# 🖥️ [관제 PC]
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
cd ~/roscamp-repo-1/services/automato_control_service
source .venv-acs/bin/activate   # (RP-78 문서 2-4에서 만든 venv)
# 이미지 저장 위치와 목 서버 주소를 지정해 기동
DETECTION_IMAGE_ROOT=/tmp/automato_detections \
AUTOMATO_WEB_SERVICE_URL=http://localhost:8100 \
python3 -m automato_control_service.patrol_node
#   → 로그에 "탐지 저장 서비스 준비: /automato/save_detection" 가 떠야 한다
```

```bash
# 🖥️ [관제 PC] 다른 터미널 — 서비스가 보이는지 확인
ros2 service list | grep save_detection          # /automato/save_detection
ros2 service type /automato/save_detection       # automato_interfaces/srv/SaveDetection
```

> **선행 데이터.** `detection_logs`는 `tasks`/`waypoints`/`robots`를 FK로 참조하고,
> `task_paths` UPDATE는 해당 `task_id`+`waypoint_id` 행이 있어야 반영된다. RP-78 순찰 접수로
> task/task_paths를 먼저 만들어 두거나, DB에 시드 task 하나를 넣고 그 `task_id`/`waypoint_id`를
> 아래 호출에 쓴다. (아래 예시의 `TASK`/`WP`를 실제 값으로 바꿔라.)

---

## 2. 시나리오

각 시나리오: **목적 / 실행 / 관찰(기대) / 판정**. `TASK`, `WP`는 1-4 선행 데이터의 실제 값.

### S1. disease < 5 — 저장 + notify만 (이미지·alert 없음) [E2-1, E3-1]

병해충 미만이라 이미지 바이트가 없다. `ros2 service call`로 빈 이미지 요청을 보낸다.

```bash
# 🖥️ [관제 PC]  (disease_image: [] = 빈 바이트)
ros2 service call /automato/save_detection automato_interfaces/srv/SaveDetection \
"{task_id: $TASK, waypoint_id: $WP, robot_id: 'dg_01',
  ripe_percent: 80, unripe_percent: 15, rotten_percent: 5, disease_percent: 3,
  disease_image: [], disease_image_encoding: ''}"
```

- **기대:** 응답 `success: true`. 목 서버에 **`/internal/v1/detections/notify`만** 찍히고
  `detection_id`가 숫자, `zone_cumulative` 키는 없음. **alert는 안 옴.** 이미지 파일도 안 생김.
- [ ] notify 1건 수신(detection_id 숫자)  - [ ] alert 없음  - [ ] `/tmp/automato_detections` 비어 있음

### S2. disease >= 5 — 이미지 저장 + notify + alert [E3-1,3,4,5]

이미지 바이트를 실어 보내야 하므로 작은 Python 클라이언트를 쓴다(`scratch_call.py`):

```python
# 🖥️ [관제 PC] python3 scratch_call.py <task_id> <waypoint_id>
import sys, rclpy
from rclpy.node import Node
from automato_interfaces.srv import SaveDetection

task_id, wp = int(sys.argv[1]), int(sys.argv[2])
rclpy.init(); node = Node("rp79_test_client")
cli = node.create_client(SaveDetection, "/automato/save_detection")
cli.wait_for_service(timeout_sec=5.0)
req = SaveDetection.Request()
req.task_id, req.waypoint_id, req.robot_id = task_id, wp, "dg_01"
req.ripe_percent, req.unripe_percent, req.rotten_percent = 70, 10, 5
req.disease_percent = 20                          # >= 5 → 게이트 통과
req.disease_image = list(b"\xff\xd8FAKEJPEGBYTES")  # 레이블 JPEG(테스트용 더미)
req.disease_image_encoding = "jpeg"
fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut)
print("응답:", fut.result())
node.destroy_node(); rclpy.shutdown()
```

```bash
python3 scratch_call.py $TASK $WP
```

- **기대:** 응답 `success: true`. 목 서버에 **notify + alert 둘 다** 찍힘. alert payload에
  `disease_percent: 20`, `image_path: "YYYY-MM-DD/wp<WP>_dg_01_HHMMSS.jpg"`, `detected_at` 포함.
  이미지 파일이 실제로 생성됨:
  ```bash
  find /tmp/automato_detections -type f      # 저장된 .jpg 경로 확인
  ```
  DB에 상대경로 기록:
  ```bash
  docker compose exec postgres psql -U robot8 -d automatodb -c \
   "SELECT detection_id, disease_percent, disease_image_path, detected_at
      FROM detection_logs ORDER BY detection_id DESC LIMIT 3;"
  ```
- [ ] notify+alert 둘 다 수신  - [ ] 이미지 파일 생성  - [ ] `disease_image_path`가 상대경로
- [ ] `task_paths.is_visited=TRUE`로 바뀜:
  ```bash
  docker compose exec postgres psql -U robot8 -d automatodb -c \
   "SELECT task_id, waypoint_id, is_visited FROM task_paths WHERE task_id=$TASK ORDER BY point_index;"
  ```

### S3. 매 순찰 발송 확인 — dedup 없음 [E3-2]

S2를 **연속 2번** 실행한다.

- **기대:** 목 서버에 alert가 **2번** 찍히고, `detection_logs`에 행이 2개 늘어난다(중복 억제 없음).
- [ ] 같은 지점 반복 호출에도 alert 매번 발송

### S4. DB 실패해도 notify/alert 발송 + success=false [E2-2]

존재하지 않는 `task_id`(예: 999999)로 호출 → FK 위반으로 DB INSERT 실패.

```bash
python3 scratch_call.py 999999 $WP        # 없는 task_id
```

- **기대:** 응답 `success: false`, `message`에 DB 오류 사유. 그래도 목 서버엔 **notify + alert가
  찍히고**, notify의 `detection_id`는 **null**. (안전 위해 알림은 발송)
- [ ] success=false + 사유  - [ ] notify(detection_id=null)·alert 그래도 발송

### S5. alert 재시도 (목 서버를 끈 상태) [E3-5]

목 서버(1-3)를 **Ctrl+C로 끄고** S2를 실행한다.

- **기대:** ACS 로그에 `disease alert 최종 실패(3회) ...` (최대 3회 시도 후 포기). notify도
  `notify 실패(재시도 안 함) ...` 1줄. **하지만 ACS는 안 죽고, 응답 `success:true`는 정상**(DB는
  떠 있으니). 순찰 루프 비블로킹 확인.
- [ ] alert 3회 시도 로그  - [ ] notify는 재시도 없이 1회 실패 로그  - [ ] ACS 정상 동작 유지

---

## 3. 트러블슈팅

| 증상 | 원인 후보 | 조치 |
| --- | --- | --- |
| `ros2 service list`에 save_detection 없음 | ACS 미기동 / interfaces 재빌드 안 함 | 1-1 재빌드, ACS 로그 "탐지 저장 서비스 준비" 확인 |
| service call이 **필드 없음** 에러 | 새 srv 필드 반영 안 됨 | 관제 PC·HQ **양쪽** `colcon build --packages-select automato_interfaces` |
| 응답 success=false (정상 데이터인데) | task_id/waypoint_id가 DB에 없음(FK 위반) | 선행 task/task_paths 생성(RP-78 접수) 후 그 값 사용 |
| 이미지 파일이 안 생김 | disease_percent<5 / 바이트 안 실림 / 루트 권한 | `disease_percent>=5`+바이트 확인, `DETECTION_IMAGE_ROOT` 쓰기 권한 |
| notify/alert가 목 서버에 안 옴 | `AUTOMATO_WEB_SERVICE_URL` 불일치 / 목 서버 미기동 | ACS 기동 시 URL, 목 서버 포트(8100) 확인 |
| `disease_image_path`가 절대경로 | (버그) 상대경로만 저장해야 함 | 저장값이 `YYYY-MM-DD/...` 형태인지 확인 |

---

## 부록: 단위테스트 (로봇/DB 없이 로직만)

순서·게이트·실패 정책은 협력자를 fake로 주입한 단위테스트로 이미 검증된다.

```bash
cd ~/roscamp-repo-1/services/automato_control_service
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_detection_service.py -v
```

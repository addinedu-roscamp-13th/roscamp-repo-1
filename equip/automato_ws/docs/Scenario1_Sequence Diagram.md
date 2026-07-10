# 시나리오 1 Sequence Diagram

## E0 : 상시 모니터링 루프

**목적**: 순찰 요청 여부와 무관하게 상시 도는 텔레메트리 스트리밍 루프. 저장 없이 실시간으로만 흘려보낸다.

**참여자**

- DdaGo Control Service, Ddagi Control Service: 각각 독립적으로 HQ에 보고 (하나로 묶이지 않음)
- HQ (DG Control Service): 두 로봇의 데이터를 받아 취합 후 상위로 전달
- Automato Control Service: 취합된 텔레메트리를 다시 클라이언트용으로 가공
- System Admin APP(QT): 최종 수신자, Automato Control Service와 ROS2로 직접 연결

**메시지별 데이터**

1. DdaGo → HQ: `x, y, yaw`(위치), `battery_percent, battery_voltage`, `task_id`, `nav_status`, `is_charging`, `us_range_m`
2. Ddagi → HQ: `joint_angles`, `tcp_coords`, `servo_health`(7: 6관절+그리퍼), `task_id`, `is_paused`
3. HQ → Automato Control Service: 위 두 로봇 데이터를 robot\_id 기준 배열로 취합
4. Automato Control Service → System Admin APP(QT): 위치/배터리/현재작업/연결상태

**주기**: 1초 (loop). 배터리처럼 실제 publish 주기가 5초인 값도 있지만, HQ가 로봇별로 최신값을 모아 1초 간격으로 재전송하는 방식.

**원칙**: 이 루프의 어떤 데이터도 DB/메모리/rosbag2에 저장하지 않는다 (실시간 스트리밍 전용).

**메세지**

**Ddago (주행 로봇)**

**"is\_charging"(충전 중인지) 핑키에서 충전 중인 상태를 알 수 있는 방법 X → false로 표기하고 이 필드는 쓰지 않는 걸로 합니다.**

```json
{
  "robot_id": "dg_01",
  "timestamp": "2026-07-06T09:12:33.512Z",
  "task_id": 1024,
  "position": { "x": 3.21, "y": 1.05, "yaw": 1.57 },
  "nav_status": "NAVIGATING",
  "is_charging": false,
  "battery": { "percent": 78.5, "voltage": 12.1 },
  "obstacle": { "us_range_m": 0.42 }
}
```

**Ddagi (로봇팔)**

```json
{
  "robot_id": "dg_01",
  "timestamp": "2026-07-06T09:12:33.512Z",
  "task_id": 1024,
  "joint_angles": [10.2, -30.5, 45.0, 0.0, -12.3, 5.5],
  "tcp_coords": { "x": 160, "y": 30, "z": 200, "rx": 0, "ry": 0, "rz": 0 },
  "is_paused": false,
  "servo_health": [
    { "joint": 1, "voltage_ok": true, "temperature": 42, "current": 0.3, "overload": false },
    { "joint": 2, "voltage_ok": true, "temperature": 41, "current": 0.3, "overload": false },
    { "joint": 3, "voltage_ok": true, "temperature": 40, "current": 0.2, "overload": false },
    { "joint": 4, "voltage_ok": true, "temperature": 39, "current": 0.2, "overload": false },
    { "joint": 5, "voltage_ok": true, "temperature": 40, "current": 0.2, "overload": false },
    { "joint": 6, "voltage_ok": true, "temperature": 38, "current": 0.2, "overload": false },
    { "joint": 7, "voltage_ok": true, "temperature": 37, "current": 0.4, "overload": false, "gripper_value": 85 }
  ]
}
```


## E0 통신 규격

### 1) DdaGo Control Service → HQ (ROS2 Topic)

**Topic Name**: `/dg_01/ddago/telemetry`

**Message Type**: `automato_interfaces/msg/DdagoTelemetry`

**Publish 주기**: 1Hz

```
# automato_interfaces/msg/DdagoTelemetry
# robot_id : dg_01, dg_02, dg_03
std_msgs/Header  header
string           robot_id 
int64            task_id
string           nav_status
bool             is_charging

# position
float64          x
float64          y
float64          yaw

# battery
float32          battery_percent
float32          battery_voltage

# obstacle
float32          us_range_m
```


### 2) Ddagi Control Service → HQ (ROS2 Topic)

**Topic Name**: `/dg_01/ddagi/telemetry`

**Message Type**: `automato_interfaces/msg/DdagiTelemetry`

**Publish 주기**: 1Hz

```
# automato_interfaces/msg/DdagiTelemetry
# robot_id : dg_01, dg_02, dg_03
std_msgs/Header  header
string           robot_id
int64            task_id
bool             is_paused

float32[6]       joint_angles
float32[6]       tcp_coords

ServoStatus[7]   servo_health
```

**보조 메시지**: `automato_interfaces/msg/ServoStatus`

```
# automato_interfaces/msg/ServoStatus

int8    joint_no
bool    voltage_ok
int16   temperature
float32 current
bool    overload
int16   gripper_value
```


### 3) HQ → Automato Control Service (ROS2 Topic)

**Topic Name**: `/automato/telemetry/fleet`

**Message Type**: `automato_interfaces/msg/FleetTelemetry`

**Publish 주기**: 1Hz

```
# automato_interfaces/msg/FleetTelemetry

std_msgs/Header    header
DdagoTelemetry[]   ddagos
DdagiTelemetry[]   ddagis
```


### 4) Automato Control Service → System Admin APP (QT) (ROS2 Topic)

**Topic Name**: `/automato/dashboard/fleet_telemetry`

**Message Type**: `automato_interfaces/msg/FleetTelemetry` (3번과 동일 재사용)

**Publish 주기**: 1Hz

QT는 시스템 진단이 목적이므로 축약본이 아니라 로봇 원본 데이터 그대로 전달합니다.

---

## E1 : 순찰 시작

**목적**: Farm Admin(=Owner)이 순찰을 요청하는 시점부터, 그 요청이 실제 로봇에게 명령으로 전달되기까지의 흐름.

**참여자**

- Farm Admin (Owner): 요청 주체
- Automato Web Service: 클라이언트(App)와 백엔드 사이의 중계
- Automato Control Service: 실제 로봇을 어떤 걸 쓸지 결정하는 주체
- HQ (DG Control Service): 로봇에게 실제 명령을 하달하는 오케스트레이터
- DdaGo Control Service: 순찰은 주행 로봇만 담당하므로 Ddagi는 이 흐름에 등장하지 않음

**메시지별 데이터**

1. Farm Admin → Web Service : 즉시 요청. 시스템 자동으로 로봇 선정 / 관리자가 로봇 지정
2. Web Service → Automato Control Service: 즉시 요청. robot\_id / auto
3. Automato Control Service 내부에서 "가능한 로봇"(대기중이면서 배터리 많은 로봇) 판단
    1. 로봇 상태 대기중
    2. 농장 관리자가 기능별로 % 설정할 수 있게
        - 순찰 : 배터리 70% 이상(기본)
        - 수확 : 배터리 50% 이상(기본)
4. Automato Control Service → Automato DB: task 생성(tasks) + 순찰 경로 저장(task\_paths) + 배정 스냅샷 저장(task\_assignment\_snapshot) + tasks 상태 IN\_PROGRESS로 변경
5. Automato Control Service → HQ: 확정된 task\_id, 배정된 robot\_id, waypoint 목록
6. HQ → DdaGo Control Service: task\_id, waypoint 목록

## **E1 API 스펙 및 ROS2 통신 규격**

### **0) Farm Admin App → Automato Web Service (HTTP)**

**Endpoint**

```
GET /api/v1/robots/patrol/available
```

**Response 200 OK**

```json
{
  "requested_at": "2026-07-06T09:12:33.512Z",
  "min_battery_percent": 70,
  "robots": [
    { "robot_id": "dg_01", "status": "IDLE",
      "battery_percent": 85.2, "current_position": { "x": 3.21, "y": 1.05 }, "available": true },
    { "robot_id": "dg_02", "status": "IDLE",
      "battery_percent": 62.0, "current_position": { "x": 5.10, "y": 2.30 },
      "available": false, "unavailable_reason": "BATTERY_TOO_LOW" },
    { "robot_id": "dg_03", "status": "PATROLLING",
      "battery_percent": 78.0, "current_position": { "x": 1.50, "y": 4.00 },
      "available": false, "unavailable_reason": "ROBOT_BUSY" }
  ]
}
```


### **1) Automato Web Service → Automato Control Service (HTTP 내부 API)**

**Endpoint**

```
GET /internal/v1/robots/patrol/available
```

Response는 0)과 동일.


### **2) Farm Admin App → Automato Web Service (HTTP)**

**Endpoint**

```
POST /api/v1/patrol/requests
```

**Request Body**

```json
// robot_id : dg_01, dg_02, dg_03
{
  "robot_selection": "auto",
  "robot_id": null
}
```

**Response 200 OK**

```json
{
  "task_id": 1024,
  "assigned_robot_id": "dg_01",
  "status": "ACCEPTED",
  "message": "순찰 요청이 접수되었습니다."
}
```

**Response 409 Conflict**

```json
{
  "request_id": "req_20260706_001",
  "status": "REJECTED",
  "reason": "NO_AVAILABLE_ROBOT",
  "message": "요청 가능한 로봇이 없습니다."
}
```


### **3) Automato Web Service → Automato Control Service (HTTP 내부 API)**

**Endpoint**

```
POST /internal/v1/tasks/patrol
```

Request/Response는 2)와 동일한 필드 구조.


### 4) Automato Control Service → **Automato DB**

FK 의존성 때문에 **tasks → task\_paths → task\_assignment\_snapshot** 순으로 저장한 뒤, 순찰 시작으로 상태를 전환한다. (①② 선행 저장, ③④ 기존 스텝. robot\_state\_snapshot엔 선정 근거로 순찰이니 Ddago 상태 position/battery\_percent/nav\_status 저장)

```sql
-- ① task 생성 (WAITING), 선정된 로봇을 assigned_robot_id에 기록
INSERT INTO tasks (task_type, status, assigned_robot_id, created_at, updated_at)
VALUES ('PATROL', 'WAITING', ?, NOW(), NOW())
RETURNING task_id;

-- ② 순찰 경로 적재 (waypoint 목록 → task_paths, waypoint 개수만큼 배치 INSERT)
INSERT INTO task_paths (task_id, waypoint_id, point_index, is_visited, created_at, updated_at)
VALUES (?, ?, 0, FALSE, NOW(), NOW()),
       (?, ?, 1, FALSE, NOW(), NOW());
       -- ... waypoint 개수만큼

-- ③ 배정 근거 스냅샷
INSERT INTO task_assignment_snapshot (task_id, robot_id, robot_state_snapshot, assigned_at)
VALUES (?, ?, ?::jsonb, NOW());

-- ④ 순찰 시작 → IN_PROGRESS
UPDATE tasks
   SET status = 'IN_PROGRESS', started_at = NOW(), updated_at = NOW()
 WHERE task_id = ?;
```


### **5) Automato Control Service → HQ (DG Control Service) (ROS2 Action)**

**Action Name**: `/{robot_id}/patrol`

**Action Interface**: `automato_interfaces/action/Patrol`

waypoint 정보는 **Automato Control Service**에서 **DG Control Service**로 넘겨준다.

```
# automato_interfaces/action/Patrol

# ---------- Goal ----------
int64   task_id
WaypointGoal[]   waypoints
---
# ---------- Result ----------
# result_code(0: 성공, 1: 실패, 2: 중단)
int32   result_code 
string  message
int32   visited_count
---
# ---------- Feedback ----------
int32   current_waypoint_id
int32   visited_count
```

**순찰 종료 처리**: 이 Action의 Result가 ACS로 돌아오면 `UPDATE tasks SET status = (result_code=0 이면 'DONE', 아니면 'FAILED'), ended_at = NOW() WHERE task_id = ?;`


### **6) HQ → DdaGo Control Service (ROS2 Action)**

**Action Name**: `/dg_01/ddago/patrol` (로봇마다 네임스페이스로 구분)

**Action Interface**: `automato_interfaces/action/DdagoPatrol`

```
# automato_interfaces/action/DdagoPatrol

# ---------- Goal ----------
int64   task_id
WaypointGoal   waypoint
---
# ---------- Result ----------
# result_code(0: 성공, 1: 실패, 2: 중단)
int32   result_code
string  message
---
# ---------- Feedback ----------
int32    current_waypoint_id
float64  current_x
float64  current_y
float64  current_yaw
```

**보조 메세지** : `automato_interfaces/msg/WaypointGoal`

```
# automato_interfaces/msg/WaypointGoal
int32    waypoint_id
float64  x
float64  y
```

---

## E2 : 웨이포인트별 체크 및 DB 저장 (숙도·부패·병해충, RGB 카메라)

**목적**: 순찰의 핵심 루프. HQ가 waypoint 하나를 하달할 때마다 촬영→분석→저장→다음 waypoint 하달 사이클이 반복됩니다. 익음/부패/병해충 어떤 결과가 나오든 상관없이 항상 다음 waypoint로 넘어갑니다(별도 분기 없음).

**참여자**

- **DdaGo Control Service**: waypoint 도착 후 RGB 카메라 촬영, 분석 요청 발신
- **HQ (DG Control Service)**: waypoint를 하나씩 하달하며 전체 루프의 중계자, 처음부터 끝까지 계속 활성 상태
- **DG AI Service**: RGB 이미지 분석만 수행
- **Automato Control Service**: 저장 요청을 받아 DB 저장·화면 갱신까지 처리(내부 DB 저장은 이 서비스 안에서 이루어짐)
- **Automato DB**: 순찰 결과(익음/부패/병해충 수치) 저장소
- **Automato Web Service**: Automato Control Service와 Farm Admin App 사이 중계
- **Farm Admin App**: 순찰 현황 최종 수신자

**메시지별 데이터**

1. DdaGo(self): waypoint 도착 후 RGB 카메라 촬영 트리거
2. DdaGo → HQ: 분석 요청 (이미지, task\_id, `waypoint_id`)
3. HQ → AI Service: 분석 요청 전달
4. AI Service → HQ: 분석 결과 (`ripe_count`, `rotten_count`, `disease_count`, `total_count`)
5. HQ → Automato Control Service: 결과 저장 요청 (`task_id`, `waypoint_id`, 위 수치)
6. HQ → DdaGo: 다음 waypoint 전달 — ACS 응답을 기다리지 않고 즉시 발신
7. Automato Control Service → Automato DB: 결과 저장 (`detection_logs` INSERT)
8. Automato DB → Automato Control Service: 저장 완료
9. Automato Control Service → Automato Web Service: 순찰 현황 전달 (순찰 누적치)
10. Automato Web Service → Farm Admin App: 순찰 현황 갱신


## E2 API 스펙 및 통신 규격

### 1) DdaGo 내부: waypoint 도착 후 RGB 카메라 촬영 트리거

내부 함수 호출. 통신 없음.


### 2) DdaGo Control Service → HQ: 분석 요청 (ROS2 Service)

**Service Name**: `/dg/analyze_frame`

**Service Interface**: `automato_interfaces/srv/AnalyzeFrame`

```
# automato_interfaces/srv/AnalyzeFrame

# ---------- Request ----------
int64                task_id
int32                waypoint_id
sensor_msgs/Image    image
---
# ---------- Response ----------
bool                 accepted
string               request_id
```


### 3) HQ → DG AI Service: 분석 요청 전달 (TCP)

**연결 방식**: 지속 연결(persistent TCP connection) 유지. 매 요청마다 새로 열지 않음.

**메시지 프레이밍**: 길이 접두어 방식 — 앞 4바이트에 payload 크기(big-endian int32), 이어서 payload

**Payload 포맷**: JSON

**Request Payload**

```json
// request_id는 DG Control Service가 생성
{
  "message_type": "analyze_frame_request",
  "request_id": "req_20260706_001",
  "task_id": 1024,
  "waypoint_id": 3,
  "image_encoding": "jpeg",
  "image_data": "<base64 encoded bytes>"
}
```


### 4) DG AI Service → HQ: 분석 결과 (TCP)

같은 TCP 연결로 응답. 프레이밍 방식 동일.

**Response Payload (성공)**

```json
{
  "message_type": "analyze_frame_response",
  "request_id": "req_20260706_001",
  "status": "OK",
  "result": {
    "ripe_percent": 50,
    "unripe_percent": 50,
    "rotten_percent": 0,
    "disease_percent": 0
  }
}
```

**Response Payload (에러)**

```json
{
  "message_type": "analyze_frame_response",
  "request_id": "req_20260706_001",
  "status": "ERROR",
  "error_code": "IMAGE_DECODE_FAILED",
  "error_message": "이미지 디코딩 실패"
}
```


### 5) HQ → Automato Control Service: 저장 요청 (ROS2 Service)

**Service Name**: `/automato/save_detection`

**Service Interface**: `automato_interfaces/srv/SaveDetection`

```
# automato_interfaces/srv/SaveDetection

# ---------- Request ----------
int64   task_id
int32   waypoint_id
string  robot_id
int32   ripe_percent
int32   unripe_percent
int32   rotten_percent
int32   disease_percent
---
# ---------- Response ----------
bool    success
string  message
```


### 6) HQ → DdaGo Control Service: 다음 waypoint 전달

**Action Name**: `/dg_01/ddago/patrol` (로봇마다 네임스페이스로 구분)

**Action Interface**: `automato_interfaces/action/DdagoPatrol`

```
# automato_interfaces/action/DdagoPatrol

# ---------- Goal ----------
int64   task_id
WaypointGoal   waypoint
---
# ---------- Result ----------
# result_code(0: 성공, 1: 실패, 2: 중단)
int32   result_code 
string  message
---
# ---------- Feedback ----------
int32    current_waypoint_id
float64  current_x
float64  current_y
float64  current_yaw
```


### 7) Automato Control Service → Automato DB: 결과 저장 (SQL)

```sql
-- detection_logs 저장
INSERT INTO detection_logs
    (task_id, robot_id, waypoint_id,
     ripe_count, rotten_count, disease_count, total_count, detected_at)
VALUES (?, ?, ?, ?, ?, ?, ?, NOW())
RETURNING detection_id;

-- 방문 처리 (같은 트랜잭션)
UPDATE task_paths
   SET is_visited = TRUE, updated_at = NOW()
 WHERE task_id = ? AND waypoint_id = ?;
```


### 8) Automato DB → Automato Control Service: 저장 완료 (SQL)

생성된 `detection_id` 반환. 이 값은 Automato Control Service 내부에서만 사용되고 HQ로 다시 올라가지는 않습니다.


### 9) Automato Control Service → Automato Web Service: 순찰 현황 전달 (내부 HTTP)

**Endpoint**

```
POST /internal/v1/detections/notify
```

**Request Body**

```json
{
  "task_id": 1024,
  "waypoint_id": 3,
  "robot_id": "dg_01",
  "detection_id": 55123,
  "ripe_percent": 50,
  "unripe_percent": 50,
  "rotten_percent": 0,
  "disease_percent": 0,
  "detected_at": "2026-07-06T09:12:33.512Z"
}
```

**Response 200 OK**

```json
{ "success": true }
```


### 10) Automato Web Service → Farm Admin App: 순찰 현황 갱신 (WebSocket)

**WebSocket Endpoint**

```
wss://{host}/ws/farm-admin?token={jwt}
```

앱은 로그인 시 이 연결을 열어두고 유지합니다. 여러 이벤트 타입이 하나의 커넥션 위에 흘러가며, 각 메시지의 `event` 필드로 종류를 구분합니다.

**Event: **`patrol_progress` (waypoint마다)

```json
{
  "event": "patrol_progress",
  "task_id": 1024,
  "waypoint_id": 3,
  "ripe_percent": 50,
  "unripe_percent": 50,
  "rotten_percent": 0,
  "disease_percent": 0,
  "detected_at": "2026-07-06T09:12:33.512Z"
}
```

**Event: **`patrol_completed` (마지막 waypoint 완료 후)

```json
{
  "event": "patrol_completed",
  "task_id": 1024,
  "robot_id": "dg_01",
  "completed_at": "2026-07-06T09:35:12.000Z",
  "summary": {
    "ripe_percent": 50,
    "unripe_percent": 50,
    "rotten_percent": 0,
    "disease_percent": 0
  }
}
```

앱은 `patrol_completed` 수신 시 다음 두 채널로 사용자에게 알림을 전달합니다.

- **브라우저 Notification API**: 웹앱이 활성화되어 있을 때 화면 팝업으로 알림 표시. Notification 권한은 앱 초기 로딩 시 `Notification.requestPermission()`으로 사용자 승인 받아 사용합니다.
- **텔레그램 알림**: 웹앱을 열어두지 않은 상황에서도 알림을 받을 수 있도록 텔레그램 봇을 통해 메시지 발송. 사용자는 사전에 텔레그램 봇을 자신의 계정과 연동해두어야 합니다.

---

## E3 : 병해충 알림

**참여자**

- Automato Control Service: E2에서 저장된 결과 중 병해충 수치를 확인해 알림을 트리거
- Automato Web Service: Farm Admin App으로 알림을 중계
- Farm Admin App: Web Service를 거쳐 긴급 알림 수신

**메시지별 데이터**

1. Automato Control Service → Automato Web Service: 위와 동일한 내용 전달
2. Automato Web Service → Farm Admin App: 위치 정보, disease\_count, 발생 시각

## E3 API 스펙 및 통신 규격

**트리거 조건**: E2 4번(AI Service 분석 결과)에서 `disease_count > 0`일 때만 발동. 그 외엔 이 흐름 전체 스킵.

### 1) Automato Control Service → Automato Web Service (내부 HTTP)

**Endpoint**

```
POST /internal/v1/alerts/disease
```

**Request Body**

```json
{
  "task_id": 1024,
  "waypoint_id": 3,
  "robot_id": "dg_01",
  "disease_count": 2,
  "detected_at": "2026-07-06T09:12:33.512Z"
}
```

**Response 200 OK**

```json
{ "success": true }
```


### 2) Automato Web Service → Farm Admin App (WebSocket 재사용)

E2 10번에서 이미 열려 있는 `/ws/farm-admin` 커넥션을 그대로 재사용합니다. 새 이벤트 타입 `disease_alert`가 추가됩니다.

**Event: **`disease_alert`

```json
{
  "event": "disease_alert",
  "task_id": 1024,
  "waypoint_id": 3,
  "robot_id": "dg_01",
  "disease_count": 2,
  "detected_at": "2026-07-06T09:12:33.512Z"
}
```

앱은 `disease_alert` 수신 시 다음 두 채널로 사용자에게 알림을 전달합니다.

- **브라우저 Notification API**: 웹앱이 활성화되어 있을 때 화면 팝업으로 알림 표시. Notification 권한은 앱 초기 로딩 시 `Notification.requestPermission()`으로 사용자 승인 받아 사용합니다.
- **텔레그램 알림**: 웹앱을 열어두지 않은 상황에서도 알림을 받을 수 있도록 텔레그램 봇을 통해 메시지 발송. 사용자는 사전에 텔레그램 봇을 자신의 계정과 연동해두어야 합니다.

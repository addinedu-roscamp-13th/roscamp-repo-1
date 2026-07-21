# Automato Control Service

Automato Web Service의 순찰 요청을 받아 **가용 로봇을 선정**하고, task를 **DB에 생성**한 뒤
HQ(DG Control Service)로 **순찰 Action**을 하달한다. (시나리오 1 E1 / JIRA **RP-78**)
순찰 중 waypoint마다 HQ가 넘긴 탐지 결과를 받아 **저장·순찰 현황 중계·병해충 알림**을 처리한다
(시나리오 1 E2/E3 / JIRA **RP-79**). 또한 Fleet 텔레메트리를 QT로 중계한다(RP-77).

## 구성 (RP-78 / RP-79)

관심사를 파일로 분리했다(한 프로세스에서 함께 실행):

| 파일 | 담당 |
|---|---|
| `automato_control_service/patrol_api.py` | **① API** — FastAPI 엔드포인트 + 가용 판정(4조건) + 로봇 선정 |
| `automato_control_service/patrol_node.py` | **② ROS2 노드** — 텔레메트리 캐시 + Navigate 액션 클라이언트 + 세그먼트 디스패치 + SaveDetection 서비스 등록 + `main()` |
| `automato_control_service/automato_db.py` | **③ DB 저장** — 가용 조회 / 접수 트랜잭션(①~④) / 종료 갱신 / 그래프 로드 (psycopg v3) |
| `automato_control_service/routing_engine.py` | **④ 라우팅/예약 엔진** — Dijkstra 경로탐색 + 예약표(통로·지점 자리, 독립 모듈, 순찰 외 재사용) |
| `automato_control_service/detection_service.py` | **(RP-79) 탐지 오케스트레이션** — 이미지 저장/DB/notify/alert 조율 + HTTP(urllib) |
| `automato_control_service/detection_db.py` | **(RP-79) 탐지 DB 저장** — detection_logs INSERT + task_paths.is_visited (단일 트랜잭션) |
| `automato_control_service/fleet_telemetry_relay.py` | (RP-77) 텔레메트리 → QT 대시보드 중계 |

### 탐지 저장·중계·알림 (RP-79)

HQ가 `/automato/save_detection`(ROS2 Service)로 waypoint별 탐지 결과를 넘기면, ACS가 1콜에서
**아래 순서**로 처리한다. 진입 시 `detected_at`을 한 번만 캡처해 전 단계가 같은 시각을 쓴다.

```
detected_at 1회 캡처
 → (disease_percent>=5 && 이미지 존재) 이미지 파일 저장 → 상대경로  ┐ 게이트 공유
 → DB 트랜잭션(detection_logs INSERT + task_paths.is_visited=TRUE)  │ (disease>=5)
 → notify  (순찰 현황 중계, fire-and-forget, 재시도 없음)            │
 → (disease_percent>=5) disease alert (3회 재시도)                  ┘
 → 응답 success = DB 저장 성공 여부만 (notify/alert 성패는 무관)
```

- **비블로킹:** DB까지만 동기로 끝내 `success`/`detection_id`를 확정하고, notify/alert는
  백그라운드 스레드풀로 던져 순찰 루프를 막지 않는다. HTTP 재시도/지연도 응답을 지연시키지 않는다.
- **DB 실패해도** notify·alert는 발송하되 HQ엔 `success=false`. notify의 `detection_id`는 null.
- **이미지:** `disease_percent>=5`일 때만 AI가 바이트를 실어 보내고, 파일 저장은 **ACS에서만**.
  경로 규칙 `{YYYY-MM-DD}/wp{waypoint_id}_{robot_id}_{HHMMSS}.jpg`, DB엔 루트 제외 **상대경로만**.
  파일 쓰기 실패 시 경고 로그 + `image_path=null`로 진행(alert엔 `""`).
- **수신처(대시보드/알림 백엔드) 미정:** notify/alert의 base URL은 `AUTOMATO_WEB_SERVICE_URL`
  환경변수로 뺐다. 실제 수신 엔드포인트 구현은 RP-79 범위 밖(다른 티켓).
- **공유 계약 변경:** `SaveDetection.srv`에 `disease_image`(uint8[]) + `disease_image_encoding`
  필드를 추가했다. **HQ·ACS 양쪽에서 `automato_interfaces` 재빌드 필요**(HQ 연계 협의 대상).

**Phase 2 교통관제(구현됨)**: 노드가 세그먼트(연속 예약 가능한 홉 묶음) 단위로
`예약→배열 하달→도착→해제`를 반복하고, 주행 중 다음 구간을 미리 잡아(룩어헤드) 끊김 없이 잇는다.
예약 자원은 **통로(간선)와 지점 자리(정점)** 둘이다 — 홉 하나가 `(통로, 도착 자리)` 한 쌍이라,
자리를 못 잡으면 통로도 잡지 않는다. 통로 잠금이 좁은 길 마주보기를, 자리 잠금이 같은 지점
겹침을 막는다(자리는 `node_slot(n) = -n` 가상 통로로 같은 예약표에 들어간다).
막힘 보고 시 통로 N초 블랙리스트 → Dijkstra 우회 → 우회 불가 시 건너뛰고 마지막에 재시도.
남이 점유하면 예약 대기 후 타임아웃 시 순찰(최하위)이 양보하고, 대기 사이클(데드락)은 즉시 우회.
로봇마다 별도 스레드라 3대가 동시에 움직이며 자원을 공유(`routing_engine`이 락으로 안전 보장).
죽은 예약은 하트비트 TTL + 주기 회수(`reap_expired`)로 자동 정리. **실제 구동 전제**: `corridors` 시드(마이그레이션 `0003_seed_corridors`)가 적용돼 있어야 함.

## 사전 준비

```bash
# 1) DB 기동 + 스키마/시드 (services/database)
cd ../database && docker compose up -d && alembic upgrade head

# 2) ROS2 인터페이스 빌드 (Navigate.action 등)
cd ../../equip/automato_ws && colcon build --packages-select automato_interfaces
source install/setup.bash

# 3) 파이썬 런타임 의존 설치
cd ../../services/automato_control_service && pip install -r requirements.txt
```

접속 정보는 `services/database/.env`의 `DATABASE_URL`을 공유한다(별도 설정 없으면 자동 로딩).

## 실행

```bash
source /opt/ros/jazzy/setup.bash
source <automato_interfaces install>/setup.bash
# 노드(백그라운드 spin) + FastAPI(:8200)를 한 프로세스로 기동
ros2 run automato_control_service patrol_node
#   포트 변경: ACS_API_PORT=8200 ros2 run automato_control_service patrol_node
```

## API

| 메서드 | 경로 | 용도 |
|---|---|---|
| GET | `/internal/v1/robots/patrol/available` | 가용 로봇 조회(4조건 판정 결과) |
| POST | `/internal/v1/tasks/patrol` | 순찰 접수 — `{"robot_selection":"auto"\|"manual","robot_id":null\|"dg_0x"}` |
| GET | `/health` | 헬스체크 |

**가용 판정(4조건 AND)**: ①활성 task 없음(DB) ②nav_status=='IDLE'(캐시) ③battery≥임계값(캐시)
④최근 3초 이내 수신(캐시, ddago header.stamp). `is_charging`은 판정에 쓰지 않는다(항상 false 고정).

**`unavailable_reason`**(시나리오1 E0 5) enum 준수, 우선순위 순): `ROBOT_BUSY`(①위반) →
`ROBOT_OFFLINE`(④위반) → `ROBOT_BUSY`(②위반) → `BATTERY_TOO_LOW`(③위반).
`CHARGING`은 쓰지 않는다(`is_charging`이 항상 false 고정이라 판정 불가).

**접수 성공(200)**: `{"task_id":1024,"assigned_robot_id":"dg_01","status":"ACCEPTED","message":"..."}`
**접수 실패(409)**: `{"status":"REJECTED","reason":"NO_AVAILABLE_ROBOT"|"BATTERY_TOO_LOW"|...,"message":"..."}`

## 통신 요약

- 수신(HTTP :8200): 위 엔드포인트 (Automato Web Service / Postman 호출)
- 수신(ROS2 Service): `/automato/save_detection` (SaveDetection) ← HQ (RP-79 탐지 저장)
- 구독(ROS2): `/automato/telemetry/fleet` (FleetTelemetry) → 로봇별 최신 상태 캐시
- 발신(ROS2 Action): `/{robot_id}/navigate` (Navigate, **Waypoint[] 배열**) → HQ
- 발신(HTTP, RP-79): `POST {AUTOMATO_WEB_SERVICE_URL}/internal/v1/detections/notify` (순찰 현황),
  `POST {AUTOMATO_WEB_SERVICE_URL}/internal/v1/alerts/disease` (병해충 알림)

### 설정(환경변수) — RP-79

| 변수 | 기본값 | 용도 |
|---|---|---|
| `DETECTION_IMAGE_ROOT` | `~/automato_detections` | 병해충 이미지 저장 루트(DB엔 루트 제외 상대경로만) |
| `AUTOMATO_WEB_SERVICE_URL` | `http://localhost:8100` | notify/alert 수신 백엔드 base URL(경로는 코드 상수) |
| `ACS_ALERT_RETRIES` | `3` | disease alert 최대 시도 횟수 |
| `ACS_NOTIFY_TIMEOUT_SEC` / `ACS_ALERT_TIMEOUT_SEC` | `3.0` | 각 HTTP 요청 타임아웃(초) |

## 테스트

```bash
# 가용 판정/선정 순수 로직(ROS/DB 불필요)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_patrol_availability.py -v
# 라우팅/예약 엔진 — 그래프만으로 검증(ROS/DB 불필요)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_routing_engine.py -v
# 텔레메트리 릴레이(RP-77)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_fleet_relay.py -v
# 탐지 저장/중계/알림 순서·게이트·실패정책(RP-79, ROS/DB 불필요 — 협력자 주입)
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_detection_service.py -v
```

### E2E 시뮬 (물리 로봇 없이 접수~주행 전체)

가짜 스탠드인(fake_telemetry / patrol_bridge / fleet_aggregator)을 launch 하나로 띄운다.
ACS 본체는 `.env` 자동 탐색과 로그 분리를 위해 별도 터미널에서 리포 안에서 띄운다.

```bash
# 터미널 1 — 스탠드인 일괄 (로봇 수·배터리·막힘 지점을 인자로 조절)
ros2 launch automato_control_service patrol_e2e_sim.launch.py
ros2 launch automato_control_service patrol_e2e_sim.launch.py \
    robots:=dg_01,dg_02,dg_03 batteries:=90.0,65.0,80.0 fail_waypoint_ids:=14

# 터미널 2 — ACS 본체 (리포 안에서)
ros2 run automato_control_service patrol_node

# 터미널 3 — 조회 / 접수
curl -s localhost:8200/internal/v1/robots/patrol/available | jq
curl -s -X POST localhost:8200/internal/v1/tasks/patrol \
     -H 'Content-Type: application/json' \
     -d '{"robot_selection":"auto","robot_id":null}' | jq
```

순찰은 **전역 1건**만 허용되므로(부분 유니크 인덱스 `ux_tasks_single_active_patrol`),
로봇을 여러 대 띄워도 동시에 주행하는 건 1대다 → 통로 경합·데드락 회피는 이 경로로
관측되지 않는다. 강제 종료로 `IN_PROGRESS`가 남으면 다음 접수가 409가 되니 정리한다:

```bash
docker exec automato-db psql -U robot8 -d automatodb -c \
  "UPDATE tasks SET status='FAILED', ended_at=NOW() WHERE status IN ('WAITING','IN_PROGRESS');"
```

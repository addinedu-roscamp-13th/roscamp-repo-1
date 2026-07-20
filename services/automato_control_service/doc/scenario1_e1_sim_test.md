# 시나리오1 E1(순찰 시작) — ACS 단독 시뮬 테스트 런북

> **목적**: 물리 로봇 없이 `automato_control_service`의 **E1 전 구간**(가용 조회 → 순찰 접수 →
> 로봇 선정 → DB 기록 → Navigate 경로 하달)을 관제 PC 한 대에서 돌려 확인한다.
> 위에서 아래로 따라 하면 된다.
>
> **범위 밖**: 실물 주행(Nav2), E2의 촬영·분석·탐지 저장, 다중 로봇 통로 경합(§7 참고).
> 실물 로봇을 붙이는 절차는 [rp78_field_test.md](rp78_field_test.md)에 따로 있다.

---

## 1. 무엇이 무엇을 흉내내나

E1은 원래 이런 사슬이다: **Web Service → ACS → DG Control Service → DdaGo**.
여기서 ACS만 진짜고, 나머지는 테스트 스탠드인이 대신한다.

| 띄우는 것 | 흉내내는 실물 | 하는 일 |
| --- | --- | --- |
| `fake_telemetry` (로봇마다 1개) | DdaGo 텔레메트리 | `/ddago/telemetry` 1Hz 발행 → **가용 판정의 재료** |
| `fleet_aggregator` (1개) | DG Control Service(취합) | robot_id로 묶어 `/automato/telemetry/fleet` 발행 |
| `patrol_bridge` (로봇마다 1개) | DG Control Service + DdaGo(주행) | `/<robot_id>/navigate` 액션 서버. `sim` 모드면 "도착했다"고만 응답 |
| `curl` | Automato Web Service | `POST /internal/v1/tasks/patrol` 호출 |

앞의 셋은 `patrol_e2e_sim.launch.py` 하나로 함께 뜬다.

> **왜 스탠드인이 필요한가**: ACS의 가용 판정은 "로봇들이 지금 어떤 상태인가"를 보고 내리는데,
> 그 상태를 올려주는 게 로봇의 텔레메트리다. 또 ACS가 경로를 하달하려면 그걸 **받아줄 액션
> 서버**가 있어야 한다(없으면 `wait_for_server` 타임아웃 → 즉시 FAILED). 둘 다 원래 DG Control
> Service 몫인데 아직 없어서 최소 대역만 흉내낸다. 진짜가 준비되면 이 노드들은 버린다.

---

## 2. 사전 준비 (최초 1회 / 코드 바꿨을 때)

```bash
# ① DB 기동 확인 (컨테이너 automato-db)
docker ps --filter name=automato-db
#   안 떠 있으면: cd services/database && docker compose up -d && alembic upgrade head

# ② 워크스페이스 2개 빌드 — 인터페이스(메시지) 먼저, 그 다음 ACS
cd ~/roscamp-repo-1/equip/automato_ws && colcon build --packages-select automato_interfaces
cd ~/roscamp-repo-1/services/automato_control_service && colcon build --packages-select automato_control_service
```

> **왜 워크스페이스가 2개인가**: 메시지/액션 정의(`automato_interfaces`)와 ACS 노드가 서로 다른
> ws에 있다. ACS는 그 메시지 타입을 가져다 쓰므로 **인터페이스를 먼저 빌드·소싱**해야 한다.
> ⚠️ 리포 루트에서 `colcon build`를 돌리면 안 된다(관계 없는 패키지까지 긁는다).

**모든 터미널에서 아래 3줄을 먼저 친다** (순서 중요):

```bash
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
source ~/roscamp-repo-1/services/automato_control_service/install/setup.bash
```

---

## 3. 실행 — 터미널 3개

### 터미널 1 — 스탠드인 일괄 (가짜 로봇들)

```bash
ros2 launch automato_control_service patrol_e2e_sim.launch.py
```

인자로 상황을 바꾼다:

| 인자 | 기본 | 용도 |
| --- | --- | --- |
| `robots` | `dg_01` | 콤마구분 robot_id (`dg_01,dg_02,dg_03`) |
| `batteries` | 전부 90.0 | robots와 같은 순서의 배터리% (`90.0,65.0,80.0`) |
| `sim_seconds` | `1.0` | waypoint 하나당 가짜 이동 시간(초) |
| `fail_waypoint_ids` | 없음 | 막힘으로 응답할 waypoint_id (`14` 또는 `14,7`) |

### 터미널 2 — ACS 본체

```bash
cd ~/roscamp-repo-1        # ★ 리포 안에서 실행할 것
ros2 run automato_control_service patrol_node
```

> **왜 리포 안에서?** ACS는 DB 접속 문자열을 **현재 디렉터리에서 위로 올라가며**
> `services/database/.env`를 찾아 얻는다. 리포 밖에서 띄우면 `DATABASE_URL` 없다고 죽는다.
>
> **왜 launch에 안 넣었나?** 위 이유 + FastAPI 로그가 섞이면 API 확인이 어렵고,
> ACS만 재기동하는 일이 잦기 때문.

### 터미널 3 — 호출

```bash
# ① 가용 조회
curl -s localhost:8200/internal/v1/robots/patrol/available | jq

# ② 순찰 접수 (auto = 시스템이 고름)
curl -s -X POST localhost:8200/internal/v1/tasks/patrol \
     -H 'Content-Type: application/json' \
     -d '{"robot_selection":"auto","robot_id":null}' | jq

# ②' 관리자 지정
curl -s -X POST localhost:8200/internal/v1/tasks/patrol \
     -H 'Content-Type: application/json' \
     -d '{"robot_selection":"manual","robot_id":"dg_01"}' | jq
```

---

## 4. E1 단계별 확인 체크리스트

문서(시나리오1 E1)의 순서대로, **무엇을 보면 그 단계가 됐다고 할 수 있는지**.

| # | E1 단계 | 확인 방법 | 기대 결과 |
| --- | --- | --- | --- |
| 1 | 가용 로봇 조회 | `GET .../patrol/available` | `min_battery_percent: 70`, 로봇별 `available`·`unavailable_reason` |
| 2 | 가용 판정(4조건) | 같은 응답 | 텔레메트리 없는 로봇 = `ROBOT_OFFLINE` |
| 3 | 로봇 선정 | `POST .../tasks/patrol` 응답 | `assigned_robot_id` = 가용 후보 중 **배터리 최고** |
| 4 | task 생성 + 스냅샷 | 아래 SQL ① | `tasks` 1행(`IN_PROGRESS`→ 종료 후 `COMPLETED`), `task_assignment_snapshot` 1행 |
| 5 | 출발점 = 전용 충전소 | ACS 로그 | `dg_01 순찰 시작 노드 = 22(전용 충전소)` |
| 6 | 경로 탐색 + 예약 | ACS 로그 | `세그먼트 하달 task=N 22→[15, 12] 통로=[20, 16] 촬영=True` |
| 7 | Navigate 하달 | 터미널1(브릿지) 로그 | `[TEST] Navigate 수신 task=N waypoints=[15, 12]` |
| 8 | 짝(같은 자리 회전) 촬영 | ACS 로그 | `짝 촬영 하달 ... 부모 10 → 짝 18` / `짝 촬영 완료` |
| 9 | 종료 마감 | ACS 로그 + SQL ① | `순찰 종료 task=N → COMPLETED` |

```sql
-- ① 접수 결과 확인
docker exec automato-db psql -U robot8 -d automatodb -c \
"SELECT task_id,status,assigned_robot_id,started_at,ended_at FROM tasks ORDER BY task_id DESC LIMIT 3;"

-- ② 배정 근거 스냅샷(명령 직전 로봇 상태 전체)
docker exec automato-db psql -U robot8 -d automatodb -c \
"SELECT task_id, robot_id, jsonb_pretty(robot_state_snapshot) FROM task_assignment_snapshot ORDER BY task_id DESC LIMIT 1;"
```

**ACS 로그에서 나오면 정상인 문구**

```
라우팅 그래프 로드: 노드 23(짝 2 제외) / 통로 25
dg_01 순찰 시작 노드 = 22(전용 충전소)
세그먼트 하달 task=3 22→[15, 12] 통로=[20, 16] 촬영=True
목표 도달 task=3 위치 12 (통로 16 유지 — 촬영·짝 처리 후 반납)
짝 촬영 하달 task=3 부모 10 → 짝 18 (같은 자리 제자리 회전, 통로 [11] 유지)
순찰 종료 task=3 → COMPLETED
```

---

## 5. 시나리오별 재현

| 보고 싶은 것 | 방법 |
| --- | --- |
| `BATTERY_TOO_LOW` | `ros2 param set /dg_01/fake_telemetry battery_percent 65.0` (임계 70) |
| `ROBOT_BUSY`(주행 중) | `ros2 param set /dg_01/fake_telemetry nav_status NAVIGATING` |
| `ROBOT_BUSY`(활성 task) | 순찰 도는 중에 다시 `POST` |
| `ROBOT_OFFLINE` | 터미널1을 Ctrl+C → 3초 뒤 조회 |
| auto 선정 규칙 | `robots:=dg_01,dg_02,dg_03 batteries:=80.0,90.0,70.0` → **dg_02**가 뽑히면 정상 |
| `PATROL_IN_PROGRESS` | 순찰 도는 중에 다른 로봇으로 `POST` → 409 |
| 막힘 → 우회 | `fail_waypoint_ids:=14` → 로그에 `세그먼트 막힘 ... 블랙리스트 후 우회` |
| 우회 불가 → 부분 완료 | 막힘 지점을 여러 개 → `COMPLETED_PARTIAL` |

> ⚠️ `ros2 param set`은 **선언된 타입과 정확히** 맞춰야 한다. 실수는 `65.0`처럼 소수점 포함,
> `fail_waypoint_ids`는 문자열(`-p fail_waypoint_ids:='"14"'`).

---

## 6. 정리

```bash
# 노드 종료: 각 터미널 Ctrl+C

# 강제 종료로 IN_PROGRESS가 남으면 다음 접수가 409가 된다 → 정리
docker exec automato-db psql -U robot8 -d automatodb -c \
"UPDATE tasks SET status='FAILED', ended_at=NOW() WHERE status IN ('WAITING','IN_PROGRESS');"
```

---

## 7. 이 방법으로 **관측되지 않는 것** (중요)

### 다중 로봇 통로 경합 · 데드락 회피

순찰은 **전역 1건**만 허용된다(부분 유니크 인덱스 `ux_tasks_single_active_patrol`).
두 번째 `POST`는 409 `PATROL_IN_PROGRESS`로 막히므로, 가짜 로봇을 3대 띄워도
**실제로 주행하는 건 언제나 1대**다. 예약표에 로봇이 하나뿐이면 `try_reserve`가 실패할 일이
없어 `reserve_or_wait`의 `waiting`/`deadlock` 분기는 한 줄도 실행되지 않는다.

→ 이 영역은 `verify_web`의 **순수 파이썬 시뮬**(`PatrolDispatcher` 여러 개 + `FakeNavigateClient`)로
검증한다. HTTP·DB 접수 단계를 건너뛰고 디스패처를 직접 여러 개 돌리는 방식.

### 룩어헤드(다음 구간 선예약)

두 겹의 이유로 안 돈다.

1. **틱이 안 돈다**: 룩어헤드는 결과 대기 루프의 하트비트 틱마다 호출되는데
   기본 `ACS_HEARTBEAT_SEC=5.0`이라, `sim_seconds=1.0`이면 세그먼트가 한 틱보다 빨리 끝나
   틱이 0번 돈다. → `sim_seconds:=6.0` 또는 `ACS_HEARTBEAT_SEC=0.3 ros2 run ...` 으로 해결.
2. **미리 잡을 게 없다**: 로봇이 1대면 경합이 없어 세그먼트가 **목표까지 통째로** 예약된다.
   그러면 세그먼트 끝 = 목표라서 선예약할 다음 구간 자체가 없다(정상 동작).

→ 결국 룩어헤드도 다중 로봇 상황이라야 의미가 생긴다. 위와 같은 이유로 `verify_web` 몫.

# Ddagi 로봇팔 제어 — 선택적 전환(Selective Switching) 구조 설계

> 구현 대상 저장소: `addinedu-roscamp-13th/roscamp-repo-1` (branch: `sprint4-minho`)
> 구현 위치: `equip/automato_ws/src/ddagi_control/ddagi_control/`
> 이 문서는 설계안이며, 실제 코드 작성은 RPi5에 연결된 Claude Code 에이전트가 수행한다.

---

## 1. 배경 / 목표

토마토 수확 시 두 가지 상황이 있다.
- **바깥쪽 노출된 열매**: 장애물 없이 직선 접근 가능
- **잎/줄기에 가려진 안쪽 열매**: 충돌 회피 경로가 필요할 수 있음

이 두 상황을 하나의 인터페이스에서 **자동/수동으로 분기**해 처리하는 구조를 만든다.

---

## 2. 핵심 원리 (구현 전 반드시 숙지)

### 2-1. 두 제어 경로는 결국 같은 하드웨어 API(pymycobot)로 수렴한다

| | 자체 솔버 경로 | MoveIt2 경로 |
|---|---|---|
| 호출 함수 | `pymycobot.send_coords([x,y,z,rx,ry,rz], speed, mode)` | `pymycobot.send_angles([j1..j6], speed)` |
| IK 계산 주체 | 펌웨어 자체 솔버 (그때그때) | MoveIt2가 미리 계산 (OMPL 등) |
| 장애물 인지 | **없음** — API에 장애물 파라미터 자체가 없음 | Planning Scene(옥트리)로 인지 → 충돌검사 |
| 실패 시 | 해가 없으면 그냥 안 움직임, 사유 파악 어려움 | Plan 단계에서 실패 사유 반환 가능 |
| 속도 | 빠름 | 상대적으로 느림 (경로탐색 오버헤드) |

→ **동시 사용 불가** (같은 순간 두 경로가 팔을 잡으면 충돌). **순차 전환은 가능**하되, MoveIt2 실행 후 컨트롤러를 release 하거나 필요한 순간에만 MoveIt2 노드를 활성화해야 함.

### 2-2. MoveIt2 경로 실행 흐름 (제조사 공식 `mycobot_ros2` 패키지 기준)

```
1) ros2 launch mycobot_280arduino_moveit2 demo.launch.py   # Plan (시뮬레이션만, 실물 안 움직임)
2) ros2 run mycobot_280_moveit2_control sync_plan_arduino  # Execute (내부에서 send_angles 호출)
```

### 2-3. 알려진 함정

- MoveIt2 관련 "실물 팔 떨림" 이슈는 대부분 **MoveIt Servo(실시간 연속 스트리밍)** 모드에서 보고됨. 우리는 plan-and-execute(1회성) 방식만 쓰므로 해당 없음.
- 잎이 흔들리는 경우 MoveIt2 기본 구조(스냅샷 1장 → Plan → Execute)는 실행 중 재계획을 하지 않음. 단, **우리 화분은 조화(인공식물)라 흔들림 없음을 전제로 정적 장애물 취급 가능** — 별도 재계획 로직 불필요.
- 실물 연결(pymycobot 시리얼 포트)은 **단일 인스턴스만 열 수 있음**. `telemetry_publisher.py`의 헬스 조회(`get_servo_status` 등)와 팔 제어(`send_coords`/`send_angles`)가 **같은 pymycobot 객체를 공유**해야 시리얼 포트 충돌이 안 남.

### 2-4. 판단 주체 및 데이터 흐름 (확정)

`use_moveit` 판단은 RPi(`ddagi_control`)가 아니라 **AI 서비스 PC가 초기 스캔 시점에 미리 계산**해서 넘겨준다. RPi 쪽에서 실패 후 재시도하는 reactive 방식은 채택하지 않음.

- 로봇팔에 장착된 depth 카메라는 **USB로 AI 서비스 PC에 직결** — 네트워크 전송 지연 없음.
- 순찰용 일반(RGB) 카메라(구역별 숙성도 판단용, `AnalyzeFrame.srv` 경로)와는 **별개의 카메라**.
- 초기 스캔 1회에서 AI 서비스가 한꺼번에 계산해 리스트로 전달:
  1. YOLO로 열매 검출 + 익음/썩음/질병 분류
  2. TF2로 카메라 좌표 → 로봇 base frame 좌표 변환
  3. depth 상 각 좌표 주변 장애물(잎) 밀도 분석 → `use_moveit` 판단
  4. 수확 순서 정렬 (초기 기준: 좌→우, 상→하 — 임의 설정, 실측 후 조정 예정)
- RPi는 **순서 + 좌표 + use_moveit이 이미 결정된 리스트**를 받아 순회만 한다. 접근 직전 실시간 재판단 없음(화분이 조화라 정적 장애물로 취급 가능하므로 사전 판단으로 충분하다고 판단).

---

## 3. 패키지 구조

```
ddagi_control/ddagi_control/
├── __init__.py
├── telemetry_publisher.py     # [기존, 수정 최소화] 헬스상태 1Hz 발행
├── arm_hardware.py            # [신규] pymycobot 실물 연결 싱글턴 — telemetry/controller 공용
├── arm_solver_direct.py       # [신규] send_coords 기반 자체 솔버 경로
├── moveit_bridge.py           # [신규] MoveIt2 액션 클라이언트 (plan + sync_execute 래핑)
├── arm_controller.py          # [신규] 선택적 전환 wrapper — pick_tomato() 진입점
└── harvest_server.py          # [신규 재작성] 액션 서버, arm_controller 호출
```

### 3-1. `arm_hardware.py` — 공용 하드웨어 싱글턴

역할: `pymycobot.MyCobot(...)` 객체를 프로세스 내 단일 인스턴스로 생성해 다른 모듈에 제공.

```python
_arm_instance = None

def get_arm():
    global _arm_instance
    if _arm_instance is None:
        _arm_instance = MyCobot(PORT, BAUD)
    return _arm_instance
```

- `telemetry_publisher.py`의 `read_servo_health()` TODO를 이 함수로 교체할 때도 동일 인스턴스 사용.

### 3-2. `arm_solver_direct.py` — 자체 솔버 경로

```python
GRIPPER_ORIENTATION = [-133, 7, -100]  # 「4. 로봇팔로 집기」 검증된 자세

def pick_direct(arm, coord_xyz, speed=25):
    x, y, z = coord_xyz
    arm.set_gripper_value(100, 50)
    arm.send_coords([x, y, z + 60] + GRIPPER_ORIENTATION, speed, 1)
    arm.send_coords([x, y, z - 7] + GRIPPER_ORIENTATION, speed, 1)
    arm.set_gripper_value(0, 50)
    arm.send_coords([x, y, z + 93] + GRIPPER_ORIENTATION, speed, 1)
    return arm.get_gripper_value() > 10  # 잡힘 판정 (0=빈 그리퍼, 18~22=성공)
```

### 3-3. `moveit_bridge.py` — MoveIt2 경로

```python
def pick_via_moveit(target_pose) -> bool:
    """MoveIt2 action client 로 plan 요청 → 성공 시 sync 실행.
    실패(경로 없음) 시 False 반환, 상위(arm_controller)가 폴백 여부 결정.
    """
    # ros2 action: /move_action (MoveGroup) 등 액션 클라이언트 구현
    # 실행 후 컨트롤러 release 처리 (다음 send_coords 호출과 충돌 방지)
    ...
```

### 3-4. `arm_controller.py` — 선택적 전환 진입점

```python
def pick_tomato(coord_xyz, use_moveit) -> bool:
    """use_moveit은 항상 AI 서비스의 초기 스캔 결과(HarvestTarget.use_moveit)로
    미리 결정되어 전달됨. 로컬 재판단/재시도 없음."""
    arm = get_arm()
    if use_moveit:
        ok = pick_via_moveit(coord_xyz)
        release_moveit_controller()
        return ok  # 실패해도 자체 솔버로 자동 폴백하지 않음 (§6 미확정 사항 참고)
    return pick_direct(arm, coord_xyz)
```

**분기 기준 (확정):**
- `use_moveit` 플래그는 dg_ai_service가 초기 스캔에서 depth 기반 장애물 밀도로 미리 계산해 `HarvestTarget` 리스트에 담아 전달 (§2-4, §3-6 참고).
- RPi 쪽 실패 후 재시도(reactive) 방식은 채택하지 않음.

### 3-5. `harvest_server.py` — 액션 서버

- 인터페이스: `automato_interfaces/action/DdagiHarvest` — **저장소에 아직 존재하지 않음, 신규 정의 필요** (이전 버전 문서에는 "기존 유지"로 되어 있었으나 확인 결과 미정의 상태였음)
- Goal: `HarvestTarget[]` (§3-6) — `order` 순으로 순회
- `execute_callback` 내부에서 각 target마다 `arm_controller.pick_tomato(coord, use_moveit)` 호출
- Feedback 상태(APPROACHING/GRASPING/RETRACTING/SORTING/DONE/ERROR)는 유지

### 3-6. `automato_interfaces` 신규 메시지 — 수확 대상 리스트 (다른 패키지, 참고)

AI 서비스 PC가 초기 스캔 후 넘겨주는 정보를 담을 신규 메시지. 필드/타입은 초안이며 구현 중 확정.

```
# automato_interfaces/msg/HarvestTarget.msg (초안)
int32   target_id
float32 x
float32 y
float32 z
string  ripeness       # ripe / rotten / disease
bool    use_moveit
int32   order          # 수확 순서
```

`harvest_server.py`는 `HarvestTarget[]` 리스트(=`DdagiHarvest` 액션 goal)를 받아 `order` 순으로 순회하며 `arm_controller.pick_tomato()`를 호출한다.

> **구현 확정 (2026-07-09, 실물 테스트 완료)**: 위 초안과 달리 실제로는 저장소 기존 컨벤션(`Patrol.action`/`WaypointGoal.msg`)에 맞춰 좌표는 `float64`, `ripeness`는 `int32`(0:미숙/1:적숙/2:과숙)로 정의했다. 실제 정의는 `automato_interfaces/msg/HarvestTarget.msg`, `automato_interfaces/action/DdagiHarvest.action` 참고.

---

## 4. 구현 순서 (Claude Code 에이전트 작업 단위)

1. [x] `arm_hardware.py` — pymycobot 연결 싱글턴 (실물 연결 테스트 완료 — **baud=1000000**, 115200 아님)
2. [x] `arm_solver_direct.py` — 기존 검증된 자체 솔버 로직 이식, 실물 픽 성공 확인(그리퍼 값 23, 판정 기준 18~22와 일치)
3. [x] `telemetry_publisher.py`의 `read_servo_health()` TODO를 `arm_hardware.get_arm()` 기반으로 교체, RPi5 실물로 pytest 3종 통과 확인
4. [ ] `moveit_bridge.py` — RPi5에 `mycobot_ros2`/MoveIt2 설치 확인 → 액션 클라이언트 구현 (미착수, `arm_controller.pick_tomato()`는 현재 `NotImplementedError` 스텁)
5. [x] `arm_controller.py` — 전환 wrapper 작성 (MoveIt2 분기는 스텁)
6. [x] `harvest_server.py` — `DdagiHarvest` ActionServer 신규 작성(레포 최초 ActionServer), 실물 팔로 목표 1개 픽 성공(`result_code=0`) 확인

---

## 5. 트러블슈팅 참고

| 증상 | 원인 후보 | 대응 |
|---|---|---|
| `send_coords` 호출해도 안 움직임 | 작업반경(~280mm) 밖 또는 특이점 근처 → IK 해 없음 | 좌표 사전 범위체크, 실패 시 MoveIt2 폴백 |
| telemetry와 arm_controller 동시 실행 시 시리얼 오류 | pymycobot 인스턴스 중복 생성 | `arm_hardware.get_arm()` 싱글턴으로 통일 |
| MoveIt2 전환 후 자체 솔버 명령이 씹힘 | 이전 MoveIt2 컨트롤러가 팔을 계속 점유 | 전환 전 `release_moveit_controller()` 확인 |
| **(실물 확인)** `MyCobot280(PORT, BAUD)` 연결해도 `get_gripper_value()` 등이 전부 `-1` 반환 | baud rate가 115200이면 응답 없음 | **baud=1000000**으로 연결해야 정상 통신 (실측 확인) |
| **(실물 확인)** `send_coords()` 연속 호출 시 팔이 관절이 회전한 이상한 자세로 멈춤 | `send_coords()`는 fire-and-forget(비동기) — 고정 `sleep()`으로 완료를 가정하면 다음 명령이 이전 이동을 끊어버림 | `sync_send_coords()`/`sync_send_angles()`(내부적으로 `is_in_position()` 폴링)로 교체 |
| **(실물 확인)** 매번 다른 시작 자세에서 pick을 시작하면 IK 경로가 불안정 | 시작 자세가 고정돼 있지 않음 | 티칭으로 확보한 **홈 포지션(고정 관절각)**에서 항상 시작·종료하도록 `pick_direct()`에 `move_home()` 왕복 추가 |
| **(실물 확인)** `ros2 run`으로 띄운 `telemetry_publisher`/`harvest_server`를 껐다고 생각했는데 명령이 꼬임 | `ros2 run`은 래퍼 프로세스 + 실제 노드 자식 프로세스 구조라, 래퍼만 kill하면 자식이 남아 시리얼 포트를 계속 물고 있음 | `pkill -f <노드이름>`으로 확실히 정리, 또는 `ps aux`로 잔존 프로세스 확인 후 종료 |

---

## 6. 미확정 사항 (구현 중 결정 필요)

- [x] 선택적 전환의 판단 기준 → **AI 서비스 PC가 초기 스캔에서 depth 기반 장애물 밀도로 사전 판단, 리스트로 전달** (§2-4). Reactive 재시도 방식은 채택 안 함.
- [ ] MoveIt2 plan 실패 시 정책: 폴백(자체 솔버 재시도) vs 에러 반환 — **논의 보류. 일단 폴백 없이 실패 반환으로 최소 구현.**
- [ ] `moveit_bridge.py`의 액션 클라이언트가 사용할 정확한 액션/서비스 이름 (RPi5 설치 버전 확인 후 확정)
- [x] `automato_interfaces`에 `HarvestTarget` 메시지 및 `DdagiHarvest` 액션 신규 정의 완료, 실물 RPi5에서 빌드 및 픽 테스트까지 검증됨 (§3-6)
- [ ] 수확 순서 정렬의 구체적 우선순위(좌→우 우선 vs 상→하 우선) — 초기값은 임의 설정, 실제 테스트 후 조정 예정
- [ ] **(신규)** `moveit_bridge.py` 미착수 — `arm_controller.pick_tomato(use_moveit=True)`는 현재 `NotImplementedError` 스텁
- [ ] **(신규)** `dg_ai_service`의 depth 기반 좌표/장애물 판단 파이프라인(TF2 변환, per-box 픽셀좌표 추출, 카메라-로봇베이스 extrinsic 캘리브레이션) — 저장소에 전혀 없음, AI 서비스 PC(RealSense D435/D455)에서 별도 구현·검증 필요
- [ ] **(신규)** `dg_control` 중계(`dg_ai_service → dg_control` TCP, `dg_control → ddagi_control` ROS2 액션 클라이언트) — 미착수
- [ ] **(신규)** `telemetry_publisher`와 `harvest_server`를 동시에 띄우면 각자 `arm_hardware.get_arm()`으로 별도 시리얼 연결을 열어 포트 충돌 위험 — 현재는 "동시 실행 금지"로 회피, 장기적으로 단일 프로세스 통합 또는 공유 메커니즘 필요
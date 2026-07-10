# RP-77 Fleet 텔레메트리 릴레이 — 테스트 절차서

E0 ③④ 구간(`/automato/telemetry/fleet` → `/automato/dashboard/fleet_telemetry` 1Hz 중계)이
잘 동작하는지 로봇 없이 확인하는 절차. **아직 HQ 노드가 없으므로 `ros2 topic pub`으로 HQ를 흉내**낸다.

전체 그림:
```
[터미널 B: HQ 흉내]  ──▶  /automato/telemetry/fleet
                                 │  (구독)
                          [터미널 A: 릴레이 노드]  self._pub.publish(msg)
                                 │  (발행)
                                 ▼
                          /automato/dashboard/fleet_telemetry  ──▶  [터미널 C: QT 흉내 echo/hz]
```

> 💡 ROS2 개념: **토픽(topic)** 은 노드 사이에 메시지가 흐르는 이름 붙은 채널이다.
> `topic pub` = 그 채널에 메시지를 쏘는 명령, `topic echo` = 그 채널을 엿듣는 명령,
> `topic hz` = 그 채널에 메시지가 초당 몇 번 오는지 재는 명령.

---

## 0. 사전 준비 (모든 터미널 공통)

새 터미널을 열 때마다 **매번** 아래 두 줄을 먼저 실행해야 한다.
ROS2 명령과 `automato_interfaces` 메시지 타입을 그 터미널이 알게 만드는 과정이다(=환경 소싱).

```bash
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
```

> 왜 두 번? 첫 줄은 ROS2 본체(Jazzy), 둘째 줄은 우리가 만든 메시지(`FleetTelemetry` 등)를 불러온다.
> 둘째 줄이 없으면 `FleetTelemetry` 타입을 못 찾아 에러가 난다.

**인터페이스가 빌드돼 있는지 확인**(안 돼 있으면 아래 명령으로 빌드):
```bash
ros2 interface show automato_interfaces/msg/FleetTelemetry
```
→ `std_msgs/Header header`, `DdagoTelemetry[] ddagos`, `DdagiTelemetry[] ddagis` 가 보이면 OK.
안 보이면:
```bash
cd ~/roscamp-repo-1/equip/automato_ws
colcon build --packages-select automato_interfaces
source install/setup.bash
```

---

## 1. 가장 빠른 검증 — 단위 테스트 (터미널 1개)

로봇도 발행자도 필요 없이, "원본이 손실 없이 통과되는가"를 자동으로 확인한다.
**이것만 통과해도 핵심 로직(중첩 배열 포함 무손실 전달)은 검증된 것.**

```bash
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
cd ~/roscamp-repo-1/services/automato_control_service
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_fleet_relay.py -v
```

**성공 화면:**
```
test/test_fleet_relay.py::test_relay_passthrough_no_field_loss PASSED   [100%]
============================== 1 passed in 0.31s ===============================
```

> 이 테스트는 ddago 2대 + ddagi 1대(servo 7개)를 채운 FleetTelemetry를 실제 통신(DDS)으로
> 흘려보내고, 나온 결과의 배열 길이·필드가 원본과 같은지 비교한다.

---

## 2. 실시간 파이프라인 검증 (터미널 3개)

실제로 1Hz로 흘러가는 모습을 눈으로 본다. 터미널 3개를 나란히 연다.
(각 터미널에서 **0번 사전 준비 두 줄을 먼저** 실행하는 것 잊지 말 것.)

### 터미널 A — 릴레이 노드 실행

```bash
python3 ~/roscamp-repo-1/services/automato_control_service/automato_control_service/fleet_telemetry_relay.py
```

**성공 화면(시작 직후):**
```
[INFO] [...] [fleet_telemetry_relay]: Fleet 텔레메트리 릴레이 준비: /automato/telemetry/fleet → /automato/dashboard/fleet_telemetry (직접 릴레이, 저장 없음)
```
아직 HQ(터미널 B)를 안 켰으면 1초 뒤 이런 경고가 뜬다(정상 — 워치독이 입력 없음을 알림):
```
[WARN] [...] /automato/telemetry/fleet 입력 대기 중 — HQ FleetTelemetry 발행 확인 필요
```

### 터미널 B — HQ 흉내 (1Hz 발행)

`-r 1` = 초당 1회(1Hz) 발행. 로봇 2대 + 로봇팔 1대 데이터를 흉내 낸다.

```bash
ros2 topic pub -r 1 /automato/telemetry/fleet automato_interfaces/msg/FleetTelemetry \
'{header: {frame_id: automato}, ddagos: [{robot_id: dg_01, task_id: 1024, nav_status: NAVIGATING, battery_percent: 78.5, x: 3.21, y: 1.05}, {robot_id: dg_02, nav_status: IDLE, battery_percent: 62.0}], ddagis: [{robot_id: dg_01, is_paused: false}]}'
```

이걸 켜면 **터미널 A**에 5초마다 이런 로그가 뜬다(throttle로 5초에 한 번만):
```
[INFO] [...] [fleet_telemetry_relay]: 릴레이: ddago 2대 / ddagi 1대
```

### 터미널 C — QT 흉내 ① 내용 확인 (echo)

출력 토픽을 엿들어 1건 출력하고 멈춘다.

```bash
ros2 topic echo --once /automato/dashboard/fleet_telemetry
```

**성공 화면(발췌):** 터미널 B에서 넣은 값이 그대로 나오면 성공.
```yaml
ddagos:
- robot_id: dg_01
  nav_status: NAVIGATING
  x: 3.21
  battery_percent: 78.5
- robot_id: dg_02
  nav_status: IDLE
  battery_percent: 62.0
ddagis:
- robot_id: dg_01
  servo_health: [ ...7개... ]
```

### 터미널 C — QT 흉내 ② 주기 확인 (hz)

```bash
ros2 topic hz /automato/dashboard/fleet_telemetry
```

**성공 화면:** `average rate`가 **1.0 근처**면 1Hz 중계 성공. (Ctrl+C로 멈춤)
```
average rate: 1.000
	min: 0.997s max: 1.003s std dev: 0.00317s window: 2
```

---

## 3. 워치독(입력 끊김 감지) 확인

터미널 A(릴레이)를 켜 둔 채로 **터미널 B(HQ 흉내)를 Ctrl+C로 끈다.**
3초쯤 지나면 터미널 A에 아래 경고가 뜨면 정상:
```
[WARN] [...] 입력 3.4s 끊김 — 재발행도 멈춤(진단 목적상 정상 동작)
```
> 직접 릴레이는 입력이 끊기면 출력도 멈춘다(오래된 값을 지어내지 않음).
> 워치독은 "왜 멈췄는지"만 알려줄 뿐, 재발행은 하지 않는다.

---

## 4. 완료 조건(DoD) 체크리스트

| 확인 항목 | 방법 | 통과 기준 |
| --- | --- | --- |
| 수신 → 재발행 동작 | 2번 터미널 C echo | 터미널 B에서 넣은 값이 출력 토픽에 그대로 나옴 |
| 1Hz 재발행 | 2번 터미널 C hz | `average rate` ≈ 1.0 |
| 배열 원본 손실 없음 | 1번 pytest | `1 passed` (ddago/ddagi/servo 배열 필드 일치) |
| 입력 끊김 안내 | 3번 워치독 | HQ 끄면 "입력 끊김" 경고 |

넷 다 통과하면 RP-77 구현이 정상 동작하는 것이다.
(QT 대시보드 화면 표시는 System Admin App 쪽 구현이 붙은 뒤 이 출력 토픽을 구독해 확인.)

---

## 5. (선택) `ros2 run` / `ros2 launch`로 실행하기

2번에서는 `python3 파일.py`로 바로 실행했다. ROS2 표준 방식(`ros2 run`/`ros2 launch`)으로 쓰려면
services 패키지를 colcon으로 한 번 빌드해야 한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
cd ~/roscamp-repo-1/services/automato_control_service
colcon build --symlink-install      # 이 패키지를 빌드 → install/ 생성
source install/setup.bash
```
그러면 아래처럼 실행 가능(터미널 A 대체):
```bash
ros2 run automato_control_service fleet_telemetry_relay
# 또는 launch 로:
ros2 launch automato_control_service fleet_telemetry_relay.launch.py
```
> 참고: 같은 패키지의 `control_node`(RP-54)는 `OpHarvest` 액션 미정의로 실행 시 임포트 에러가 난다.
> 이는 RP-77과 무관하며, 우리 릴레이(`fleet_telemetry_relay`)는 영향받지 않는다(다른 실행파일).

---

## 6. 트러블슈팅

| 증상 | 원인/해결 |
| --- | --- |
| `Unknown message type automato_interfaces/msg/FleetTelemetry` | 0번 둘째 소싱 줄을 안 했거나 인터페이스 미빌드. 0번 재확인 |
| 터미널 C echo에 아무것도 안 나옴 | 터미널 A(릴레이)·B(발행) 둘 다 켜졌는지 확인. 세 터미널 모두 0번 소싱했는지 확인 |
| hz가 1.0이 아님 | 터미널 B의 `-r 1`(1Hz) 확인. 여러 발행자가 동시에 떠 있지 않은지 확인 |
| 터미널마다 서로 안 보임 | 같은 `ROS_DOMAIN_ID`인지 확인(기본 0). 한 PC 같은 셸이면 보통 문제없음 |
| 로그가 안 보이고 종료 | 파이프(`|`)로 넘길 땐 `python3 -u`(언버퍼)로 실행하면 로그가 즉시 나온다 |

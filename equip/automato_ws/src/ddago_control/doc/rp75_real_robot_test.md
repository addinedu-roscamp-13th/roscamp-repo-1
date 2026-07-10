# RP-75 텔레메트리 — 실물 핑키 프로 검증 절차 (2단계)

> 1단계(로봇 없이 로직 검증)는 완료(빌드·단위테스트·린트 통과).
> 이 문서는 **실물 로봇을 붙여 DoD를 최종 확정**하는 2단계 체크리스트다.
> 로봇 앞에서 위에서 아래로 따라 하면 된다.

---

## 실행 위치 표기 규칙 (이 문서의 약속)

명령마다 **어디서 치는지**를 아래 라벨로 표시한다. 라벨을 무시하고 아무 데서나
치면 토픽이 안 잡히거나 드라이버가 안 뜬다.

| 라벨 | 실제 위치 | 접속 방법 |
| --- | --- | --- |
| 🤖 **[로봇 dg_01]** | dg_01 로봇의 RPi5 | 관제 PC에서 `ssh pinky@<dg_01 IP>` 로 접속한 터미널 |
| 🤖 **[로봇 dg_02]** | dg_02 로봇의 RPi5 | 관제 PC에서 `ssh pinky@<dg_02 IP>` 로 접속한 터미널 |
| 🖥️ **[관제 PC]** | 관제/개발 노트북 | 그냥 관제 PC의 로컬 터미널 |

> **왜 SSH?** 로봇(RPi5)엔 보통 모니터가 없어서, 관제 PC에서 SSH로 각 로봇에
> 원격 접속해 로봇 쪽 명령을 친다. SSH 터미널 = "그 로봇 안에서 치는 것"과 같다.

### 무엇을 어디서 돌리나 (확정 배치)

| 실행 대상 | 도는 기기 | 왜 여기서 |
| --- | --- | --- |
| 드라이버 wrapper (`ddago_bringup`) | 🤖 **각 로봇 RPi5** | 센서·모터에 직결. 핑키 드라이버가 **그 로봇에만** 설치돼 있어 다른 데선 안 뜸 |
| nav2 (`pinky_navigation`) | 🤖 **각 로봇 RPi5** | scan/odom/TF 등 **고주기 데이터를 로컬에서 소비**(네트워크 절약). 로봇마다 1세트 필요 |
| 텔레메트리 노드 (`ddago_telemetry`) | 🤖 **각 로봇 RPi5** | 각 로봇이 **자기 상태를 스스로 발행**. 관제 PC 부하 분산·로봇 확장에 유리 |
| 확인·명령 (`topic echo/hz`, teleop, goal 전송) | 🖥️ **관제 PC** | 사람이 보고 조작하는 검증·명령은 한 곳(관제 PC)에서 모아서 |

> **원칙 한 줄:** *각 로봇은 자기 스택 3종을 자기 RPi5에서 돌린다. 관제 PC는 오직
> 확인과 명령만 한다.* 로봇이 2대든 3대든 이 원칙을 그대로 복제하면 된다.

---

## 0. 사전 준비 (환경 전제)

- **모든 기기가 같은 `ROS_DOMAIN_ID` + 같은 공유기 네트워크** (팀 결정사항).
  - `ROS_DOMAIN_ID` = 같은 번호를 쓰는 기기끼리만 서로 토픽이 보이는 "채널 번호".
    로봇 2대 + 관제 PC가 서로 통신하려면 셋 다 같은 번호여야 한다.
  - 확인: 각 기기(🤖 dg_01, 🤖 dg_02, 🖥️ 관제 PC) 터미널에서 `echo $ROS_DOMAIN_ID`
    → 세 값이 모두 같아야 함.
  - 다르면 서로 토픽이 안 보인다. `export ROS_DOMAIN_ID=<번호>` 로 통일.
- **핑키 드라이버 패키지가 각 로봇에 설치**돼 있어야 함:
  `pinky_bringup`, `pinky_sensor_adc`, `pinky_navigation` (+ 의존 `sllidar_ros2` 등).
  → 🤖 **각 로봇 dg_01, dg_02 양쪽** 모두에 설치돼 있어야 한다.
- **우리 코드(`automato_ws`)를 관제 PC → 각 로봇으로 복사** (rsync).
  소스는 🖥️ 관제 PC에만 있으므로, 각 로봇 RPi5로 밀어넣는다. **소스만** 보내고
  `build/`·`install/`·`log/` 는 반드시 제외한다 — 이들은 관제 PC(x86)에서 빌드된
  산물이라 로봇(ARM/RPi5)에 그대로 넣으면 아키텍처가 안 맞아 깨진다. 로봇에서는
  아래 "우리 패키지 빌드"로 **다시 빌드**해야 한다.
  ```bash
  # 🖥️ [관제 PC] — dg_01 로 소스 복사 (equip/ 의 상위, 즉 automato_ws 를 가리킬 수 있는 위치에서)
  rsync -av --delete \
    --exclude 'build/' --exclude 'install/' --exclude 'log/' \
    equip/automato_ws/ pinky@<dg_01 IP>:~/automato_ws/

  # 🖥️ [관제 PC] — dg_02 로도 동일 (IP만 변경)
  rsync -av --delete \
    --exclude 'build/' --exclude 'install/' --exclude 'log/' \
    equip/automato_ws/ pinky@<dg_02 IP>:~/automato_ws/
  ```
  - `-a` 아카이브(권한·타임스탬프 보존), `-v` 진행 로그, `--delete` 관제 PC에 없는
    파일은 로봇에서도 삭제(소스를 그대로 미러링).
  - source 경로 끝의 `/`(`automato_ws/`)는 "디렉터리 **내용을**" 대상 안으로 넣으라는
    뜻. 이게 없으면 `~/automato_ws/automato_ws/` 처럼 한 겹 더 들어간다(경로 중첩 사고).
  - **코드를 고칠 때마다** 이 rsync 를 다시 돌려 로봇에 최신 소스를 반영한다.
- **우리 패키지 빌드** — 돌리는 곳/보는 곳에 따라 다르다:
  - 🤖 **각 로봇 RPi5** (텔레메트리 노드를 실행하므로 필수). 방금 rsync 로 받은 소스에서:
    ```bash
    cd ~/automato_ws
    colcon build --packages-select automato_interfaces ddago_control --symlink-install
    source install/setup.bash
    ```
  - 🖥️ **관제 PC** (직접 노드는 안 돌리지만, `ddago/telemetry` 를 `echo` 하려면
    커스텀 메시지 타입 정의가 있어야 역직렬화됨 → **`automato_interfaces` 는 필수**):
    ```bash
    cd equip/automato_ws
    colcon build --packages-select automato_interfaces --symlink-install
    source install/setup.bash
    ```
    > `automato_interfaces` 가 없으면 관제 PC의 `ros2 topic echo /dg_01/ddago/telemetry`
    > 가 "unknown type" 으로 실패한다. 토픽은 보여도 내용을 못 푼다.

---

## 1. 로봇 1대(dg_01) 기동 — 먼저 1대로 감을 잡는다

> 2대를 한꺼번에 띄우기 전에, 먼저 dg_01 한 대로 파이프라인이 도는지 확인한다.
> **아래 T1~T3 는 모두 🤖 dg_01 RPi5(SSH 터미널)에서** 실행한다.
> 각 터미널에서 `source /opt/ros/jazzy/setup.bash && source install/setup.bash` 먼저.

```bash
# 🤖 [로봇 dg_01] 터미널 T1 — 드라이버(odom, 배터리, 초음파)를 /dg_01 네임스페이스로
ros2 launch ddago_control ddago_bringup.launch.py robot_id:=dg_01

# 🤖 [로봇 dg_01] 터미널 T2 — nav2 위치추정(amcl_pose). 맵 등 인자는 팀 nav2 실행법에 맞게.
ros2 launch pinky_navigation localization_launch.xml namespace:=/dg_01
#   (nav_status/goal 주행까지 볼 거면 T2b 로 navigation_launch.xml namespace:=/dg_01 도)

# 🤖 [로봇 dg_01] 터미널 T3 — 우리 텔레메트리 발행 노드
ros2 launch ddago_control ddago_telemetry.launch.py robot_id:=dg_01
```

이제 🖥️ **관제 PC** 로 넘어가 아래 2·3장을 확인한다. 1대가 통과하면 2장으로.

---

## 2. 로봇 2대(dg_01 + dg_02) 동시 기동

핵심은 간단하다: **1장을 dg_02에도 똑같이 하되 `robot_id:=dg_02` 로만 바꾼다.**
각 로봇은 자기 RPi5에서 자기 스택을 돌리므로, 두 로봇의 명령은 **서로 다른 SSH 터미널**
(dg_01용 3개, dg_02용 3개)에서 친다.

### 2-1. dg_01 스택 — 🤖 dg_01 RPi5 (SSH 터미널 3개)
```bash
# 🤖 [로봇 dg_01] T1
ros2 launch ddago_control ddago_bringup.launch.py robot_id:=dg_01
# 🤖 [로봇 dg_01] T2
ros2 launch pinky_navigation localization_launch.xml namespace:=/dg_01
# 🤖 [로봇 dg_01] T3
ros2 launch ddago_control ddago_telemetry.launch.py robot_id:=dg_01
```

### 2-2. dg_02 스택 — 🤖 dg_02 RPi5 (SSH 터미널 3개)
```bash
# 🤖 [로봇 dg_02] T1
ros2 launch ddago_control ddago_bringup.launch.py robot_id:=dg_02
# 🤖 [로봇 dg_02] T2
ros2 launch pinky_navigation localization_launch.xml namespace:=/dg_02
# 🤖 [로봇 dg_02] T3
ros2 launch ddago_control ddago_telemetry.launch.py robot_id:=dg_02
```

> **왜 `robot_id` 만 바꾸면 되나?** `ddago_bringup`·`ddago_telemetry` launch 는
> `robot_id` 값을 그대로 **네임스페이스 접두어**로 씌운다(`PushRosNamespace`). 그래서
> dg_01 스택의 모든 토픽은 `/dg_01/...`, dg_02 스택은 `/dg_02/...` 로 자동으로 갈라진다.
> 같은 도메인·같은 네트워크에 있어도 이름공간이 달라 **섞이지 않는다.**

### 2-3. 기동 후 두 로봇이 다 보이는지 (🖥️ 관제 PC)
```bash
# 🖥️ [관제 PC]
ros2 topic list | grep ddago/telemetry
#  /dg_01/ddago/telemetry
#  /dg_02/ddago/telemetry     ← 둘 다 뜨면 성공
```
안 보이는 로봇이 있으면 → 그 로봇의 T1~T3 가 다 떴는지, `ROS_DOMAIN_ID` 가 관제 PC와
같은지(0장), 3장 네임스페이스 확인으로 내려간다.

> **로봇 3대(dg_03)로 늘릴 때도 동일:** 🤖 dg_03 RPi5에서 `robot_id:=dg_03` 으로
> T1~T3 를 한 세트 더 돌리면 끝. 관제 PC는 아무것도 새로 안 띄운다.

---

## 3. ★가장 먼저★ 네임스페이스 확인 (실패의 90%가 여기) — 🖥️ 관제 PC

드라이버 토픽이 정말 `/dg_01/...`(그리고 2대면 `/dg_02/...`)로 올라오는지부터 본다.

```bash
# 🖥️ [관제 PC]
ros2 topic list | grep dg_01     # 2대면 grep dg_02 도
```

**기대 결과** (dg_01 기준, 아래가 다 보여야 함):
```
/dg_01/odom
/dg_01/amcl_pose
/dg_01/battery/percent
/dg_01/battery/voltage
/dg_01/us_sensor/range
/dg_01/batt_state
/dg_01/navigate_to_pose/_action/status
/dg_01/ddago/telemetry            ← 우리가 발행하는 것
```

- ❌ 만약 `/odom`, `/battery/percent` 처럼 **`/dg_01` 없이** 전역으로 뜨면
  → 드라이버가 토픽 이름을 절대경로로 박아둔 것. **아래 [부록 A] 리매핑**으로 대응.
- ⚠️ `/dg_01/batt_state` 는 있지만 **우리는 안 씀**(percentage가 NaN이라). 배터리는
  `/dg_01/battery/percent`·`/battery/voltage` 를 쓴다.

---

## 4. DoD 검증 체크리스트 — 🖥️ 관제 PC

> 아래 확인은 전부 🖥️ **관제 PC** 에서 한다(사람이 값을 눈으로 보고 판정).
> 로봇을 움직이는 teleop 명령도 관제 PC에서 쏜다.

### 4-1. 발행 주기 1Hz
```bash
# 🖥️ [관제 PC]
ros2 topic hz /dg_01/ddago/telemetry
```
- [ ] `average rate: ~1.0` 확인.

### 4-2. 필드가 실제 값으로 채워지는지 (하드코딩 아님)
```bash
# 🖥️ [관제 PC]
ros2 topic echo /dg_01/ddago/telemetry
```
한 메시지를 보면서 아래를 확인:

- [ ] **battery_voltage** — 0이 아닌 실제 전압(예: 11~12V대).
- [ ] **battery_percent** — 실제 값. **⚠️ 스케일 확인**:
  - `78.0` 처럼 나오면 → 정상(0~100).
  - `0.78` 처럼 **1 미만**으로 나오면 → 소스가 0~1임. 🤖 **dg_01 RPi5의 T3** 를 끄고
    아래로 재실행:
    ```bash
    # 🤖 [로봇 dg_01] T3
    ros2 launch ddago_control ddago_telemetry.launch.py robot_id:=dg_01 \
      battery_percent_scale:=100.0
    ```
    (그래도 계속 그러면 노드 기본값을 100.0으로 바꿔 커밋)
- [ ] **is_charging** — **항상 `false`** 가 맞음(핑키는 충전상태 미제공). 충전 케이블 꽂아도
      false면 정상. (하드웨어 충전감지선 유무는 별도 팀 확인 항목)
- [ ] **us_range_m** — 손을 초음파 센서 앞에 가까이/멀리 → 값이 변함.

### 4-3. 이동 시 위치 실시간 반영
텔레옵(키보드 등)으로 로봇을 움직이며 확인. teleop 도 🖥️ **관제 PC** 에서 실행하되,
**어느 로봇에 보낼지 네임스페이스로 지정**해야 한다(안 그러면 명령이 어느 로봇에도 안 감):
```bash
# 🖥️ [관제 PC] — dg_01 을 조종: cmd_vel 을 /dg_01/cmd_vel 로 리매핑
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/dg_01/cmd_vel

# 🖥️ [관제 PC] — 다른 터미널에서 위치가 변하는지 관찰
ros2 topic echo /dg_01/ddago/telemetry --field x
```
- [ ] **x / y / yaw** 가 이동에 따라 변함.
- [ ] `header.frame_id` 확인: amcl 잡히면 `map`, 아니면 `odom`.

### 4-4. nav_status (Nav2 연동)
먼저 🤖 **해당 로봇 RPi5** 에 nav2 주행 스택(navigation)이 떠 있어야 한다.
1장 T2 에서 localization 만 띄웠다면, 주행 상태를 보려면 navigation 을 추가로 띄운다:
```bash
# 🤖 [로봇 dg_01] T2b — navigate_to_pose 액션(주행) 활성화
ros2 launch pinky_navigation navigation_launch.xml namespace:=/dg_01
```
그다음 🖥️ **관제 PC** 에서 목표점(goal)을 하나 보낸다(RViz2 의 "Nav2 Goal" 버튼이나
`/dg_01/navigate_to_pose` 액션 전송). 보낸 뒤:
- [ ] 주행 중 → `nav_status: NAVIGATING`
- [ ] 도착 후 → `nav_status: IDLE`
- [ ] (목표 취소 시 `CANCELING`, 실패 시 `FAILED` 나오면 더 좋음)

### 4-5. 네임스페이스 충돌 없음 (2대 동시)
2장에서 dg_01·dg_02 를 모두 기동한 상태로, 🖥️ **관제 PC** 에서:
```bash
# 🖥️ [관제 PC]
ros2 topic list | grep ddago/telemetry
#  /dg_01/ddago/telemetry
#  /dg_02/ddago/telemetry     ← 둘이 독립적으로 뜨면 성공
ros2 topic echo /dg_02/ddago/telemetry   # dg_01과 다른 로봇 값이 나오는지
```
- [ ] 두 토픽이 서로 안 섞이고 각자 값 발행.
- [ ] dg_01 을 teleop 으로 움직여도(4-3) **dg_02 의 x/y/yaw 는 안 변함**(반대도 동일).
      → 명령·상태가 로봇별로 완전히 분리됐다는 결정적 증거.

### 4-6. QoS 매칭 실측 (데이터가 실제로 들어오는지)
어떤 소스 토픽에 데이터는 흐르는데 우리 필드가 계속 0이고, 🤖 **로봇 T3** 로그에
`아직 수신되지 않은 소스: ...` 경고가 반복되면 → **QoS 불일치** 의심.
```bash
# 🖥️ [관제 PC] (또는 🤖 로봇에서 로컬로 확인해도 됨)
ros2 topic info /dg_01/battery/percent -v   # 배터리
ros2 topic info /dg_01/us_sensor/range -v   # 초음파
ros2 topic info /dg_01/navigate_to_pose/_action/status -v  # nav 상태
```
- [ ] 각 토픽의 Publisher/Subscriber **Reliability·Durability**가 호환되는지 확인.
- 우리 구독자 QoS(참고):
  | 토픽 | 우리 구독 QoS |
  | --- | --- |
  | odom, amcl_pose, battery/percent·voltage | RELIABLE, depth 10 |
  | us_sensor/range | BEST_EFFORT (sensor) |
  | navigate.../status | RELIABLE + TRANSIENT_LOCAL |

---

## 5. 문제 해결(Troubleshooting)

| 증상 | 원인 | 조치 (실행 위치) |
| --- | --- | --- |
| 관제 PC에서 로봇 토픽이 아예 안 보임 | ROS_DOMAIN_ID 불일치 or 다른 네트워크 | 🖥️🤖 세 기기 `echo $ROS_DOMAIN_ID` 통일(0장) |
| `echo telemetry` 가 unknown type | 관제 PC에 `automato_interfaces` 미빌드 | 🖥️ 관제 PC에서 `automato_interfaces` colcon build(0장) |
| 위치 x/y/yaw 계속 0 | odom·amcl 둘 다 미수신 | 3장 네임스페이스 확인. 🖥️ `ros2 topic echo /dg_01/odom` 데이터 오나 |
| battery_percent 0 | battery/percent 미수신 or NaN | 🖥️ `echo /dg_01/battery/percent` 존재·값 확인 |
| battery_percent가 0.xx | 소스가 0~1 스케일 | 🤖 로봇 T3 를 `battery_percent_scale:=100.0` 로 재기동(4-2) |
| nav_status 계속 IDLE | status 미수신 | 🤖 로봇에 navigation 떠 있나(4-4), TRANSIENT_LOCAL QoS, 🖥️ goal 실제로 보냈나 |
| us_range_m 0 고정 | sensor_adc 미기동/미수신 | 🤖 로봇 T1 wrapper가 `pinky_sensor_adc` 띄웠는지 |
| teleop 쳐도 로봇이 안 움직임 | cmd_vel 네임스페이스 미지정 | 🖥️ `-r cmd_vel:=/dg_01/cmd_vel` 붙였는지(4-3) |
| 토픽이 `/dg_01` 없이 전역 | 드라이버 절대경로 하드코딩 | [부록 A] 리매핑 |
| dg_01·dg_02 값이 섞임 | 한쪽이 네임스페이스 없이 전역 발행 | 3장에서 어느 토픽이 전역인지 찾아 [부록 A] |

---

## 부록 A. 드라이버가 전역 토픽으로 뜰 때(리매핑 대안)

3장에서 `/dg_01/odom` 이 아니라 `/odom` 으로 뜨면, 그 노드가 토픽명을 절대경로로
박아둔 것. 이땐 우리 텔레메트리 노드 쪽에서 **구독 대상을 실제 토픽으로 리매핑**한다.
`ddago_telemetry.launch.py` 의 `Node(...)` 에 `remappings` 를 추가:
```python
remappings=[
    ('odom', '/odom'),
    ('battery/percent', '/battery/percent'),
    # ... 전역으로 뜬 것만
]
```
단, **같은 도메인에 로봇이 여러 대면 전역 토픽은 서로 섞이므로** 이 방식은
임시방편이다(dg_01·dg_02 가 같은 `/odom` 을 두고 충돌). 근본 해결은 드라이버를
네임스페이스로 올리는 것(부록 없이 정상 케이스). **로봇 2대 테스트에서는 부록 A로
때우지 말고 반드시 네임스페이스 정상화를 우선한다.**

---

## 6. 검증 완료 후

- [ ] `battery_percent_scale` 최종값 확정(1.0 or 100.0) → 필요 시 코드 반영.
- [ ] is_charging 하드웨어 충전감지선 유무 팀 확인 결과 기록.
- [ ] 2대 동시(4-5) 통과 확인 — 명령·상태가 로봇별로 분리됨.
- [ ] 모든 체크 통과하면 → **커밋 후 push** (RP-75 완료).

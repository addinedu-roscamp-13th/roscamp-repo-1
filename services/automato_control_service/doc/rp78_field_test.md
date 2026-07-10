# RP-78 순찰 관제 — 실물 로봇 텔레메트리 검증 런북

> ACS(Automato Control Service)의 **순찰 로봇 선정·DB 기록·HQ 순찰 Action 하달·교통관제**를
> 실물 핑키 프로의 **진짜 텔레메트리**로 검증하는 절차서. 로봇 앞에서 **위에서 아래로** 따라 하면 된다.
>
> **이 문서의 범위 — 관제 로직만, 로봇은 주행하지 않는다.** RP-78은 "관제"라서 로봇이 실제로
> 움직일 필요가 없다. 실물 로봇은 **텔레메트리(자기 상태) 발행 용도로만** 연결하고, 순찰 명령을
> 받는 `patrol_bridge`는 **`sim` 모드**로 둬서 "도착했다"고 가짜로 답만 한다(로봇은 정지).
> 그래서 **Nav2(자율주행)는 띄우지 않는다.** 로봇이 실제로 순찰지점을 도는 실주행은 이 문서
> 범위 밖이며, 원래 그 대역을 맡을 HQ(DG Control Service)가 준비되면 별도로 다룬다.
>
> **중요(전제 부품):** RP-78이 돌려면 (a) 로봇 텔레메트리를 모아 `FleetTelemetry`로 발행하는
> 취합 노드, (b) ACS의 `/dg_0x/patrol` 명령을 받아줄 액션 서버가 필요하다. 이 둘은 원래
> **DG Control Service(HQ)** 몫인데 아직 없어서, 그 최소 대역만 흉내내는 **테스트 전용 스탠드인**
> 2개를 이 저장소(`automato_control_service/test_harness/`)에 넣어 두었다.
> **실제 DG Control Service가 아니다.** 그게 준비되면 이 두 노드는 걷어내면 된다.

---

## 0. 실행 위치 표기 규칙 (이 문서의 약속)

명령마다 **어디서 치는지**를 아래 라벨로 표시한다. 라벨을 무시하고 아무 데서나 치면
토픽이 안 잡히거나 노드가 안 뜬다.

| 라벨 | 실제 위치 | 접속 방법 |
| --- | --- | --- |
| 🤖 **[로봇 dg_01]** | dg_01 로봇의 RPi5 | 관제 PC에서 `ssh pinky@<dg_01 IP>` 로 접속한 터미널 |
| 🤖 **[로봇 dg_02]** | dg_02 로봇의 RPi5 | `ssh pinky@<dg_02 IP>` |
| 🤖 **[로봇 dg_03]** | dg_03 로봇의 RPi5 | `ssh pinky@<dg_03 IP>` |
| 🖥️ **[관제 PC]** | 관제/개발 노트북 | 그냥 관제 PC의 로컬 터미널 |

> **왜 SSH?** 로봇(RPi5)엔 모니터가 없어서, 관제 PC에서 SSH로 각 로봇에 원격 접속해
> 로봇 쪽 명령을 친다. SSH 터미널 = "그 로봇 안에서 치는 것"과 같다.

### 무엇을 어디서 돌리나 (확정 배치)

| 실행 대상 | 도는 기기 | 역할 |
| --- | --- | --- |
| 드라이버 `ddago_bringup` | 🤖 각 로봇 | 센서·모터 (odom, battery, us_sensor) |
| 텔레메트리 `ddago_telemetry` | 🤖 각 로봇 | 자기 상태 발행 `/dg_0x/ddago/telemetry` |
| **`patrol_bridge` (테스트)** | 🤖 각 로봇 | ACS의 `/dg_0x/patrol` 받아 **sim 응답**(도착했다고 가짜로 답, 주행 안 함) |
| PostgreSQL (docker) | 🖥️ 관제 PC | tasks/task_paths/snapshot 저장 |
| **`fleet_aggregator` (테스트)** | 🖥️ 관제 PC | 로봇 텔레메트리 → `/automato/telemetry/fleet` 취합 |
| **`fake_telemetry` (테스트)** | 🖥️ 관제 PC | 물리 로봇 없이 가짜 로봇 상태 발행 → **로봇 1대로 T3·T7** 검증 |
| **ACS `patrol_node` (RP-78 본체)** | 🖥️ 관제 PC | 선정·기록·순찰 하달·교통관제 + HTTP API(:8200) |
| 확인·명령 (`curl`, `ros2 topic/action`, `psql`) | 🖥️ 관제 PC | 사람이 보고 조작 |

> **원칙:** 각 로봇은 자기 스택 3종(드라이버·텔레메트리·patrol_bridge)을 자기 RPi5에서 돌린다.
> **Nav2 는 안 띄운다**(로봇을 주행시키지 않으므로). 관제 PC는 DB·취합·ACS·확인만 한다.
> 로봇이 1대든 3대든 이 원칙을 복제하면 된다.

---

## 1. 데이터 흐름 한눈에

```
🤖 로봇 dg_01                              🖥️ 관제 PC
┌───────────────────────────┐             ┌──────────────────────────────────┐
│ 드라이버(odom,batt,us)     │             │ fleet_aggregator (테스트)         │
│ ddago_telemetry ───────────┼──telemetry─▶│   /dg_0x/ddago/telemetry 구독     │
│  (텔레메트리만 발행)       │             │   → /automato/telemetry/fleet 발행│
│                            │             │            │                     │
│ patrol_bridge (테스트,sim) │  Patrol     │            ▼                     │
│   /dg_01/patrol 서버 ◀─────┼─────────────┼── ACS patrol_node (RP-78)         │
│     └▶ "도착" 가짜 응답    │  액션 하달  │   - available/accept API (:8200)  │
│        (로봇 정지)         │             │   - 로봇 선정·통로 예약·우회      │
└───────────────────────────┘             │   - DB 기록 ─▶ PostgreSQL(docker) │
                                          └──────────────────────────────────┘
```

핵심만: **로봇이 상태를 올리면(telemetry) → 취합(aggregator) → ACS가 보고 로봇을 고르고(available/accept)
→ DB에 기록하고 → `/dg_0x/patrol`로 "거기 가라"를 한 구간씩 하달 → patrol_bridge가 sim으로 "도착" 응답(로봇 정지).**

---

## 2. 사전 준비 (한 번만)

### 2-1. 인벤토리 — 먼저 이 표를 채워라

| 항목 | 값(채우기) |
| --- | --- |
| 팀 `ROS_DOMAIN_ID` | ____ (예: 8) |
| dg_01 IP / 계정 | ____ / pinky |
| dg_02 IP / 계정 | ____ / pinky |
| dg_03 IP / 계정 | ____ / pinky |
| 관제 PC 저장소 경로 | `~/roscamp-repo-1` (예시) |
| 로봇 저장소 경로 | `~/automato_ws` (예시, RP-75와 동일) |

> **몇 대로 테스트하나 — 이 문서는 로봇 1~3대를 모두 지원한다(3대 필수 아님).**
> 표에 dg_01\~dg_03 이 있다고 3대가 꼭 있어야 하는 게 아니다. **가진 물리 로봇 수만큼만** 채운다.
> - **물리 로봇**: 있는 대수만큼 자기 스택(드라이버·텔레메트리·bridge)을 돌린다. 아래 rsync/빌드(2-3)와
>   Step B 도 **물리 로봇 대수만큼만** 반복한다. **1대면 dg_01 만 하고 dg_02/dg_03 부분은 건너뛴다.**
> - **가짜 로봇**: 로봇이 더 있는 것처럼 보여야 하는 T3·T7 은, 물리 로봇 대신
>   **가짜 로봇(Step B-2)** 을 관제 PC 에서 띄운다. 가짜 로봇은 rsync/빌드 대상이 **아니다**.
>
> 즉 **기본 경로 = "물리 로봇 1대 + 필요 시 가짜 로봇"**. dg_02/dg_03 이 진짜 여러 대처럼 나오는
> 예시들은 물리 로봇이 여럿일 때를 위한 것이니, 1대면 그 부분만 가짜로 바꿔 읽으면 된다.

### 2-2. 네트워크 · 도메인 · 시계 (가장 흔한 실패 원인)

- **같은 공유기 + 같은 `ROS_DOMAIN_ID`.** 다르면 서로 토픽이 안 보인다.
  - `ROS_DOMAIN_ID` = 같은 번호끼리만 통신되는 "채널 번호".
  - 각 기기(🤖 로봇들, 🖥️ 관제 PC)에서 확인:
    ```bash
    echo $ROS_DOMAIN_ID        # 모두 같아야 함. 다르면: export ROS_DOMAIN_ID=<번호>
    ```
  - `~/.bashrc` 에 `export ROS_DOMAIN_ID=<번호>` 를 넣어두면 매번 안 쳐도 된다.

- **⏰ 시계 동기화 (RP-78에서 특히 중요).** ACS의 가용 판정에는 "최근 3초 이내 수신"
  조건이 있고, 이 3초를 **로봇이 찍은 시각(header.stamp) vs 관제 PC 시각**으로 비교한다.
  로봇 시계가 관제 PC보다 3초 이상 느리면 **멀쩡한 로봇도 `TELEMETRY_STALE`** 로 뜬다.
  - 각 기기에서 확인(세 값이 1초 이내로 비슷해야 함):
    ```bash
    date -u +%H:%M:%S.%N
    ```
  - 어긋나면(인터넷 되면) 각 기기에서:
    ```bash
    sudo apt install -y chrony && sudo systemctl restart chrony
    sudo chronyc makestep
    ```
  - 인터넷이 안 되면(폐쇄망) 관제 PC 시각으로 각 로봇을 맞춘다:
    ```bash
    # 🖥️ [관제 PC] 로봇 시계를 관제 PC 로 강제 동기 (로봇마다)
    ssh pinky@<dg_01 IP> "sudo date -s '$(date -u '+%Y-%m-%d %H:%M:%S') UTC'"
    ```
  - 정 안 되면 임시로 ACS의 stale 기준을 늘릴 수도 있다(권장 아님):
    `automato_control_service/patrol_api.py` 의 `STALE_SEC = 3.0` 을 키운다.

### 2-3. 소스 배포(rsync) + 빌드 — Patrol.action 이 바뀌었으니 **모든 기기 재빌드 필수**

RP-78에서 `Patrol.action` 을 "단일 waypoint"로 바꿨다. 이 인터페이스를 쓰는 **모든 기기**
(관제 PC + 각 로봇)에서 `automato_interfaces` 를 다시 빌드해야 새 타입이 반영된다.
그런데 **아직 원격 저장소에 push 하지 않았으므로 로봇에서 `git pull` 로는 최신 코드를 못 받는다.**
그래서 관제 PC의 **로컬 작업본을 로봇으로 직접 복사(rsync)** 한다. 이 절 전체가 그 절차다.

> **📦 코드와 데이터는 별개 트랙이다(중요).** rsync 는 **로봇으로 코드(ROS2 워크스페이스)만**
> 보낸다. 순찰 지점 데이터(`waypoints`·`corridors` 테이블)는 **관제 PC 의 PostgreSQL 안에만** 있고
> rsync 대상이 아니다(로봇엔 DB 가 없다). 이 데이터는 마이그레이션(0002/0003)에 이미 확정돼 있어
> 관제 PC 에서 `alembic upgrade head` 한 번이면 시드된다(3장) — 로봇 재배포와 무관하다.

> **rsync 가 뭐고 왜 쓰나 (초보자용).** `rsync` 는 두 위치의 파일을 **차이나는 부분만** 골라
> 복사·동기화하는 도구다(SSH 위로 동작). `scp` 는 매번 전체를 덮어쓰지만 `rsync` 는 바뀐 파일만
> 보내고, `--delete` 로 "원본에서 지운 파일"도 로봇에서 지워 **양쪽을 똑같이** 맞춘다.
> push 전이라 git 을 못 쓰는 지금 상황에 가장 적합하다.

#### (0) 시작 전 확인 — 관제 PC → 로봇 SSH 가 되는지

```bash
# 🖥️ [관제 PC] 로봇에 SSH 접속되는지 먼저 확인 (IP 는 2-1 인벤토리 값)
ssh pinky@<dg_01 IP> "echo OK; hostname; \
  ls ~/automato_ws >/dev/null 2>&1 && echo '워크스페이스 있음' || echo '워크스페이스 아직 없음(신규 생성됨)'"
#   매번 비밀번호 치기 싫으면(선택): ssh-copy-id pinky@<dg_01 IP>  로 키 등록
#   rsync 설치 확인(대개 기본 설치): 관제 PC → which rsync / 로봇 → ssh ... "which rsync"
```

#### (1) 먼저 `--dry-run` 으로 "무엇이 복사/삭제될지" 미리보기 (실제 변경 없음)

```bash
# 🖥️ [관제 PC]
cd ~/roscamp-repo-1
rsync -avn --delete \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  equip/automato_ws/ pinky@<dg_01 IP>:~/automato_ws/
#   -n(=--dry-run): 실제로 안 옮기고 목록만 출력. "deleting ..." 줄로 무엇이 지워질지도 보여준다.
#   목록이 예상과 맞으면 (2) 에서 -n 만 빼고 실제 실행.
```

#### (2) 실제 복사 (rsync)

```bash
# 🖥️ [관제 PC]
cd ~/roscamp-repo-1
rsync -av --delete \
  --exclude 'build/' --exclude 'install/' --exclude 'log/' \
  equip/automato_ws/ pinky@<dg_01 IP>:~/automato_ws/
```

플래그 뜻:

| 플래그 | 뜻 | 왜 |
| --- | --- | --- |
| `-a` | 아카이브(권한·심볼릭·타임스탬프 보존 + 재귀) | 소스 트리를 그대로 옮기려고 |
| `-v` | 자세히(옮긴 파일 출력) | 무엇이 갔는지 눈으로 확인 |
| `--delete` | 원본에 없는 파일은 대상에서도 삭제 | 로봇을 관제 PC 와 **완전히 동일**하게(옛 파일 잔재 제거) |
| `--exclude build/ install/ log/` | 이 3개는 복사 제외 | **아키텍처가 다름**: 관제 PC(x86_64)에서 빌드한 바이너리는 로봇(ARM64/RPi5)에서 안 돈다. 소스만 보내고 **빌드는 로봇에서 새로** 한다 |

> **⚠️ 경로 끝 슬래시(`/`) 주의 — rsync 최대 함정.** `equip/automato_ws/` 처럼 **소스 끝에 `/` 를
> 붙이면** "그 폴더의 **내용물**을 대상 폴더 안에" 넣는다. 슬래시를 빼면(`equip/automato_ws`)
> "automato_ws 폴더 **자체**를 대상 안에" 넣어 `~/automato_ws/automato_ws/` 처럼 한 겹 더 생긴다.
> **반드시 소스·대상 모두 끝에 `/` 를 붙여** 위 예시대로 쓴다.

#### (3) patrol_bridge.py 따로 복사 (automato_ws 밖의 파일)

`patrol_bridge`(테스트 스탠드인)는 `services/...` 아래라 automato_ws 에 안 들어간다. 단독 파일이라 `scp` 로 충분.

```bash
# 🖥️ [관제 PC]
scp services/automato_control_service/automato_control_service/test_harness/patrol_bridge.py \
    pinky@<dg_01 IP>:~/patrol_bridge.py
```

#### (4) 로봇에서 "제대로 갔는지" 확인 (빌드 전에)

```bash
# 🤖 [로봇 dg_01] 새 Patrol.action 이 왔는지 = 단일 waypoint 정의인지 확인
cat ~/automato_ws/src/automato_interfaces/action/Patrol.action
#   Goal 쪽에 'WaypointGoal   waypoint' 한 줄(단일 목적지)이 보이면 최신.
#   예전 정의(배열/다중)면 rsync 가 안 된 것 → (1)(2) 재확인.
ls ~/patrol_bridge.py            # 파일이 보이면 (3) 성공
```

#### (5) 로봇에서 빌드 (한 번만 — 코드 바뀔 때마다)

```bash
# 🤖 [로봇 dg_01]
source /opt/ros/jazzy/setup.bash
cd ~/automato_ws
colcon build --packages-select automato_interfaces ddago_control
source install/setup.bash
```

#### (6) 관제 PC 빌드 (인터페이스만 — 여긴 ddago_control 불필요)

```bash
# 🖥️ [관제 PC]
cd ~/roscamp-repo-1/equip/automato_ws
colcon build --packages-select automato_interfaces
source install/setup.bash
```

#### (7) 물리 로봇이 2대 이상일 때만 — dg_02, dg_03 반복

위 **(1)\~(5) 를 IP 만 바꿔 물리 로봇 대수만큼** 반복한다. 관제 PC 빌드(6)은 한 번이면 된다.
**물리 로봇이 1대면 이 (7)은 건너뛴다** — 추가 로봇은 Step B-2 의 **가짜 로봇**으로 대체하며,
가짜 로봇은 rsync/빌드가 필요 없다(관제 PC 소프트웨어).

> **한 줄 요약.** (관제 PC) rsync 코드 복사 → patrol_bridge scp → (로봇) `cat` 으로 확인 →
> (로봇·관제 PC 각각) `colcon build automato_interfaces` → 소싱. 이걸 **로봇 대수만큼**.
>
> **참고:** 이 검증은 로봇을 주행시키지 않으므로 **Nav2(`pinky_navigation`)는 띄우지도 빌드하지도
> 않는다.** 로봇에서 빌드할 건 위 `automato_interfaces`·`ddago_control` 뿐이다.

### 2-4. 관제 PC — ACS 실행 환경 (한 번만)

ACS는 rclpy(ROS) + FastAPI/psycopg(pip) 를 함께 쓴다. 시스템 rclpy가 보이는 venv를 만든다.

```bash
# 🖥️ [관제 PC]
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
cd ~/roscamp-repo-1/services/automato_control_service
python3 -m venv --system-site-packages .venv-acs   # 시스템 rclpy 를 그대로 보이게
source .venv-acs/bin/activate
pip install -r requirements.txt                     # fastapi, uvicorn, psycopg, ...
```

---

## 3. Waypoint · Corridor — 이미 시드됨(확인만)

> **개념.** *waypoint* = 순찰이 들르는 지점(맵 좌표 x,y). *corridor* = 두 지점 사이를
> 로봇이 **직접(중간 경유 없이) 갈 수 있는 통로**. ACS는 waypoint를 `patrol_order` 순서로 돌고,
> 한 통로가 막히면 다른 통로로 우회한다. 그래서 "지점 좌표"와 "어디끼리 통하는지(통로)"가 DB에 있어야 한다.

**이 데이터는 직접 넣을 필요가 없다.** 실제 맵에서 뽑은 좌표·통로가 마이그레이션에 이미 들어 있어
Step A 의 `alembic upgrade head` 가 자동으로 시드한다:

- **0002** — waypoint 19개(순찰점 12 + 비순찰점 7). 순찰점만 `patrol_order` 1~12 를 가진다.
- **0003** — 실제 맵 통로 19쌍(무방향 간선).

> **⚠️ DB 데이터를 바꿨다면 ACS 재시작.** ACS(`patrol_node`)는 라우팅 그래프를 **첫 순찰 때 한 번
> 읽어 캐시**한다(성능 목적). 실행 중 DB 를 바꿔도 이미 로드했으면 옛 그래프를 계속 쓴다. Ctrl+C 후
> 다시 띄우면 시작 로그 `라우팅 그래프 로드: 노드 N / 통로 M` 의 N/M 이 새 값으로 나온다.
> 로봇은 좌표를 DB 에서 읽지 않으므로(ACS 가 Patrol Goal 에 실어 하달) 로봇 재배포는 불필요하다.

### 3-1. 확인 — 시드가 제대로 들어갔는지

```bash
# 🖥️ [관제 PC] DB 컨테이너로 psql 접속
cd ~/roscamp-repo-1/services/database
docker compose exec postgres psql -U robot8 -d automatodb
```

```sql
-- 순찰점 목록(방문 순서). 여기의 waypoint_id 를 T8(막힘 시뮬)에서 하나 골라 쓴다.
SELECT waypoint_id, x_coord, y_coord, patrol_order
  FROM waypoints WHERE is_patrol_point ORDER BY patrol_order;   -- 순찰점 12행

-- 통로 목록
SELECT corridor_id, waypoint_a_id, waypoint_b_id FROM corridors ORDER BY corridor_id;  -- 통로 19행
```

> **T8 용 id 하나 골라 둬라.** 위 순찰점 목록에서 중간쯤 지점 하나의 `waypoint_id` 를 적어두면
> T8(막힘→우회) 에서 "그 지점을 막았을 때 관제가 우회하는지"를 볼 때 쓴다.

---

## 4. 브링업 (기동 순서) — 순서대로

> 터미널이 여러 개 필요하다. 각 로봇마다 SSH 터미널 3개(드라이버/텔레메트리/bridge),
> 관제 PC에 3개(aggregator/ACS/확인)를 열어 두면 편하다.

### Step A. 🖥️ [관제 PC] DB 기동

```bash
cd ~/roscamp-repo-1/services/database
docker compose up -d
docker compose ps                       # STATUS 가 healthy
source .venv/bin/activate
alembic upgrade head                    # 0003(corridors 시드 로직)까지 적용
python smoke_check.py                   # ✅ DB 연결 성공
```
> `alembic upgrade head` 가 waypoints/corridors 실데이터까지 시드한다(3장). 별도 삽입은 필요 없다.

### Step B. 🤖 [각 로봇] 로봇 스택 (드라이버 → 텔레메트리 → bridge)

각 로봇에서 **터미널 3개**. 모두 먼저 소싱:
```bash
source /opt/ros/jazzy/setup.bash
source ~/automato_ws/install/setup.bash
```

```bash
# 🤖 [로봇 dg_01] 터미널1 — 드라이버 (odom·배터리 등 센서 발행의 토대)
ros2 launch ddago_control ddago_bringup.launch.py robot_id:=dg_01

# 🤖 [로봇 dg_01] 터미널2 — 텔레메트리 발행 (ACS 가 읽는 자기 상태)
ros2 launch ddago_control ddago_telemetry.launch.py robot_id:=dg_01

# 🤖 [로봇 dg_01] 터미널3 — patrol_bridge (sim: 순찰 명령을 받아 "도착"으로 가짜 응답, 로봇 정지)
python3 ~/patrol_bridge.py --ros-args -r __ns:=/dg_01 -p robot_id:=dg_01 -p mode:=sim
```

> **Nav2 는 안 띄운다.** 이 검증은 로봇을 주행시키지 않으므로(bridge=sim) localization/navigation
> 이 필요 없다. 텔레메트리의 좌표(current_position)는 amcl 없이 odom 기준(또는 0,0)으로 나올 수
> 있는데, RP-78 가용 판정·선정·하달 로직은 좌표값을 조건으로 쓰지 않으므로 검증에 지장이 없다.

dg_02, dg_03 도 `dg_01` 만 바꿔 동일하게 반복 — **단, 물리 로봇이 여러 대일 때만.**
물리 로봇이 1대면 여기까진 **dg_01 만** 하고, 추가 로봇은 아래 Step B-2 의 가짜 로봇으로 대신한다.

### Step B-2. 🖥️ [관제 PC] (선택) 가짜 로봇 추가 — 물리 로봇 1대로 다중 로봇 테스트

물리 로봇이 **1대뿐이어도** 관제 PC 에서 **가짜 로봇**을 띄우면 T3(선정 비교)·T7(통로 경합)까지
검증할 수 있다. 여기서 "로봇 하나 = 텔레메트리(가짜) + patrol_bridge(sim)" 두 조각이고 둘 다
소프트웨어다. 진짜 dg_01 은 Step B 대로 두고, 아래를 **가짜 로봇 대수만큼(dg_02, dg_03 …)** 반복한다.

```bash
# 🖥️ [관제 PC] 공통 소싱
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
cd ~/roscamp-repo-1/services/automato_control_service

# 터미널 ① — 가짜 dg_02 텔레메트리 (배터리 90, IDLE = 가용 상태로 시작)
python3 -m automato_control_service.test_harness.fake_telemetry \
  --ros-args -r __ns:=/dg_02 -p battery_percent:=90.0

# 터미널 ② — 가짜 dg_02 의 patrol 액션 서버 (sim)
python3 -m automato_control_service.test_harness.patrol_bridge \
  --ros-args -r __ns:=/dg_02 -p robot_id:=dg_02 -p mode:=sim
```

> **값은 실행 중에 바꿀 수 있다(재시작 불필요).** `fake_telemetry` 는 파라미터를 매 초 다시 읽는다.
> ```bash
> ros2 param set /dg_02/fake_telemetry battery_percent 65.0   # → BATTERY_TOO_LOW 유도
> ros2 param set /dg_02/fake_telemetry nav_status NAVIGATING   # → ROBOT_BUSY 유도
> ros2 param set /dg_02/fake_telemetry nav_status IDLE         # → 다시 가용
> ```
> 멈추면(Ctrl+C) 3초 뒤 `TELEMETRY_STALE`. **진짜 로봇과 같은 id 를 가짜로 또 내지 말 것**(토픽 충돌).
> 소수 파라미터는 `65.0` 처럼 소수점을 붙여야 타입이 맞는다.

> **⚠️ 아래 Step C 의 aggregator `robot_ids` 에 가짜 id 를 반드시 포함**해야 취합된다.
> 예: 진짜 dg_01 + 가짜 dg_02 → `robot_ids:="['dg_01','dg_02']"`.

### Step C. 🖥️ [관제 PC] 텔레메트리 취합 (테스트)

```bash
# 🖥️ [관제 PC] 새 터미널
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
cd ~/roscamp-repo-1/services/automato_control_service
# 실제로 발행 중인 로봇만 지정(진짜+가짜). 1대면 ['dg_01'], 진짜1+가짜1이면 ['dg_01','dg_02']
python3 -m automato_control_service.test_harness.fleet_aggregator \
  --ros-args -p robot_ids:="['dg_01','dg_02','dg_03']"
```
> `robot_ids` 에 넣었지만 실제로 아무도 발행 안 하는 id(예: 안 띄운 dg_03)는 available 에서
> `TELEMETRY_STALE` 로 뜬다(정상). 헷갈리면 **켜져 있는 로봇만** 나열하라.

### Step D. 🖥️ [관제 PC] ACS 본체 (RP-78)

```bash
# 🖥️ [관제 PC] 새 터미널
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
cd ~/roscamp-repo-1/services/automato_control_service
source .venv-acs/bin/activate
python3 -m automato_control_service.patrol_node
#   → "라우팅 그래프 로드: 노드 N / 통로 M" 로그가 뜨면 DB 그래프 적재 OK
#   → HTTP API 는 :8200 (변경: ACS_API_PORT=8200)
```

---

## 5. 테스트 시나리오 (관제 테스트 목록)

각 시나리오: **목적 / 실행(어디서) / 관찰(기대) / 판정(체크박스)**.
모두 **patrol_bridge=sim**(로봇 정지) 기준이다 — 로봇은 안 움직이고 관제 로직만 검증한다.

> 준비물: 관제 PC에 확인용 터미널 하나. `jq` 없으면 `python3 -m json.tool` 로 대체.

> **잘 됐는지 어떻게 확인하나 (판정 3대 창구).** 각 시나리오는 아래 세 곳 중 하나(또는 둘)를
> 보고 판정한다. 시나리오마다 **기대값**과 **체크박스**를 달아 뒀으니, 관찰이 기대와 맞으면 체크한다.
> 1. **HTTP 응답** — `curl` 결과의 JSON(가용 목록/접수 결과/사유). "성공 예시" 블록과 비교.
> 2. **ACS 로그(Step D 터미널)** — 예약/하달/우회/양보/종료가 다 찍히는 **관제 로그**. T6~T8 핵심.
> 3. **DB(psql)** — `tasks`/`task_paths`/`snapshot` 상태 전이. T5·T9 핵심.
>
> **아래 전체가 통과하면 관제(선정·기록·하달·교통관제·막힘대응) 로직이 검증된 것**이다.
> 최종 합격 요약은 이 절 끝의 **합격 체크리스트** 참고.

### T0. 배선 확인 (먼저)

```bash
# 🖥️ [관제 PC]
ros2 node list                          # patrol_control_node, fleet_aggregator, 로봇 노드들 보임
ros2 topic hz /automato/telemetry/fleet # 약 1Hz 로 흐르면 취합 OK
ros2 topic echo /automato/telemetry/fleet --once   # ddagos[] 에 로봇들 보임
ros2 action list | grep patrol          # /dg_01/patrol (+02/03) 보이면 bridge OK
curl -s localhost:8200/health           # {"ok":true,...}
```
- [ ] fleet 토픽 1Hz 수신, action `/dg_0x/patrol` 존재, ACS health OK

### T1. 텔레메트리 캐시 → available (실제 로봇 상태 반영)

- **목적:** ACS가 실물 로봇의 배터리·상태를 읽어 가용 목록을 만든다.
```bash
# 🖥️ [관제 PC]
curl -s localhost:8200/internal/v1/robots/patrol/available | python3 -m json.tool
```
- **기대:** `robots[]` 에 실제 `battery_percent`, `status:"IDLE"`, `current_position` 이 찍힘.
  `min_battery_percent: 70`. 정지·충전된 로봇이면 `available: true`.
- **성공 예시(응답):**
  ```json
  {
    "requested_at": "2026-07-10T01:23:45+00:00",
    "min_battery_percent": 70,
    "robots": [
      {"robot_id": "dg_01", "status": "IDLE", "battery_percent": 88.0,
       "current_position": {"x": 3.2, "y": 1.0}, "available": true},
      {"robot_id": "dg_02", "status": "IDLE", "battery_percent": 90.0,
       "current_position": {"x": 0.0, "y": 0.0}, "available": true}
    ]
  }
  ```
- [ ] 실제 배터리/좌표가 응답에 반영됨  - [ ] IDLE·배터리충분 로봇이 `available:true`

### T2. 가용 판정 4조건 (불가 사유)

- **목적:** 조건 미달 로봇이 올바른 사유로 제외되는지.
- **가장 쉬운 방법 — 가짜 로봇(Step B-2)으로 각 사유를 즉석에서 만든다.**
  진짜 로봇은 배터리를 70 밑으로 떨어뜨리기 어렵지만, 가짜 로봇은 `ros2 param set` 한 줄로 된다.
  각 명령 후 `available` 를 다시 호출해 dg_02 의 `unavailable_reason` 을 확인한다.
  ```bash
  # 🖥️ [관제 PC]  (호출 사이에 available 재조회)
  ros2 param set /dg_02/fake_telemetry battery_percent 65.0    # → BATTERY_TOO_LOW
  ros2 param set /dg_02/fake_telemetry battery_percent 90.0    #   (원복)
  ros2 param set /dg_02/fake_telemetry nav_status NAVIGATING   # → ROBOT_BUSY
  ros2 param set /dg_02/fake_telemetry nav_status IDLE         #   (원복)
  # TELEMETRY_STALE: 가짜 텔레메트리 노드(터미널①)를 Ctrl+C 로 끄고 3초 뒤 available 재조회
  # (참고) 충전으로 만들어도 판정은 그대로 → is_charging 은 영향 없음:
  ros2 param set /dg_02/fake_telemetry is_charging true        # available 여전히 true 여야 함
  ```
- **진짜 로봇으로도 확인(선택):** dg_01 을 teleop 로 움직이면(nav_status≠IDLE) `ROBOT_BUSY`,
  텔레메트리 노드를 끄면 3초 뒤 `TELEMETRY_STALE`.
- [ ] 주행중→ROBOT_BUSY  - [ ] 저배터리→BATTERY_TOO_LOW  - [ ] 미수신→TELEMETRY_STALE
- [ ] (참고) 충전 여부(is_charging)는 판정에 **영향 없음**(true 로 바꿔도 available 유지)

### T3. auto 선정 (배터리 최고, 동점 시 id 오름차순)

> **로봇 1대로 검증하려면 가짜 로봇이 필요**하다(비교하려면 후보가 2대 이상이어야 함).
> Step B-2 로 가짜 dg_02 를 띄우고, 배터리를 진짜 dg_01 과 다르게/같게 세팅해 선정 규칙을 확인한다.
> **주의:** 접수(accept)가 성공하면 그 로봇은 활성 task 를 갖게 되어 다음 선정에서 제외된다.
> 여러 번 시도하려면 사이사이 정리한다(빠르게: 아래 SQL 로 방금 task 를 DONE 처리).

```bash
# 🖥️ [관제 PC] (1) 배터리 최고 고르기 — dg_01 이 90 이면 dg_02 를 더 높게 줘서 dg_02 가 뽑히는지
ros2 param set /dg_02/fake_telemetry battery_percent 95.0
curl -s localhost:8200/internal/v1/robots/patrol/available | python3 -m json.tool   # 배터리들 먼저 확인
curl -s -X POST localhost:8200/internal/v1/tasks/patrol \
  -H 'Content-Type: application/json' -d '{"robot_selection":"auto"}' | python3 -m json.tool
#   → assigned_robot_id 가 배터리 최고(dg_02)여야 함

# (2) 동점 시 id 오름차순 — 두 배터리를 같게 맞추고 다시 auto → 더 작은 id(dg_01) 선정
#     (아래 '초기화' 로 이전 task 를 정리한 뒤 실행)
ros2 param set /dg_02/fake_telemetry battery_percent 90.0    # dg_01 과 동일하게

# (초기화) 방금 접수로 생긴 활성 task 를 DONE 처리해 로봇을 다시 가용으로
docker compose -f ~/roscamp-repo-1/services/database/docker-compose.yml exec -T postgres \
  psql -U robot8 -d automatodb -c \
  "UPDATE tasks SET status='DONE', ended_at=NOW() WHERE status IN ('WAITING','IN_PROGRESS');"
```
- **기대:** 가용 로봇 중 **배터리 최고**가 `assigned_robot_id` 로 200 ACCEPTED. 동점이면 **id 오름차순**
  첫 번째. 가용 로봇이 없으면 409 `NO_AVAILABLE_ROBOT`.
- **성공 예시(응답):**
  ```json
  {"task_id": 12, "assigned_robot_id": "dg_02", "status": "ACCEPTED", "message": "..."}
  ```
- [ ] 배터리 최고 로봇 선정  - [ ] 동점 시 id 오름차순  - [ ] task_id 발급 + ACCEPTED

### T4. manual 선정 + 미달 시 409

```bash
# 🖥️ [관제 PC] 특정 로봇 지정
curl -s -X POST localhost:8200/internal/v1/tasks/patrol \
  -H 'Content-Type: application/json' -d '{"robot_selection":"manual","robot_id":"dg_02"}' | python3 -m json.tool
```
- **기대:** dg_02 가용이면 배정. **미달(주행중/저배터리/미수신)이면 409 + 해당 reason**.
  (GUI가 막더라도 백엔드가 배정 직전 재검증)
- [ ] 가용 로봇 배정  - [ ] 미달 로봇 지정 시 409+사유

### T5. DB 기록 (트랜잭션) + 중복 배정 차단

- **목적:** 접수가 tasks/task_paths/snapshot 을 한 트랜잭션으로 남기고, 같은 로봇 중복 배정을 막는지.
```bash
# 🖥️ [관제 PC] 방금 접수한 task 확인
cd ~/roscamp-repo-1/services/database
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT task_id,status,assigned_robot_id,started_at FROM tasks ORDER BY task_id DESC LIMIT 3;"
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT task_id,point_index,waypoint_id,is_visited FROM task_paths ORDER BY task_id DESC,point_index LIMIT 20;"
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT task_id,robot_id,robot_state_snapshot->>'robot_id' rid,assigned_at FROM task_assignment_snapshot ORDER BY id DESC LIMIT 3;"
# 중복: 같은 로봇에 순찰이 진행중일 때 다시 manual 로 그 로봇 접수 → 409
```
- [ ] tasks IN_PROGRESS + started_at  - [ ] task_paths `point_index` 0부터 연속
- [ ] snapshot 에 명령직전 상태 저장  - [ ] 진행중 로봇 재접수 → 409 NO_AVAILABLE_ROBOT

### T6. 단일 로봇 순찰 완주 (세그먼트 하달 → DONE)

- **목적:** 접수 후 ACS가 waypoint를 **한 개씩** 하달하고 bridge(sim)가 "도착"으로 답한다.
  로봇은 안 움직이지만 전체 방문 순서·DB 상태 전이가 끝까지 이어지는지 확인.
```bash
# 🖥️ [관제 PC] 접수(auto 또는 manual) 후, ACS 로그(터미널 D)와 DB를 관찰
#  - ACS 로그: "순찰 디스패치 시작", 구간별 예약/하달, "순찰 종료 task=.. → DONE"
#  - patrol_bridge 로그(🤖 터미널3): "Patrol 수신 waypoint=.. → result_code=0"
watch -n1 'docker compose -f ~/roscamp-repo-1/services/database/docker-compose.yml \
  exec -T postgres psql -U robot8 -d automatodb -c \
  "SELECT task_id,status FROM tasks ORDER BY task_id DESC LIMIT 1;"'
```
- **기대:** 모든 지점 방문 후 tasks `DONE`.
- [ ] 구간별 하달 로그 + tasks=DONE

### T7. 다중 로봇 통로 경합 → 양보 (진짜 1대 + 가짜 1대로도 가능)

- **목적:** 두 로봇이 같은 통로를 원할 때 한 대만 들어가고(안전속성) 다른 대는 대기/양보하는지.
- **로봇 1대로 검증:** 진짜 dg_01(로봇의 bridge) + **가짜 dg_02**(Step B-2, 관제 PC)로 두 대를 만든다.
  예약표는 ACS 안에 있어 로봇이 안 움직여도 경합이 생기고, `sim_seconds` 를 길게 주면 한 로봇이
  통로를 오래 잡아 경합이 잘 보인다. **두 bridge 모두 `sim_seconds:=15` 로 (재)기동:**
```bash
# 🤖 [로봇 dg_01] 터미널3 — 진짜 로봇 bridge 를 긴 이동시간으로
python3 ~/patrol_bridge.py --ros-args -r __ns:=/dg_01 -p robot_id:=dg_01 -p mode:=sim -p sim_seconds:=15
# 🖥️ [관제 PC] 가짜 dg_02 bridge(Step B-2 터미널②)를 sim_seconds 15 로 (재)기동
python3 -m automato_control_service.test_harness.patrol_bridge \
  --ros-args -r __ns:=/dg_02 -p robot_id:=dg_02 -p mode:=sim -p sim_seconds:=15
# 🖥️ [관제 PC] 두 로봇에 거의 동시에 순찰 접수(manual 로 각각)
curl -s -X POST localhost:8200/internal/v1/tasks/patrol -H 'Content-Type: application/json' \
  -d '{"robot_selection":"manual","robot_id":"dg_01"}' >/dev/null &
curl -s -X POST localhost:8200/internal/v1/tasks/patrol -H 'Content-Type: application/json' \
  -d '{"robot_selection":"manual","robot_id":"dg_02"}' >/dev/null &
```
- **관찰:** ACS 로그에 한 로봇은 통로 예약 성공, 다른 로봇은
  `통로 N 예약 대기 …` → 타임아웃 시 `→ 순찰 양보`(우회 또는 순서 미룸).
  **같은 통로가 두 로봇에 동시에 잡히는 로그는 절대 없어야 한다(안전속성).**
- [ ] 한 통로엔 한 로봇만  - [ ] 늦은 로봇이 대기→양보(우회/미룸)  - [ ] 데드락 없이 둘 다 진행

### T8. 막힘 → 블랙리스트 → 우회 / 건너뜀

- **목적:** 특정 지점을 막힘으로 응답하게 해서 우회/건너뜀 로직을 안전 검증(로봇 정지).
  (3-1 확인 SQL 의 순찰점 목록에서 `waypoint_id` 하나를 고른다. 예: 6)
```bash
# 🤖 [로봇 dg_01] 6번 waypoint 를 항상 막힘(1)으로 응답 (id 는 3-1 에서 고른 순찰점)
python3 ~/patrol_bridge.py --ros-args -r __ns:=/dg_01 -p mode:=sim -p fail_waypoint_ids:="6"
# 🖥️ [관제 PC] dg_01 에 순찰 접수 후 ACS 로그 관찰
```
- **관찰:** ACS 로그 `통로 X 막힘 보고 → 블랙리스트 후 우회 시도`.
  실제 맵 그래프에 대체 경로가 있으면 그리로 우회하고, 그래도 그 지점을 못 가면
  `경로 없음 … → 건너뜀`, 마지막에 재시도. 일부만 방문하면 tasks `PARTIAL`.
- [ ] 막힘→우회 시도 로그  - [ ] 우회 불가 시 건너뜀 + PARTIAL

### T9. 결과 상태 확인

```bash
# 🖥️ [관제 PC] 최종 상태
docker compose exec postgres psql -U robot8 -d automatodb -c \
 "SELECT task_id,status,started_at,ended_at FROM tasks ORDER BY task_id DESC LIMIT 5;"
```
- [ ] 완주 DONE / 일부 PARTIAL / 실패 FAILED 가 상황에 맞게 기록

### ✅ 합격 체크리스트 (관제 5기능 ↔ 시나리오)

아래가 모두 체크되면 **RP-78 관제 로직이 검증된 것**이다(실물 로봇 1대 + 필요 시 가짜 로봇 기준).

| 관제 기능 | 확인 시나리오 | 판정 창구 | 통과 |
| --- | --- | --- | --- |
| ① 로봇 선정 (가용 4조건) | T1, T2 | HTTP 응답(사유) | [ ] |
| ① 로봇 선정 (auto/manual) | T3, T4 | HTTP 응답(assigned/409) | [ ] |
| ② DB 기록 (트랜잭션·중복차단) | T5 | psql(tasks/paths/snapshot) | [ ] |
| ③ 순찰 하달 (세그먼트 완주) | T6 | ACS 로그 + tasks=DONE | [ ] |
| ④ 교통관제 (경합→양보, 한 통로 한 대) | T7 | ACS 로그(예약/대기/양보) | [ ] |
| ⑤ 막힘 대응 (블랙리스트→우회→건너뜀) | T8 | ACS 로그 + tasks=PARTIAL | [ ] |
| (결과 마감) DONE/PARTIAL/FAILED | T9 | psql(tasks 상태) | [ ] |

> **가장 중요한 안전속성(T7):** ACS 로그에 **같은 통로가 두 로봇에 동시에 잡히는 로그가 단 한 번도
> 없어야** 한다. 이게 깨지면 실주행에서 좁은 통로 정면충돌로 이어지므로 최우선 확인 항목이다.

---

## 6. 관찰·디버깅 도구 모음

```bash
# 🖥️ [관제 PC]
ros2 node list                                   # 살아있는 노드
ros2 topic list | grep -E 'telemetry|fleet'      # 텔레메트리/취합 토픽
ros2 topic echo /dg_01/ddago/telemetry --once    # 개별 로봇 상태(원본)
ros2 topic echo /automato/telemetry/fleet --once # 취합 결과(ACS 입력)
ros2 action list -t | grep patrol                # Patrol 액션 서버 존재/타입
ros2 action info /dg_01/patrol                    # 서버(bridge)/클라이언트(ACS) 연결 확인
# ACS 는 실행 터미널(D) 로그가 곧 관제 로그. 예약/우회/양보/종료가 다 찍힌다.
# patrol_bridge 는 각 로봇 터미널3 로그(수신/결과).
```

---

## 7. 안전 수칙

이 검증은 **로봇을 주행시키지 않으므로**(patrol_bridge=sim) 별도의 주행 안전조치는 필요 없다.
로봇은 텔레메트리만 발행하며 제자리에 정지해 있다. 확인할 것은 두 가지뿐:

- **전원·네트워크.** 실물 로봇의 전원과 같은 공유기·`ROS_DOMAIN_ID` 연결만 유지되면 된다.
- **배터리.** 순찰 임계값(70%) 근처면 가용 판정이 흔들릴 수 있으니 여유 있게 충전 후 시작.

> **실주행(로봇이 실제로 도는 것)은 이 문서 범위 밖.** 그건 HQ(DG Control Service)가 Nav2 로
> 주행을 맡을 때 별도로 다룬다. 지금은 관제(ACS) 로직 검증에만 집중한다.

---

## 8. 정리 (teardown)

```bash
# 각 터미널 Ctrl+C (역순: ACS → aggregator → 로봇 bridge/telemetry/driver)
# 🖥️ [관제 PC] DB 중지(데이터 보존)
cd ~/roscamp-repo-1/services/database && docker compose down
#   데이터까지 초기화하려면: docker compose down -v (주의: 전체 삭제)
```

---

## 9. 트러블슈팅 (자주 나는 문제)

| 증상 | 원인 후보 | 조치 |
| --- | --- | --- |
| available 이 **전부 TELEMETRY_STALE** | 시계 어긋남 / fleet 토픽 미수신 / DOMAIN_ID 불일치 | `date -u` 비교·동기(2-2), `ros2 topic hz /automato/telemetry/fleet`, `echo $ROS_DOMAIN_ID` |
| fleet 토픽이 안 뜸 | 로봇 텔레메트리 미기동 / robot_ids 파라미터 누락 | 🤖 `ros2 topic echo /dg_01/ddago/telemetry`, aggregator `robot_ids` 확인 |
| **가짜 로봇**이 available 에 안 뜸 | fake_telemetry id 가 aggregator `robot_ids` 에 없음 / id 오타 / 노드 미기동 | Step C `robot_ids` 에 가짜 id 포함, `ros2 topic echo /dg_02/ddago/telemetry` 로 발행 확인 |
| `ros2 param set` 이 타입 에러 | 소수 파라미터에 정수 입력 | `battery_percent 65.0` 처럼 **소수점** 포함, 노드명 `/dg_02/fake_telemetry` 정확히 |
| accept 는 됐는데 **로봇이 안 움직임** | 정상 — bridge 가 `mode:=sim` 이라 주행하지 않음 | 이 문서는 주행을 검증하지 않는다. ACS 로그·DB 상태 전이로 판정하면 된다 |
| `/dg_01/patrol` 액션 없음 | patrol_bridge 미기동 / 네임스페이스 오타 | 🤖 bridge 로그 확인, `-r __ns:=/dg_01` 정확히 |
| ACS 가 **DB_UNAVAILABLE** | DB 미기동 / DATABASE_URL 불일치 | Step A, `python smoke_check.py`, 다른 호스트면 `export DATABASE_URL=...` |
| "라우팅 그래프 로드: 통로 0" 경고 | corridors 시드 안 됨 | `alembic upgrade head` 재실행, 3-1 확인 SQL 로 통로 19행 확인 |
| accept 가 409 만 남 | 이전 테스트 task 가 IN_PROGRESS 로 로봇 점유 | `SELECT * FROM tasks WHERE status IN ('WAITING','IN_PROGRESS');` 확인 후 정리 |
| 로봇 좌표가 (0,0)/odom 기준 | Nav2(amcl) 안 띄움 — 이 문서에선 **정상** | 좌표는 가용·선정 판정에 안 쓰이므로 무시해도 된다 |
| interfaces 임포트/타입 에러 | Patrol.action 재빌드 안 함 | 관제 PC·로봇 **양쪽** `colcon build --packages-select automato_interfaces` |
| 로봇에 옛 Patrol.action / `~/automato_ws/automato_ws/` 중첩 생김 | rsync 경로 끝 슬래시 누락 or 미실행 | 2-3(2) 슬래시(`.../automato_ws/`) 확인, `cat .../Patrol.action` 으로 검증 후 재빌드 |
| 데이터 새로 시드했는데 순찰이 옛 그래프로 돔 | ACS 가 그래프를 캐시(첫 순찰 때 1회 로드) | ACS(`patrol_node`) **재시작** → 로그 `라우팅 그래프 로드: 노드 N/통로 M` 새 값 확인(3장 ⚠️ 주의) |

---

## 부록: 이 문서의 범위 (한 장)

| | 이 문서 (관제 로직 검증) | 범위 밖 (실주행) |
| --- | --- | --- |
| patrol_bridge | `mode:=sim` | `mode:=nav2` |
| 로봇 움직임 | ❌ (정지, 텔레메트리만 발행) | ✅ (Nav2 주행) |
| 검증 대상 | 선정·DB·예약·우회·양보 **관제 로직** | + 실제 자율주행·물리 장애물 우회 |
| 로봇 스택 | 드라이버·텔레메트리·bridge(sim) | + Nav2(localization/navigation) |
| 담당 | RP-78(ACS) | HQ(DG Control Service) — 준비되면 |

> 실물 로봇의 **진짜 텔레메트리**로 T0~T9 를 모두 통과시키면 RP-78 관제 로직이 검증된 것이다.
> 로봇이 실제로 순찰지점을 도는 실주행은 HQ(DG Control Service)가 Nav2 로 주행을 맡을 때
> 별도로 다룬다(이 문서 범위 밖).

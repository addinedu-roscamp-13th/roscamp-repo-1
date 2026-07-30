# 예냉실 이송·하역 데모 촬영 런북

수확지에 서 있는 로봇을 **예냉실로 보내고 → 도킹시키고 → 로봇팔이 바구니를 들어올리는**
장면만 따로 실행한다. 영상 촬영용이다.

정식 경로(`POST /internal/v1/tasks/harvest`)로는 '수확 접수 → 수확지 주행 → 도킹 → 수확 →
이송 → 하역' 전체를 돌려야 이 구간에 닿는다. 앞단(수확)이 몇 분씩 걸리는데 영상에 담을 건
뒤의 세 장면뿐이라, 그 부분만 떼어 실행하는 스크립트를 쓴다.

도구: `services/automato_control_service/tools/goto_precool.py`
경로탐색·도킹·하역은 ACS 코드를 그대로 불러 쓰므로, **여기서 움직인 경로 = 실제 수확
task가 움직이는 경로**다.

---

## 0. 미리 준비할 것

### (1) 하역 모션 티칭 — 로봇마다 각자 해야 한다

팔이 바구니 손잡이를 잡아 드는 동작은 **손으로 가르친 관절각**을 재생하는 것이다.
그 파일(`unload_path.json`)은 로봇·예냉실 배치마다 달라서 `.gitignore` 대상이고,
**없으면 팔이 Goal을 거절한다.**

```bash
# 로봇팔 PC에서
python3 ddagi_harvest/teach_unload.py check    # 있는지·성한지 확인
python3 ddagi_harvest/teach_unload.py teach    # 없으면 티칭
```

> 왜 좌표가 아니라 관절각인가: 같은 좌표라도 IK(역기구학) 해가 매번 달라져 경로가 튄다.
> 짐을 든 채로 경로가 바뀌면 예냉실 벽을 친다.

### (2) 두 워크스페이스 빌드

```bash
cd ~/roscamp-repo-1/equip/automato_ws            && colcon build
cd ~/roscamp-repo-1/services/automato_control_service && colcon build
```

> `--symlink-install` 은 쓰지 않는다. 빌드하지 않으면 install 트리의 옛 파일이 도는데,
> 새로 생긴 파라미터가 **에러 없이 조용히 무시되는** 형태로 나타나 원인을 찾기 어렵다.

### (3) ROS_DOMAIN_ID 를 네 터미널 모두 같은 값으로

ROS2는 같은 DOMAIN_ID끼리만 서로를 본다. 하나라도 다르면 "액션 서버 미기동"으로 보인다.

```bash
echo $ROS_DOMAIN_ID        # 각 기기에서 확인 — 로봇마다 값이 다르다
export ROS_DOMAIN_ID=<그 로봇 값>
```

### (4) 로봇을 수확지에 둔다

`HARVEST_01`(노드 20) 또는 `HARVEST_02`(노드 21). 스크립트는 로봇이 그 자리에 있다고
믿고 첫 구간을 계산하므로, 실제 위치와 다르면 첫 주행부터 엉뚱하게 간다.

---

## 1. 실행 — 터미널 4개

### 터미널 ① 따고(주행 로봇)

```bash
ros2 launch ddago_control ddago_bringup.launch.py dry_run:=false
```

- 주행(`/ddago/navigate`)과 바닥 H마커 도킹(`/ddago/floor_dock`)이 함께 뜬다.
- **`dry_run:=false` 를 빠뜨리면 바퀴가 안 굴러간다.** 기본값이 `true`인데, 이는 도킹
  서버가 첫 투입에서 사고를 내지 않도록 "검출·계획만 하고 `/cmd_vel`은 안 낸다"로
  안전하게 잡아둔 것이다.
- 로봇이 바뀌면 캘리브 파일도 바뀐다: `floor_calib_file:=...`(바닥 도킹),
  `config_file:=.../ddago02.yaml`(반사 도킹). 이번 데모는 예냉실만 쓰므로 바닥 쪽만 맞으면 된다.

### 터미널 ② 따기(로봇팔 PC)

```bash
ros2 launch ddagi_harvest harvest_with_ai.launch.py with_ai:=false
```

- 하역 서버(`/ddagi/unload`)가 뜬다.
- **`with_ai:=false`** — 이번엔 수확을 안 하니 AI 검출 서비스를 띄우지 않는다.
  카메라는 한 프로세스만 열 수 있어서, 안 쓸 노드를 띄우면 서로 카메라를 다툰다.
- 팔 주소가 기본값과 다르면 `arm_ip:=192.168.3.xx`.

### 터미널 ③ DG (중계자)

```bash
ros2 run dg_control dcs_node --ros-args -p robot_id:=dg_01
```

- ACS ↔ 로봇 사이의 중계자다. 여기서 `/dg_01/navigate`, `/dg_01/floor_dock`,
  `/dg_01/unload` 가 열리고, 스크립트는 이 셋에 붙는다.
- `robot_id` 는 액션 이름이 되므로 ④의 인자와 **반드시 같아야 한다.**

### 터미널 ④ 스크립트 (ACS 자리)

```bash
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash              # 액션 정의
source ~/roscamp-repo-1/services/automato_control_service/install/setup.bash
set -a; source ~/roscamp-repo-1/services/database/.env; set +a            # DATABASE_URL

cd ~/roscamp-repo-1/services/automato_control_service
python3 tools/goto_precool.py dg_01
```

> 소싱이 두 번인 이유: 워크스페이스가 둘이다(로봇/인터페이스 = `automato_ws`, ACS =
> `automato_control_service`). 나중에 소싱한 것이 앞에 오는 겹쳐쓰기라 **순서가 중요**하다.

옵션:

| 명령 | 언제 |
|---|---|
| `python3 tools/goto_precool.py dg_01` | 본편 — 이송 → 도킹 → 하역 |
| `... dg_01 --from HARVEST_02` | 두 번째 수확지에서 출발 |
| `... dg_01 --no-unload` | 팔 없이 주행·도킹만 리허설 |

---

## 2. 잘 되고 있는지 보는 법

터미널 ④의 로그가 이 순서로 나오면 정상이다.

```
[PRECOOL-DEMO] 라우팅 그래프 로드: 노드 21 / 통로 23
[PRECOOL-DEMO] HARVEST_01(노드 20) → PRECOOL_01(노드 25)
[PRECOOL-DEMO] 액션 대상: /dg_01/navigate, /dg_01/floor_dock, /dg_01/unload
[PRECOOL-DEMO] 이송 시작 task=9001 dg_01 20 → 25
세그먼트 하달 task=9001 20→[1, 2, 3, 8, 11, 14, 17, 25] ...
조기 반납 task=9001 로봇 위치 2 → 자원 [...] 해제          ← 지나온 통로를 놓는 중
[PRECOOL-DEMO] 예냉실 도착 노드 25
도킹 시도 1/3 task=9001 dg_01 @ PRECOOL_01 (floor)
[PRECOOL-DEMO] 예냉실 도킹 완료 @ PRECOOL_01
[HARVEST] 하역 진행 task=9001 phase=GRIP_HANDLE       ← 손잡이 잡기
[HARVEST] 하역 진행 task=9001 phase=LIFT              ← 들어올리기
[HARVEST] 하역 진행 task=9001 phase=WAIT              ← 쏟아지길 기다림(3초)
[HARVEST] 하역 진행 task=9001 phase=RETURN            ← 복귀
[PRECOOL-DEMO] 하역 완료
[PRECOOL-DEMO] 노드 25 자리 반납
```

종료코드로도 갈린다: **0 = 전부 성공 / 1 = 어딘가 실패.** 촬영 중엔 로그를 다 못 보므로
`echo $?` 로 바로 확인하면 된다.

> `SHAKE` phase는 나오지 않는다. 2026-07-30에 털기 단계를 뺐기 때문이며 정상이다.

---

## 3. 자주 나는 실패와 원인

| 증상 | 원인 / 조치 |
|---|---|
| `액션 서버 미기동: /dg_01/navigate, ...` | ①②③ 중 안 뜬 게 있거나 `ROS_DOMAIN_ID` 불일치. `ros2 action list` 로 확인 |
| 셋 다 미기동인데 노드는 떠 있음 | 도메인 불일치가 대부분. Wi-Fi를 나중에 붙였으면 `ros2 daemon stop` 후 재시도 |
| 로봇이 안 움직이는데 로그는 정상 | ①에 `dry_run:=false` 를 빠뜨렸다 |
| `도킹 실패(code=1) 마커 미검출` | 바닥 H마커가 카메라에 안 보인다. 진입 노드 정차 위치·조명 확인 |
| `하역 goal 거부: 도킹 안 됨` | 도킹과 하역 사이에 다른 주행이 끼었다. 스크립트를 처음부터 다시 |
| `티칭 경로 없음` | 그 로봇에 `unload_path.json` 이 없다 → 0-(1) 티칭 |
| `DATABASE_URL 환경변수가 필요합니다` | ④의 `.env` 소싱을 빠뜨렸다 |

---

## 4. 지켜야 할 것 3가지

1. **ACS(`automato_node`)를 동시에 돌리지 않는다.**
   스크립트는 자기만의 통로 예약표를 만든다. ACS가 순찰을 돌리는 중이면 두 예약표가
   서로를 못 봐서, 같은 통로에 두 로봇이 들어갈 수 있다.

2. **도킹과 하역을 쪼개지 않는다.**
   DG는 "직전 도킹에 성공한 task_id"의 하역만 받아준다. 도킹 안 된 자리에서 팔이
   움직이는 걸 막는 안전장치라, 두 단계 사이에 다른 명령이 끼면 하역이 거부된다.
   스크립트가 한 번에 이어서 하는 이유다.

3. **DB에 기록이 남지 않는다.**
   `task_id` 는 임의 번호(기본 9001)이고 `tasks`·`harvest_batches`·`unload_logs` 에
   아무것도 쓰지 않는다. 실적 기록까지 검증해야 한다면 정식 수확 접수를 써야 한다.
   이 도구는 촬영용이다.

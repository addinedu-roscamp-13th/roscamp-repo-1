# ddagi_harvest — 로봇팔 수확 (시나리오2 E3~E5)

`send_coords`(제조사 IK)로 파지하는 Ddagi 수확 구현. 세 구현 비교에서 채택됨(RP-129).
경로를 IK 에 통째로 맡기지 않고 안전한 중간 자세로 쪼개는 방식.

설계 배경·실측·판단 근거는 Confluence
[로봇팔 수확 구현 (Ddagi)](https://robot8.atlassian.net/wiki/spaces/Robot8/pages/52428802) 참고.

---

## 실행 — 로봇팔 PC (로봇마다 각자)

로봇팔별로 **그 팔을 제어하는 PC 에서 AI 서비스와 수확 노드를 함께** 돌린다.
카메라는 그 PC 에 USB 직결되고, Ddagi ↔ AI 는 같은 기기 안의 ROS2 Service 호출이라
네트워크를 타지 않는다.

```
[ 로봇팔 PC ]                                    [ Pi (팔) ]
  detect_tomatoes_server  ─/ai/detect_tomatoes─▶
  harvest_node            ─────────────────────▶  ──TCP 9010──▶  arm_server.py
        ▲
        └── /ddagi/harvest  ◀── DG Control Service ◀── ACS
```

### 1) 로봇마다 다른 값 — 처음 한 번만 설정

`~/.bashrc` 에 넣어두면 매번 안 쳐도 된다.

```bash
export ROS_DOMAIN_ID=10                                     # 전 로봇 공통
export ARM_IP=192.168.3.12                                  # ★ 로봇마다 다름
export DG_AI_MODEL_PATH=$HOME/roscamp-repo-1/equip/automato_ws/src/dg_ai_service/models/tomato_4cls_v8.pt
export AUTOMATO_PYTHON=$HOME/venv/automato/bin/python3      # venv 경로가 다르면
```

`ARM_IP` 는 그 로봇의 Pi 주소다. 모르면 9010 포트가 열린 호스트를 찾는다(문제해결 참고).

**가중치(.pt) 배치** — `.gitignore` 로 커밋이 금지돼 있다(18MB, 재학습마다 누적).
각 PC 가 직접 받아서 아래 위치에 둔다. `Downloads` 같은 개인 폴더가 아니라
**프로젝트 안의 정해진 자리**에 두면 PC 마다 경로가 같아진다.

```bash
mkdir -p ~/roscamp-repo-1/equip/automato_ws/src/dg_ai_service/models
cp <받은 파일>/tomato_4cls_v8.pt ~/roscamp-repo-1/equip/automato_ws/src/dg_ai_service/models/
```

> `DG_AI_MODEL_PATH` 를 반드시 설정해야 한다. `dg_ai_service` 의 기본 경로는
> `os.path.realpath(__file__)` 기준이라 **install 공간**(`install/dg_ai_service/.../models/`)
> 으로 해석되는데, 거기엔 파일이 없고 `rm -rf install` 로 날아간다. src 에 두고
> 환경변수로 가리키는 것이 안전하다.

> ⚠ **`raspi.local`(mDNS)을 쓰지 말 것.** Pi 들의 호스트명이 모두 `raspi` 라
> 먼저 응답한 쪽으로 해석된다. 실측에서 같은 세션 안에 다른 기계 두 곳을 가리켰다 —
> **남의 로봇팔에 파지 명령이 갈 수 있다.**

### 2) Pi 에서 팔 브리지 (수동 — 자동 실행 아님)

```bash
ssh jetcobot@$ARM_IP
cd ~/kdh_ws/m1_arm_basics && ~/venv/automato/bin/python3 arm_server.py
```

시스템 python 에는 `pymycobot` 이 없으므로 **venv 파이썬으로** 띄운다.
**공유기 재부팅·전원 재인가 때마다 죽으므로 다시 띄워야 한다.**

### 3) AI + 수확 노드 (한 번에)

```bash
cd ~/roscamp-repo-1/equip/automato_ws && source install/setup.bash
ros2 launch ddagi_harvest harvest_with_ai.launch.py
```

환경변수를 안 썼다면 인자로 넘긴다:

```bash
ros2 launch ddagi_harvest harvest_with_ai.launch.py \
  arm_ip:=192.168.3.12 \
  model:=$HOME/roscamp-repo-1/equip/automato_ws/src/dg_ai_service/models/tomato_4cls_v8.pt
```

### 4) rviz (선택 — 검출 결과 확인)

```bash
rviz2 -d $(ros2 pkg prefix ddagi_harvest)/share/ddagi_harvest/rviz/ddagi_harvest.rviz
```

Fixed Frame·토픽·시점이 잡혀서 뜬다. **파란 와이어프레임 상자가 보이면**
토픽·프레임·`ROS_DOMAIN_ID` 가 모두 맞았다는 뜻이다.

### 5) 이제 기다린다

수확·하역 노드는 액션 서버다. **ACS → DG → Ddagi 로 Goal 이 내려오면 자동으로 시작한다.**
DG 는 도킹 성공(`is_docked`)한 task 의 goal 만 수락한다.

| 액션 | 언제 | 하는 일 |
|---|---|---|
| `/ddagi/harvest` | 따고가 **수확 위치**에 도킹 | 관측 → 검출 → 파지 → 바구니 (라운드 반복) |
| `/ddagi/unload` | 따고가 **예냉실**에 도킹 | 손잡이 파지 → 들기 → 쏟기 → 놓기 → 복귀 |

```
대기 → Goal 수신 → 라운드 반복 → Result 반환 → 대기 → ...
```

Goal 이 들어오면 로그가 이렇게 시작한다:

```
수확 시작 task_id=2048 max_capacity=7
로봇팔 연결 192.168.3.12:9010
===== 라운드 1/5 =====
```

**팔이 알아서 움직인다.** 반경을 비우고 손 닿는 곳에 있을 것.

---

## launch 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `arm_ip` | `$ARM_IP` 또는 `192.168.3.12` | Pi 의 `arm_server.py` 주소 (9010) |
| `model` | 비움 → `dg_ai_service` 기본값(`$DG_AI_MODEL_PATH`) | YOLO 가중치. 관례를 두 곳에 두지 않으려 위임 |
| `python` | `$AUTOMATO_PYTHON` 또는 `~/venv/automato/bin/python3` | ultralytics 가 있는 파이썬 |
| `dry_run` | `false` | `true` = 파지 없이 검출·마커만 |
| `conf` | `0.4` | YOLO 신뢰도 임계 |
| `max_rounds` | `5` | 촬영-수확 라운드 상한 |
| `with_ai` | `true` | `false` = AI 는 딴 데서 돌 때 |
| `arm` | `network` | `fake` = 팔 없이 배선만 확인 |

`dry_run` 과 `conf` 는 **실행 중에도** 바꿀 수 있다:

```bash
ros2 param set /ddagi_harvest_node dry_run true
ros2 param set /ddagi_harvest_node conf 0.25
```

---

## 수동 실행 (디버깅)

DG 없이 직접 Goal 을 보낼 때:

```bash
ros2 action send_goal /ddagi/harvest automato_interfaces/action/Harvest \
  "{task_id: 1, max_capacity: 7}" --feedback
```

AI 서비스만 따로 확인할 때 (팔·노드 불필요):

```bash
ros2 service call /ai/detect_tomatoes automato_interfaces/srv/DetectTomatoes \
  "{task_id: 1, round: 1}"
```

응답 좌표가 `0.0x ~ 0.5` 범위면 미터(정상), `20 ~ 500` 이면 mm 다.

하드웨어 없이 전 경로를 볼 때:

```bash
ros2 launch ddagi_harvest harvest_with_ai.launch.py with_ai:=false arm:=fake
ros2 param set /ddagi_harvest_node detector list
```

---

## 문제해결

### `wait_for_service` 실패 / 액션 서버를 못 찾음

`ROS_DOMAIN_ID` 불일치가 가장 흔하다. **모든 터미널에서 10** 이어야 한다.

```bash
echo $ROS_DOMAIN_ID          # 전부 10
ros2 action list | grep harvest
ros2 service list | grep detect
```

### 팔 연결 실패

| 증상 | 원인 | 조치 |
|---|---|---|
| `ConnectionRefused` | 호스트는 살아있고 `arm_server.py` 만 미실행 | Pi 에서 띄운다 |
| `No route to host` / 타임아웃 | IP 가 바뀜(DHCP) | 아래로 찾는다 |

```bash
# 서브넷에서 9010 열린 호스트 찾기
for i in $(seq 2 60); do
  (timeout 1 bash -c "echo >/dev/tcp/192.168.3.$i/9010" 2>/dev/null \
     && echo "192.168.3.$i ← Pi") &
done; wait
```

Pi 에서 프로세스 확인은 **대괄호를 넣어서** 한다:

```bash
pgrep -af "arm_ser[v]er"     # 없으면 pgrep 이 자기 명령줄을 매칭해 오탐한다
```

### `Device or resource busy` / `Frame didn't arrive`

카메라를 두 프로세스가 잡으려 한 것이다. 수확 노드가 중복 실행됐거나
`detector` 가 `yolo`/`mock` 인 경우다(launch 는 `ros` 로 고정해 둠).

```bash
pgrep -af "harvest_[n]ode"   # 하나만 있어야 한다
pkill -f harvest_node        # 정리 후 다시 띄운다
```

장치가 이상한 상태로 남았으면 리셋:

```bash
$AUTOMATO_PYTHON -c "
import pyrealsense2 as rs, time
d = rs.context().query_devices()[0]; d.hardware_reset(); time.sleep(6)
print('재검출:', len(rs.context().query_devices()), '대')"
```

### AI 서비스가 뜨자마자 죽는다 (모델을 못 찾음)

`DG_AI_MODEL_PATH` 가 없으면 기본 경로가 **install 공간**으로 해석되는데 가중치는
거기에 없다. 확인:

```bash
echo $DG_AI_MODEL_PATH        # 비어 있으면 그것이 원인
ls -la $DG_AI_MODEL_PATH      # 파일이 실제로 있는지
```

### `ModuleNotFoundError: ultralytics`

`ros2 run` 으로 띄운 것이다. colcon 이 만든 실행 스크립트의 셔뱅은 시스템 python 을
가리키므로 **반드시 launch 로** 띄운다(`Node(prefix=)` 로 venv 를 앞에 붙인다).

### 검출은 되는데 하나도 안 딴다

AI 가 좌표를 mm 로 보내면 1000배가 되어 전량 작업공간 밖으로 걸러진다.
`ros2 service call` 로 원시 좌표를 확인한다(위 참고).

---

## 예냉실 하역 (E6)

따고가 예냉실에 도킹하면 DG 가 `/ddagi/unload` 로 Goal 을 보낸다. 티칭한 관절각 경로를
재생해 바구니를 쏟고 완료를 돌려준다.

```bash
# 티칭 (로봇마다 각자 — 예냉실 배치가 다르다)
python3 ddagi_harvest/teach_unload.py teach
python3 ddagi_harvest/teach_unload.py check     # 명령 한계 검사 (필수)
python3 ddagi_harvest/teach_unload.py run       # 수동 재생

# 수동 Goal
ros2 action send_goal /ddagi/unload automato_interfaces/action/Unload \
  "{task_id: 1, shake_delay_sec: 3.0}" --feedback
```

티칭 값은 **`~/.config/automato/unload_path.json`** 에 저장된다. 패키지 안에 두면
티칭은 `src` 에 쓰이는데 액션 서버는 `install` 공간에서 읽어 못 찾고, `rm -rf install`
로 날아간다. `UNLOAD_PATH` 로 경로를 바꿀 수 있다.

| `result_code` | 의미 |
|---|---|
| `0` | 성공 |
| `1` | 손잡이 파지 실패 — 그리퍼를 열고 준비 자세로 복귀한다 |
| `2` | 중단(취소) 또는 티칭 경로 없음 |

`shake_delay_sec` 은 '들어올린 뒤 대기 시간'이라 쏟아지는 걸 기다리는 구간에 쓴다.

> **털기는 빼기로 했다(2026-07-30).** 경로에서 `shake` 스텝의 `act` 를 해제했으므로
> Feedback 에 `SHAKE` phase 가 나오지 않는다. 되살리려면 `unload_path.json` 의 그
> 스텝 `act` 를 `"shake"` 로 되돌린다(코드는 기능을 남겨 두었다).

---

## 종료 사유 (`exit_reason`)

| 값 | 의미 |
|---|---|
| `FULL` | 수확품 바구니 만차(7개) — 밭에 작물이 남아 있음, 재출동 필요 |
| `DEPLETED` | 딸 것이 없음 — 정상 종료 |
| `MAX_ROUNDS_EXCEEDED` | 라운드 상한 도달 — 점검 필요 |
| `CANCELED` / `BUSY` / `ERROR` | 취소 / 다른 수확 진행 중 / 예외 (스펙 외 확장) |

---

## 구성

| 파일 | 역할 |
|---|---|
| `harvest_node.py` | 액션 서버. Goal·Feedback·취소·Result 만 담당하는 ROS 껍데기 |
| `harvest.py` | 수확 루프. ROS 의존 없음 → `python3 harvest.py` 단독 실행 가능 |
| `pick.py` | 파지 1개. 구역·자세·standoff·후퇴·판정 |
| `tf_transform.py` | 관측자세 camera(optical) → flange 변환 (직접 피팅) |
| `detector.py` | 검출 추상화. `RosDetector` / `YoloDetector` / `MockColorDetector` / `ListDetector` |
| `arm_backend.py` | 팔 추상화. `NetworkArm`(TCP) / `RealArm` / `FakeArm` |
| `markers.py`·`log.py` | rviz 시각화 · 출력 싱크 주입 |
| `unload_node.py` | 하역 액션 서버(`/ddagi/unload`). `teach_unload.replay` 를 감싼다 |
| `teach_unload.py` | 예냉실 하역 모션 티칭·재생 (RP-130) |
| `fit_observe_tf.py` | 관측자세 변환 피팅 (rigid/affine + LOO 교차검증) |
| `tf_verify.py` | 클릭 검증·게이지 측정·손목 자세 티칭 |
| `calib/` | 캘리브레이션 3종 결과·스크립트 (이건수, 원본 보존) |

# ACS 교통관제 검증 웹

`routing_engine.py`(경로 탐색 + 통로 예약)와 `patrol_dispatcher.py`(세그먼트 하달 ·
룩어헤드 · 막힘 우회 · 데드락 양보)가 **실제로** 어떻게 움직이는지 눈으로 보는 도구.

## 가장 중요한 원칙 — 검증 대상 코드는 고치지 않는다

`RoutingEngine`과 `PatrolDispatcher`를 **그대로 import 해서 실행**한다.
경로 탐색을 JavaScript로 다시 구현하면 "웹 페이지가 잘 돈다"만 증명될 뿐,
정작 검증하려는 파이썬 코드는 하나도 검증되지 않는다.
**가짜인 것은 로봇(Navigate 액션 클라이언트)뿐이다.**

관측도 같은 원칙을 따른다 — 상태를 훔쳐보기 위해 엔진에 훅을 심지 않고,
이미 있는 공개 API(`engine.holder_of()`)를 밖에서 물어본다.

## 두 가지 모드

|  | SIM | LIVE |
|---|---|---|
| 데이터 출처 | 이 프로세스 안의 `RoutingEngine` + 가짜 로봇 | 실물 ACS(8200)의 예약표를 HTTP 폴링 |
| 로봇 위치 | 시뮬이 계산 (노드 번호를 안다) | 텔레메트리 x/y/yaw (노드는 **좌표에서 역추정**) |
| 조작 | 순찰 · 이동 · 통로 막기 · 데드락 시나리오 | **없음 (관측 전용)** |
| 이벤트 | 디스패처 로그를 그대로 가로챔 | 예약표 **변화**를 감지해 생성 |

LIVE가 조작을 막는 이유: 이 화면의 조작은 *가짜 로봇 세계*를 흔드는 것이라
LIVE에서 눌러도 화면(실물)엔 아무 변화가 없다. 조용히 씹히는 것보다
`409 LIVE_READONLY`로 거절하는 편이 덜 헷갈린다.

LIVE가 **거짓말하지 않도록** 지키는 것:
- ACS와 끊기면 예약표를 **비운다**. 확인 못 하는 교통관제 정보는 위험하다
  (1분 전에 반납했을 수도 있다). 로봇 위치는 "마지막으로 본 자리"로 남기되
  점선 빈 원으로 바꿔 확인되지 않은 값임을 드러낸다.
- `~wp13`의 `~`는 좌표에서 역추정한 값이라는 표시다. 실물 로봇은 자기가
  몇 번 지점에 있는지 모른다.
- ACS의 `RoutingEngine`은 첫 순찰 때 만들어진다 → 그전에는 "순찰 대기"로 표시
  (예약이 *없는 것*과 예약표가 *아직 없는 것*은 다른 상태).

## 실행

### SIM 모드 (로봇 없이 — 기본)

이것만 띄우면 된다. ACS도, 로봇도, ROS 노드도 필요 없다.

```bash
cd ~/roscamp-repo-1

# ① ROS 환경 → ② automato_interfaces → ③ automato_control_service 순서로 소싱.
#    순서가 중요하다: 뒤에 소싱한 워크스페이스가 앞의 것을 덮어쓰기 때문에
#    '더 구체적인 것'을 나중에 얹는다. 리포 루트에서 colcon build 하지 말 것.
source /opt/ros/jazzy/setup.bash
source equip/automato_ws/install/setup.bash                 # automato_interfaces
source services/automato_control_service/install/setup.bash # routing_engine 등

python3 services/automato_control_service/verify_web/server.py
```

→ 브라우저에서 **http://127.0.0.1:8300**

빌드가 안 돼 있으면 먼저:
```bash
cd ~/roscamp-repo-1/services/automato_control_service && colcon build
```
`--symlink-install`은 쓰지 않는다 — 같은 워크스페이스를 어떤 때는 붙이고 어떤 때는
빼고 빌드하면 install 트리에 실파일과 심링크가 섞여 import 가 엉킨다.

DB 접속은 `DATABASE_URL` 또는 `services/database/.env`를 쓴다(ACS와 동일).

#### 자주 쓰는 변형

```bash
# 로봇을 느리게 → 통로 점유가 눈에 잘 보인다
VERIFY_SPEED_MPS=0.06 python3 .../verify_web/server.py

# 다른 노트북(Tailscale)에서 보기. tailnet 안에서만 열리고 LAN에는 노출 안 됨
VERIFY_WEB_HOST=$(tailscale ip -4) python3 .../verify_web/server.py

# 예약 대기·하트비트를 느슨하게 → 데드락 양보 판단을 관찰하기 좋다
ACS_HEARTBEAT_SEC=1.0 ACS_RESERVE_WAIT_SEC=20 ACS_RESERVE_POLL_SEC=0.5 \
  python3 .../verify_web/server.py
```

### LIVE 모드 (실물 로봇 관측)

**터미널 2개**가 필요하다. verify_web은 ACS를 *들여다볼 뿐* 직접 띄우지 않는다.

```bash
# ── 터미널 1: 실물 ACS (여기 안에 진짜 예약표가 산다) ──
cd ~/roscamp-repo-1/services/automato_control_service
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
source install/setup.bash
python3 -m automato_control_service.patrol_node      # → 0.0.0.0:8200

# ── 터미널 2: 검증 웹 ──
#   (소싱은 SIM 과 동일)
python3 services/automato_control_service/verify_web/server.py
```

→ 화면 오른쪽 위 **LIVE** 버튼을 누르면 전환된다.
처음부터 LIVE로 뜨게 하려면 `VERIFY_MODE=LIVE`,
ACS가 다른 기기면 `ACS_BASE_URL=http://<ACS주소>:8200`.

동작 확인:
```bash
curl -s http://127.0.0.1:8200/internal/v1/debug/traffic | python3 -m json.tool
```
`engine_ready: false` + `robots: []`는 **정상**이다 — 순찰을 아직 안 걸었고
(엔진은 첫 순찰 때 생성) 로봇 텔레메트리도 안 들어온 상태라는 뜻.

### 종료

```bash
fuser -k 8300/tcp        # verify_web
fuser -k 8200/tcp        # ACS
```
`pkill -f verify_web/server.py`는 쓰지 말 것 — 패턴이 **자기 자신의 명령줄에도
매칭돼** 실행한 셸까지 같이 죽는다.

### 환경변수

| 이름 | 기본 | 설명 |
|---|---|---|
| `VERIFY_WEB_PORT` | `8300` | ACS(8200)·텔레메트리 WS(8000)와 겹치지 않게 |
| `VERIFY_WEB_HOST` | `127.0.0.1` | 다른 기기에서 보려면 그 인터페이스 주소. Tailscale이면 `$(tailscale ip -4)` — tailnet에만 열리고 LAN에는 노출되지 않는다 |
| `VERIFY_SPEED_MPS` | `0.06` | 가짜 로봇 속도. 맵이 작아(통로 0.03~0.44m) 느려야 통로 점유가 눈에 보인다 |
| `VERIFY_MODE` | `SIM` | `LIVE`로 두면 부팅부터 실물 관측 |
| `ACS_BASE_URL` | `http://127.0.0.1:8200` | LIVE가 들여다볼 ACS |

## 파일

| 파일 | 역할 |
|---|---|
| `server.py` | FastAPI. 그래프 API · WebSocket 10Hz 방송 · 조작 API · 모드 스위치 |
| `sim.py` | SIM 세계. 진짜 엔진/디스패처를 조립하고 스냅샷을 뜬다 |
| `fake_navigate.py` | 가짜 로봇 + `PatrolDispatcher`가 기대하는 액션 클라이언트 흉내 |
| `live.py` | LIVE 어댑터. ACS를 2Hz로 폴링해 SIM과 **같은 모양**으로 변환 |
| `map_layout.py` | DB가 모르는 것(토마토 베드 · 로봇팔 · 방 외곽) |
| `static/` | SVG 렌더링 · 실시간 갱신 |

`live.py`가 SIM과 같은 키를 돌려주기 때문에 **프런트엔드는 모드를 거의 신경 쓰지 않는다**
(조작 잠금과 배지만 다르다).

### 왜 패키지 밖에 있나

이 디렉터리는 ROS 노드가 아니라 평범한 웹 서버다. 패키지 안에 넣으면 `setup.py`의
`data_files`로 html/js까지 install 트리에 복사해야 하고 colcon 빌드에 영향을 준다.
밖에 두면 `find_packages()`가 잡지 않아 빌드 리스크가 0이다.

## ACS 쪽에 추가된 것

LIVE를 위해 ACS에 **읽기 전용 창문 하나**를 뚫었다.

- `automato_control_service/traffic_debug.py` (신규) — `GET /internal/v1/debug/traffic`
- `patrol_api.py` — import 1줄 + `traffic_debug.register(app, node)` 1줄
- `telemetry_cache.py` — `robot_ids()` 공개 메서드 추가
  (밖에서 내부 dict를 락 없이 순회하지 않도록)

쓰기 API는 하나도 없다. 엔진을 강제로 생성하지도 않는다 — 관찰이 대상을 바꾸면 안 된다.

## 화면 읽는 법

통로 색은 `RoutingEngine`의 예약표 그 자체다. 우선순위는
**막힘 > 예약(누가 쥐었나) > 회피(블랙리스트) > 빈 통로**.
색만으로 구분하지 않는다 — 굵기와 점선이 함께 바뀐다(적록색맹 대응:
`막힘 빨강`과 `로봇2 초록`은 deutan 시야에서 ΔE 5.9로 구분되지 않는다).

`↻18` 배지는 **짝(pair)** — 같은 자리에서 제자리 회전해 반대 방향으로 한 번 더
촬영하는 지점. 부모와 x·y가 완전히 같아 점을 두 개 찍으면 겹치므로 배지로 표시한다.
짝은 `corridors`에 없어서 라우팅 그래프에 넣지 않는다(넣으면 Dijkstra가
도달 불가능한 목적지를 후보로 잡는다).

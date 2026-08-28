# 🍅 Automato — 스마트팜 방울토마토 자율수확·선별 로봇

> ROS 2 기반 이기종 멀티로봇이 방울토마토 온실을 **스스로 순찰하고, 익은 것만 골라 수확해, 예냉실까지 옮기는** 오픈소스 로보틱스 플랫폼
>
> 애드인에듀 KDT 「ROS2와 AI를 활용한 자율주행 로봇개발자 부트캠프」 심화 13기 · **1팀 Robot8**

**▶ 시연영상 (2분 28초) — https://youtu.be/o6JPAzeayL0**
📊 발표자료 — https://automato-robot8.netlify.app · 🌐 라이브 데모 — https://geonsulee.pythonanywhere.com

---

## 왜 만들었나

- 농가 경영주의 **56.6%가 65세 이상**. 시설원예 방울토마토는 `착색 판별 → 수확 → 선별 → 이송`이 연중 반복되는 고빈도 수작업이라 인건비 비중이 가장 큽니다.
- 기존 스마트팜 자동화는 온·습도·관수 등 **환경 제어**에 집중되어 있고, 정작 노동이 몰리는 **수확·선별**은 사람의 몫으로 남아 있습니다.
- 상용 수확 로봇은 대당 수억 원대이고 코드·데이터가 비공개라 개선·재현이 불가능합니다.

그래서 **라즈베리파이5 기반 저가 UGV + 6축 협동로봇팔 + RGB-D 카메라**만으로 `순찰 → 탐지 → 수확 → 선별 → 이송` 폐루프를 실제로 구동하고, 전 과정을 오픈소스로 공개합니다.

---

## 두 가지 시나리오

| | PATROL 순찰 | HARVEST 수확 |
|---|---|---|
| 1 | 관리자가 모바일에서 `순찰하기` | 관리자가 모바일에서 `수확하기` |
| 2 | DDaGo가 지점마다 방울토마토 촬영 | DDaGo가 작업 지점에 도킹 |
| 3 | 숙도·병해충을 AI로 판별 | DDagi가 수확 — 일반품/폐기품 분리 |
| 4 | 농장 지도에 **히트맵**으로 표시 | DDaGo가 예냉실로 이송 후 바구니 하역 |
| 5 | 충전소 복귀 | 충전소 복귀 |

**따고(DDaGo)** 주행로봇이 다니고·나르고·살피면, **따기(DDagi)** 로봇팔이 딴다 — 한 세트로 움직이고, 관제 서비스(ACS)가 여러 세트를 동시에 지휘합니다.

---

## 핵심 기술

### 🚗 경로 탐색 & 교통 관제 — 회피가 불가능한 환경에서 출발
통로 폭 **27 cm**에 로봇 두 대(각 12 cm)면 남는 여유가 3 cm. **교행이 물리적으로 불가능**합니다. 게다가 두 로봇의 라이다 높이(12.5 cm)가 상대 몸통 꼭대기(9.5 cm)보다 높아 **서로를 장애물로 인식하지도 못합니다.**

→ 회피가 아니라 **중앙 사전 배차**로 설계했습니다.
- 온실 맵을 **21 지점 / 23 통로 그래프**로 모델링, 경로는 **Dijkstra**(비용 = 실측 거리)
- **통로(지나갈 권리)** 와 **자리(서 있을 권리)** 를 모두 예약 대상으로 삼고, 자리는 `길이 0 · id < 0` 가상 통로로 표현해 예약·만료·교착 검사 로직을 그대로 재사용
- **교착은 탐지가 아니라 회피** — 예약 요청 시 대기 사슬을 따라가 사이클이면 기다리지 않고 즉시 우회 (실물 로봇의 예약을 강제 해제하면 진짜로 충돌하므로)
- **무정지 주행** — 다음 구간을 먼저 잡고 앞엣것을 놓는 hand-over-hand 예약으로 예약이 비는 순간이 없음
- 예약은 소유가 아니라 **임대(lease)** — 하트비트 5초 / TTL 15초로 죽은 로봇의 예약을 자동 회수

### 🤖 AI 검출 — 벤치마크가 아니라 현장 성능으로
YOLO11s 기반 **토마토 4분류**(익음·안익음·썩음·병해충). 공개 데이터셋 점수가 아니라 **실제 재배 환경 성능**을 개발 기준으로 삼았습니다.

| 지표 | 개선 전 | 개선 후 |
|---|---|---|
| 현장 mAP@50 | 0.561 | **0.850** |
| 현장 Recall | 0.482 | **0.810** |
| 공개 검증셋(878장) mAP@50 | 0.801 | 0.796 *(사실상 불변)* |

공개셋 점수를 지키면서 현장 성능만 끌어올렸습니다. 학습 데이터 **12,742장**, SAM 기반 오토라벨링 파이프라인 사용.

### 🎯 정밀 도킹
- **H 마커리스 도킹** — ArUco 같은 표식 없이 바닥에 그려진 `H` 모양만 보고 작업 자리에 정렬
- **반사 테이프 도킹** — 라이다 반사 강도로 90° 코너 마커를 찾아 위치·방향을 계산, 정면 축에 정렬 후 곧게 후진해 충전소 진입

### 🦾 로봇팔 수확
- **동작 분할** — 수확 한 번을 여러 단계로 쪼개 단계마다 경로를 따로 생성
- **REAP** — `정렬 → 직진 → 후퇴 → 검증` 4단계로 나눠 실패해도 어디서 틀렸는지 알 수 있게
- **PQS(수확 후보 선별)** — 목표 하나마다 MoveIt 2 경로 계획을 **250회** 수행하고 6가지 지표로 계획 품질을 측정해 `통과 / 경고 / 실패` 3등급으로 분류
- **핸드-아이 캘리브레이션** — ChArUco 보드 기반 카메라·로봇 좌표계 정합

---

## 시스템 구성

```
CLIENT                        SERVICES                         EQUIP
─────────────────────────────────────────────────────────────────────────────
System Admin App (Qt)  ──┐                              ┌── DDaGo (RPi 4)
                         ├─ Automato Control Service ───┤    Lidar·Ultrasonic
Farm Admin App (Web) ────┤     (배차·교통관제·task)      │    IMU·Motor·LED
                         │                              │
                         ├─ Automato Web Service ───────┤── DDagi (RPi 5)
                         │     (App HTTP/WS 중계)        │    6축 Motor·Gripper
                         │                              │    Depth Camera
                         └─ Automato DB (PostgreSQL 16) └── DG Controller
                                                             (로봇 두뇌 · AI Service)
```

| 계층 | 통신 |
|---|---|
| 로봇 사이 | ROS 2 (DDS, `ROS_DOMAIN_ID` 분리) |
| 웹·모바일 ↔ 서버 | HTTP / WebSocket |
| 서버 ↔ 앱·DB | TCP |
| 센서·구동부 | 직결 |

---

## 저장소 구조

```
├── client/                     프론트엔드
│   ├── farm_admin_app/         농장 관리자 웹앱 (순찰·수확 요청, 히트맵)
│   └── system_admin_app/       시스템 관리자 Qt 앱 (1Hz 텔레메트리 모니터링)
├── equip/automato_ws/          ROS 2 워크스페이스
│   └── src/
│       ├── automato_interfaces/  공용 msg·srv·action 정의
│       ├── ddago_control/        주행로봇 제어 (Nav2·도킹)
│       ├── ddagi_harvest/        로봇팔 수확 (MoveIt 2·PQS·캘리브레이션)
│       ├── dg_control/           로봇 두뇌 — 상위 관제 연동
│       └── dg_sim/               시뮬레이션
├── services/                   백엔드
│   ├── automato_control_service/  배차·교통관제·task 생성 (FastAPI)
│   ├── automato_web_service/      App-facing API·실시간 피드 (Flask)
│   └── database/                  PostgreSQL 16 + Alembic (12 테이블)
├── evidence/                   실험·검증 기록
└── .github/workflows/          ROS 2 CI
```

---

## 기술 스택

**하드웨어** · PinkyPro(Raspberry Pi 5) 3대 · myCobot 280 6축 로봇팔 2대 · Intel RealSense D435 · NVIDIA GPU 노트북

**로보틱스** · ROS 2 Jazzy Jalisco · Nav2 · SLAM · MoveIt 2 · Octomap · tf2 · pymoveit2 · easy_handeye2 · colcon/rosdep/vcstool

**AI · 비전** · Ultralytics YOLO11s · OpenCV · NumPy · SAM 기반 오토라벨링 · ONNX 변환

**백엔드 · DB** · FastAPI + Uvicorn · Flask + Gunicorn · PostgreSQL 16 · psycopg3 · SQLAlchemy · Alembic

**클라이언트** · PyQt6 + pyqtgraph · HTML/JS

---

## 시작하기

### ROS 2 워크스페이스
```bash
cd equip/automato_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 데이터베이스
```bash
cd services/database
cp .env.example .env
docker compose up -d          # PostgreSQL 16
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
alembic upgrade head          # 스키마 + 시드
python smoke_check.py         # 연결 확인
```

### 웹 서비스
```bash
cd services/automato_web_service
pip install -r requirements.txt
PORT=8899 python3 app.py                                    # 로컬 데모
CONTROL_SERVICE_URL=http://127.0.0.1:7001 PORT=8899 python3 app.py   # ACS 연동
```

각 모듈의 상세 문서는 하위 폴더 README를 참고하세요 — `services/*/README.md`, `equip/automato_ws/src/ddagi_harvest/README.md`, `client/system_admin_app/README.md` 등 **총 27개 문서**가 있습니다.

---

## 팀 Robot8

| | 담당 |
|---|---|
| **이보연** | 주행·관제·반사 도킹 — 경로탐색·교통관제, 반사 테이프 도킹, ACS·DCS 구현, 설계문서 |
| **김동현** | 수확 동작 — 자체 경로 생성(동작 분할), REAP, DDagi 수확 구현, 3D 프린터 토마토 베드 제작 |
| **김희석** | 지도·도킹 — SLAM·주행 파라미터 조정, H 마커리스 도킹, DG Control Service |
| **이건수** | AI·캘리브레이션·앱 — YOLO 학습(숙도·병해충), 로봇팔 캘리브레이션, Farm Admin App·Web Service |

### 개발 방식
- **스프린트 8회전** · Jira 이슈 **87건** · Confluence 문서 **203페이지**
- 브랜치 전략 `main ← dev ← feature/` · 브랜치명 `<타입>/<Jira키>-<설명>` (예: `feature/RP-78-corridor-reservation`)
- PR은 **Squash and merge**로 통일 (Jira 이슈 하나 = dev 커밋 하나)
- **PR 전 로컬 `colcon build` 성공 + 최소 실행 확인 필수** — 리뷰어가 없는 프로젝트라 이것이 dev를 지키는 안전선

---

## 라이선스

**Apache License 2.0** — ROS 2 패키지 `package.xml` 선언 기준, 자체 작성 코드 전체에 적용.

> ⚠️ 추론 백엔드로 사용한 **Ultralytics는 AGPL-3.0(카피레프트)** 이므로 네트워크 서비스 형태로 배포할 경우 결합 범위 검토가 필요합니다. 팀은 (a) 결합 부분 AGPL-3.0 준수 공개, (b) 추론 백엔드를 **ONNX Runtime(MIT)** 으로 교체(변환본 확보 완료) 두 경로를 로드맵에 포함했습니다. 그 외 의존성은 MIT/BSD/Apache-2.0 계열로 상호 충돌이 없으며, LGPL-3.0(easy_handeye2, psycopg)은 동적 링크·별도 프로세스 사용으로 준수합니다.

---

<sub>2026 오픈소스 개발자대회 출품작 (접수번호 647) · 팀 Robot8 Automato</sub>

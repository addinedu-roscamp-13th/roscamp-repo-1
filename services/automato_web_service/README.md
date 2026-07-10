# Automato Web Service

Farm Admin App ↔ **Automato Web Service** ↔ Automato Control Service 중계 (담당: 이건수).
시나리오1(주간 순찰)의 App-facing HTTP API + 실시간 이벤트 피드 + 원격 농장 릴레이를 구현.

- 프론트(Farm Admin App)는 `client/farm_admin_app/index.html`. 라이브 배포(pythonanywhere)에서는 이 서비스가 프론트도 함께 서빙한다.
- 라이브: https://geonsulee.pythonanywhere.com

## 실행
```bash
pip install -r requirements.txt
# (A) 로컬 데모 — 외부 ACS 없이 Web이 순찰을 자체 시뮬
PORT=8899 python3 app.py
# (B) ACS 직결 — Web이 Control로 HTTP 중계
CONTROL_SERVICE_URL=http://127.0.0.1:7001 PORT=8899 python3 app.py
```

## 주요 엔드포인트 (시나리오1)
| 메서드 | 경로 | 용도 |
|---|---|---|
| GET  | `/api/v1/robots/patrol/available` | 순찰 가능 로봇 조회 (E1-0) |
| POST | `/api/v1/patrol/requests` | 순찰 요청 auto/specific (E1-2) · 409 거절 |
| GET  | `/api/v1/patrol/events?since=` | App 실시간 피드(폴링, WS 대체) — patrol_progress/disease_alert/patrol_completed |
| WS   | `/ws/farm-admin` | 스펙 WebSocket (flask-sock, 미지원 환경은 폴링 폴백) |
| POST | `/internal/v1/detections/notify` | ACS→Web 검출현황 콜백 (E2-9) |
| POST | `/internal/v1/alerts/disease` | ACS→Web 병해충 알림 콜백 (E3-1, disease_percent≥5) |
| POST | `/internal/v1/patrol/completed` | ACS→Web 순찰 완료 콜백 |
| POST | `/internal/v1/farm/poll` | **원격 농장(edge) 아웃바운드 폴링** — 명령 수령 + 로봇상태 보고 |
| GET  | `/api/v1/farm/status` · `/api/v1/weblog` | 농장 연결상태 · 웹서비스 로그 tail |

## 원격 농장 릴레이 (edge 아웃바운드)
농장은 NAT 뒤(공개주소 없음)이고 무료 호스팅은 아웃바운드가 막혀 있어, **농장 ACS가 공개 Web으로 먼저 폴링**하는 방식으로 지휘한다.
브라우저→Web(명령 큐)→농장이 폴링으로 수령·실행→콜백→App. 워커 간 유실 방지를 위해 릴레이 상태는 **파일락(`relay.json`)** 으로 공유.
상세: `docs/원격농장_아키텍처_폴링릴레이.md`.

## 알림 정책 (2026-07-08 회의 반영)
- 병해충 알림은 `disease_percent >= 5` 일 때만. dedup 없음(매 순찰). 사진 첨부(레이블본), 파일 저장은 ACS.
- 검출 데이터는 퍼센트(`ripe/unripe/rotten/disease_percent`), `total_count` 삭제.
- 텔레그램·브라우저 알림은 **disease_alert + patrol_completed** 두 이벤트에서만.

## 테스트
```bash
# 로컬 자동 검증(직결 모드): 모의 ACS + 3링크 체인
python3 selftest_chain.py
# 라이브 시나리오1 전 케이스(원격 농장 폴링) 자동 검증
python3 test_scenario1_live.py
```
- 로컬 수동: 터미널① `python3 web_log_tail.py`(웹서비스 로그) + 터미널② `WEB_SERVICE_URL=... python3 farm_agent.py`(농장 대역) + 브라우저.
- Postman: `docs/Automato_WebService.postman_collection.json` (base `http://localhost:8899`).
- 가이드: `docs/시나리오1_라이브_터미널2개_버튼가이드.md`.

## 파일
- `app.py` — Web Service 본체
- `farm_agent.py` — 원격 농장 ACS 대역(폴링). 내일 실기엔 보연님 실제 ACS가 이 폴링 규약으로 대체.
- `mock_control_service.py` — ACS 직결(CONTROL_SERVICE_URL) 모의 서버
- `web_log_tail.py` — 라이브 웹서비스 로그 실시간 tail
- `test_scenario1_live.py` · `selftest_chain.py` — 자동 통신 검증

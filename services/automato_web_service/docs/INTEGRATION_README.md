# 이건수 담당 통신 통합 (시나리오1 Sequence Diagram · 2026-07 개정판)

담당: **Farm Admin App(WebApp) + Automato Web Service**
상대: **Automato Control Service (보연님 컴퓨터)**

## 개정 스펙 반영 (2026-07-08~09 회의)
- 검출 데이터: 개수 → **퍼센트** (`ripe_percent, unripe_percent, rotten_percent, disease_percent`). `total_count`/`*_count` 폐기.
- 병해충 알림: **`disease_percent >= 5`** 일 때만 발동, 중복 억제 없음(매 순찰 발송), **사진(`image_path`) 첨부**.

## 내가 구현한 계약 (Automato Web Service = app.py)
App-facing (Farm Admin App ↔ Web)
- `GET  /api/v1/robots/patrol/available`   → Control 연동 시 `/internal/v1/robots/patrol/available` 중계
- `POST /api/v1/patrol/requests`           → Control 연동 시 `/internal/v1/tasks/patrol` 중계 (200 ACCEPTED / 409 REJECTED 그대로)
- `GET  /api/v1/patrol/events?since=<seq>` → 실시간 피드(폴링). 스펙의 WebSocket `/ws/farm-admin` 대체
                                             (pythonanywhere가 WebSocket 미지원). 이벤트 payload 필드는 스펙과 동일.

Control → Web 콜백 (내부 API, 인바운드)
- `POST /internal/v1/detections/notify`  (E2-9)  → `patrol_progress` 이벤트로 App 에 푸시
- `POST /internal/v1/alerts/disease`     (E3-1)  → `disease_alert` 이벤트 (image_path 포함)
- `POST /internal/v1/patrol/completed`           → `patrol_completed` 이벤트 + 로봇 IDLE 복귀

이벤트 종류: `patrol_progress` / `patrol_completed` / `disease_alert` (스펙 필드 동일)

## 자체 테스트 (Control 없이 단독)
```bash
cd automato-live
python3 selftest_chain.py       # Web(app.py) + 모의 ACS 를 로컬로 띄워 3개 링크 자동 검증
```
검증 항목: App↔Web / Web↔Control 중계 / 전체 체인(순찰요청→퍼센트 검출현황→disease≥5 알림→완료).
→ 2026-07-09 기준 **13/13 통과**.

## 보연님 실제 ACS 와 붙일 때 (최종 통신)
```
Farm Admin App(나) ── HTTP ──▶ Automato Web Service(나, app.py) ── HTTP ──▶ Automato Control Service(보연)
                                          ▲                                          │
                                          └──────── 콜백(내부 API) ◀─────────────────┘
```
1. 내 Web Service 실행 (보연 ACS 주소를 CONTROL_SERVICE_URL 로):
   ```bash
   CONTROL_SERVICE_URL=http://<보연ACS_IP>:<port> PORT=7000 python3 app.py
   ```
2. 보연 ACS 는 콜백을 내 Web 으로 (WEB_SERVICE_URL=http://<내IP>:7000) 보내도록 설정.
3. 같은 LAN 이면 사설 IP, 아니면 ngrok 등 터널.

### ⚠ pythonanywhere 배포 관련 주의
- 무료 pythonanywhere 는 **아웃바운드 화이트리스트**가 있어 임의 호스트(보연 ACS) 호출이 막힘.
  → **최종 통신 테스트는 Web Service 를 내 노트북에서 로컬 실행**(위 방식)하는 게 확실.
- 반대 방향(보연 ACS → 내 Web 의 `/internal/...` 콜백)은 pythonanywhere 공개 URL 로도 잘 들어옴(인바운드).
- 라이브(geonsulee.pythonanywhere.com)는 CONTROL_SERVICE_URL 미설정 → 기존 데모 폴백으로 계속 동작.

## 병해충 알림 '실제 사진' 첨부 (E3)
알림에는 병해충 레이블(bbox) 사진을 **실제 파일로** 첨부한다(텍스트 경로 아님). Web Service가 사진 소스를 이 우선순위로 해석해 텔레그램 `sendPhoto`로 보냄:
1. 알림 payload의 `image_url`(전체 URL) — 있으면 그걸로
2. `ACS_IMAGE_BASE_URL` 설정 + `image_path` → `<base>/<image_path>` 를 telegram 이 fetch
3. Web 로컬 `DETECTION_IMG_DIR/<image_path>` 파일 → 파일 업로드
4. 다 없으면 텍스트로 폴백
웹앱 배너/알림도 `GET /detections/<image_path>` 로 실제 이미지를 표시.

> **실연동 시 보연님과 조율 필요:** ACS가 파일을 저장하므로, Web가 사진을 붙이려면 **ACS가 이미지를 URL로 서빙**(→ `ACS_IMAGE_BASE_URL` 설정)하거나 알림 payload에 **`image_url`/`image_data`(base64)** 를 실어주면 됨. 셋 중 하나만 되면 실제 사진 첨부 완성.

## 남은 App-side 작업 (프론트)
- 웹앱이 `GET /api/v1/patrol/events` 를 폴링해 순찰현황 갱신 + `disease_alert`/`patrol_completed` 시
  **브라우저 Notification** 팝업, **텔레그램 봇** 발송 (텔레그램은 봇토큰+사용자 연동 사전설정 필요).

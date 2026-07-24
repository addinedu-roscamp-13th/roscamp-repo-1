# Automato DB

시나리오 1·2(순찰·관제·수확)용 **PostgreSQL 16 + Alembic 마이그레이션**. (JIRA RP-82, RP-88)
Confluence "DB ERD" 문서(v33)의 12개 테이블을 PostgreSQL로 구성한다.

## 구성 파일

| 파일 | 역할 |
|---|---|
| `docker-compose.yml` | PostgreSQL 16 컨테이너 (DB 기동) |
| `.env` / `.env.example` | 접속정보 (`.env`는 커밋 안 됨) |
| `requirements.txt` | 마이그레이션 도구 (alembic, sqlalchemy, psycopg) |
| `alembic/versions/0001_initial_schema.py` | 12개 테이블 + 트리거 + 인덱스 + 부분 유니크 |
| `alembic/versions/0002_seed.py` | 시드 (배터리 임계값, 로봇, 순찰 지점 waypoints) |
| `smoke_check.py` | ACS→DB 연결 확인 |
| `Makefile` | 자주 쓰는 명령 단축 |

## 빠른 시작

```bash
# 0) (최초 1회) 접속정보 준비
cp .env.example .env          # 필요하면 값 수정

# 1) DB 컨테이너 기동
docker compose up -d
docker compose ps             # STATUS 가 healthy 인지 확인

# 2) 마이그레이션 도구 설치 (가상환경)
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 3) 스키마 + 시드 적용
alembic upgrade head

# 4) 연결 확인
python smoke_check.py
```

> `docker` / `docker compose` 설치: `sudo apt install -y docker.io docker-compose-v2`
> (PostgreSQL 자체는 컨테이너로 뜨므로 따로 설치하지 않는다.)

## Alembic 명령 요약

| 명령 | 설명 |
|---|---|
| `alembic upgrade head` | 최신 리비전까지 전부 적용 |
| `alembic downgrade base` | 전부 롤백 (모든 테이블 삭제) |
| `alembic downgrade -1` | 한 단계만 롤백 |
| `alembic current` | 현재 적용된 리비전 |
| `alembic history` | 리비전 이력 |

`Makefile` 이 있으면 `make up` / `make down` / `make smoke` 로도 가능.

## 스키마 개요 (12개 테이블)

`waypoints` · `robots` · `task_points` · `tasks` · `detection_logs`
· `harvest_batches` · `unload_logs` · `event_logs` · `task_assignment_snapshot`
· `operation_battery_thresholds` · `corridors` · `charuco_boards`

**핵심 규칙**
- **부분 유니크 인덱스** `ux_tasks_active_robot`: 한 로봇에 활성(`WAITING`/`IN_PROGRESS`) task 중복 배정 차단.
- **시드**: `operation_battery_thresholds` = PATROL 70 / HARVEST 50 / TRANSFER 50.
- **`updated_at` 자동 갱신**: `set_updated_at()` 트리거로 UPDATE 시 자동 반영.

## ERD(MySQL 표기) → PostgreSQL 변환

| ERD | PostgreSQL | 이유 |
|---|---|---|
| `INT AUTO_INCREMENT` | `INTEGER GENERATED ALWAYS AS IDENTITY` | PG 표준 자동 증가 |
| `ENUM('A','B')` | `VARCHAR + CHECK` | 값 추가/변경 유연 (팀 결정) |
| `... ON UPDATE CURRENT_TIMESTAMP` | `set_updated_at()` 트리거 | PG엔 해당 문법 없음 |
| `FLOAT` | `DOUBLE PRECISION` | 좌표 정밀도 |
| `TIMESTAMP` | `TIMESTAMPTZ` | UTC 시점 저장, 타임존 버그 예방 |

## 시나리오 2 반영 (ERD v33 · RP-88)

- **`task_paths` 폐기(삭제)**: 경로는 실행 중 재계획되는 휘발성 데이터라 ACS 메모리에서 관리하고, DB엔 요청(`tasks`)과 결과만 남긴다.
- **`charuco_boards` 신설**: 정밀 도킹용 ChArUco 보드/도킹 오프셋. `task_points`와 1:1(`task_point_id` UNIQUE), `marker_id`는 보드가 점유하는 마커 ID 범위의 시작 번호라 자연키.
  단일 ArUco 가 아니라 보드를 쓰는 이유는 자세를 체스판 코너에서 얻어 서브픽셀 정밀도가 나오고, 접붙임 직전 보드가 잘려도 남은 코너로 계산되기 때문. 그래서 보드 구성(`squares_x/y`, `square_size_m`)까지 저장한다.
  CHECK 2개(`marker_size_m < square_size_m`, `squares_x/y >= 3`)로 값 뒤집힘을 막는다 — 뒤집혀도 검출은 되고 거리 추정만 조용히 틀어져 원인 찾기가 어렵다.
- **`robots.charge_point_id`** (FK→`task_points`, NULLABLE): 로봇별 전용 충전소.
- **`task_points.point_type`** (`HARVEST`/`PRECOOL`/`CHARGE`): 작업 위치 종류.
- **`harvest_batches.failed_count` / `exit_reason`**(`DEPLETED`/`FULL`/`MAX_ROUNDS_EXCEEDED`): 수확 성공률·종료 사유.
- **`tasks.status`** 값 `DONE`/`PARTIAL` → `COMPLETED`/`COMPLETED_PARTIAL` 로 문서와 일치.

## 비고

- 이전 초안 `schema.sql`(SQLite 문법, 예시 테이블)은 **RP-82에서 이 Alembic 마이그레이션으로 대체**되어 삭제됨.
- docker 없이 검증하려면 `pgserver`(pip, 개발 전용) 로 로컬 PG를 띄워
  `DATABASE_URL` 을 그 인스턴스로 지정해 동일하게 `alembic upgrade/downgrade` 가능.

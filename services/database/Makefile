# Automato DB 편의 명령 (services/database 에서 실행)
# 사전조건: docker/compose 설치, venv 활성화(source .venv/bin/activate)
.PHONY: help db-up db-down db-logs up down redo status smoke

help:
	@echo "db-up    - PostgreSQL 컨테이너 기동 (docker compose up -d)"
	@echo "db-down  - 컨테이너 정지 (docker compose down)"
	@echo "db-logs  - 컨테이너 로그"
	@echo "up       - 마이그레이션 최신까지 적용 (alembic upgrade head)"
	@echo "down     - 전부 롤백 (alembic downgrade base)"
	@echo "redo     - 전부 롤백 후 재적용 (down + up)"
	@echo "status   - 현재 리비전/이력"
	@echo "smoke    - DB 연결 스모크 체크"

db-up:
	docker compose up -d

db-down:
	docker compose down

db-logs:
	docker compose logs -f postgres

up:
	alembic upgrade head

down:
	alembic downgrade base

redo: down up

status:
	alembic current
	alembic history

smoke:
	python smoke_check.py

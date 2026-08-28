"""Alembic 실행 환경.

DB 접속 URL은 .env 의 DATABASE_URL 에서 읽는다.
ORM 모델을 쓰지 않고 마이그레이션을 손으로 작성하므로 target_metadata = None
(autogenerate 미사용).
"""
import os
from logging.config import fileConfig

from alembic import context
from sqlalchemy import engine_from_config, pool

# --- .env 로드 (services/database/.env) ---
try:
    from dotenv import load_dotenv

    _here = os.path.dirname(__file__)
    load_dotenv(os.path.join(_here, os.pardir, ".env"))
except ImportError:
    # python-dotenv 미설치 시엔 이미 export 된 환경변수를 사용
    pass

config = context.config

# DATABASE_URL 이 있으면 alembic.ini 의 값보다 우선 적용
_db_url = os.environ.get("DATABASE_URL")
if _db_url:
    config.set_main_option("sqlalchemy.url", _db_url)

if config.config_file_name is not None:
    fileConfig(config.config_file_name)

target_metadata = None


def run_migrations_offline() -> None:
    """offline 모드: DB 접속 없이 SQL 스크립트만 생성."""
    url = config.get_main_option("sqlalchemy.url")
    context.configure(
        url=url,
        target_metadata=target_metadata,
        literal_binds=True,
        dialect_opts={"paramstyle": "named"},
    )
    with context.begin_transaction():
        context.run_migrations()


def run_migrations_online() -> None:
    """online 모드: 실제 DB에 접속해 마이그레이션 적용."""
    connectable = engine_from_config(
        config.get_section(config.config_ini_section, {}),
        prefix="sqlalchemy.",
        poolclass=pool.NullPool,
    )
    with connectable.connect() as connection:
        context.configure(connection=connection, target_metadata=target_metadata)
        with context.begin_transaction():
            context.run_migrations()


if context.is_offline_mode():
    run_migrations_offline()
else:
    run_migrations_online()

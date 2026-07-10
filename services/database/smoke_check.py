#!/usr/bin/env python3
"""Automato DB 연결 스모크 체크 (ACS -> DB 배선 확인용).

RP-82 완료조건 "ACS에서 DB 연결 성공"을 만족하는 최소 스크립트.
정밀 검증이 아니라 "연결이 통하고 스키마가 올라와 있나"만 딱 확인한다.

사용:
    # .env 의 DATABASE_URL 사용
    python smoke_check.py
    # 또는 URL 직접 지정
    DATABASE_URL=postgresql://user:pass@host:5432/db python smoke_check.py

종료코드: 성공 0 / 실패 1
"""
import os
import sys

try:
    import psycopg
except ImportError:
    sys.exit("psycopg 가 필요합니다:  pip install 'psycopg[binary]'")


def _load_dsn() -> str:
    """DATABASE_URL(우선) 또는 POSTGRES_* 조합으로 libpq DSN을 만든다."""
    try:
        from dotenv import load_dotenv

        load_dotenv(os.path.join(os.path.dirname(__file__), ".env"))
    except ImportError:
        pass

    url = os.environ.get("DATABASE_URL")
    if url:
        # SQLAlchemy 표기(postgresql+psycopg://)를 libpq 표기(postgresql://)로 정규화
        return url.replace("postgresql+psycopg://", "postgresql://", 1)

    user = os.environ.get("POSTGRES_USER", "automato")
    pw = os.environ.get("POSTGRES_PASSWORD", "automato")
    host = os.environ.get("POSTGRES_HOST", "localhost")
    port = os.environ.get("POSTGRES_PORT", "5432")
    db = os.environ.get("POSTGRES_DB", "automato")
    return f"postgresql://{user}:{pw}@{host}:{port}/{db}"


def main() -> int:
    dsn = _load_dsn()
    # 비밀번호는 로그에 노출하지 않도록 마스킹해서 표기
    shown = dsn
    if "@" in dsn and "://" in dsn:
        scheme, rest = dsn.split("://", 1)
        if "@" in rest:
            shown = f"{scheme}://***@{rest.split('@', 1)[1]}"
    print(f"→ 연결 시도: {shown}")

    try:
        with psycopg.connect(dsn, connect_timeout=5) as conn:
            # ① 살아있나?
            one = conn.execute("SELECT 1").fetchone()[0]
            assert one == 1

            # ② public 스키마 테이블 목록
            rows = conn.execute(
                """
                SELECT table_name FROM information_schema.tables
                WHERE table_schema = 'public' AND table_type = 'BASE TABLE'
                ORDER BY table_name
                """
            ).fetchall()
            tables = [r[0] for r in rows]

            # ③ 시드 확인 (있으면 표시, 없어도 연결 자체는 성공)
            seeded = None
            if "operation_battery_thresholds" in tables:
                seeded = conn.execute(
                    "SELECT count(*) FROM operation_battery_thresholds"
                ).fetchone()[0]
    except Exception as exc:  # noqa: BLE001
        print(f"❌ DB 연결 실패: {exc}")
        return 1

    print(f"✅ DB 연결 성공 — public 테이블 {len(tables)}개")
    if tables:
        print("   " + ", ".join(tables))
    if seeded is not None:
        print(f"   operation_battery_thresholds 시드: {seeded} 행")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

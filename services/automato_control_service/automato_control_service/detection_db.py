#!/usr/bin/env python3
"""RP-79 DB 저장 계층 — 탐지 결과 저장(E2)의 SQL을 여기 모은다.

이 파일은 "탐지 1건을 DB에 어떻게 쓰는가"만 담당한다(순수 데이터 계층).
이미지 파일 저장/순찰 현황 중계(notify)/병해충 알림(alert)은 오케스트레이션
계층(detection_service.py)이 맡는다. 순찰 접수/조회/종료 SQL은 automato_db.py 에 있다.

핵심: 탐지 저장은 detection_logs INSERT 한 문으로 한다.
  ① detection_logs INSERT     → detection_id 확보(ACS 내부용, HQ 로 미반환)
  RP-88: 예전엔 task_paths.is_visited=TRUE 도 같이 찍었으나, task_paths(휘발성 경로)가
  폐기되어 방문 기록은 detection_logs 행 존재로 대신한다(단문이지만 트랜잭션 블록은 유지).

배경 지식(왜 detected_at 을 파라미터로 받나):
  이슈 요구대로 핸들러 진입 시 시각을 '한 번만' 캡처해서 DB·notify·alert 가
  전부 같은 값을 쓰게 한다(타임스탬프 일관성). 그래서 SQL 안에서 NOW() 를 쓰지 않고
  호출부가 만든 datetime 을 그대로 바인딩한다.

주의(스키마 실측): detection_logs 의 이미지 컬럼명은 image_path 가 아니라
  disease_image_path 다(0001_initial_schema.py). 상대경로만 저장한다(루트는 설정값).
"""
from datetime import datetime

from psycopg_pool import ConnectionPool

# ① 탐지 결과 INSERT. detected_at 은 호출부가 캡처한 값을 바인딩(NOW() 안 씀).
#    disease_image_path 는 상대경로(루트 제외) 또는 None.
_INSERT_DETECTION = (
    "INSERT INTO detection_logs "
    "  (task_id, robot_id, waypoint_id, "
    "   ripe_percent, unripe_percent, rotten_percent, disease_percent, "
    "   disease_image_path, detected_at) "
    "VALUES (%s, %s, %s, %s, %s, %s, %s, %s, %s) "
    "RETURNING detection_id"
)


def save_detection_log(
    pool: ConnectionPool,
    *,
    task_id: int,
    robot_id: str,
    waypoint_id: int,
    ripe_percent: int,
    unripe_percent: int,
    rotten_percent: int,
    disease_percent: int,
    image_path,           # str | None (루트 제외 상대경로; 이미지 없음/쓰기실패면 None)
    detected_at: datetime,
) -> int:
    """탐지 1건을 detection_logs 에 저장하고 detection_id 를 반환한다.

    ① detection_logs INSERT (RETURNING detection_id)
    RP-88: task_paths 폐기로 방문 표시(is_visited)는 없앴다 — 방문 사실은
    detection_logs 행 존재로 대신한다.

    반환: detection_id (INSERT 로 새로 발급된 PK)
    예외: 실패 시 psycopg 예외를 그대로 올린다(호출부가 잡아 success=false 처리).
    """
    with pool.connection() as conn:
        with conn.transaction():   # BEGIN ~ COMMIT/ROLLBACK 자동 관리
            row = conn.execute(
                _INSERT_DETECTION,
                (
                    int(task_id), robot_id, int(waypoint_id),
                    int(ripe_percent), int(unripe_percent),
                    int(rotten_percent), int(disease_percent),
                    image_path, detected_at,
                ),
            ).fetchone()
            detection_id = row["detection_id"]
    return detection_id

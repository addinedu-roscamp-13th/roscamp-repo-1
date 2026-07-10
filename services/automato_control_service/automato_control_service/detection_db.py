#!/usr/bin/env python3
"""RP-79 DB 저장 계층 — 탐지 결과 저장(E2)의 SQL을 여기 모은다.

이 파일은 "탐지 1건을 DB에 어떻게 쓰는가"만 담당한다(순수 데이터 계층).
이미지 파일 저장/순찰 현황 중계(notify)/병해충 알림(alert)은 오케스트레이션
계층(detection_service.py)이 맡는다. 순찰 접수/조회/종료 SQL은 patrol_db.py 에 있다.

핵심(원자성): 탐지 저장은 '하나의 트랜잭션'으로 묶는다.
  ① detection_logs INSERT     → detection_id 확보(ACS 내부용, HQ 로 미반환)
  ② task_paths.is_visited=TRUE (해당 task_id + waypoint_id 행)
  두 문이 한 트랜잭션이라 중간 실패 시 전부 롤백된다(반쯤 기록된 탐지가 안 남음).

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

# ② 방문 표시. updated_at 은 set_updated_at() 트리거가 자동 갱신하지만,
#    patrol_db 관례(명시 갱신)와 맞춰 명시적으로도 NOW() 를 넣는다.
_MARK_VISITED = (
    "UPDATE task_paths "
    "   SET is_visited = TRUE, updated_at = NOW() "
    " WHERE task_id = %s AND waypoint_id = %s"
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
    """탐지 1건을 단일 트랜잭션으로 저장하고 detection_id 를 반환한다.

    ① detection_logs INSERT (RETURNING detection_id)
    ② task_paths.is_visited = TRUE (task_id + waypoint_id)

    반환: detection_id (INSERT 로 새로 발급된 PK)
    예외: 실패 시 psycopg 예외를 그대로 올린다(호출부가 잡아 success=false 처리).
          트랜잭션 블록이라 예외 시 두 문 모두 자동 롤백된다.
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
            conn.execute(_MARK_VISITED, (int(task_id), int(waypoint_id)))
    return detection_id

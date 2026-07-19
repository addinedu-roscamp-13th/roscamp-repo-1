#!/usr/bin/env python3
"""ACS 순찰 노드 설정값 — 토픽/서비스 이름(코드 상수) + 타이밍 튜닝값(.env → 환경변수).

왜 이렇게 나눴나:
  * 토픽/서비스 이름은 시스템 '구조'라 배포마다 바뀌지 않는다 → 코드 상수로 고정.
  * 타이밍(대기·타임아웃·TTL)은 현장/시뮬마다 조정하고 싶은 '튜닝값'이라 .env 로 외부화.
    코드를 안 건드리고 값만 바꿔 재기동할 수 있고, dev/sim/실물마다 .env 를 달리 둘 수 있다.

로딩 규칙:
  * import 시 .env 를 한 번 읽어 os.environ 에 넣는다(이미 설정된 값은 '덮지 않음').
    → 셸 export 나 ROS launch 가 넣은 값이 .env 보다 우선(운영 오버라이드 가능).
  * .env 를 못 찾아도 조용히 넘어가고 아래 _envf 기본값이 쓰인다(파일은 필수 아님).
  * 이 모듈은 '순수 설정'이라 로그를 두지 않는다(로깅 규칙: 순수 모듈은 비우고 호출부에서).
"""
import os


def _load_dotenv_once() -> None:
    """가장 가까운 .env 를 찾아 KEY=VALUE 를 os.environ 에 채운다(이미 있으면 유지).

    탐색 순서(먼저 찾은 것 하나만 사용):
      1) 환경변수 ACS_ENV_FILE 이 가리키는 파일(명시적 지정, 최우선)
      2) 현재 작업 디렉터리(CWD)에서 위로 올라가며 .env
      3) 이 소스 파일 위치에서 위로 올라가며 .env (colcon symlink-install 대비)
    단순 KEY=VALUE 만 해석한다(따옴표/치환 없음 — 우리 값은 숫자뿐이라 충분).
    """
    def _walk_up(start):
        d = os.path.abspath(start)
        while True:
            yield os.path.join(d, ".env")
            parent = os.path.dirname(d)
            if parent == d:      # 루트('/')에 도달
                return
            d = parent

    candidates = []
    explicit = os.environ.get("ACS_ENV_FILE")
    if explicit:
        candidates.append(explicit)
    candidates.extend(_walk_up(os.getcwd()))
    candidates.extend(_walk_up(os.path.dirname(os.path.realpath(__file__))))

    for path in candidates:
        if not path or not os.path.isfile(path):
            continue
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, _, val = line.partition("=")
                # setdefault: 이미 설정돼 있으면 안 덮음 → 셸/launch 값이 우선
                os.environ.setdefault(key.strip(), val.strip())
        return                    # 첫 번째로 찾은 .env 만 사용


_load_dotenv_once()


def _envf(name: str, default: float) -> float:
    """환경변수를 float 로 읽되, 없거나 이상하면 기본값."""
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _envi(name: str, default: int) -> int:
    """환경변수를 int 로 읽되, 없거나 이상하면 기본값."""
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


# ── 토픽/서비스 이름 (구조적 상수, .env 대상 아님) ─────────────────────────── #
FLEET_TOPIC = "/automato/telemetry/fleet"
# RP-79: DG 가 waypoint 마다 탐지 결과를 넘기는 ROS2 Service (ACS 가 서버).
SAVE_DETECTION_SRV = "/automato/save_detection"

# ── 타이밍 튜닝값 (.env 로 외부화, 아래 숫자는 기본값) ─────────────────────── #
# 액션 서버 접속 대기 / Goal 수락 대기 / 세그먼트(1 waypoint) 결과 대기(초)
SERVER_WAIT_SEC = _envf("ACS_SERVER_WAIT_SEC", 5.0)
GOAL_ACCEPT_TIMEOUT_SEC = _envf("ACS_GOAL_ACCEPT_TIMEOUT_SEC", 30.0)
# 순찰은 로봇 Nav2가 자체 재시도(2분×3=6분)를 하므로 그보다 넉넉히 기다린다.
SEGMENT_TIMEOUT_SEC = _envf("ACS_SEGMENT_TIMEOUT_SEC", 420.0)
# 통로 예약 대기(양보 전) / 예약 폴링 간격
RESERVE_WAIT_SEC = _envf("ACS_RESERVE_WAIT_SEC", 30.0)
RESERVE_POLL_SEC = _envf("ACS_RESERVE_POLL_SEC", 1.0)
# 막힘/양보 통로를 재계획에서 제외해 둘 시간(N초 블랙리스트)
BLOCK_TTL_SEC = _envf("ACS_BLOCK_TTL_SEC", 30.0)
# 이동 대기 중 예약 유지용 하트비트 간격 / 엔진 예약 TTL(하트비트보다 커야 함)
HEARTBEAT_SEC = _envf("ACS_HEARTBEAT_SEC", 5.0)
RESERVATION_TTL_SEC = _envf("ACS_RESERVATION_TTL_SEC", 15.0)

# ── 순찰 시작(정적) 노드 ─────────────────────────────────────────────────── #
# 순찰은 항상 로봇이 충전소에 있을 때 시작한다(시작 위치 고정). 라우터는 waypoint_id 로만
# 경로를 계산하는데, 충전소(task_points)와 그래프 노드(waypoints)는 FK 로 안 이어져 있어
# '어느 waypoint 가 시작점인지'를 여기서 정적으로 지정한다. 지금은 순찰이 전역 1대뿐이라
# 전역 상수 하나로 충분하다. (향후: task_points↔waypoints 연결이 생기면 로봇별
# charge_point_id 로 시작 노드를 유도하도록 대체 예정.)
# 이 값이 그래프(wp_meta)에 없으면 디스패처가 옛 동작(첫 지점 예약 없이 직행)으로 폴백한다.
PATROL_START_WAYPOINT_ID = _envi("ACS_PATROL_START_WAYPOINT_ID", 15)

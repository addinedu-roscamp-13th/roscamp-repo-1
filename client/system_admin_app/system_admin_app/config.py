"""앱 전역 설정. 토픽/서비스 이름, 로봇 구성, 임계값을 한곳에서 관리한다.

임계값은 팀 협의로 바뀔 수 있으므로 매직넘버를 UI/모델에 흩지 않고 여기 모은다.
"""
import os

# --- 배경 맵 (SLAM 점유격자) ---
# "file"  : 정적 .pgm/.yaml 로드 (기본, 맵 고정 농장 관제에 적합)
# "topic" : /map (nav_msgs/OccupancyGrid) 구독  (추후 옵션)
# "none"  : 배경 없이 좌표 평면만
MAP_SOURCE = os.environ.get("AUTOMATO_MAP_SOURCE", "file")
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def _default_map_yaml() -> str:
    """소스 실행(python3 -m)과 설치 실행(ros2 run) 양쪽에서 맵을 찾는다."""
    src = os.path.join(_PKG_ROOT, "maps", "automato_map.yaml")
    if os.path.exists(src):
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory("system_admin_app"),
            "maps", "automato_map.yaml",
        )
    except Exception:
        return src


MAP_YAML_PATH = os.environ.get("AUTOMATO_MAP_YAML", _default_map_yaml())
MAP_TOPIC = "/map"

# --- ROS2 토픽 / 서비스 이름 (E0 통신 규격 4번) ---
TOPIC_FLEET_TELEMETRY = "/automato/dashboard/fleet_telemetry"
# [제안] 제어탭 유지보수 명령: QT -> ACS -> HQ
SERVICE_MAINTENANCE = "/automato/maintenance/command"

# --- 로봇 편대 구성 ---
ROBOT_IDS = ["dg_01", "dg_02", "dg_03"]
# 주행 전용(로봇팔 Ddagi 없음) 로봇. dg_03은 주행 전용.
DRIVE_ONLY_ROBOTS = {"dg_03"}

# 로봇팔 서보 개수 (6관절 + 그리퍼)
SERVO_COUNT = 7
GRIPPER_JOINT_NO = 7

# --- 실시간 그래프 ---
GRAPH_HZ = 1                     # 텔레메트리 1Hz
GRAPH_WINDOW_SEC = 60            # 1분 창
GRAPH_MAXLEN = GRAPH_HZ * GRAPH_WINDOW_SEC  # 60 포인트
# E0 원칙: 저장하지 않음. 링버퍼는 메모리에만 존재하며 앱 종료 시 사라진다.

# --- 수신 감시(liveness) ---
STALE_SEC = 3.0                  # 마지막 수신이 이보다 오래되면 통신 두절로 간주

# --- 판정 임계값 ---
BATTERY_WARN_PERCENT = 30.0      # 이하: 주의
BATTERY_CRIT_PERCENT = 20.0      # 이하: 위험
SERVO_TEMP_WARN_C = 60           # 이상: 주의
SERVO_TEMP_CRIT_C = 65           # 이상: 과열(위험). 그래프 기준선도 이 값.

# --- nav_status 표시 매핑 (라벨/레벨) ---
# level: ok / busy / warn / crit
NAV_STATUS_META = {
    "IDLE":        {"label": "대기",   "level": "ok"},
    "NAVIGATING":  {"label": "주행 중", "level": "busy"},
    "PATROLLING":  {"label": "순찰 중", "level": "busy"},
    "CHARGING":    {"label": "충전 중", "level": "ok"},
    "STOP":        {"label": "정지",   "level": "warn"},
    "ERROR":       {"label": "오류",   "level": "crit"},
}


def nav_status_meta(nav_status: str) -> dict:
    """알 수 없는 상태는 원문 라벨 + warn 레벨로 안전하게 처리."""
    return NAV_STATUS_META.get(
        nav_status, {"label": nav_status or "-", "level": "warn"}
    )

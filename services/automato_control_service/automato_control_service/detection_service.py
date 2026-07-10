#!/usr/bin/env python3
"""RP-79 오케스트레이션 계층 — save_detection 1콜의 저장/중계/알림을 조율한다.

시나리오 1 E2(탐지 저장·순찰 현황 중계) + E3(병해충 알림)를 한 진입점
(`DetectionHandler.on_request`)에서 이어 처리하되, 이슈 요구대로 **저장/중계/알림을
별도 함수로 분리**한다:

  1) 이미지 저장   store_disease_image()      — disease_percent >= 5 && 이미지 존재일 때만
  2) DB 저장      detection_db.save_detection_log()  (단일 트랜잭션)
  3) 순찰 중계    send_notify()               — fire-and-forget(재시도 없음)
  4) 병해충 알림  send_disease_alert()        — disease_percent >= 5 일 때만, 3회 재시도

처리 순서(1콜 = 아래 순서대로):
  detected_at 1회 캡처 → (게이트면)이미지 저장 → DB 트랜잭션 → notify → (게이트면)alert → 응답

비블로킹 원칙(순찰 루프를 막지 않음):
  응답 success 는 'DB 저장 성공 여부'만 반영한다. 그래서 이미지 저장·DB 는 동기로
  끝내 detection_id/success 를 확정하고, HTTP(notify/alert)는 **백그라운드 스레드풀**로
  던진 뒤 즉시 응답한다. HTTP 재시도/지연이 서비스 응답이나 순찰을 막지 않는다.

타임스탬프 일관성:
  핸들러 진입 시 detected_at 을 **한 번만** 캡처(UTC)해서 DB INSERT·notify·alert·
  이미지 경로에 **동일 값**을 쓴다. 그래서 SQL 안 NOW() 를 쓰지 않는다.

게이트(disease_percent >= 5) 공유:
  '이미지 저장'과 '병해충 알림'은 동일 임계값을 공유한다. 미만이면 둘 다 건너뛴다.

HTTP 클라이언트:
  형제 서비스(automato_web_service)와 동일하게 stdlib urllib.request 를 쓴다(의존성 추가 없음).
  수신처(대시보드/알림 백엔드)는 아직 미정이라 base URL 을 설정값(AUTOMATO_WEB_SERVICE_URL)으로
  빼둔다. 경로는 계약이라 상수로 고정한다.
"""
import json
import os
import time
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone

# NOTE: detection_db(psycopg 의존)는 __init__ 의 기본 db_fn 을 만들 때만 '지연 임포트'한다.
# 그래야 DB 드라이버 없이도 이 모듈을 임포트해 순수 로직/오케스트레이션을 단위테스트할 수 있다.

# --------------------------------------------------------------------------- #
# 설정 상수 / env
# --------------------------------------------------------------------------- #
# 이미지 저장·병해충 알림 공통 게이트 임계값.
DISEASE_ALERT_THRESHOLD = 5

# notify/alert 수신 백엔드 base URL(미정 → 실행 시 지정). 경로는 아래 상수로 고정.
NOTIFY_PATH = "/internal/v1/detections/notify"
ALERT_PATH = "/internal/v1/alerts/disease"


def _env(name: str, default: str) -> str:
    return os.environ.get(name, default)


def _envf(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


def _envi(name: str, default: int) -> int:
    try:
        return int(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default


# --------------------------------------------------------------------------- #
# 순수 함수: 경로 규칙 / payload 구성 (ROS/DB/네트워크 의존 없음 → 단위테스트 대상)
# --------------------------------------------------------------------------- #
def relative_image_path(waypoint_id: int, robot_id: str, dt: datetime) -> str:
    """병해충 이미지의 '루트 제외 상대경로'를 만든다.

    규칙: {YYYY-MM-DD}/wp{waypoint_id}_{robot_id}_{HHMMSS}.jpg
    dt 는 detected_at(UTC). 날짜 폴더로 묶고 파일명에 지점/로봇/시각을 담아
    한 순찰에서도 파일이 안 겹치게 한다. DB엔 이 상대경로만 저장(루트는 설정값).
    """
    day = dt.strftime("%Y-%m-%d")
    hhmmss = dt.strftime("%H%M%S")
    return f"{day}/wp{int(waypoint_id)}_{robot_id}_{hhmmss}.jpg"


def build_notify_payload(*, task_id, waypoint_id, robot_id, detection_id,
                         ripe_percent, unripe_percent, rotten_percent,
                         disease_percent, detected_at: datetime) -> dict:
    """순찰 현황 중계(notify) 몸통. zone_cumulative 는 없음(제거됨).

    detection_id 는 DB 저장 성공 시 정수, 실패 시 None(null 로 직렬화).
    """
    return {
        "task_id": int(task_id),
        "waypoint_id": int(waypoint_id),
        "robot_id": robot_id,
        "detection_id": detection_id,
        "ripe_percent": int(ripe_percent),
        "unripe_percent": int(unripe_percent),
        "rotten_percent": int(rotten_percent),
        "disease_percent": int(disease_percent),
        "detected_at": detected_at.isoformat(),
    }


def build_alert_payload(*, task_id, waypoint_id, robot_id, disease_percent,
                        image_path, detected_at: datetime) -> dict:
    """병해충 알림(alert) 몸통. image_path 가 없으면(쓰기 실패 등) "" 로 보낸다."""
    return {
        "task_id": int(task_id),
        "waypoint_id": int(waypoint_id),
        "robot_id": robot_id,
        "disease_percent": int(disease_percent),
        "image_path": image_path or "",
        "detected_at": detected_at.isoformat(),
    }


# --------------------------------------------------------------------------- #
# 1) 이미지 저장
# --------------------------------------------------------------------------- #
def store_disease_image(root: str, waypoint_id: int, robot_id: str,
                        dt: datetime, image_bytes: bytes, log=None):
    """병해충 이미지 바이트를 파일로 저장하고 '상대경로'를 반환한다.

    저장 위치: {root}/{상대경로}.  DB엔 상대경로만 기록하므로 이 함수도 상대경로를 반환.
    실패(디렉터리 생성/파일 쓰기 오류) 시: 경고 로그 + None 반환(저장·알림은 계속 진행).
    """
    rel = relative_image_path(waypoint_id, robot_id, dt)
    abs_path = os.path.join(root, rel)
    try:
        os.makedirs(os.path.dirname(abs_path), exist_ok=True)
        with open(abs_path, "wb") as f:
            f.write(image_bytes)
        if log is not None:
            log.info(f"병해충 이미지 저장: {rel} ({len(image_bytes)} bytes)")
        return rel
    except Exception as exc:  # noqa: BLE001  (파일 실패는 삼키지 않고 로그로 남김)
        if log is not None:
            log.warn(f"병해충 이미지 저장 실패({abs_path}): {exc} → image_path=null 로 진행")
        return None


def image_msg_to_jpeg_bytes(image_msg, log=None) -> bytes:
    """sensor_msgs/Image(raw 픽셀) → JPEG 바이트. 빈 이미지·변환 실패 시 b"" 반환.

    ROS 어댑터(on_request)에서만 호출한다. cv_bridge/cv2 는 여기서만 필요하므로
    detection_db 처럼 '지연 임포트'해서, DB·이미지 라이브러리 없이도 이 모듈을 임포트해
    순수 로직/오케스트레이션을 단위테스트할 수 있게 둔다.

    왜 인코딩이 필요한가: sensor_msgs/Image 는 관례상 무압축 raw 픽셀이라 그대로 .jpg 로
    저장할 수 없다. store_disease_image 는 'JPEG 바이트'를 파일에 쓰므로, 그 전에 여기서
    한 번 인코딩한다. disease_percent<5 로 빈 Image(height==0/width==0)면 b"" → 저장 게이트가
    '이미지 없음'으로 처리한다.
    """
    if getattr(image_msg, "height", 0) == 0 or getattr(image_msg, "width", 0) == 0:
        return b""
    try:
        from cv_bridge import CvBridge  # 지연 임포트(테스트 시 미요구)
        import cv2
        cv_img = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_img)
        if not ok:
            if log is not None:
                log.warn("병해충 이미지 JPEG 인코딩 실패 → 이미지 없이 진행")
            return b""
        return buf.tobytes()
    except Exception as exc:  # noqa: BLE001
        if log is not None:
            log.warn(f"병해충 이미지 변환 실패({exc}) → 이미지 없이 진행")
        return b""


# --------------------------------------------------------------------------- #
# HTTP 유틸 (stdlib urllib) — 2xx 면 상태코드 반환, 아니면 예외
# --------------------------------------------------------------------------- #
def post_json(url: str, payload: dict, timeout: float) -> int:
    """JSON 을 POST 하고 HTTP 상태코드를 반환. 비2xx/네트워크 오류는 예외로 올린다."""
    data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(
        url, data=data,
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return int(getattr(r, "status", 200))


# --------------------------------------------------------------------------- #
# 3) 순찰 현황 중계 — notify (fire-and-forget, 재시도 없음)
# --------------------------------------------------------------------------- #
def send_notify(base_url: str, payload: dict, timeout: float = 3.0, log=None) -> bool:
    """notify 를 1회 발송. 비200/예외여도 **재시도 없이 로그만** 남기고 넘어간다.

    반환: 성공 여부(bool). 성공/실패 모두 순찰 루프엔 영향 없음(호출측이 백그라운드로 던짐).
    """
    url = base_url.rstrip("/") + NOTIFY_PATH
    try:
        status = post_json(url, payload, timeout)
        if log is not None:
            log.info(f"notify 발송 OK({status}) wp={payload.get('waypoint_id')}")
        return True
    except Exception as exc:  # noqa: BLE001
        if log is not None:
            log.warn(f"notify 실패(재시도 안 함) {url}: {exc}")
        return False


# --------------------------------------------------------------------------- #
# 4) 병해충 알림 — disease alert (조건부 + 가벼운 재시도)
# --------------------------------------------------------------------------- #
def send_disease_alert(base_url: str, payload: dict, timeout: float = 3.0,
                       retries: int = 3, retry_delay: float = 0.5,
                       log=None, sleep=time.sleep) -> bool:
    """alert 를 최대 retries 회 시도. 비200/예외면 잠깐 쉬고 재시도, 최종 실패면 로그.

    retries=3 → 최대 3회 시도(최초 + 재시도 포함). 재시도 사이 retry_delay 초 대기(가벼운 백오프).
    이 함수는 백그라운드 스레드에서 돌기에 sleep 이 순찰 루프를 막지 않는다.
    반환: 성공 여부(bool).
    """
    url = base_url.rstrip("/") + ALERT_PATH
    attempts = max(1, int(retries))
    last_err = None
    for i in range(1, attempts + 1):
        try:
            status = post_json(url, payload, timeout)
            if log is not None:
                log.info(
                    f"disease alert 발송 OK({status}) "
                    f"wp={payload.get('waypoint_id')} (시도 {i}/{attempts})")
            return True
        except Exception as exc:  # noqa: BLE001
            last_err = exc
            if i < attempts:
                sleep(retry_delay)
    if log is not None:
        log.error(
            f"disease alert 최종 실패({attempts}회) {url}: {last_err}")
    return False


# --------------------------------------------------------------------------- #
# 오케스트레이터 — ROS2 Service 콜백 + 순서/실패 정책
# --------------------------------------------------------------------------- #
class DetectionHandler:
    """/automato/save_detection 서비스 콜백을 처리한다(저장/중계/알림 조율).

    의존은 생성자에서 주입한다(테스트에서 fake 로 갈아끼우기 쉽게):
      pool        : psycopg 커넥션 풀(DB 저장)
      logger      : rclpy 로거(없으면 로깅 생략)
      executor    : 백그라운드 HTTP 디스패치용 ThreadPoolExecutor(없으면 자동 생성)
      *_fn / now_fn : 이미지 저장/DB/ notify/alert/시각 캡처 함수(테스트 주입용)
      dispatch    : 백그라운드 실행기(기본 executor.submit; 테스트는 동기 실행)
    """

    def __init__(self, pool, *, logger=None,
                 image_root=None, web_service_url=None,
                 notify_timeout=None, alert_timeout=None,
                 alert_retries=None, threshold=DISEASE_ALERT_THRESHOLD,
                 executor=None, dispatch=None, now_fn=None,
                 store_fn=None, db_fn=None, notify_fn=None, alert_fn=None):
        self._pool = pool
        self._log = logger
        self._threshold = int(threshold)

        self._image_root = image_root if image_root is not None else _env(
            "DETECTION_IMAGE_ROOT",
            os.path.expanduser("~/automato_detections"))
        self._base_url = web_service_url if web_service_url is not None else _env(
            "AUTOMATO_WEB_SERVICE_URL", "http://localhost:8100")
        self._notify_timeout = (
            notify_timeout if notify_timeout is not None
            else _envf("ACS_NOTIFY_TIMEOUT_SEC", 3.0))
        self._alert_timeout = (
            alert_timeout if alert_timeout is not None
            else _envf("ACS_ALERT_TIMEOUT_SEC", 3.0))
        self._alert_retries = (
            alert_retries if alert_retries is not None
            else _envi("ACS_ALERT_RETRIES", 3))

        # 백그라운드 디스패치: notify/alert 를 서비스 응답과 분리해 순찰 비블로킹 보장.
        self._executor = executor or ThreadPoolExecutor(
            max_workers=4, thread_name_prefix="detection-http")
        self._dispatch = dispatch or (lambda fn: self._executor.submit(fn))

        # 주입 가능한 협력자(기본은 이 모듈의 실제 구현).
        self._now = now_fn or (lambda: datetime.now(timezone.utc))
        self._store_fn = store_fn or store_disease_image
        if db_fn is None:
            # DB 기본 구현만 psycopg 를 끌어오므로 여기서 지연 임포트한다.
            from automato_control_service import detection_db
            db_fn = detection_db.save_detection_log
        self._db_fn = db_fn
        self._notify_fn = notify_fn or send_notify
        self._alert_fn = alert_fn or send_disease_alert

    # ---- ROS2 Service 콜백 어댑터 ---- #
    def on_request(self, request, response):
        """rclpy 서비스 콜백. ROS 요청을 평범한 값으로 풀어 _process 에 넘긴다."""
        # disease_image 는 sensor_msgs/Image(raw). 저장은 JPEG 파일이므로 여기서 인코딩해
        # bytes 로 바꾼다(빈 Image → b""). _process 이하 코어는 계속 bytes 로만 다룬다.
        image_bytes = image_msg_to_jpeg_bytes(request.disease_image, log=self._log)
        success, message = self._process(
            task_id=request.task_id,
            waypoint_id=request.waypoint_id,
            robot_id=request.robot_id,
            ripe_percent=request.ripe_percent,
            unripe_percent=request.unripe_percent,
            rotten_percent=request.rotten_percent,
            disease_percent=request.disease_percent,
            image_bytes=image_bytes,
        )
        response.success = bool(success)
        response.message = message
        return response

    # ---- 순서/실패 정책 본체(ROS 비의존) ---- #
    def _process(self, *, task_id, waypoint_id, robot_id,
                 ripe_percent, unripe_percent, rotten_percent,
                 disease_percent, image_bytes) -> tuple:
        """1콜 처리. 반환: (success, message).  success 는 DB 저장 성공 여부만 반영."""
        detected_at = self._now()                      # ★ 한 번만 캡처(전 단계 공유)
        gate = int(disease_percent) >= self._threshold

        # 1) 이미지 저장 — 게이트 통과 && 이미지 바이트 존재일 때만
        image_path = None
        if gate and image_bytes:
            image_path = self._store_fn(
                self._image_root, waypoint_id, robot_id, detected_at,
                image_bytes, log=self._log)
        elif gate and not image_bytes and self._log is not None:
            self._log.warn(
                f"disease_percent>={self._threshold} 인데 이미지 바이트 없음 "
                f"wp={waypoint_id} → image_path=null")

        # 2) DB 저장(단일 트랜잭션). 실패해도 아래 notify/alert 는 계속 발송.
        detection_id = None
        try:
            detection_id = self._db_fn(
                self._pool,
                task_id=task_id, robot_id=robot_id, waypoint_id=waypoint_id,
                ripe_percent=ripe_percent, unripe_percent=unripe_percent,
                rotten_percent=rotten_percent, disease_percent=disease_percent,
                image_path=image_path, detected_at=detected_at)
            success = True
            message = f"detection {detection_id} 저장 완료"
        except Exception as exc:  # noqa: BLE001
            success = False
            message = f"DB 저장 실패: {exc}"
            if self._log is not None:
                self._log.error(
                    f"detection DB 저장 실패 task={task_id} wp={waypoint_id}: {exc}")

        # 3) 순찰 현황 중계 — notify(fire-and-forget). DB 실패면 detection_id=null.
        notify_payload = build_notify_payload(
            task_id=task_id, waypoint_id=waypoint_id, robot_id=robot_id,
            detection_id=detection_id,
            ripe_percent=ripe_percent, unripe_percent=unripe_percent,
            rotten_percent=rotten_percent, disease_percent=disease_percent,
            detected_at=detected_at)
        self._dispatch(
            lambda: self._notify_fn(
                self._base_url, notify_payload,
                timeout=self._notify_timeout, log=self._log))

        # 4) 병해충 알림 — 게이트 통과일 때만. DB 실패해도 안전 위해 발송.
        if gate:
            alert_payload = build_alert_payload(
                task_id=task_id, waypoint_id=waypoint_id, robot_id=robot_id,
                disease_percent=disease_percent, image_path=image_path,
                detected_at=detected_at)
            self._dispatch(
                lambda: self._alert_fn(
                    self._base_url, alert_payload,
                    timeout=self._alert_timeout,
                    retries=self._alert_retries, log=self._log))

        # 5) HQ 응답 — success 는 DB 결과만, notify/alert 성패는 반영 안 함.
        return success, message

    def shutdown(self) -> None:
        """백그라운드 스레드풀 정리(프로세스 종료 시)."""
        try:
            self._executor.shutdown(wait=False)
        except Exception:  # noqa: BLE001
            pass

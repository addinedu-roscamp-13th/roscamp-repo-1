import base64
import os
from collections import Counter
from typing import Any, Dict, Optional, Tuple

try:
    import cv2
    import numpy as np
    from ultralytics import YOLO
except ImportError:  # pragma: no cover
    cv2 = None
    np = None
    YOLO = None

CLASSES = ['ripe', 'unripe', 'rotten', 'disease']
# 이 클래스가 하나라도 감지되면 응답에 레이블링된 이미지를 함께 반환한다.
LABEL_TRIGGER_CLASSES = ('rotten', 'disease')

# RP-127  DetectTomatoes(시나리오2) 등급 매핑. ripe -> 수확품(NORMAL),
# rotten/disease -> 폐기품(DISCARD). unripe는 매핑에 없음 -> 검출 결과에서
# 제외한다(안익은 열매는 따지 않는다).
GRADE_BY_LABEL = {
    'ripe': 'NORMAL',
    'rotten': 'DISCARD',
    'disease': 'DISCARD',
}

# 레이블링 이미지(encode_labeled_image)에 그려지는 박스 텍스트만 한글로 표시.
# 모델 매칭(CLASSES)과 JSON 응답 필드명(ripe_percent 등)은 영문을 그대로 쓴다.
KOREAN_LABELS = {
    'ripe': '익음',
    'unripe': '미숙',
    'rotten': '무름',
    'disease': '병충해',
}


class ModelNotReadyError(RuntimeError):
    pass


class TomatoDetector:
    def __init__(self, model_path: str, conf: float = 0.4):
        if YOLO is None or cv2 is None or np is None:
            raise ModelNotReadyError(
                'Missing dependency: install ultralytics, opencv-python, numpy to enable image analysis.'
            )
        if not os.path.exists(model_path):
            raise ModelNotReadyError(
                f'Model file not found: {model_path}.\n'
                'Set DG_AI_MODEL_PATH environment variable or pass --model-path to point to tomato_4cls_model.pt.'
            )
        self.model = YOLO(model_path)
        self.conf = conf

    def warmup(self) -> None:
        """더미 이미지로 최초 1회 추론을 미리 실행.

        ultralytics/torch 는 로딩 후 첫 predict() 호출에서 커널 준비 등
        추가 비용이 들어(수백ms~수 초) 실제 첫 analyze_frame 요청이 그
        비용을 고스란히 떠안는다. 서버 기동 직후 백그라운드에서 이걸
        미리 태워 실제 요청은 항상 빠르게 응답하도록 한다.
        """
        dummy = np.zeros((64, 64, 3), dtype=np.uint8)
        self.model.predict(dummy, conf=self.conf, verbose=False)

    def decode_image(self, image_data: str) -> Any:
        try:
            raw = base64.b64decode(image_data, validate=True)
        except Exception as exc:
            raise ValueError(f'IMAGE_DECODE_FAILED: base64 디코딩 실패 ({exc})') from exc
        arr = np.frombuffer(raw, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            head = raw[:8].hex() if raw else '(empty)'
            raise ValueError(
                f'IMAGE_DECODE_FAILED: {len(raw)} bytes 수신, 유효한 이미지 포맷이 아님 '
                f'(첫 8바이트: {head}). JPEG/PNG로 인코딩 후 base64 했는지 확인 필요.'
            )
        return img

    def analyze(self, image_data: str) -> Tuple[Counter, Any]:
        """이미지를 분석해 (클래스별 개수, YOLO 결과 객체)를 반환.

        결과 객체는 rotten/disease 감지 시 레이블링 이미지를 그리는 데 쓰인다.
        """
        img = self.decode_image(image_data)
        result = self.model.predict(img, conf=self.conf, verbose=False)[0]
        counts = Counter()
        boxes = getattr(result, 'boxes', None)
        if boxes is None:
            return counts, result
        for box in boxes:
            try:
                cls_index = int(box.cls[0])
            except Exception:
                continue
            label = self.model.names.get(cls_index, str(cls_index))
            if label in CLASSES:
                counts[label] += 1
        return counts, result

    def detect_instances(self, img: Any, padding_px: float = 6.0) -> list:
        """개별 검출 결과를 픽셀 원 마스크 정보로 반환 (RP-110, Ddagi/D435용).

        analyze()는 프레임 전체의 클래스별 개수/비율(시나리오1 순찰 분석)을
        반환하지만, 이건 검출된 각 토마토를 그리퍼 URDF가 이미 실제 충돌검사를
        하므로 padding은 bbox 경계/검출 오차 정도의 작은 여유만 둔다.

        반환 원소(dict): label, confidence, center_u, center_v(픽셀 중심),
        radius_px(= bbox의 min(width,height)/2 + padding_px). 3D 좌표는 여기서
        계산하지 않음 — depth 조회는 호출측(카메라 토픽을 든 노드) 책임.
        """
        result = self.model.predict(img, conf=self.conf, verbose=False)[0]
        detections = []
        boxes = getattr(result, 'boxes', None)
        if boxes is None:
            return detections
        for box in boxes:
            try:
                cls_index = int(box.cls[0])
                confidence = float(box.conf[0])
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            except Exception:
                continue
            label = self.model.names.get(cls_index, str(cls_index))
            if label not in CLASSES:
                continue
            radius_px = min(x2 - x1, y2 - y1) / 2.0 + padding_px
            detections.append({
                'label': label,
                'confidence': confidence,
                'center_u': int(round((x1 + x2) / 2.0)),
                'center_v': int(round((y1 + y2) / 2.0)),
                'radius_px': int(round(radius_px)),
            })
        return detections

    @staticmethod
    def percentages(counts: Counter) -> Dict[str, int]:
        total = sum(counts.values())
        if total == 0:
            return {f'{label}_percent': 0 for label in CLASSES}
        return {
            f'{label}_percent': int(round(counts.get(label, 0) / total * 100))
            for label in CLASSES
        }

    @staticmethod
    def needs_labeled_image(counts: Counter) -> bool:
        """rotten 또는 disease 가 하나라도 감지됐는지."""
        return any(counts.get(label, 0) > 0 for label in LABEL_TRIGGER_CLASSES)

    @staticmethod
    def encode_labeled_image(result: Any) -> str:
        """탐지 박스를 그린 이미지를 JPEG로 인코딩해 base64 문자열로 반환.

        박스 라벨 텍스트만 한글로 바꿔서 그린다(result.names 는 이 결과 객체
        인스턴스에만 있는 속성이라, 여기서 새 dict 로 바꿔치기해도 모델의
        원본 클래스 이름(analyze() 매칭에 쓰이는)에는 영향이 없다)."""
        result.names = {idx: KOREAN_LABELS.get(name, name) for idx, name in result.names.items()}
        annotated = result.plot()  # BGR ndarray, 박스+한글 라벨이 그려진 상태
        ok, buf = cv2.imencode('.jpg', annotated)
        if not ok:
            raise ValueError('LABELED_IMAGE_ENCODE_FAILED')
        return base64.b64encode(buf.tobytes()).decode('ascii')

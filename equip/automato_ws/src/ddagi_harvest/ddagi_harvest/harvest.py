#!/usr/bin/env python3
"""수확 루프 — 검출된 토마토들을 순서대로 따서 등급별 바구니에 담는다.

한 개 파지(pick.pick)를 여러 개로 엮는다. 매 라운드: 관측자세 복귀 → 검출 →
가까운 것부터 1개 파지 → 재검출(장면 변화·가림 대응). 실패한 자리는 제외목록에
넣어 무한 재시도를 막는다. MAX_CAPACITY 차거나 딸 게 없으면 종료.

    관측 → 검출 → (가장 가까운 1개) 파지+바구니 → 관측 → 재검출 → ... → 종료

- 검출 소스는 detector(TomatoDetector)로 주입 — 지금은 MockColorDetector, 나중에
  실물 AI 서비스로 교체(인터페이스 동일).
- 등급(NORMAL/DISCARD)은 pick 이 바구니로 라우팅한다.
- 접근 방향/자세는 pick 이 결정(단일 일관 접근 + 위치별 티칭 모델). AI 기울기(RP-115)가
  오면 pick(orientation=...)로 덮어쓴다.

실행 (노트북, 팔·카메라 연결):
    python3 ddagi_harvest/harvest.py            # 팔 IP 기본 192.168.3.12
    DRY_RUN=1 python3 ddagi_harvest/harvest.py  # 파지 없이 검출·순서만 출력
"""
from __future__ import annotations

import math
import os
import sys
import time

# 직접 실행(python3 ddagi_harvest/harvest.py) 시 패키지 루트를 import 경로에 추가
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ddagi_harvest import pick as pk                    # noqa: E402
from ddagi_harvest.arm_backend import ArmBackend        # noqa: E402
from ddagi_harvest.detector import TomatoDetector       # noqa: E402

MAX_CAPACITY = 7          # 정상품(NORMAL) 바구니 용량 — NORMAL 7개면 만차 후 종료.
                          # 폐기품(DISCARD)은 별도 바구니라 이 용량에 세지 않는다.
MAX_ATTEMPTS = 30         # 총 시도 상한(무한루프 방지)
MAX_RETRY = 2             # 한 자리 파지 실패 재시도 횟수(넘으면 제외)
RETRY_Z_BUMP = 8.0        # 재시도마다 목표 z를 이만큼(mm) 위로 — 같은 실패 반복 방지(너무 아래 잡던 것 보정)
EXCLUDE_RADIUS = 25.0     # 실패/수확한 자리 반경(mm) 내 재검출은 같은 것으로 보고 건너뜀.
                          # TF 노이즈(~10mm)보단 크고, 토마토 간격(~30mm+)보단 작게.
SETTLE = 0.4              # 관측 복귀 후 카메라 안정 대기(s)


def _dist(a, b) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _near_any(base, spots, radius: float) -> bool:
    return any(_dist(base, s) < radius for s in spots)


def _grid_key(base) -> tuple:
    """10mm 격자 키 — 재검출 시 같은 토마토 실패횟수 누적용."""
    return (round(base[0] / 10), round(base[1] / 10), round(base[2] / 10))


def harvest(arm: ArmBackend, detector: TomatoDetector,
            max_capacity: int = MAX_CAPACITY, max_attempts: int = MAX_ATTEMPTS,
            exclude_radius: float = EXCLUDE_RADIUS, max_retry: int = MAX_RETRY,
            dry_run: bool = False) -> dict:
    """검출→파지 루프 1회 수확 세션. 결과 요약 dict 반환.

    **카메라 검증** 방식: 그리퍼 값은 줄기·잎을 물어도 '성공'으로 오판하므로, 성공
    집계를 위치값이 아니라 **재검출로 확인**한다 — 직전 배치에서 시도한 토마토가 다음
    검출에서 사라졌으면 진짜 수확, 아직 있으면 실패(재시도). 배치마다 어차피 관측
    복귀+재검출을 하므로 검증에 추가 이동이 없다(끝에 마지막 배치 확인용 관측 1번만).

    배치 방식: 관측자세에서 한 번 검출 → 그 배치를 연속 파지(파지 사이 관측 복귀 없음).
    실패 자리는 max_retry 회까지 재시도(재시도마다 z 위로 보정) 후 제외. 만차는 NORMAL 기준.
    """
    harvested = {"NORMAL": 0, "DISCARD": 0}
    done: list[list] = []           # 검증으로 확정된 성공 자리(제외)
    excluded: list[list] = []       # 재시도 소진해 제외한 자리
    fail_count: dict = {}           # 격자키 → 실패 횟수
    prev: list[dict] = []           # 직전 배치에서 시도한 {base, grade} (다음 검출로 검증)
    attempts = 0

    def verify(detections: list) -> None:
        """직전 배치 시도들을 현재 검출과 비교 — 사라졌으면 성공, 남아있으면 실패."""
        if not prev:
            return
        print(f"\n[검증] 관측자세 재검출({len(detections)}개)로 직전 배치 "
              f"{len(prev)}건 확인 — 사라짐+파지=수확, 남아있음=실패")
        cur = [t["base"] for t in detections]
        for a in prev:
            pos = [round(c, 1) for c in a["base"]]
            if _near_any(a["base"], cur, exclude_radius):        # 아직 보임 = 실패
                k = _grid_key(a["base"])
                fail_count[k] = fail_count.get(k, 0) + 1
                tail = ("제외" if fail_count[k] >= max_retry else "재시도 예정")
                if fail_count[k] >= max_retry:
                    excluded.append(a["base"])
                print(f"  ✗ [검증] {pos} 아직 있음 — 실패 {fail_count[k]}회, {tail}")
            elif a.get("grabbed"):                               # 사라짐+파지 = 진짜 수확
                harvested[a["grade"]] += 1
                done.append(a["base"])
                print(f"  ✓ [검증] {pos} 사라짐+파지 — {a['grade']} 수확 확정 "
                      f"(NORMAL {harvested['NORMAL']}/{max_capacity})")
            else:                                                # 사라짐+미파지 = 낙과/유실
                done.append(a["base"])
                print(f"  ⚠ [검증] {pos} 사라졌으나 미파지 — 낙과/유실로 보고 카운트 제외")
        prev.clear()

    while attempts < max_attempts:
        pk.move_observe(arm)                 # 검출은 관측자세에서만(FK 가정)
        time.sleep(SETTLE)
        detections = detector.detect()
        verify(detections)                   # 직전 배치 결과를 재검출로 확정
        if harvested["NORMAL"] >= max_capacity:
            print(f"\n만차(정상품 {max_capacity}개) — 종료. 실전이면 바구니 비움 요청.")
            break
        batch = [t for t in detections
                 if not _near_any(t["base"], done + excluded, exclude_radius)]
        if not batch:
            print("남은 토마토 없음 — 수확 종료")
            break
        # 카메라 우선순위: 카메라 depth 가까운(앞쪽) 것부터 — 앞을 치우면 뒤 가림이 풀림.
        # depth_cm 없는 검출기(테스트용)는 베이스 3D 거리로 폴백.
        batch.sort(key=lambda t: t.get("depth_cm", _dist(t["base"], (0.0, 0.0, 0.0))))
        print(f"\n[배치] 검출 {len(batch)}개 — 카메라 depth 가까운 순(파지 우선순위):")
        for i, t in enumerate(batch):
            dc = t.get("depth_cm")
            dtxt = f"depth {dc:.1f}cm" if dc is not None \
                else f"거리 {_dist(t['base'], (0.0, 0.0, 0.0)):.0f}mm"
            print(f"   {i + 1}. {t.get('color', '?')}/{t['grade']} "
                  f"base={[round(c, 1) for c in t['base']]}  {dtxt}")

        projected = harvested["NORMAL"]      # 이번 배치에서 성공 가정한 NORMAL 누계(만차 방지)
        for t in batch:
            if attempts >= max_attempts:
                break
            if t["grade"] == "NORMAL" and projected >= max_capacity:
                continue                     # NORMAL 만차 예상 — 더 안 땀(DISCARD는 계속)
            b = t["base"]
            if _near_any(b, done + excluded, exclude_radius):
                continue
            retries = fail_count.get(_grid_key(b), 0)
            target = list(b)
            if retries:                      # 재시도면 z를 조금 위로 보정
                target[2] += RETRY_Z_BUMP * retries
            attempts += 1
            print(f"  [시도 {attempts}] {t.get('color', '?')}/{t['grade']} "
                  f"base={[round(c, 1) for c in target]}"
                  + (f"  (재시도 {retries}회, z+{RETRY_Z_BUMP * retries:.0f})"
                     if retries else ""))

            grabbed = True                          # dry_run 은 파지했다고 가정
            if not dry_run:
                try:
                    # pick 은 그리퍼가 물었을 때만 True + 바구니 투하. 최종 수확 판정은
                    # 이 파지여부 + 재검출 '사라짐'을 함께 본다(줄기 건드려 떨군 것 배제).
                    grabbed = pk.pick(arm, target, t["grade"])
                except RuntimeError as e:
                    print(f"    예외(도달 불가 등): {e}")
                    grabbed = False
                if not grabbed:
                    print("    (그리퍼 미파지 — 사라져도 수확 카운트 안 함)")
            prev.append({"base": b, "grade": t["grade"], "grabbed": grabbed})
            if t["grade"] == "NORMAL":
                projected += 1

    if prev:                                 # 마지막 배치 검증 (관측 1회 더)
        pk.move_observe(arm)
        time.sleep(SETTLE)
        print("\n[마지막 배치 검증]")
        verify(detector.detect())

    total = sum(harvested.values())
    pk.move_observe(arm)
    summary = {"harvested": harvested, "total": total,
               "excluded": len(excluded), "attempts": attempts}
    print(f"\n=== 수확 요약 === {summary}")
    return summary


def main() -> int:
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from ddagi_harvest.arm_backend import NetworkArm
    from ddagi_harvest.detector import MockColorDetector, YoloDetector

    dry = os.environ.get("DRY_RUN", "") not in ("", "0", "false")
    ip = os.environ.get("ARM_IP", "192.168.3.12")
    weights = os.environ.get("WEIGHTS", "")   # 실물 YOLO .pt 경로. 없으면 색 목업.
    print(f"팔 연결 {ip}:9010  (DRY_RUN={dry})")
    arm = NetworkArm(ip)
    if weights:
        print(f"검출기: YOLO ({weights})")
        det = YoloDetector(weights)
    else:
        print("검출기: 색 목업(MockColorDetector) — 실모델 쓰려면 WEIGHTS=경로")
        det = MockColorDetector()
    try:
        if not dry:
            print("\n!! 팔이 자동으로 여러 번 움직입니다. 반경 확보!")
            if input("수확을 시작할까요? (y/N) ").strip().lower() != "y":
                print("취소")
                return 0
        harvest(arm, det, dry_run=dry)
    finally:
        det.close()
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

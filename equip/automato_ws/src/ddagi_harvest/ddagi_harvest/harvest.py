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
from ddagi_harvest.log import log, warn                 # noqa: E402

MAX_CAPACITY = 7          # 정상품(NORMAL) 바구니 용량 — NORMAL 7개면 만차 후 종료.
                          # 폐기품(DISCARD)은 별도 바구니라 이 용량에 세지 않는다.
MAX_ATTEMPTS = 30         # 총 시도 상한(무한루프 방지)
MAX_RETRY = 3             # 한 자리 파지 실패 재시도 횟수(넘으면 제외)
MAX_ROUNDS = 5            # 촬영-수확 라운드 상한(시나리오2 스펙). 정상 종료는 보통 2~3
                          # 라운드이므로 그 2배를 상한으로 둔다. 넘으면 비정상 종료로
                          # 보고 MAX_ROUNDS_EXCEEDED 로 빠져나온다.

# 파지 우선순위 기준.
#   'base_x'    : base x(팔 정면 거리)가 작은 것부터 — 팔에 가까운 것 우선(현재 기본)
#   'base_dist' : base 원점에서의 3D 거리 순 — 시나리오2 스펙 문언("가까운 순")
#   'depth'     : 카메라 depth(광축 방향 거리)가 작은 것부터 — 화면 앞쪽 우선
#   'base_z'    : 낮은 것부터 — 아래 열매를 먼저 비워 위 열매를 안 건드림
# ⚠ base_dist 는 z(250~350mm)가 x·y 보다 커서 사실상 '낮은 것 우선'에 가깝게 동작한다.
#   튜닝·실측이 전부 base_x 로 이뤄졌으므로 기본값은 base_x 를 유지한다. 스펙 문언에
#   맞추려면 이 한 줄만 'base_dist' 로 바꾸면 된다.
SORT_KEY = "base_x"
RETRY_Z_BUMP = 8.0        # 재시도마다 목표 z를 이만큼(mm) 위로 — 같은 실패 반복 방지(너무 아래 잡던 것 보정)
EXCLUDE_RADIUS = 25.0     # 실패/수확한 자리 반경(mm) 내 재검출은 같은 것으로 보고 건너뜀.
                          # TF 노이즈(~10mm)보단 크고, 토마토 간격(~30mm+)보단 작게.
SETTLE = 0.4              # 관측 복귀 후 카메라 안정 대기(s)


def _dist(a, b) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _near_any(base, spots, radius: float) -> bool:
    return any(_dist(base, s) < radius for s in spots)


def _find_fail(fails: list, base, radius: float):
    """실패 기록 중 이 자리와 같은 것으로 볼 항목을 찾는다(없으면 None).

    ⚠ 예전엔 10mm 격자 키로 셌는데, 좌표 잡음(±5mm)에 키가 바뀌면 실패 횟수가 리셋돼
    같은 열매를 무한 재시도했다(실측: y 6.7/4.5/4.3/5.3 → 키 1/0/0/1). 근접 매칭이 안전.
    """
    for rec in fails:
        if _dist(base, rec["base"]) < radius:
            return rec
    return None


def harvest(arm: ArmBackend, detector: TomatoDetector,
            max_capacity: int = MAX_CAPACITY, max_attempts: int = MAX_ATTEMPTS,
            exclude_radius: float = EXCLUDE_RADIUS, max_retry: int = MAX_RETRY,
            dry_run: bool = False, max_rounds: int = MAX_ROUNDS,
            on_progress=None, should_cancel=None,
            on_detect=None, on_target=None) -> dict:
    """검출→파지 루프 1회 수확 세션. 결과 요약 dict 반환.

    ROS2 액션 서버(harvest_node)에서도 이 함수를 그대로 쓴다. 그래서 ROS 의존을
    들이지 않고 두 개의 콜백만 받는다:
      on_progress(round, normal, discard, failed, remaining_in_round) -> None
          라운드 시작마다·파지 1건마다 호출. 액션 Feedback 발행에 쓴다.
      should_cancel() -> bool
          True 면 진행 중인 배치를 중단하고 exit_reason='CANCELED' 로 종료한다.
          파지 도중이 아니라 '파지 1건이 끝난 경계'에서만 검사한다 — 팔이 열매를
          문 채로 멈추면 사람이 빼줘야 한다.
      on_detect(batch) -> None
          제외 필터를 통과하고 정렬까지 끝난 배치. rviz 마커 발행에 쓴다.
      on_target(base_mm, grade) -> None
          지금 파지하러 가는 열매. 재시도 보정(z bump)이 반영된 실제 목표다.

    **카메라 검증** 방식: 그리퍼 값은 줄기·잎을 물어도 '성공'으로 오판하므로, 성공
    집계를 위치값이 아니라 **재검출로 확인**한다 — 직전 배치에서 시도한 토마토가 다음
    검출에서 사라졌으면 진짜 수확, 아직 있으면 실패(재시도). 배치마다 어차피 관측
    복귀+재검출을 하므로 검증에 추가 이동이 없다(끝에 마지막 배치 확인용 관측 1번만).

    배치 방식: 관측자세에서 한 번 검출 → 그 배치를 연속 파지(파지 사이 관측 복귀 없음).
    실패 자리는 max_retry 회까지 재시도(재시도마다 z 위로 보정) 후 제외. 만차는 NORMAL 기준.
    """
    # 수확 확정 = 파지 후 '상승 후 확인'을 통과한 것(=바구니에 넣은 것). 만차 기준.
    harvested = {"NORMAL": 0, "DISCARD": 0}
    excluded: list[list] = []       # 재시도 소진해 제외한 자리
    fails: list[dict] = []          # [{base, n}] 실패 자리와 횟수(근접 매칭)
    prev: list[dict] = []           # 직전 배치에서 시도한 {base, grade} (다음 검출로 검증)
    attempts = 0
    round_no = 0                    # 촬영 라운드 (1부터). 관측+검출 1회 = 1라운드
    exit_reason = "DEPLETED"        # 종료 사유. 아래 분기에서 덮어쓴다

    def progress(remaining: int = 0) -> None:
        if on_progress is None:
            return
        on_progress(round_no, harvested["NORMAL"], harvested["DISCARD"],
                    len(excluded), remaining)

    def verify(detections: list) -> None:
        """직전 배치의 **실패분**만 재검출과 대조해 재시도/제외를 정한다.

        성공은 여기서 세지 않는다 — 파지 후 '상승 후 확인'(그리퍼 재확인)을 통과하면
        그 시점에 확정한다. 상승 동작이 식물에 붙은 잎·가지를 자연스럽게 걸러내므로
        (실측: 파지값 80·92·67 이 모두 상승 후 0), 거기서 살아남았으면 떼어낸 열매다.
        거기서 바구니까지는 놓치지 않는다. 반면 재검출은 가려져 있던 뒤 열매를 보고
        성공을 실패로 오판한 사례가 있었다.
        """
        if not prev:
            return
        log(f"\n[검증] 재검출 {len(detections)}개로 실패분 {len(prev)}건 확인 "
              f"— 아직 있으면 재시도, 사라졌으면 낙과")
        cur = [t["base"] for t in detections]
        for a in prev:
            pos = [round(c, 1) for c in a["base"]]
            if _near_any(a["base"], cur, exclude_radius):        # 아직 있음 = 재시도
                rec = _find_fail(fails, a["base"], exclude_radius)
                if rec is None:
                    rec = {"base": a["base"], "n": 0}
                    fails.append(rec)
                rec["n"] += 1
                tail = ("제외" if rec["n"] >= max_retry else "재시도 예정")
                if rec["n"] >= max_retry:
                    excluded.append(a["base"])
                warn(f"  ✗ {pos} 아직 있음 — 실패 {rec['n']}회, {tail}")
            else:
                # 못 물었는데 사라짐 = 건드려 떨어뜨렸거나 검출 흔들림.
                # 기록하지 않는다 — 정말 떨어졌으면 다음 검출에 안 나오고,
                # 남아 있으면 자연히 재시도된다.
                warn(f"  ⚠ {pos} 사라졌으나 미파지 — 낙과 또는 검출 흔들림")
        prev.clear()

    full = False
    canceled = False
    while attempts < max_attempts and not full and not canceled:
        if round_no >= max_rounds:           # 라운드 상한 — 비정상 종료로 본다
            log(f"\n라운드 상한 {max_rounds} 도달 — 수확 중단(점검 필요)")
            exit_reason = "MAX_ROUNDS_EXCEEDED"
            break
        round_no += 1
        log(f"\n===== 라운드 {round_no}/{max_rounds} =====")
        progress()
        pk.move_observe(arm)                 # 검출은 관측자세에서만(FK 가정)
        time.sleep(SETTLE)
        detections = detector.detect()
        verify(detections)                   # 직전 배치 결과를 재검출로 확정
        batch = [t for t in detections
                 if not _near_any(t["base"], excluded, exclude_radius)]
        if not batch:
            if detections:      # 검출은 됐지만 전부 수확완료/제외 자리 → 구분해 알린다
                log(f"검출 {len(detections)}개가 모두 제외 자리 "
                      f"(제외 {len(excluded)}건) — 수확 종료")
            else:
                log("검출 0개 — 수확 종료")
            exit_reason = "DEPLETED"
            break
        keyfn = {"base_x": lambda t: t["base"][0],
                 "base_dist": lambda t: math.sqrt(sum(c * c for c in t["base"])),
                 "base_z": lambda t: t["base"][2],
                 "depth": lambda t: t.get("depth_cm", 1e9)}[SORT_KEY]
        label = {"base_x": "base x 가까운 순(팔 정면 거리)",
                 "base_dist": "base 원점 거리 가까운 순",
                 "base_z": "낮은 순(base z)",
                 "depth": "카메라 depth 가까운 순"}[SORT_KEY]
        batch.sort(key=keyfn)
        if on_detect is not None:            # 정렬 뒤에 넘긴다 — 마커의 번호가 파지 순서
            on_detect(batch)
        log(f"\n[배치] 검출 {len(batch)}개 — {label} (파지 우선순위):")
        for i, t in enumerate(batch):
            dc = t.get("depth_cm")
            log(f"   {i + 1}. {t.get('color', '?')}/{t['grade']} "
                  f"base={[round(c, 1) for c in t['base']]}  x={t['base'][0]:.0f}mm"
                  + (f"  depth {dc:.1f}cm" if dc is not None else ""))

        for idx, t in enumerate(batch):
            progress(len(batch) - idx)       # 이번 라운드 잔여 개수
            if attempts >= max_attempts:
                break
            if should_cancel is not None and should_cancel():
                # 파지 1건이 끝난 경계에서만 검사한다 — 파지 도중에 멈추면 팔이 열매를
                # 문 채로 서고, 사람이 빼줘야 한다.
                log("\n[중단] 취소 요청 — 이번 라운드를 여기서 종료")
                exit_reason = "CANCELED"
                canceled = True
                break
            b = t["base"]
            if _near_any(b, excluded, exclude_radius):
                continue
            _fr = _find_fail(fails, b, exclude_radius)
            retries = _fr["n"] if _fr else 0
            target = list(b)
            if retries:                      # 재시도면 z를 조금 위로 보정
                target[2] += RETRY_Z_BUMP * retries
            attempts += 1
            if on_target is not None:        # 재시도 z 보정이 반영된 실제 목표
                on_target(target, t["grade"])
            log(f"  [시도 {attempts}] {t.get('color', '?')}/{t['grade']} "
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
                    # 진입 불가(standoff 확보 실패) — 즉시 포기하되 **영구 제외는 안 한다**.
                    # 다음 배치에는 옆 열매가 빠져 도달성이 달라질 수 있어 재시도 가치가
                    # 있고, 실패 판정 자체는 재검출이 해준다(prev 에 미파지로 기록).
                    warn(f"    ✗ {e}\n    → 이번엔 건너뜀, 다음 배치에서 재시도")
                    grabbed = False
                if not grabbed:
                    log("    (그리퍼 미파지 — 사라져도 수확 카운트 안 함)")
            if grabbed:
                # 상승 후 확인을 통과 = 떼어낸 열매를 들고 있다 → 수확 확정.
                harvested[t["grade"]] += 1
                # 성공 자리를 별도 목록에 넣지 않는다 — 떼어냈으면 다음 검출에 안 나오고,
                # 그게 곧 제외다. 목록으로 막으면 (a) 25mm 안의 이웃 열매가 영구 스킵되고
                # (b) 뒤에 가려 있다가 드러난 열매를 못 따게 된다.
                log(f"    ✓ 수확 확정 — {t['grade']} "
                      f"(NORMAL {harvested['NORMAL']}/{max_capacity}, "
                      f"DISCARD {harvested['DISCARD']})")
                # 만차는 수확품(NORMAL)만 본다 — 폐기품은 소량이라 기준으로 삼지 않는다
                # (시나리오2 E4). 폐기품이 넘치는 경우는 설계상 감수한 제약이다.
                if harvested["NORMAL"] >= max_capacity:
                    log(f"\n만차: 수확품 바구니 {max_capacity}개 — 수확 종료")
                    exit_reason = "FULL"
                    full = True
                    break
            else:
                prev.append({"base": b, "grade": t["grade"]})   # 실패분만 재검출로 확인

    if attempts >= max_attempts and not full and not canceled:
        # 라운드 상한과 별개인 내부 안전장치. 스펙에 대응 값이 없어 '점검 필요'로 묶는다.
        log(f"\n시도 상한 {max_attempts} 도달 — 수확 중단(점검 필요)")
        exit_reason = "MAX_ROUNDS_EXCEEDED"

    if prev and not canceled:                # 마지막 배치 검증 (관측 1회 더)
        pk.move_observe(arm)
        time.sleep(SETTLE)
        log("\n[마지막 배치 검증]")
        verify(detector.detect())

    pk.move_observe(arm)
    summary = {
        "normal_count": harvested["NORMAL"],
        "discard_count": harvested["DISCARD"],
        "failed_count": len(excluded),   # 3회 시도 후 포기한 개수(제외 목록)
        "exit_reason": exit_reason,
        "rounds": round_no,
        "attempts": attempts,
        "total": sum(harvested.values()),
        "harvested": harvested,          # 하위호환(기존 호출부·로그)
        "excluded": len(excluded),
    }
    progress()
    log(f"\n=== 수확 요약 === {summary}")
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
    log(f"팔 연결 {ip}:9010  (DRY_RUN={dry})")
    arm = NetworkArm(ip)
    if weights:
        log(f"검출기: YOLO ({weights})")
        det = YoloDetector(weights, angles_provider=arm.get_angles)
    else:
        log("검출기: 색 목업(MockColorDetector) — 실모델 쓰려면 WEIGHTS=경로")
        det = MockColorDetector(angles_provider=arm.get_angles)
    try:
        if not dry:
            log("\n!! 팔이 자동으로 여러 번 움직입니다. 반경 확보!")
            if input("수확을 시작할까요? (y/N) ").strip().lower() != "y":
                log("취소")
                return 0
        harvest(arm, det, dry_run=dry)
    finally:
        det.close()
        arm.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

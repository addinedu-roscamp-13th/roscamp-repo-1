#!/usr/bin/env python3
"""엔드투엔드 검증 (노트북, 화면 필요) — 카메라 클릭 → TF → 실제 파지.

관측자세에서 라이브 화면의 토마토를 클릭하면, 그 픽셀을 camera 좌표로 역투영하고
tf_transform 으로 base 좌표를 구한다. 'p' 로 그 좌표에 pick()을 실행해 실제로
집히는지 본다 — 카메라→TF→pick 전 파이프라인의 실물 검증.

  python3 tf_verify.py                 # 팔 IP 기본 192.168.100.12

키/마우스:
  (마우스 좌클릭)  그 픽셀의 토마토를 목표로 → camera·base 좌표 표시
  h  그랩점으로 이동(겨냥 확인, 안 잡음)   g  게이지+조그(TF/오프셋 실측)
  t  수확 준비 자세 티칭(드래그)   w  측면별 손목 자세 티칭(grasp 1회)
  a  접근 경로 티칭(pre-grasp+grasp — 구버전, 지금은 w 를 쓴다)
  o  관측자세로 복귀   p  파지만(바구니X, 확인후)   b  전체 수확(바구니 투하까지)
  q  종료

!! p·b 는 팔이 자동 이동한다. 팔 반경 확보, 이상하면 Ctrl+C.
좌표 mm.
"""
import json
import os
import sys

import numpy as np
import pyrealsense2 as rs

APPROACHES_FILE = "taught_approaches.json"   # 'a' 접근경로 티칭 저장(위치별 접근 모델용)
PAIRS_FILE = "observe_tf_pairs.json"         # 'g'→'x' 게이지 측정 쌍(camera→flange) 누적
WRIST_FILE = "wrist_ori.json"                # 'w' 측면별 손목 자세 티칭 누적

W, H, FPS = 640, 480, 30
_click = {"uv": None}


def _on_mouse(event, x, y, flags, param):
    import cv2
    if event == cv2.EVENT_LBUTTONDOWN:
        _click["uv"] = (x if x < W else x - W, y)  # 오른쪽(depth) 클릭도 컬러 픽셀로


def main():
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from ddagi_harvest.arm_backend import NetworkArm
    from ddagi_harvest import tf_transform as tf
    from ddagi_harvest import pick as pk
    try:
        import cv2
    except Exception:
        print("opencv 필요")
        return 1
    if len(rs.context().query_devices()) == 0:
        print("RealSense 장치 없음")
        return 1

    ip = os.environ.get("ARM_IP", "192.168.100.12")
    print(f"팔 연결 → {ip}:9010")
    arm = NetworkArm(ip)
    print("관측자세로 이동...")
    arm.move_angles(pk.OBSERVE_ANGLES, 30)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    pipe.start(cfg)
    align = rs.align(rs.stream.color)
    cv2.namedWindow("tf verify")
    cv2.setMouseCallback("tf verify", _on_mouse)

    target = None       # 클릭한 목표 {uv, cam, base}
    jog = None          # 게이지 조그 중인 flange 좌표(6)
    gauge0 = None       # 게이지 시작 flange(3) — 오차 = 현재 jog - gauge0
    print(__doc__)
    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            intr = color.profile.as_video_stream_profile().get_intrinsics()

            if _click["uv"] is not None:
                u, v = _click["uv"]
                _click["uv"] = None
                # 클릭 픽셀 주변 11x11 창의 유효 depth를 모아 중앙값 사용 — 거품
                # 토마토는 IR 반사가 나빠 단일 픽셀이 배경을 읽는 일이 잦다. 분포도
                # 같이 찍어 토마토(가까움)와 배경(멀다)이 섞였는지 눈으로 본다.
                ds = []
                for dy in range(-5, 6):
                    for dx in range(-5, 6):
                        uu, vv = u + dx, v + dy
                        if 0 <= uu < W and 0 <= vv < H:
                            dd = depth.get_distance(uu, vv)
                            if dd > 0:
                                ds.append(dd)
                if not ds:
                    print(f"클릭 ({u},{v}) 주변 depth 전부 없음 — 다른 지점")
                else:
                    ds.sort()
                    d = ds[len(ds) // 2]  # 중앙값
                    cam = [c * 1000 for c in
                           rs.rs2_deproject_pixel_to_point(intr, [u, v], d)]
                    ang = arm.get_angles() or None   # 명령각이 아닌 실측각 기준
                    base = [float(v) + pk.TCP_CORRECTION[i]
                            for i, v in enumerate(tf.observe_cam_to_flange(cam))]
                    target = {"uv": (u, v), "cam": cam, "base": base}
                    print(f"\n클릭 ({u},{v}) depth중앙값={d*100:.1f}cm "
                          f"(유효 {len(ds)}/121, 범위 {ds[0]*100:.1f}~{ds[-1]*100:.1f}cm)")
                    print(f"  camera = [{cam[0]:.1f}, {cam[1]:.1f}, {cam[2]:.1f}]")
                    print(f"  base   = [{base[0]:.1f}, {base[1]:.1f}, {base[2]:.1f}]")
                    z = pk.zone_of(base[1])
                    if z is None:
                        print("  구역   = (줄기 미설정 — 진입 방향 분기 꺼짐)\n")
                    else:
                        zname = {(0, -1): "우-오 (우측줄기의 오른쪽)",
                                 (0, 1): "우-왼 (우측줄기의 왼쪽)",
                                 (1, -1): "좌-오 (좌측줄기의 오른쪽)",
                                 (1, 1): "좌-왼 (좌측줄기의 왼쪽)"}[z]
                        side = "오른쪽에서 진입" if z[1] < 0 else "왼쪽에서 진입"
                        stem = pk.STEM_REFS_Y[z[0]]
                        print(f"  구역   = {zname}  → {side}")
                        print(f"           (기준 줄기 y={stem:.1f}, 열매 y={base[1]:.1f})")
                        print("  ⚠ 눈으로 본 좌우와 다르면 STEM_REFS_Y 재측정 필요\n")

            img = np.asanyarray(color.get_data()).copy()
            dimg = cv2.applyColorMap(
                cv2.convertScaleAbs(np.asanyarray(depth.get_data()), alpha=0.03),
                cv2.COLORMAP_JET)
            if target:
                cv2.drawMarker(img, target["uv"], (0, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
                b = target["base"]
                cv2.putText(img, f"base [{b[0]:.0f},{b[1]:.0f},{b[2]:.0f}]",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                _z = pk.zone_of(b[1])
                if _z is not None:
                    _tag = {(0, -1): "R-stem RIGHT", (0, 1): "R-stem LEFT",
                            (1, -1): "L-stem RIGHT", (1, 1): "L-stem LEFT"}[_z]
                    _dir = "enter from RIGHT" if _z[1] < 0 else "enter from LEFT"
                    cv2.putText(img, f"{_tag} / {_dir}", (10, 48),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
            cv2.putText(img, "click | o:observe h:hover p:pick b:pick+basket q:quit",
                        (10, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("tf verify", np.hstack([img, dimg]))

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("h"):
                if not target:
                    print("먼저 토마토를 클릭하세요"); continue
                tgt = pk.flange_target(target["base"])   # 손끝→플랜지 보정
                appr = [float(tgt[i]) + pk.DESCEND_OFFSET[i] for i in range(3)] + pk.GRIPPER_ORI
                if not pk.in_workspace(appr):
                    print(f"그랩점 {[round(x,1) for x in appr[:3]]} 범위 밖 — 목표/TF 확인")
                    continue
                print(f"그랩점으로 이동(그리퍼 열고, 안 잡음) 명령flange={[round(x,1) for x in appr[:3]]} ...")
                arm.open_gripper()
                try:
                    arm.move_coords(appr, 25, 0)
                    rc = arm.get_coords() or []
                    if rc:
                        tip = [rc[i] + pk.TCP_CORRECTION[i] for i in range(3)]
                        b = target["base"]
                        derr = [round(tip[i] - b[i], 1) for i in range(3)]
                        print(f"  도착flange={[round(c,1) for c in rc[:3]]}"
                              f"  손끝추정={[round(t,1) for t in tip]}")
                        print(f"  클릭base ={[round(c,1) for c in b]}  손끝-base오차={derr}")
                    print("  그리퍼 끝이 토마토에 오나 확인 (맞으면 'p'로 파지)")
                except RuntimeError as e:
                    print(f"  ✗ {e}\n  → 더 가운데/낮은 토마토로.")
            elif k == ord("g"):
                # 게이지 시작: 손끝을 '계산 base'에 정확히 두고(DESCEND 없이),
                # 실제 토마토까지 조그한 이동량으로 TF 오차를 실측한다.
                if not target:
                    print("먼저 토마토를 클릭하세요"); continue
                gf = pk.flange_target(target["base"])   # 손끝을 계산 base에
                appr = [float(gf[i]) for i in range(3)] + pk.GRIPPER_ORI
                if not pk.in_workspace(appr):
                    print(f"게이지점 {[round(x,1) for x in appr[:3]]} 범위 밖")
                    continue
                arm.open_gripper()
                try:
                    arm.move_coords(appr, 25, 0)
                    jog = list(appr)
                    gauge0 = list(gf)
                    print("게이지 모드: 손끝이 '계산 base' 위치에 옴.")
                    print("  실제 토마토에 손끝이 닿을 때까지 조그(각 5mm):")
                    print("    i/k = 앞/뒤(x)   j/l = 좌/우(y)   u/n = 위/아래(z)")
                    print("    x = 오차 기록/출력    o = 관측복귀(게이지 취소)")
                except RuntimeError as e:
                    print(f"  ✗ {e}"); jog = None
            elif jog is not None and k in (ord("i"), ord("k"), ord("j"),
                                           ord("l"), ord("u"), ord("n")):
                step = 5.0
                axis, dv = {ord("i"): (0, step), ord("k"): (0, -step),
                            ord("j"): (1, step), ord("l"): (1, -step),
                            ord("u"): (2, step), ord("n"): (2, -step)}[k]
                jog[axis] += dv
                try:
                    arm.move_coords(jog, 20, 0)
                    print(f"  조그 flange={[round(c,1) for c in jog[:3]]}")
                except RuntimeError as e:
                    print(f"  ✗ {e}")
            elif k == ord("x"):
                if jog is None or gauge0 is None:
                    print("먼저 'g'로 게이지를 시작하세요"); continue
                err = [round(jog[i] - gauge0[i], 1) for i in range(3)]
                corrected = [round(target["base"][i] + err[i], 1) for i in range(3)]
                print("\n=== TF 오차 실측 ===")
                print(f"  camera 좌표   = {[round(c,1) for c in target['cam']]}")
                print(f"  계산 base     = {[round(c,1) for c in target['base']]}")
                print(f"  TF 오차(실제-계산) = {err}   ← 이 값 보내줘")
                print(f"  보정된 실제 base   = {corrected}\n")
                # 관측자세 변환 피팅용 쌍 저장: camera 좌표 → '손끝이 닿는 flange'
                # (이 flange 가 곧 파지 때 명령할 좌표. TCP·DESCEND·처짐이 모두 포함됨)
                pair = {"camera": [round(c, 1) for c in target["cam"]],
                        "flange": [round(jog[i], 1) for i in range(3)],
                        "ori": [round(jog[i], 1) for i in range(3, 6)]}
                try:
                    with open(PAIRS_FILE) as f:
                        pairs = json.load(f)
                except Exception:
                    pairs = []
                pairs.append(pair)
                with open(PAIRS_FILE, "w") as f:
                    json.dump(pairs, f, indent=2, ensure_ascii=False)
                print(f"  [저장 #{len(pairs)}] camera={pair['camera']} → flange={pair['flange']}")
                print(f"    → {PAIRS_FILE} (5~6점 모이면 fit_observe_tf.py 로 변환 피팅)")
                jog = gauge0 = None   # 게이지 종료 — 다음 조작이 옛 좌표를 움직이지 않게
            elif k == ord("t"):
                # 수확 준비 자세(STAGING_ANGLES) 티칭 — 드래그로 잡아 각도 캡처.
                print("\n수확 준비 자세 티칭 — 서보 풉니다. 팔을 '베드 위·접근 준비'")
                print("  자세로 잡고(반드시 손으로 받치기!) 이 창에서 아무 키.")
                arm.release_servos()
                cv2.waitKey(0)
                ang = arm.get_angles()
                arm.focus_servos()
                if ang:
                    print(f"  STAGING_ANGLES = {[round(a,1) for a in ang]}"
                          f"   ← 이 값 보내줘 (pick.py에 박음)\n")
                else:
                    print("  각도 못 읽음 — 다시\n")
            elif k == ord("w"):
                # 손목 자세 티칭 — grasp 자세 하나만 잡는다(pre-grasp 는 계산됨).
                # 여기서 얻는 것: ① 그 측면의 손목 자세(rx,ry,rz) ② 자세별 flange 보정
                #   (= 티칭 flange − 피팅 flange. 두 고정 자세 사이 차이라 상수)
                if not target:
                    print("먼저 토마토를 클릭하세요"); continue
                z = pk.zone_of(target["base"][1])
                zt = ({(0, -1): "우-오", (0, 1): "우-왼",
                       (1, -1): "좌-오", (1, 1): "좌-왼"}[z] if z else "?")
                print(f"\n[손목 자세 티칭] 구역 {zt} "
                      f"({'오른쪽에서 진입' if z and z[1] < 0 else '왼쪽에서 진입'})")
                print("  서보 풉니다 — 팔 받치기! 이 열매를 '그 방향에서 감싸 무는'")
                print("  자세(손목 각도 + 손끝 위치)로 잡고 창에서 아무 키.")
                arm.release_servos()
                cv2.waitKey(0)
                g = arm.get_coords()
                arm.focus_servos()
                if not g:
                    print("  좌표 못 읽음 — 다시\n"); continue
                fit_fl = pk.flange_target(target["base"])   # 피팅이 준 flange(기준 자세)
                delta = [round(g[i] - fit_fl[i], 1) for i in range(3)]
                rec = {"zone": list(z) if z else None, "zone_name": zt,
                       "base": [round(c, 1) for c in target["base"]],
                       "grasp": [round(c, 1) for c in g],
                       "fit_flange": [round(c, 1) for c in fit_fl],
                       "delta": delta}
                try:
                    with open(WRIST_FILE) as f:
                        recs = json.load(f)
                except Exception:
                    recs = []
                recs.append(rec)
                with open(WRIST_FILE, "w") as f:
                    json.dump(recs, f, indent=2, ensure_ascii=False)
                print(f"  [저장 #{len(recs)}] 자세(rx,ry,rz)={[round(c,1) for c in g[3:6]]}")
                print(f"    flange 보정 = {delta}  (기준자세 대비)")
                print(f"    → {WRIST_FILE}\n")
            elif k == ord("a"):
                # 접근 경로 티칭 — 클릭한 토마토에 대해 pre-grasp·grasp 자세를 드래그로
                # 잡아 {base, pregrasp, grasp} 저장. 여러 위치 모아 위치별 접근모델 산출.
                if not target:
                    print("먼저 토마토를 클릭하세요"); continue
                b = target["base"]
                print("\n[접근 경로 티칭] 서보 풉니다 — 손으로 받치기!")
                print("  ① 원하는 pre-grasp(접근 시작) 자세로 잡고 창에서 아무 키")
                arm.release_servos()
                cv2.waitKey(0)
                pre = arm.get_coords()
                print(f"    pre-grasp = {[round(c,1) for c in pre] if pre else '실패'}")
                print("  ② grasp(실제 파지) 자세로 잡고 아무 키")
                cv2.waitKey(0)
                grasp = arm.get_coords()
                arm.focus_servos()
                if not (pre and grasp):
                    print("  좌표 못 읽음 — 다시\n"); continue
                rec = {"base": [round(float(x), 1) for x in b],
                       "pregrasp": [round(c, 1) for c in pre],
                       "grasp": [round(c, 1) for c in grasp]}
                try:
                    with open(APPROACHES_FILE) as f:
                        data = json.load(f)
                except Exception:
                    data = []
                data.append(rec)
                with open(APPROACHES_FILE, "w") as f:
                    json.dump(data, f, indent=2, ensure_ascii=False)
                print(f"  저장됨 (#{len(data)}) → {APPROACHES_FILE}")
                print(f"    base={rec['base']}")
                print(f"    grasp-base오프셋={[round(grasp[i]-b[i],1) for i in range(3)]}"
                      f"  grasp자세(rx,ry,rz)={[round(c,1) for c in grasp[3:6]]}")
                print(f"    접근벡터(grasp-pre)={[round(grasp[i]-pre[i],1) for i in range(3)]}\n")
            elif k == ord("o"):
                print("관측자세 복귀")
                arm.move_angles(pk.OBSERVE_ANGLES, 30)
            elif k == ord("p"):
                if not target:
                    print("먼저 토마토를 클릭하세요")
                    continue
                b = target["base"]
                print(f"\n[파지 검증] base={[round(x,1) for x in b]} — 파지→후퇴까지만"
                      f"(바구니 X). 팔 반경 확인!")
                print("  진행하려면 창에서 'y', 취소는 다른 키:")
                if (cv2.waitKey(0) & 0xFF) != ord("y"):
                    print("  취소됨")
                    continue
                try:
                    ok = pk.pick(arm, [float(b[0]), float(b[1]), float(b[2])], "NORMAL",
                                 to_basket=False)
                    print(f"  결과: {'파지 성공' if ok else '실패'}  (그리퍼값 {arm.gripper_value()})")
                except RuntimeError as e:
                    print(f"  ✗ {e}\n  → 더 가운데/낮은 토마토로.")
                print("  관측자세로 복귀")
                arm.move_angles(pk.OBSERVE_ANGLES, 30)
                target = None
            elif k == ord("b"):
                if not target:
                    print("먼저 토마토를 클릭하세요")
                    continue
                b = target["base"]
                print(f"\n[전체 수확] base={[round(x,1) for x in b]} — 파지→바구니(NORMAL)"
                      f" 투하까지. 팔 반경 확인!")
                print("  진행하려면 창에서 'y', 취소는 다른 키:")
                if (cv2.waitKey(0) & 0xFF) != ord("y"):
                    print("  취소됨")
                    continue
                try:
                    ok = pk.pick(arm, [float(b[0]), float(b[1]), float(b[2])], "NORMAL",
                                 to_basket=True)
                    print(f"  결과: {'수확 성공(바구니 투하)' if ok else '실패'}")
                except RuntimeError as e:
                    print(f"  ✗ {e}\n  → 더 가운데/낮은 토마토로.")
                print("  관측자세로 복귀")
                arm.move_angles(pk.OBSERVE_ANGLES, 30)
                target = None
    finally:
        arm.close()
        pipe.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

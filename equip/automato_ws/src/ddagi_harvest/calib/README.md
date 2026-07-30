# JetCobot(myCobot 280 Pi) 캘리브레이션 3종 — 전달 패키지

토마토 수확 로봇의 "카메라가 본 토마토 → 로봇이 손끝으로 집을 좌표" 변환에 필요한
**독립적인 캘리브레이션 3종**의 결과·스크립트·증거를 폴더별로 정리한 것.

## 좌표 변환 사슬 (셋이 순서대로 곱해져 완성)

```
 [① 카메라 내부]        [② 핸드아이(TF)]        [③ TCP]
 픽셀 → 카메라3D   ──▶   카메라 → 로봇(joint6)  ──▶   플랜지 → 그리퍼 손끝
   (렌즈·왜곡·깊이)         (카메라 장착위치)          (그리퍼 손끝 오프셋)
```

- ① 은 카메라 자체 성질(로봇 무관), ② 는 카메라↔로봇 장착관계, ③ 은 그리퍼↔로봇 손끝관계.
- 각자 독립 파일 → 하나 다시 캘리브해도 나머지는 안 건드림.

## 폴더별 핵심 결과

### 1_카메라캘리브/  (① 카메라 내부·스테레오)
- `my_stereo_calib.json` — 우리가 측정한 스테레오 캘리브 (baseline 50.1mm)
- `intel_factory_calib.json` — 인텔 공장 기본값(비교용)
- `calibrate_camera_only.py`, `cal_cam.py` — 실행 스크립트
- `checkerboard_*.pdf/png`, charuco 관련 — 캘리브 타깃

### 2_TF핸드아이/  (② 손눈, eye-in-hand)  ★가장 핵심
- **`jetcobot_handeye.calib`** — 결과(X 변환). joint6→camera:
  - translation [x,y,z] = [-40.3, 28.9, 79.4] mm
  - rotation quat [x,y,z,w] = [-0.7485, -0.0261, -0.0136, 0.6625]
  - calibration_type: eye_in_hand / robot_effector_frame: joint6 / tracking: camera_color_optical_frame
- `검증결과_3.4mm.txt` — 재투영 정확도 3.4mm
- `00_핸드아이_가이드.md` — 캘리브 절차
- `calibrate_handeye_full.py` 등 — easy_handeye2 기반 실행 스크립트
- `charuco_tf_publisher.py` — 마커 TF 발행 / `verify_handeye.py` — 검증
- `handeye_ws/urdf/mycobot_280_pi.urdf` — 로봇 URDF(회사 순정)
- `handeye_ws/src/easy_handeye2` — 소스 (colcon build 필요, build/install/log는 제외됨)

### 3_TCP캘리브/  (③ 손끝 오프셋)
- `2026-07-20_③TCP캘리브/TCP_최종결과.txt` — 결과:
  - **TCP_flange = [-3.2, -10.3, 109.2] mm** (플랜지 기준, J6 무관)
  - 두 위치 교차검증 z편차 0.6mm
- `calibrate_grasp_offset.py`, `auto_tcp.py`, `live_tcp_view.py` — 스크립트

## 참고
- URDF는 카메라용으로 수정하지 않음. 카메라 위치는 ②의 `.calib`(별도 파일)로 표현.
- `handeye_ws/build·install·log`(ROS 빌드 산출물)는 기계 종속이라 제외 → 받는 쪽에서 `colcon build`.
- 각 폴더의 날짜 하위폴더(2026-07-1x_…)는 증거 이미지·녹화(과정 기록).

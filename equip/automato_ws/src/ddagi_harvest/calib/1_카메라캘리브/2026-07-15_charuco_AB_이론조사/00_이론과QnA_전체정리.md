# 차루코 1장으로 A(카메라 내부)+B(핸드아이) 동시 캘리브 — 이론·Q&A·조사 (2026-07-15)

> 이건수 ↔ Claude 대화 정리. 방울토마토 스마트팜(jetcobot_aac0 = MyCobot280 + RealSense D435, eye-in-hand).

## 0. 배경 질문
강사님이 캘리브 3가지를 말씀함: ①카메라 내부(체크보드) ②TCP(손끝 오프셋) ③카메라↔base(차루코).
팀원은 "③은 로봇 회사에서 만들 때 적용돼 나왔다"고 함. → 진짜 3개 다 해야 하나? ③은 공장 게 맞나?

## 1. 용어 정리 — 흰검판은 "같은 종이, 다른 측정"
- 흰검 체커판/차루코는 그냥 **기준자(ruler)**. 같은 판으로 서로 다른 두 캘리브를 함.
- **A. 카메라 내부(intrinsic)**: 카메라 렌즈 혼자 검사. fx,fy,cx,cy,왜곡. 로봇 불필요.
- **B. 카메라↔base(핸드아이)**: 카메라와 로봇의 관계(변환). 로봇을 여러 자세로 움직여야 함.
- **차루코 = 체커판(흰검) + 흰칸마다 아르코마커(고유 ID)**. 판이 일부 잘려도 인식 → 화면 구석까지
  밀 수 있어 내부캘리브 왜곡값에 유리. (일반 체커판은 조금만 잘려도 프레임 통째 버림)

## 2. 핵심 결론 — A와 B를 차루코 1장으로 동시에 가능
- 카메라가 그리퍼에 달림(eye-in-hand) → 판 1장 고정 + 로봇 자세순회 → 판이 저절로 다양한 각도로 찍힘.
- 그 한 세트로 A(calibrateCamera)와 B(calibrateHandEye) 둘 다 뽑힘.
- 단 조건: ①계산은 A 먼저(K 없이는 판 3D자세 못 구함→B 불가) ②자세를 화면 구석·기울기·거리로
  다양하게(A·B 공통 이득) ③차루코라야 구석까지 밀어도 인식.

## 3. 원리 A — 핀홀 방정식 역풀이(Zhang's method)
- 투영: u=fx·X/Z+cx, v=fy·Y/Z+cy (+왜곡 k1,k2,k3,p1,p2)
- 아는 것: 판 코너 실제 mm(objP) + 찍힌 픽셀(imgP). 모르는 것: fx,fy,cx,cy,왜곡 + 자세별 R,t.
- 재투영오차(추정 K로 3D→픽셀 투영 vs 실제 픽셀) 최소가 되게 반복 최적화(LM).
- 다양성 필수 이유: 자세당 방정식(코너24×2=48) ≫ 미지수9 → 과결정이라야 안정. 거리 다양(초점거리 vs
  거리 분리), 화면 구석(왜곡은 가장자리에서 결정), 기울기 다양.

## 4. 원리 B — AX = XB
- 카메라↔그리퍼 X는 상수(볼트 고정). 각 자세 i:
  - T_base←gripper(i): 로봇이 관절각으로 아는 손끝(순기구학)
  - T_cam←board(i): 카메라가 본 판(solvePnP, **K 필요→A 먼저**)
  - 판 고정 → T_base←board 상수
- 두 자세 걸면 정리돼 **A·X = X·B**. A=로봇이 움직인 양, B=카메라가 본 판 움직임.
  손목을 여러 축 회전시켜야 X 회전이 풀림. OpenCV calibrateHandEye(TSAI/PARK/HORAUD/DANIILIDIS).
- 자가검증: 판 고정이니 X 구한 뒤 모든 자세의 base←board가 한 점에 모여야 함. 퍼짐(mm)=품질.
  (2026-07-03 체커판 결과 = 퍼짐 2.6mm)

## 5. 조사 결론 — jetcobot은 B가 공장 적용돼서 나오나? → 아니오(D435 셋업 기준)
- JetCobot = myCobot 280(+Jetson/RPi). 공식은 끝에 **기본 2D 카메라** eye-in-hand.
- Elephant 공식문서: "카메라 (재)장착 시 hand-eye 캘리브 필요", "앞단 카메라 교체 시 6축 캘리브" →
  **사용자가 직접** EyesInHand_matrix 산출. 즉 공식 제품도 핸드아이는 사용자 작업.
  - https://docs.elephantrobotics.com/docs/mycobot-m5-en/12-ApplicationBaseROS/12.1-ROS1/12.1.4-rivzIntroductionAndUse/myCobot-280.html
  - https://docs.elephantrobotics.com/docs/gitbook-en/13-AdvancedKit/13.1%E4%BA%BA%E5%B7%A5%E6%99%BA%E8%83%BD/13.1.5-aruco%E7%A0%81%E8%AF%86%E5%88%AB.html
- 결정타: 너희는 기본 2D 카메라를 떼고 **D435 자가 장착** → 공장 기본 매트릭스가 있어도 2D 전용이라 무효.
- 로컬 증거: 너희 handeye.json = "jetcobot_aac0 전용, 다른 로봇엔 재캘리브 필요" = 개체별 값 = 공장 공통 상수 아님.
- 팀원이 들은 "공장에서 됨"의 정체 = **관절 영점·순기구학(base→플랜지)**. 이건 진짜 공장 캘리브(개체별).
  하지만 **카메라↔base(B)와는 다른 것**. → 강사님 ③번은 "우리가 직접 해야 하고 이미 한" 것.

## 6. 산출물
- 스크립트: `deploy/calibrate_cam_handeye_charuco.py` (차루코 A+B 통합, ROS2 자동순회, 증거저장)
  - OpenCV 4.7+ 신 API / 4.6 이하 구 API 자동 폴백. 4.6에서 전 경로 오프라인 검증 통과(RMS 0.32px).
  - 실행: `--make-board board.png` → 인쇄·실측 → `--square 30 --marker 22` 로 캘리브.
  - 출력: cam_calib.npz(A: K,dist) + handeye_charuco.json(B: R,t,규약,K).
- 기존 대비: 판을 체커판→차루코로 교체, calibrateCamera(A) 단계 추가. 나머지(AX=XB 규약 자가검증)는 동일.

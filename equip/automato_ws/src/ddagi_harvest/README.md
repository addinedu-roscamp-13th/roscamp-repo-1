# ddagi_harvest (김동현 경쟁 구현본)

시나리오 2 수확의 로봇팔 부분을 **직접 `send_coords`(제조사 IK 솔버)** 로 구현한다.
MoveIt2 없이 고정방향 직선접근으로 파지. 로봇팔 3인 경쟁 구현 중 하나.

> 최종 채택 시 표준 이름으로 rename → 나머지 삭제. 그래서 **독립 패키지**로 격리.

## 구조 (진행 중)

```
ddagi_harvest/
├── ddagi_harvest/
│   └── tf_transform.py     ✅ camera(optical) → base 변환 (기초 모듈)
├── calib/                  건수님 캘리 3종 결과·스크립트 (원본, 사진 제외)
├── package.xml / setup.py
```

## tf_transform — 좌표 변환 기초

AI(DetectTomatoes)가 준 camera 좌표 → 팔이 움직일 base 좌표. **파지·루프가 전부 이 위에 선다.**

변환 사슬 (건수님 캘리, 검증 3.4mm):
```
p_camera(optical) → [② 핸드아이 T(j6←cam), 고정] → p_j6
                  → [get_coords T(base←j6), 관측자세] → p_base
③ TCP(손끝 오프셋)는 send_coords 목표 계산 때.
```

- `camera_to_base(p_camera_mm, arm_coords)` — 핵심 함수
- 자기검증: `python3 ddagi_harvest/tf_transform.py` (왕복 0mm·회전 정규직교 확인)

### ⚠️ 확인 필요 (하드웨어/팀)
1. **프레임**: 핸드아이는 `camera_color_optical_frame`(z=앞) 기준. AI 출력도 optical 이어야 함 → 손민호 님 확인. (camera_link면 90° 틀어짐)
2. **오일러 규약**: `get_coords` rx,ry,rz 규약이 애매 → 기본 ZYX. 실물에서 검증.
3. **end-to-end 수치**: 관측자세에서 실제 get_coords + 알려진 camera점으로 base 결과가 자로 잰 위치와 맞는지 (±수mm) 실측 확정.

## 다음
- [ ] `arm_backend.py` — Arm 추상화 (FakeArm 로깅 / RealArm pymycobot)
- [ ] `pick.py` — send_coords 파지 시퀀스 (pre-grasp→직선접근→그립→후퇴→바구니)
- [ ] `harvest.py` — 루프 (가까운순·제외목록·라운드·grade→바구니)
- [ ] DetectTomatoes 클라이언트 + Mock AI
- [ ] Harvest 액션 서버 래핑

# 로봇팔 제어 방식: 자체 솔버 vs MoveIt2, 선택적 전환 구조

## 1. 목표 / 배경

토마토 수확 로봇팔이 **바깥쪽 노출된 열매**와 **잎에 가려진 안쪽 열매** 두 가지 상황을 모두 처리해야 한다. 단순 직선 접근(자체 솔버)만으로는 안쪽 열매에서 잎/줄기와 충돌할 가능성이 있고, 반대로 모든 동작을 MoveIt2로만 처리하면 불필요하게 느려진다. 두 방식을 상황에 맞게 골라 쓰는 구조를 만드는 것이 목표.

---

## 2. 핵심 원리

### 2-1. 결국 둘 다 같은 제조사 API를 거친다

로봇을 움직이는 마지막 관문은 항상 `pymycobot`이다. 차이는 **"IK(역기구학)를 누가, 언제 푸느냐"** 뿐이다.

| | `send_coords([x,y,z,rx,ry,rz])` | `send_angles([j1..j6])` |
|---|---|---|
| 입력 | 공간 좌표 | 관절 각도(기계좌표) |
| IK 계산 | **펌웨어 자체 솔버**가 그때그때 계산 | 이미 계산되어 있음 (전달만) |
| 실패 시 | 해가 없으면 그냥 안 움직임, 이유를 알기 어려움 | 해당 없음 (각도라 항상 실행 가능) |

MoveIt2 경로는:
```
MoveIt2 (경로계획 + IK, OMPL 등) → joint trajectory 생성
    → 브릿지 노드가 그 각도를 pymycobot.send_angles()로 전달
```
→ **MoveIt2를 쓸 때 IK를 두 번 푸는 게 아니다.** MoveIt2가 미리 다 풀어서 각도만 넘기기 때문에 이중 계산/부하 충돌은 원칙적으로 발생하지 않는다.

### 2-2. 언제 뭘 쓰나

- **자체 솔버(`send_coords`)**: 장애물 없는 단순 직선 접근. 빠르고 구현 간단. 단, 해가 없거나 특이점 근처면 실패해도 원인 파악이 어려움.
- **MoveIt2**: 잎/줄기 사이로 우회 경로가 필요하거나, 자체 솔버가 실패를 반복하는 자세. 충돌 회피 + 여러 IK 후보 탐색 가능. 대신 계획 시간이 더 걸림.

### 2-3. 동시 사용은 불가, 선택적 전환은 가능

같은 순간에 두 방식이 동시에 팔을 붙잡으면 명령이 충돌한다. 하지만 **매 동작마다 어느 쪽을 쓸지 코드에서 분기**하는 것은 문제없다. 핵심 조건: MoveIt2로 실행한 뒤에는 컨트롤러를 놓아주거나(release), 애초에 필요할 때만 MoveIt2 노드를 켜는 구조로 가야 한다.

---

## 3. 단계별 절차

### Part A. 자체 솔버 방식 (기존 검증 완료 — 「4. 로봇팔로 집기」)

```python
ori = [-133, 7, -100]                       # 검증된 그리퍼 자세
mc.set_gripper_value(100, 50)               # 1) 그리퍼 열기
mc.send_coords([X, Y, Z+60]+ori, 25, 1)     # 2) 목표 위쪽 접근
mc.send_coords([X, Y, Z-7]+ori, 25, 1)      # 3) 내려가기 (7mm 보정)
mc.set_gripper_value(0, 50)                 # 4) 그리퍼 닫기
mc.send_coords([X, Y, Z+93]+ori, 25, 1)     # 5) 들어올림
```
잡힘 판정: 그리퍼 값이 0(빈 그리퍼) vs 18~22(토마토 걸림)으로 성공 여부 확인.

### Part B. MoveIt2 설치 및 실행 (제조사 공식 `mycobot_ros2` 패키지 기준)

1. `mycobot_ros2` 워크스페이스 클론 및 빌드
2. 계획용 launch 실행:
   ```
   ros2 launch mycobot_280arduino_moveit2 demo.launch.py
   ```
   RViz에서 목표 자세 지정 → Plan (여기까지는 시뮬레이션, 실물 안 움직임)
3. 실물 동기 실행:
   ```
   ros2 run mycobot_280_moveit2_control sync_plan_arduino
   ```
   → 이 노드 내부에서 `pymycobot.send_angles()`를 호출해 계산된 관절각을 실제 로봇에 전달

> RPi5에서 `demo.launch.py`까지는 이미 동작 확인된 상태.

### Part C. 선택적 전환 wrapper 구조 (설계안)

```python
def pick_tomato(coord, use_moveit=False):
    if use_moveit:
        # MoveIt2 액션 클라이언트로 목표 pose 전달 → planning
        joint_traj = call_moveit_plan(coord)
        if joint_traj is None:
            return False  # 계획 실패
        for angles in joint_traj:
            mc.send_angles(angles, speed)
        # 실행 후 MoveIt2 컨트롤러 release (다음 자체 솔버 호출과 충돌 방지)
        release_moveit_controller()
    else:
        mc.send_coords(coord + ori, 25, 1)
    return True
```

**분기 기준 예시:**
- 카메라 depth 상 목표 좌표 주변 장애물(잎) 밀도가 낮음 → `use_moveit=False`
- 밀도 높음, 또는 `send_coords` 1차 시도 실패(응답 타임아웃/미도달) → `use_moveit=True`로 재시도

**주의:** MoveIt2 노드는 상시 실행하지 말고, 필요한 순간에만 launch하거나 이미 떠 있는 노드에 액션 요청만 보내는 방식으로 설계 — 자체 솔버 호출과 겹치는 시간대를 만들지 않는 게 핵심.

---

## 4. 트러블슈팅

| 증상 | 원인 후보 | 대응 |
|---|---|---|
| `send_coords` 호출해도 안 움직임 | 목표 좌표가 작업반경(약 280mm) 밖이거나 특이점 근처 → IK 해 없음 | 좌표 범위 사전 체크, 실패 시 MoveIt2로 재시도 |
| MoveIt2 실물 실행 시 팔이 떨림 | (일반적으로 알려진 원인은 아님) 대부분 **MoveIt Servo(실시간 연속 스트리밍)** 모드에서 보고되는 문제 — publish rate/필터링 이슈로 추정. `sync_plan` 방식(계획 후 1회 실행)은 이 문제와 무관 | 실시간 서보잉 대신 plan-and-execute 방식 사용 |
| MoveIt2와 자체 솔버 전환 후 로봇이 명령 씹음 | 이전 컨트롤러가 팔을 계속 붙잡고 있음 | 전환 전 컨트롤러 release 확인 |

---

## 5. 결과 / 검증 (체크리스트)

- [ ] 자체 솔버로 바깥쪽 열매 집기 성공률 확인 (기존 완료분)
- [ ] MoveIt2 `demo.launch.py` + `sync_plan_arduino` 로 안쪽 열매 1개 이상 실물 집기 성공
- [ ] wrapper 함수로 두 방식 순차 전환 시 충돌/명령 씹힘 없는지 확인
- [ ] 분기 기준(장애물 밀도 등) 설정 후 자동 선택 vs 수동 지정 결과 비교

---

*참고: Part B의 실물 동기 실행 명령은 제조사 공식 문서 기준이며, 실제 사용 중인 `mycobot_280_moveit2_control` 패키지명/명령어는 설치된 버전에 맞춰 확인 필요.*
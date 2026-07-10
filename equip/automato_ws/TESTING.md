# automato_ws 테스트 방법

`equip/automato_ws` 액션/노드 테스트(pytest) 방법을 기록한다.
설치 항목은 [SETUP.md](SETUP.md) 참고. 테스트 접근이 바뀌면 이 문서를 갱신한다.

---

## ✅ 최종 버전 (현재 권장 방법)

### 1. 환경 준비
```bash
source /home/ane/dev_ws/.venv/bin/activate
source /opt/ros/jazzy/setup.bash
cd /home/ane/dev_ws/roscamp-repo-1/equip/automato_ws
colcon build                 # 인터페이스/노드 변경 시
source install/setup.bash
```

### 2. 테스트 실행
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/<pkg>/test/ -v
```
예) `pytest src/ddago_control/test/test_move_action.py -v`

### 3. 필수 조건 (이유는 아래 버전 이력 참고)
- **`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`** 를 붙인다.
- 액션 테스트 코드는 **`SingleThreadedExecutor`** 로 서버+클라이언트를 한 스레드에서 스핀한다.
- future 는 `future.done()` 폴링으로 대기(백그라운드 스핀 스레드가 완료시킴).
- 각 goal 후 `ActionClient.destroy()` 로 정리한다.

### 4. 액션 테스트 코드 골격 (최종형)
```python
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient

# fixture: rclpy.init() → 서버노드+클라이언트노드를 SingleThreadedExecutor 에 add
#          → executor.spin() 을 daemon 스레드로 실행
# 전송: client.send_goal_async(goal, feedback_callback=...) 후 _wait(future) 폴링
#       → goal_handle.get_result_async() 폴링 → client.destroy()
```
전체 예시: [src/ddago_control/test/test_move_action.py](src/ddago_control/test/test_move_action.py)

---

## 🕓 버전 이력 (버전별로 달라지는 것)

각 버전에서 **무엇이 문제였고 무엇을 바꿨는지** 기록. 위 "최종 버전"이 v3 이다.

### v1 — 최초 시도 (실패)
- 방식: `MultiThreadedExecutor` 로 서버+클라이언트 스핀, `pytest src/.../test_move_action.py`
- 문제 A: `ModuleNotFoundError: No module named 'yaml'` — venv 의 rclpy 가 yaml 필요
- 문제 B: pytest 가 `launch_testing` 플러그인 자동로드 중 yaml import 실패 (수집 단계 중단)
- 문제 C: `RuntimeError: Two goals were accepted with the same ID` — MultiThreadedExecutor +
  rclpy 액션 클라이언트 동시성 버그로 1개 테스트 타임아웃

### v2 — yaml/플러그인 해결 (부분 성공: 2 pass / 1 fail)
- 바꾼 것:
  - `pip install pyyaml` (문제 A 해결) → [SETUP.md](SETUP.md) 기록
  - `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 추가 (문제 B 해결)
- 남은 문제: 문제 C (Two goals same ID) 그대로 → `test_result_success` 실패

### v3 — Executor 교체 (최종, 3 pass)
- 바꾼 것:
  - `MultiThreadedExecutor` → **`SingleThreadedExecutor`** (문제 C 해결)
  - 각 goal 후 `ActionClient.destroy()` 추가
- 결과: `3 passed` — 최종 방법으로 확정

### 요약: 버전별 핵심 차이

| 항목 | v1 | v2 | v3 (최종) |
|---|---|---|---|
| Executor | MultiThreaded | MultiThreaded | **SingleThreaded** |
| pyyaml | 없음 | 설치 | 설치 |
| PLUGIN_AUTOLOAD | 기본(on) | **disable** | **disable** |
| ActionClient.destroy | 없음 | 없음 | **있음** |
| 결과 | 수집 실패 | 2/3 | **3/3 ✅** |

---

## 변경 이력

| 날짜 | 내용 |
|---|---|
| 2026-07-01 | 최초 작성. ddago_control move 액션 테스트 기준 v1→v3 정리, 최종 방법 확정 |

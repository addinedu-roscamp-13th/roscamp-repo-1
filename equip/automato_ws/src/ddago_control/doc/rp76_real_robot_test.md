# RP-76 Patrol 서버 — 실물 검증 절차 (2단계)

> 1단계(로봇 없이 로직 검증)는 완료(빌드·단위테스트·린트 통과).
> 이 문서는 **실물 로봇 + 카메라 + Nav2 를 붙여 DoD를 최종 확정**하는 2단계 체크리스트다.
> 로봇 앞에서 위에서 아래로 따라 하면 된다.

## 선행 조건

- **RP-85 완료**: USB 카메라 드라이버 bringup 및 이미지 토픽 확정.
  (RP-76은 RP-85에 blocked. 코드는 토픽을 파라미터화해 두었으므로, 이 문서 단계에서
  실제 토픽 이름만 `camera_topic` 인자로 주입하면 된다.)
- **Nav2 기동**: `pinky_navigation` localization/navigation launch 를 같은
  `/<robot_id>` 네임스페이스로 띄워 `navigate_to_pose` 액션과 map 이 살아 있어야 한다.
- **HQ AnalyzeFrame 서버**: `/dg/analyze_frame` 서비스가 떠 있어야 응답
  `accepted=true`/`request_id` 를 받는다. (HQ측 구현은 RP-76 범위 밖 — 없으면
  "서비스 미준비" 경고만 뜨고 주행은 계속 성공한다.)

## 실행 위치 표기

| 라벨 | 실제 위치 |
| --- | --- |
| 🤖 **[로봇 dg_01]** | dg_01 로봇의 RPi5 (관제 PC에서 `ssh pinky@<IP>`) |
| 🖥️ **[관제 PC]** | 관제/개발 노트북 로컬 터미널 |

---

## 0단계 — 카메라 토픽 확정 (RP-85 산출물 확인)

🤖 **[로봇 dg_01]** 카메라 드라이버 bringup 후:

```bash
ros2 topic list | grep -i image          # 실제 이미지 토픽 이름 확인
ros2 topic info -v /dg_01/image_raw      # 타입(sensor_msgs/Image)·QoS 확인
ros2 topic echo /dg_01/image_raw --field encoding --once   # 인코딩 확인(예: rgb8, yuyv)
ros2 topic hz /dg_01/image_raw           # 프레임이 실제로 흐르는지
```

> 확정된 토픽 이름을 아래 launch 의 `camera_topic:=` 에 넣는다. 코드엔 하드코딩하지 않는다.
> 인코딩은 메시지에 실려 그대로 HQ로 전달되므로 DdaGo에서 별도 변환하지 않는다.

## 1단계 — Patrol 서버 기동

🤖 **[로봇 dg_01]**

```bash
cd ~/roscamp-repo-1/equip/automato_ws
source install/setup.bash
# camera_topic 은 0단계에서 확인한 실제 토픽으로 교체(예: 네임스페이스 상대명)
ros2 launch ddago_control ddago_patrol.launch.py \
    robot_id:=dg_01 \
    camera_topic:=image_raw
```

기대 로그: `Patrol 서버 준비됨: robot_id=dg_01 → 서버 ddago/patrol, Nav2=navigate_to_pose, ...`

🖥️ **[관제 PC]** 서버가 보이는지 확인:

```bash
ros2 action list | grep patrol          # /dg_01/ddago/patrol 보여야 함
ros2 action info /dg_01/ddago/patrol -t  # 타입 automato_interfaces/action/Patrol
```

## 2단계 — 단일 waypoint 수동 테스트 (티켓 DoD 명령)

🖥️ **[관제 PC]** map 좌표계에서 실제로 갈 수 있는 인접 노드 좌표를 넣는다:

```bash
ros2 action send_goal --feedback /dg_01/ddago/patrol \
    automato_interfaces/action/Patrol \
    "{task_id: 1, waypoint: {waypoint_id: 1, x: 1.0, y: 0.5}}"
```

**확인 항목(DoD):**
- [ ] 로봇이 (1.0, 0.5) 인접 노드까지 Nav2로 주행한다.
- [ ] 주행 중 Feedback(`current_waypoint_id`, `current_x/y/yaw`)이 뜬다(`--feedback`).
- [ ] 도착 후 정지 → (약 300ms 후) 프레임 grab.
- [ ] 🤖 로그에 `analyze_frame 요청 전송 ...` → `analyze_frame 수락됨 ... request_id=...`.
- [ ] 최종 `result_code=0`(성공/도착) 반환.

🖥️ 분석요청이 실제로 나갔는지 별도 확인:

```bash
ros2 service echo /dg/analyze_frame        # (Jazzy) 또는 HQ 서버 로그로 request_id 확인
```

## 3단계 — 연속 순찰(끊김 없음) 확인

🖥️ **[관제 PC]** 서로 다른 인접 노드를 연달아 하달해, 도착→촬영→반환→다음 goal
루프가 끊기지 않는지 본다:

```bash
ros2 action send_goal /dg_01/ddago/patrol automato_interfaces/action/Patrol \
    "{task_id: 2, waypoint: {waypoint_id: 2, x: 1.0, y: 1.0}}"
ros2 action send_goal /dg_01/ddago/patrol automato_interfaces/action/Patrol \
    "{task_id: 3, waypoint: {waypoint_id: 3, x: 0.0, y: 1.0}}"
```

- [ ] 각 goal 마다 도착 촬영이 1회씩 일어난다(프레임 흔들림 없음 — 반환 전에 grab하므로).
- [ ] 반환 직후 다음 goal 이 즉시 처리된다.

## 4단계 — 실패/취소 경로(선택)

- **막힘**: 도달 불가 좌표를 주면 Nav2가 abort → `result_code=1`.
- **취소**: goal 실행 중 `Ctrl-C`(send_goal) 또는 취소 요청 → Nav2 goal 취소 →
  `result_code=2`. 두 경우 모두 촬영·분석요청은 일어나지 않는다.

---

## 트러블슈팅

| 증상 | 원인 / 조치 |
| --- | --- |
| `Nav2 navigate_to_pose 서버 없음 → 실패(1)` | Nav2가 같은 네임스페이스로 안 떠 있음. `pinky_navigation` 을 `namespace:=/dg_01` 로 기동 |
| `카메라 프레임 미수신 → 분석요청 스킵` | `camera_topic` 이 실제 토픽과 불일치, 또는 드라이버 미기동. 0단계 재확인 |
| `analyze_frame 서비스 미준비 → 스킵` | HQ `/dg/analyze_frame` 서버 미기동(범위 밖). 주행 자체는 정상 |
| 도착했는데 흔들린 사진 | `arrival_settle_sec` 를 키운다(예: `-p arrival_settle_sec:=0.5`) |

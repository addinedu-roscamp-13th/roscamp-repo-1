# 통합테스트 런북 — DdaGo + ACS (dg_01 / pinky1)

> 실행 명령만. 각 항목은 **별도 터미널**.

## 공통 소싱

ROS 쓰는 모든 터미널:
```bash
source /opt/ros/jazzy/setup.bash
source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash
```
로컬의 **ACS · 검증 웹** 터미널은 위 2줄 + 아래 1줄:
```bash
source ~/roscamp-repo-1/services/automato_control_service/install/setup.bash
```

---

## 🤖 로봇 PC (dg_01 / pinky1)

```bash
# ① 드라이버 + 센서 (odom / scan / cmd_vel / 카메라)
ros2 launch pinky_bringup bringup_robot.launch.xml

# ② DdaGo 스택 (주행+촬영+텔레메트리+도킹2종). 도킹은 기본 안전모드(dry_run=true)
ros2 launch ddago_control ddago_bringup.launch.py

#   └ 실제로 도킹을 붙일 때만:
ros2 launch ddago_control ddago_bringup.launch.py dry_run:=false
```

---

## 🖥️ 로컬 PC (관제)

> ROS 터미널마다 먼저 도메인을 로봇과 맞춘다: **`pinky1`** (ROS_DOMAIN_ID=10)

```bash
# ① Nav2 (주행)
pinky1
nav1

# ② DB
cd ~/roscamp-repo-1/services/database && docker compose up -d

# ③ ACS (automato_node:8200 + telemetry_ws:8000 + aggregator) — 리포 안에서
pinky1
cd ~/roscamp-repo-1
ros2 launch automato_control_service acs_bringup.launch.py

# ④ DCS 중계 (팀원 담당)
pinky1
ros2 run dg_control dcs_node

# ⑤ 검증 웹 (LIVE) — 브라우저: http://127.0.0.1:8300
pinky1
VERIFY_MODE=LIVE python3 ~/roscamp-repo-1/services/automato_control_service/verify_web/server.py
```

---

## ▶ 순찰 시작 (로컬)

```bash
curl -s -X POST localhost:8200/internal/v1/tasks/patrol \
  -H 'Content-Type: application/json' \
  -d '{"robot_selection":"manual","robot_id":"dg_01"}'
```

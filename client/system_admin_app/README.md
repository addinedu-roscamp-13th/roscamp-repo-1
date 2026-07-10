# system_admin_app (Automato System Admin APP · QT)

시나리오 1 **E0 상시 모니터링 루프**의 최종 수신자. `/automato/dashboard/fleet_telemetry`
(`automato_interfaces/msg/FleetTelemetry`, 1Hz)를 ROS2로 구독해 주행(Ddago)/로봇팔(Ddagi)
상태를 실시간 진단 표시하고, 주행 로봇 유지보수 명령을 낸다.

## 구성

| 탭 | 내용 |
|----|------|
| 실시간 상태 | DG 카드 3개(dg_01/02/03). 주행·로봇팔 섹션 분리, 관절 상세 표, 정상/점검필요 판정, 수신 감시. 하단 1분 그래프(로봇팔 온도 ↔ 주행 배터리 토글, 65℃ 기준선) |
| 로봇 제어 | [예시/초안] QT→ACS→HQ 유지보수 명령: E-STOP / RESUME / TELEOP / DOCK / RESTART / ENABLE / DISABLE |

- **저장 없음**: E0 원칙대로 DB/파일 없이 메모리 링버퍼(최근 60포인트)만 사용.
- **미지원 타일**: 라이다·IMU·주행모터온도는 현재 메시지에 없어 "미지원"으로 표시(추후 협의 시 메시지 확장).
- **어댑터 격리**: ROS 타입 의존은 `ros/telemetry_node.py` 한 곳뿐. 필드명이 바뀌면 여기만 수정.

## 의존성

```bash
# GUI
sudo apt install -y python3-pyqt6 python3-pyqtgraph
# ROS2 Jazzy + automato_interfaces (텔레메트리 msg 4종 포함) 빌드 필요
```

## 실행

```bash
source /opt/ros/jazzy/setup.bash
source <workspace>/install/setup.bash        # automato_interfaces + system_admin_app 빌드된 워크스페이스

# 1) 실제 로봇/HQ가 발행 중이면 앱만 실행
python3 -m system_admin_app.main
#   또는 (colcon 오버레이 빌드 시)
#   ros2 run system_admin_app system_admin_app

# 2) 로봇 없이 시연 — 모의 발행기 + 모의 ACS 함께 실행 (터미널 3개)
python3 -m system_admin_app.ros.sim_publisher   # 가짜 텔레메트리 1Hz (dg_03 주행전용)
python3 -m system_admin_app.ros.mock_acs        # 제어탭 명령 수신 서버
python3 -m system_admin_app.main
```

## 제어탭 제안 인터페이스 (선택)

제어탭 명령은 `proposals/RobotMaintenanceCommand.srv`(팀 협의 전 초안)에 의존한다. 이 인터페이스가
없어도 앱은 정상 실행되며(모니터링 100%), 제어탭 전송만 비활성으로 표시된다. 제어 시연/mock_acs를
쓰려면 협의 후 소유자가 이 srv를 `automato_interfaces`에 추가하거나, 로컬에서 임시로 복사해 빌드한다:

```bash
cp proposals/RobotMaintenanceCommand.srv <automato_interfaces>/srv/
# CMakeLists.txt의 rosidl_generate_interfaces에 "srv/RobotMaintenanceCommand.srv" 한 줄 추가 후 colcon build
```

## 임계값

`system_admin_app/config.py`에서 배터리/서보온도/장애물 임계값과 로봇 구성(주행 전용 로봇)을
한곳에서 조정한다.

## 남은 협의 사항 (팀)

1. **텔레메트리 msg 4종**은 `automato_interfaces` 소유자와 병합 필요 → `INTERFACES_HANDOFF.md` 참고.
2. **제어탭 명령 집합/경로**(QT→ACS→HQ)는 초안. `RobotMaintenanceCommand.srv` 확정 필요.
3. 라이다·IMU·주행모터온도를 실제로 쓸지 → `DdagoTelemetry` 확장 여부.

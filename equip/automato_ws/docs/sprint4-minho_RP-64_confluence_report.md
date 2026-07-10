# [RP-64] Ddagi(로봇팔) 시나리오1 E0 텔레메트리 퍼블리셔 구현 및 harvest 잔재 코드 정리

- 작성자: minho
- 작성일: 2026-07-08
- 브랜치: `sprint4-minho`
- 관련 티켓: RP-64
- 관련 문서: [Scenario1_Sequence Diagram.md](Scenario1_Sequence%20Diagram.md), [sprint4-minho_jira_RP-69.md](sprint4-minho_jira_RP-69.md)

## 1. 개요

시나리오1 Sequence Diagram을 기준으로 Ddagi(로봇팔) 관련 코드
(`ddagi_control` 및 연계된 `dg_control`/`ddago_control`)를 검토했다.
검토 결과 Ddagi가 실제로 구현해야 할 E0(상시 텔레메트리) 발행 기능이
전혀 없는 반면, 이전 스프린트(harvest 데모)의 잔재 코드는 인터페이스가
이미 삭제됐음에도 지워지지 않아 빌드가 깨진 상태로 남아있었다. sprint4
범위(헬스 상태 토픽 발행)에 맞춰 텔레메트리 퍼블리셔를 신규 구현하고,
깨진 잔재 코드를 정리했다.

## 2. 작업 배경

- 커밋 `43034ba`("시나리오1 주간순찰 ROS2 통신 인터페이스 정의")에서
  `automato_interfaces`가 Scenario1용 인터페이스(`DdagoPatrol`,
  `Patrol`, `DdagiTelemetry` 등)로 교체되며 이전 Sprint3 harvest 데모용
  액션(`Move`, `DdagiHarvest`, `OpHarvest`)이 삭제됐다.
- 해당 커밋 메시지는 "Sprint3 통합테스트용 노드/액션 제거"라고 적혀
  있었으나, 실제 diff는 `automato_interfaces`의 인터페이스 정의만
  변경했을 뿐 이를 사용하던 `ddagi_control/harvest_server.py`,
  `ddago_control/move_server.py`, `dg_control/op_harvest_server.py`
  및 관련 테스트/launch 파일은 그대로 남아 있었다.
- 그 결과 이 세 노드는 존재하지 않는 액션 타입을 참조하는 깨진 코드가
  되어 있었고, 이후 커밋(RP-69)에서도 `dg_ai_service`/`dg_control`만
  다뤄서 이 문제가 방치된 상태였다.
- 한편 시나리오1 문서상 Ddagi의 역할은 E0(로봇팔 헬스 상태를 담은
  `DdagiTelemetry` 토픽 1Hz 발행)뿐이며 E1~E3에는 관여하지 않는다.
  이 발행 기능은 메시지 정의(`DdagiTelemetry.msg`, `ServoStatus.msg`)만
  존재할 뿐 발행 노드 자체가 없었다.

## 3. 발견된 문제

| # | 문제 | 원인 |
|---|------|------|
| 1 | `ddagi_control/harvest_server.py` 및 테스트가 import 단계에서 실패 | 삭제된 `automato_interfaces.action.DdagiHarvest` 참조 |
| 2 | `dg_control/op_harvest_server.py`, `ddago_control/move_server.py` 및 각 테스트도 동일하게 실패 | 삭제된 `Move`, `OpHarvest` 참조 |
| 3 | `dg_control/launch/automato_bringup.launch.py`도 위 노드들을 기동하도록 되어 있어 실행 불가 | 위와 동일 |
| 4 | Ddagi의 시나리오1 E0 텔레메트리 발행 기능 자체가 미구현 | 메시지 정의만 있고 발행 노드 부재 |

## 4. 신규 기능: Ddagi 텔레메트리 퍼블리셔

`ddagi_control/telemetry_publisher.py`를 신규 작성했다.

- **토픽**: `/{robot_id}/ddagi/telemetry` (기본 `robot_id=dg_01`)
- **메시지**: `automato_interfaces/msg/DdagiTelemetry` (1Hz, 파라미터로 조정 가능)
- **내용**: `servo_health`(6관절 + 그리퍼 7개)를 문서 스펙의 예시
  페이로드와 동일한 값으로 시뮬레이션해 발행. 실물 myCobot 미연동
  상태이므로 `read_servo_health()`에 실제 하드웨어 조회로 교체할 지점을
  TODO로 남겨뒀다.
- sprint4 범위는 헬스 상태 발행까지이므로, 작업 연동이 필요한
  `task_id`/`is_paused`는 유휴 상태 고정값(0 / false)을 사용한다.
- 통합 테스트 3건(`test_telemetry_publisher.py`): 토픽 발행 여부,
  `servo_health` 7개 중 7번째가 그리퍼인지, 주기적 발행 여부.

## 5. 코드 정리 (harvest 잔재 제거)

인터페이스가 이미 삭제되어 더 이상 동작할 수 없는 Sprint3 harvest
데모 코드를 제거했다.

**삭제한 파일**
- `ddagi_control/ddagi_control/harvest_server.py`, `ddagi_control/test/test_harvest_action.py`
- `ddago_control/ddago_control/move_server.py`, `ddago_control/test/test_move_action.py`
- `dg_control/dg_control/op_harvest_server.py`, `dg_control/test/test_orchestration.py`
- `dg_control/launch/automato_bringup.launch.py` (빈 `launch/` 디렉터리도 함께 제거)

**함께 정리한 설정 파일**
- `ddagi_control/setup.py`, `ddago_control/setup.py`, `dg_control/setup.py`:
  삭제된 파일을 가리키던 `console_scripts` entry point 제거
- `dg_control/package.xml`: 더 이상 쓰지 않는 `launch`/`launch_ros`/
  `ddago_control`/`ddagi_control` exec_depend 제거. `dg_ai_service`는
  `test_analyze_frame_client.py`가 여전히 import하므로 `exec_depend`
  대신 `test_depend`로 이동(용도에 맞게 재분류)

`ai_client.py`의 레거시 TCP 프로토콜(`analyze()`)과 시나리오1 E2용
`analyze_frame()`/`decode_labeled_image()`는 삭제된 액션과 무관하게
독립적으로 동작하므로 그대로 유지했다.

## 6. 검증 결과

- `colcon build`(워크스페이스 전체 clean rebuild) → 5개 패키지 전부 성공
- `pytest` 전체 실행 → 15건 중 11건 통과. 나머지 4건은 `dg_ai_service`의
  `cv2`가 opencv 전용 venv 없이 시스템 파이썬에서 깨지는 기존 환경
  이슈로, 이번 정리와 무관함을 확인(`ddagi_control`/`ddago_control`
  관련 3건은 전부 통과, `dg_control`도 import 에러 없이 정상 수집·실행)
- 실제 ROS2(Jazzy) 환경에서 `ros2 run ddagi_control telemetry_publisher`
  실행 후 `ros2 topic list`/`ros2 topic echo --once`로 토픽 발행과
  메시지 내용을 직접 확인 — 문서 예시 페이로드와 필드값 일치
- 저장소 전체 grep으로 삭제된 코드에 대한 잔여 참조 점검. `equip/automato_ws`
  안에서는 `dashboard.sh`/`web/control_server.py`(데모용 웹 대시보드)가
  삭제된 실행 파일(`move_server`/`harvest_server`/`op_harvest_server`/
  `automato_bringup.launch.py`)을 여전히 참조하고 있음을 확인했으나,
  이번 스프린트 범위 밖으로 판단해 그대로 두기로 함

## 7. 향후 과제

- `dashboard.sh`/`web/control_server.py`의 harvest 관련 항목(`move`,
  `ddagi`, `dg`) 정리 — Scenario1의 DdaGo 순찰 액션 서버/HQ 취합 노드가
  준비되면 그 시점에 대시보드도 함께 갱신하는 것을 권장
- HQ(`dg_control`) 쪽의 텔레메트리 구독·`FleetTelemetry` 취합 노드는
  아직 미구현(E0 루프의 나머지 절반)
- DdaGo의 E0 텔레메트리 퍼블리셔, E1/E2 `DdagoPatrol` 액션 서버도
  아직 미구현

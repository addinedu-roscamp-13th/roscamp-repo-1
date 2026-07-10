# RP-64: Ddagi(로봇팔) 시나리오1 E0 텔레메트리 퍼블리셔 구현 및 harvest 잔재 코드 정리

- 유형: Task
- 브랜치: `sprint4-minho`
- 담당: minho
- 완료일: 2026-07-08

## 작업 설명

시나리오1 Sequence Diagram 기준으로 `ddagi_control`(로봇팔) 코드가 스펙과
맞는지 검토한다. Ddagi는 시나리오1에서 E0(상시 텔레메트리) 발행만
담당하므로, sprint4 범위(헬스 상태 토픽 발행)에 맞춰 이를 신규
구현한다. 검토 중 발견된, 인터페이스 마이그레이션(43034ba) 이후
존재하지 않는 액션을 참조한 채 방치되어 있던 Sprint3 harvest 관련
코드(빌드 자체가 깨진 상태)를 함께 정리한다.

## 상세 작업 내용

1. **기존 코드 검토**
   - `ddagi_control/harvest_server.py`, `dg_control/op_harvest_server.py`,
     `ddago_control/move_server.py`가 커밋 43034ba에서 삭제된
     `DdagiHarvest`/`OpHarvest`/`Move` 액션을 여전히 import하고 있어
     빌드/실행이 불가능한 상태임을 확인
   - Ddagi가 시나리오1에서 실제로 맡아야 할 범위는 E0 텔레메트리
     발행뿐이며(E1/E2/E3는 DdaGo/HQ/AI Service/ACS만 관여), 이 발행
     노드 자체가 어디에도 구현돼 있지 않음을 확인(`DdagiTelemetry`/
     `ServoStatus` 메시지 정의는 이미 존재)

2. **Ddagi 텔레메트리 퍼블리셔 신규 구현** (`ddagi_control/telemetry_publisher.py`)
   - `/{robot_id}/ddagi/telemetry`(`DdagiTelemetry`)를 1Hz로 발행
     (파라미터: `robot_id` 기본 `dg_01`, `publish_rate_hz` 기본 1.0)
   - `servo_health`(6관절+그리퍼 7개)를 문서 스펙 예시 페이로드와 동일한
     값으로 시뮬레이션 발행, 실물 myCobot 연동 지점은 TODO로 표시
   - sprint4 범위 밖인 `task_id`/`is_paused`는 유휴 고정값(0/false) 사용
   - 통합 테스트 3건(`test_telemetry_publisher.py`) 작성 및 통과

3. **깨진 Sprint3 harvest 잔재 코드 정리**
   - `harvest_server.py`/`move_server.py`/`op_harvest_server.py`와 각각의
     테스트, `automato_bringup.launch.py` 삭제
   - `ddagi_control`/`ddago_control`/`dg_control`의 `setup.py`
     entry_points, `dg_control/package.xml` 의존성 정리(더 이상 쓰지
     않는 `launch`/`launch_ros`/`ddago_control`/`ddagi_control`
     exec_depend 제거, `dg_ai_service`는 테스트 전용으로 남아있어
     `test_depend`로 이동)

4. **검증**
   - `colcon build` 워크스페이스 전체 재빌드 성공(5개 패키지)
   - `pytest` 전체 실행, `ddagi_control`/`ddago_control`/`dg_control`
     정상 통과 확인(`dg_ai_service`의 `cv2` 환경 이슈로 인한 기존 실패
     4건은 이번 작업과 무관함을 별도 확인)
   - 실제 ROS2 환경에서 `ros2 run` + `ros2 topic echo`/`hz`로 토픽 발행
     수동 검증(발행 내용이 문서 예시 페이로드와 일치)

## 작업 완료 조건 (Definition of Done)

- [x] Ddagi가 시나리오1에서 실제로 맡아야 할 범위(E0 헬스 텔레메트리)를
      문서와 대조해 확인
- [x] `/{robot_id}/ddagi/telemetry` 1Hz 발행 노드 구현 및 통합 테스트
      3건 작성·통과
- [x] 실물 ROS2 환경(`colcon build` + `ros2 run` + `topic echo`/`hz`)에서
      수동 검증 완료
- [x] 인터페이스 마이그레이션 이후 깨져 있던 Sprint3 harvest 관련
      노드/테스트/launch 파일 제거 및 각 패키지 의존성 정리
- [x] 정리 후 워크스페이스 전체 재빌드 및 회귀 테스트로 부작용 없음 확인
- [ ] (보류, 후속 과제) `dashboard.sh`/`web/control_server.py`의 harvest
      관련 항목 정리 — 삭제된 실행 파일을 여전히 참조 중이나, 이번
      스프린트 범위 밖으로 판단해 이월

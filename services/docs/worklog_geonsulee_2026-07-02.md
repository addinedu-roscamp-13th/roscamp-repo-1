# Automato services 작업 정리 — geonsulee / 2026-07-02

SERVICES 계층(Web/Control/DB)을 `services/`에 구현. 팀원(hskim) equip 예시·팀 계약 준수.

## 1. 구현
| 폴더 | 역할 | 통신 | 상태 |
|---|---|---|---|
| automato_web_service | Farm App↔Web (Flask :8100) | HTTP/WS | /api/v1/operation/start(RP-53) 등 |
| automato_control_service | Web/앱→DG 중계 (rclpy :8200) | HTTP + ROS2 OpHarvest | /dg1/op/harvest 클라이언트 |
| database | 저장소 | TCP | 스키마 초안 |

## 2. 통합 계약 (팀 automato_interfaces)
- **OpHarvest** (/dg1/op/harvest, RP-60): Automato Control → DG. Goal(없음)/Result(string)/Feedback(count,x,y,yaw).
- Control Service가 이 액션 클라이언트로 동작. equip/dg_control/test/test_orchestration.py의 호출 패턴 참고.

## 3. 테스트
- Control 단위: test_control.py (모의 DG OpHarvest 서버, SingleThreadedExecutor)
- 종단: Web→Control→DG (TESTING.md)

## 4. 남은 일
- DB 스키마 확정(팀 ERD)
- System Admin App ↔ Control (ROS2) 연동
- 팀 계약 원문(Confluence 19496961) 대조

## 5. 참고
- ../equip/automato_ws (팀원 hskim 예시)
- SETUP.md / TESTING.md / docs/test_flow.md

# dg_sim — HQ 테스트용 상대편 시뮬 4종

`dg_control`(HQ)을 실제 팀원 코드 없이 테스트하기 위한 **즉시-응답 시뮬레이터**.
팀원들의 실제 패키지와 겹치지 않게 이 한 패키지에 노드 4개로 모아 둔다.

| 노드 | 대역 | 핵심 |
|---|---|---|
| `acs_sim`   | Automato Control Service | Patrol 클라(waypoint 하나씩 하달·루프) / SaveDetection 서버 / Fleet 구독 |
| `ddago_sim` | DdaGo Control Service | Patrol 서버(단일 waypoint, 즉시 도착) / DdagoTelemetry / AnalyzeFrame 클라 |
| `ddagi_sim` | Ddagi Control Service | DdagiTelemetry 1Hz |
| `dg_ai_sim` | DG AI Service | TCP 서버(:9100, 4B len+JSON, 가짜 분석결과) |

## 실행
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch dg_sim dg_sim.launch.py            # 4종 동시 기동
# 또는 개별:  ros2 run dg_sim ddago_sim
```
대시보드에서 개별 on/off: `../dashboard.sh start|stop <acs|ddago|ddagi|dg_ai>`.

## 통합 테스트
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_sim/test/test_e2e.py -v
```
상세는 `docs/dg_control_dev_2026-07-08.md`.

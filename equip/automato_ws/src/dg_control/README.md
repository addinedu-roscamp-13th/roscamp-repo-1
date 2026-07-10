# dg_control — DG Control Service (HQ)

시나리오1 순찰 오케스트레이터 **본체(실제 구현)**. 팀원들이 개발 중인 상대편은
`dg_sim` 시뮬로 대역하여 혼자 end-to-end 테스트한다.

## 노드: `hq_node`
- E0: Ddago/DdagiTelemetry 구독 → **FleetTelemetry** 1Hz 발행 (`/automato/telemetry/fleet`)
- E1: **Patrol 액션 서버** (`/dg_01/patrol`, ACS←) ↔ **Patrol 액션 클라** (`/dg_01/ddago/patrol`, →DdaGo) — 단일 waypoint
- E2: **AnalyzeFrame 서비스 서버** (`/dg/analyze_frame`) → AI(TCP) → **SaveDetection 클라** (`/automato/save_detection`)

AI TCP 접속 대상은 `../../dg_web/dg_ai_target.json` 의 `active`("real"|"sim")를 따르며,
대시보드에서 dg_ai 시뮬을 끄면 실서버로 자동 전환된다(`ai_client.py`).

## 실행
```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run dg_control hq_node
```

## 테스트
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest src/dg_control/test/test_ai_switch.py -v
```
전체 통합 테스트는 `dg_sim/test/test_e2e.py` 참조. 상세는 `docs/dg_control_dev_2026-07-08.md`.

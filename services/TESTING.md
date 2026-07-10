# services 테스트 방법

## Control Service 단위 테스트 (모의 DG)
automato_interfaces 빌드·source 후:
```bash
cd automato_control_service
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest test/test_control.py -v
```
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 필수(venv에서 launch_testing yaml import 실패 방지).
- 액션 테스트는 **SingleThreadedExecutor** 사용(MultiThreaded는 "Two goals same ID" 충돌 — equip/TESTING.md와 동일 교훈).

## 종단 통신 테스트 (Web → Control → DG)
1. DG 서버 기동(equip):  `ros2 launch dg_control automato_bringup.launch.py`
2. Control 기동:        `python3 automato_control_service/automato_control_service/control_node.py`
3. Web 기동:            `PORT=8100 CONTROL_URL=http://localhost:8200 python3 automato_web_service/app.py`
4. 트리거(Postman/curl): `POST http://localhost:8100/api/v1/operation/start`
   → 200 + "운영 시작" 로그 + Control이 DG로 OpHarvest → DG Result 'success' 가 응답 relay에 포함

## 변경 이력
| 날짜 | 내용 |
|---|---|
| 2026-07-02 | 최초 작성 |

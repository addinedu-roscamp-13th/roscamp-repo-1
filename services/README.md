# services — Automato SERVICES 계층 (담당: 이건수)

SW 아키텍처 SERVICES 계층. Client(앱)와 EQUIP(로봇/DG) 사이 중계.

```
Farm Admin App ─HTTP─▶ automato_web_service ─HTTP─▶ automato_control_service ─ROS2(OpHarvest)─▶ DG(/dg1/op/harvest)
                                                            └─TCP─▶ database
```

| 폴더 | 역할 | 통신 |
|---|---|---|
| `automato_web_service/` | Farm App ↔ Web (Flask) | HTTP/WS |
| `automato_control_service/` | Web/앱 → DG 중계 (ROS2 액션) | HTTP + ROS2(OpHarvest) |
| `database/` | 데이터 저장소 | TCP |

문서: [SETUP.md](SETUP.md) · [TESTING.md](TESTING.md) · [docs/test_flow.md](docs/test_flow.md)
계약: 팀 Confluence "Sprint 3 인터페이스 통합 테스트 메세지 규격", `equip/automato_ws/src/automato_interfaces`

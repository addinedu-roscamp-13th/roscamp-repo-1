# services 통신 흐름 (VS Code 미리보기로 Mermaid 렌더)

```mermaid
flowchart LR
    FA[Farm Admin App] -->|"HTTP POST /api/v1/operation/start"| WEB[Automato Web Service<br/>:8100]
    WEB -->|"HTTP 중계"| CTL[Automato Control Service<br/>:8200]
    CTL -->|"ROS2 Action /dg1/op/harvest (OpHarvest, RP-60)"| DG[DG Control Service]
    CTL -->|"TCP"| DB[(Automato DB)]
    DG -.->|"Result: success"| CTL
    CTL -.->|"relay 응답"| WEB
    WEB -.->|"200 + 운영 시작"| FA
```

## 확인 포인트
- WEB `/api/v1/operation/start` → **200 + '운영 시작' 로그** (RP-53)
- CTL → DG **OpHarvest** Goal 발행 → **Result 'success'** 수신
- 응답 `relay.dg_result == "success"` 면 Web→Control→DG 전 구간 성립

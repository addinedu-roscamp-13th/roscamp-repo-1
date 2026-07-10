# 인터페이스 현황 (automato_interfaces)

## E0 텔레메트리 msg — 공식본 사용 (조율 완료)

`ServoStatus / DdagoTelemetry / DdagiTelemetry / FleetTelemetry`는 sprint4에서 소유자가
이미 정의했고, 본 QT 앱은 그 공식 정의를 그대로 구독한다. (필드가 스펙과 100% 일치함을
확인 → 어댑터 `ros/telemetry_node.py` 수정 불필요.)

→ **automato_interfaces는 이 앱 브랜치에서 건드리지 않는다(소유자 영역).**

## 제어탭 제안 서비스 (아직 미확정)

```
client/system_admin_app/proposals/RobotMaintenanceCommand.srv   ← 제안 원본(여기 보관)
```

- System Admin APP 제어탭(정비)용 **초안**. 경로: QT → ACS → HQ.
- 아직 팀 협의 전이라 **automato_interfaces에 넣지 않았다.** 협의가 끝나면 소유자(이보연)가
  이 파일을 `automato_interfaces/srv/`에 추가하고 `CMakeLists.txt`의
  `rosidl_generate_interfaces`에 한 줄 등록하면 된다.
- 그 전까지 앱은 이 인터페이스가 없어도 **정상 실행**된다(모니터링 100% 동작).
  제어탭의 명령 전송만 비활성 상태로 표시되며, 인터페이스가 빌드되면 자동 활성화된다.
- command 집합(ESTOP/RESUME/TELEOP/DOCK/RESTART/ENABLE/DISABLE)과 라우팅은 협의로 확정.
  바뀌면 이 srv와 `ui/control_tab.py`만 조정.

## 남은 논의

- 라이다·IMU·주행모터온도 → 현재 `DdagoTelemetry`에 없음. UI는 "미지원" 타일로 표기 중.
  실제로 쓰려면 `DdagoTelemetry` 확장 필요(소유자 협의).

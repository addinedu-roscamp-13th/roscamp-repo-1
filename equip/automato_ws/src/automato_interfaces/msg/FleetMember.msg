# 시나리오1 E0 ④ : ACS 가 취합하면서 만드는 항목 (FleetTelemetry.robots 원소)
# 로봇 쪽(DdaGo/Ddagi/DG)은 이 타입을 알지 못한다.
#
# 왜 여기에만 robot_id 가 있나 —
#   상위 구간(DG -> ACS)은 /{robot_id}/telemetry 라는 네임스페이스로 로봇이 구분되지만,
#   ACS -> QT 는 /automato/dashboard/fleet_telemetry 라는 '네임스페이스 없는 단일 토픽'이라
#   어느 로봇의 데이터인지를 메시지 안에 실어야 한다. 그리고 robot_id 를 아는 주체는
#   구독을 로봇별로 만든 ACS 뿐이다. 그래서 취합 전용 항목 타입을 따로 두고 여기에만 붙인다.
string           robot_id     # ACS 가 구독 네임스페이스에서 채운다
RobotTelemetry   telemetry    # DG 에게서 받은 것을 가공 없이 그대로 담는다

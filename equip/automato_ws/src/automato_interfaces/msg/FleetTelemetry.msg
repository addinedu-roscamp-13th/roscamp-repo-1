# 시나리오1 E0 ④ : ACS 가 로봇 세트 전체를 취합해 QT 로 보내는 메시지
# Automato Control Service -> System Admin APP/QT
#   (ROS2 Topic  /automato/dashboard/fleet_telemetry, 1Hz)
#
# RP-114 로 '내용물이 재정의'되었다. 이름은 그대로지만 뜻이 바뀌었다 —
#   이전: DG 가 로봇 3대분을 하나로 묶어 ACS 로 보내던 메시지 (DG 는 로봇당 하나뿐인데
#         3대분 배열을 갖는 잘못된 설계였다)
#   이후: 로봇별 구독을 가진 ACS 가 취합해 QT 로 보내는 메시지
# DG -> ACS 구간은 RobotTelemetry(/{robot_id}/telemetry)가 대신한다.
#
# 발행 규칙(문서 E0 ④):
#   * ACS 는 로봇별 마지막 수신값을 캐시해두고 자체 1Hz 타이머로 배열을 만들어 발행한다.
#     세 스트림의 도착 시각이 제각각이라 수신 시점에 맞춰 발행하지 않는다.
#   * 텔레메트리가 끊긴 로봇도 배열에서 빼지 않는다(마지막 값을 그대로 넣는다). 빼면 QT
#     화면에서 로봇이 깜빡이고 '연결 끊김'과 '존재하지 않음'을 구분할 수 없다.
#   * QT 는 각 telemetry.header.stamp 와 현재 시각을 비교해 3초 이상이면 오프라인 표시.
std_msgs/Header   header       # 취합 시각 (ACS 기준)
FleetMember[]     robots       # 로봇 세트 수만큼

# --------------------------------------------------------------------------- #
# [삭제 예정] 아래 두 필드는 재정의 이전의 구조다.
# QT(system_admin_app)·dg_sim 이 아직 이 필드를 읽고 있어, 그쪽 이전이 끝날 때까지
# ACS 가 robots 와 '함께' 채워 발행한다(과도기 하위호환). 소비자 이전 후 제거한다.
# 새로 짜는 코드는 robots 만 본다.
# --------------------------------------------------------------------------- #
DdagoTelemetry[]  ddagos
DdagiTelemetry[]  ddagis

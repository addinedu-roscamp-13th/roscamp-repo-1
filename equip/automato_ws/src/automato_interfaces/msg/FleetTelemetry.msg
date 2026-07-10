# 시나리오1 E0 : HQ가 두 로봇 텔레메트리를 robot_id 기준 배열로 취합
# HQ -> Automato Control Service (ROS2 Topic  /automato/telemetry/fleet, 1Hz)
# Automato Control Service -> System Admin APP/QT (ROS2 Topic  /automato/dashboard/fleet_telemetry, 동일 재사용)
std_msgs/Header    header
DdagoTelemetry[]   ddagos
DdagiTelemetry[]   ddagis

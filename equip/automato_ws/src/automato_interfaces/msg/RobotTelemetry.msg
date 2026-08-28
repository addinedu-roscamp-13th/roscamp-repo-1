# 시나리오1 E0 ③ : DG 한 세트(주행 로봇 + 로봇팔)의 텔레메트리 묶음
# DG Control Service -> Automato Control Service (ROS2 Topic  /{robot_id}/telemetry, 1Hz)
#
# 어느 로봇인지는 토픽 네임스페이스(/dg_01/telemetry)가 말해주므로 메시지 안에
# robot_id 를 두지 않는다. DG 는 로봇 세트마다 하나씩 뜨므로 자기 세트분만 담는다
# (로봇 3대분을 배열로 취합하는 일은 전체를 보는 ACS 가 한다 -> FleetTelemetry).
#
# 배열인 이유: 한 세트에 주행 로봇/팔이 각각 0개 또는 1개다. 팔이 없는 세트(dg_03)는
# ddagis 가 빈 배열이 된다. 옵셔널을 표현할 방법이 배열뿐이라 길이 0/1 로 쓴다.
std_msgs/Header    header     # 취합 시각 (DG 기준)
DdagoTelemetry[]   ddagos
DdagiTelemetry[]   ddagis

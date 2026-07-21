# 시나리오1 E0 ② : Ddagi(로봇팔) 상시 텔레메트리
# Ddagi Control Service -> DG Control Service (ROS2 Topic  /ddagi/telemetry, 1Hz)
#
# 로봇별 물리망 분리 이후 이 토픽에는 로봇 식별자가 없다. 어느 로봇인지는 수신한
# DG Control Service 가 자기 robot_id 로 판단하며, 상위 구간(DG -> ACS)에서
# /{robot_id}/telemetry 네임스페이스로 표현된다. (DdagoTelemetry 와 같은 규칙)
std_msgs/Header  header
string           robot_id     # [삭제 예정] 네임스페이스로 대체됨. 소비자 이전 후 제거
int64            task_id
bool             is_paused

float32[6]       joint_angles
float32[6]       tcp_coords

ServoStatus[7]   servo_health

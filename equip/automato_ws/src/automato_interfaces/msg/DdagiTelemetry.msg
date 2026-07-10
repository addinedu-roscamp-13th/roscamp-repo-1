# 시나리오1 E0 : Ddagi(로봇팔) 상시 텔레메트리
# Ddagi Control Service -> HQ (ROS2 Topic  /dg_01/ddagi/telemetry, 1Hz)
# robot_id : dg_01, dg_02, dg_03
std_msgs/Header  header
string           robot_id
int64            task_id
bool             is_paused

float32[6]       joint_angles
float32[6]       tcp_coords

ServoStatus[7]   servo_health

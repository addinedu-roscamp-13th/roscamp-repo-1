# 시나리오1 E0 : DdaGo(주행 로봇) 상시 텔레메트리
# DdaGo Control Service -> DG Control Service (ROS2 Topic  /ddago/telemetry, 1Hz)
#
# 로봇별 물리망 분리 이후 이 토픽에는 로봇 식별자가 없다. 어느 로봇인지는 수신한
# DG Control Service 가 자기 robot_id 로 판단하며, 상위 구간(DG -> ACS)에서
# /{robot_id}/telemetry 네임스페이스로 표현된다.
std_msgs/Header  header
string           robot_id     # [삭제 예정] ddago 는 채우지 않는다(빈 문자열). 소비자 이전 후 제거
int64            task_id      # 진행 중 goal 의 task_id. goal 이 끝나도 유지, 받기 전엔 0
string           nav_status   # IDLE | NAVIGATING — 이 두 값뿐 ("지금 움직이는 중인가"만 답한다)
bool             is_charging  # 핑키는 충전 상태를 제공하지 않아 현재 항상 false

# position
float64          x
float64          y
float64          yaw

# battery
float32          battery_percent
float32          battery_voltage

# obstacle
float32          us_range_m

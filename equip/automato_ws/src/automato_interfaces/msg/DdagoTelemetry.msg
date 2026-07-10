# 시나리오1 E0 : DdaGo(주행 로봇) 상시 텔레메트리
# DdaGo Control Service -> HQ (ROS2 Topic  /dg_01/ddago/telemetry, 1Hz)
# robot_id : dg_01, dg_02, dg_03
std_msgs/Header  header
string           robot_id
int64            task_id
string           nav_status
bool             is_charging

# position
float64          x
float64          y
float64          yaw

# battery
float32          battery_percent
float32          battery_voltage

# obstacle
float32          us_range_m

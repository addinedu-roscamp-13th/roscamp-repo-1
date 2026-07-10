# 시나리오1 E0 : Ddagi 로봇팔 관절 상태 (DdagiTelemetry.servo_health 원소)
# 관절 7개 = 6관절 + 그리퍼. gripper_value는 그리퍼(7번)에서만 의미가 있다.
int8    joint_no
bool    voltage_ok
int16   temperature
float32 current
bool    overload
int16   gripper_value

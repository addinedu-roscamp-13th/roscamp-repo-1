# 시나리오1 E1/E2 : 순찰 경로 waypoint 1개 (Navigate 액션 Goal 의 waypoints[] 원소)
# 이전 WaypointGoal 에서 yaw, capture 필드가 추가되었다.
int32    waypoint_id
float64  x
float64  y
float64  yaw            # 이 지점에서 로봇이 향할 방향(rad). 촬영 지점은 볼 베드 쪽을 향한다.
bool     capture        # true 면 도착·정지 후 CaptureFrame 촬영(E2 3단계), false 면 통과만

# 시나리오1 E1/E2 : 순찰 경로 waypoint 1개 (Navigate 액션 Goal 의 waypoints[] 원소)
# 이전 WaypointGoal 에서 capture 필드가 추가되었다.
int32    waypoint_id
float64  x
float64  y
bool     capture        # true 면 도착·정지 후 CaptureFrame 촬영(E2 3단계), false 면 통과만

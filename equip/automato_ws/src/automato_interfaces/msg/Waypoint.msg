# 시나리오1 E1/E2 : 순찰 경로 waypoint 1개 (Navigate 액션 Goal 의 waypoints[] 원소)
# 이전 WaypointGoal 에서 yaw, capture 필드가 추가되었다.
int32    waypoint_id
float64  x
float64  y
float64  yaw            # 이 지점에서 로봇이 향할 방향(rad). 촬영 지점은 볼 베드 쪽을 향한다.
bool     capture        # true 면 도착·정지 후 CaptureFrame 촬영(E2 3단계), false 면 통과만
# true 면 도착 후 위 yaw 로 정확히 돌아선다(촬영 없이 방향만). 도킹 진입 노드 전용이다.
#
# 왜 필요한가: 로봇은 주행 효율을 위해 '가는 방향'을 보고 달리고, 도착한 뒤 목표 yaw 로
# 고쳐 선다(navigate_server 의 정밀 조준). 그런데 이 '고쳐 서기'가 capture 지점에서만
# 돌아서, 촬영하지 않는 도킹 진입 노드는 어느 쪽에서 왔느냐에 따라 자세가 90° 가까이
# 달라진다(실측: wp15→wp24 는 176.6°, DB 값은 87.3°). 마커 도킹은 카메라·라이다로
# 마커를 정면에서 봐야 시작되므로, 비스듬히 서면 '마커 없음'으로 실패한다.
# capture 를 대신 켜면 쓸데없는 사진이 찍히므로 방향 전용 플래그를 따로 둔다.
#
# 통과 노드 전반에 켜면 안 된다 — 지나가는 지점마다 고개를 돌리느라 두리번거린다.
# 켜는 곳은 충전소·수확지·예냉실의 도킹 진입 노드뿐이다(ACS route_runner.entry_yaw).
bool     hold_yaw

#!/usr/bin/env python3
"""검출 결과·파지 목표를 rviz 마커로 발행.

수확 노드가 "무엇을 어떻게 봤는지"를 3D 로 보여준다. 발표·데모용이면서 동시에
진단 도구다 — 도착 정확도는 ±3mm 인데 검출 좌표가 ~9mm 어긋나는 것이 현재 한계라,
마커를 띄우면 그 어긋남이 눈에 보인다(숫자로만 알던 것).

    토픽 : /ddagi/markers   (visualization_msgs/MarkerArray)
    프레임: ddagi_base      (팔 base. rviz 의 Fixed Frame 을 여기로 맞춘다)

**TF 를 발행하지 않는다.** rviz 는 Fixed Frame 과 마커의 frame_id 가 같으면 TF 없이
그린다. 팔 모델(robot_state_publisher)을 붙일 때 이 프레임을 URDF 의 base 링크에
연결하면 되고, 그 전까지는 마커만으로 완결된다.

단위: 내부 좌표는 mm, ROS 는 m(REP-103). 여기서 1/1000 로 환산한다.
"""
from __future__ import annotations

FRAME_ID = "ddagi_base"     # 팔 base. URDF 를 붙일 때 이 이름을 base 링크에 맞춘다
PARENT_FRAME = "world"      # ddagi_base 의 부모. 정적 변환 하나로 트리를 만든다
TOPIC = "/ddagi/markers"

TOMATO_D = 0.022          # 방울토마토 지름(m) — 마커 크기
_COLOR = {                # r, g, b, a
    "NORMAL": (0.88, 0.20, 0.16, 0.95),    # 수확품 — 빨강
    "DISCARD": (0.55, 0.55, 0.55, 0.95),   # 폐기품 — 회색
}
_TARGET_COLOR = (0.20, 0.75, 0.35, 0.45)   # 현재 파지 목표 — 초록 반투명
_ZONE_COLOR = (0.30, 0.55, 0.95, 0.10)     # 성공 실측 대역 — 파랑 아주 옅게

# 2026-07-29 실측: 열매 13개 중 성공 7개가 전부 이 대역 안이었다(x 246~266,
# z 270~355). 밖은 standoff 확보 실패나 빈손으로 끝났다. 열매를 어디에 달아야
# 하는지 눈으로 보라고 그려 둔다. 실측이 바뀌면 이 값을 갱신할 것.
SUCCESS_ZONE_MM = {"x": (246.0, 266.0), "y": (-100.0, 155.0), "z": (270.0, 355.0)}


class MarkerPublisher:
    """노드에 붙어 마커를 쏜다. rclpy 가 없으면 만들지 않는다(단독 실행 경로 보호)."""

    def __init__(self, node, show_zone: bool = True):
        from visualization_msgs.msg import MarkerArray
        self._node = node
        self._pub = node.create_publisher(MarkerArray, TOPIC, 1)
        self._show_zone = show_zone
        self._last_n = 0
        self._publish_static_tf()

    def _publish_static_tf(self) -> None:
        """world -> ddagi_base 항등 변환을 한 번 쏜다.

        rviz2 는 **Fixed Frame 이 TF 트리에 실재해야** 렌더링한다. 마커의 frame_id 와
        Fixed Frame 이 같기만 하면 되는 게 아니다(실측: 마커는 도착하는데
        "Frame [ddagi_base] does not exist" 로 아무것도 안 그려졌다). 정적 변환 하나로
        두 프레임을 만들어 두면 별도 터미널에서 static_transform_publisher 를 띄울
        필요가 없다. 나중에 URDF·robot_state_publisher 를 붙이면 그쪽 base 링크가
        ddagi_base 를 잇는다.
        """
        from geometry_msgs.msg import TransformStamped
        from tf2_ros import StaticTransformBroadcaster
        self._tf = StaticTransformBroadcaster(self._node)
        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = PARENT_FRAME
        t.child_frame_id = FRAME_ID
        t.transform.rotation.w = 1.0        # 나머지 성분은 0 = 항등
        self._tf.sendTransform(t)

    # ---- 내부 ------------------------------------------------------------ #

    def _new(self, ns: str, mid: int, mtype: int):
        from visualization_msgs.msg import Marker
        m = Marker()
        m.header.frame_id = FRAME_ID
        m.header.stamp = self._node.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    @staticmethod
    def _set_xyz(m, base_mm) -> None:
        m.pose.position.x = base_mm[0] / 1000.0
        m.pose.position.y = base_mm[1] / 1000.0
        m.pose.position.z = base_mm[2] / 1000.0

    @staticmethod
    def _set_color(m, rgba) -> None:
        m.color.r, m.color.g, m.color.b, m.color.a = rgba

    def _zone_marker(self):
        from visualization_msgs.msg import Marker
        m = self._new("zone", 0, Marker.CUBE)
        z = SUCCESS_ZONE_MM
        mid = {k: (v[0] + v[1]) / 2000.0 for k, v in z.items()}
        m.pose.position.x, m.pose.position.y, m.pose.position.z = mid["x"], mid["y"], mid["z"]
        m.scale.x = (z["x"][1] - z["x"][0]) / 1000.0
        m.scale.y = (z["y"][1] - z["y"][0]) / 1000.0
        m.scale.z = (z["z"][1] - z["z"][0]) / 1000.0
        self._set_color(m, _ZONE_COLOR)
        return m

    def _publish(self, markers) -> None:
        from visualization_msgs.msg import MarkerArray
        arr = MarkerArray()
        arr.markers = markers
        self._pub.publish(arr)

    # ---- 공개 API --------------------------------------------------------- #

    def clear(self) -> None:
        """전부 지운다. 라운드가 바뀌면 이전 검출이 남지 않도록."""
        from visualization_msgs.msg import Marker
        m = self._new("", 0, Marker.SPHERE)
        m.action = Marker.DELETEALL
        self._publish([m])

    def show_detections(self, batch: list) -> None:
        """이번 라운드의 (제외 필터를 통과한) 검출 목록을 그린다."""
        from visualization_msgs.msg import Marker
        self.clear()
        out = []
        if self._show_zone:
            out.append(self._zone_marker())
        for i, t in enumerate(batch):
            grade = t.get("grade", "NORMAL")
            s = self._new("tomatoes", i, Marker.SPHERE)
            self._set_xyz(s, t["base"])
            s.scale.x = s.scale.y = s.scale.z = TOMATO_D
            self._set_color(s, _COLOR.get(grade, _COLOR["NORMAL"]))
            out.append(s)

            # 파지 순서를 붙여 둔다 — 정렬 기준(base_x)이 눈에 보여야 진입 순서를
            # 검증할 수 있다. 열매 위 25mm.
            lb = self._new("labels", i, Marker.TEXT_VIEW_FACING)
            self._set_xyz(lb, [t["base"][0], t["base"][1], t["base"][2] + 25.0])
            lb.scale.z = 0.018
            self._set_color(lb, (1.0, 1.0, 1.0, 0.9))
            lb.text = f"{i + 1}. {grade[:1]}"
            out.append(lb)
        self._last_n = len(batch)
        self._publish(out)

    def show_idle(self) -> None:
        """대기 중에도 성공 대역만 계속 쏜다.

        수확이 돌 때만 발행하면 Goal 전에 rviz 가 완전히 비어 있어, '연결이 안 된
        건지 아직 안 보낸 건지' 구분이 안 된다(실제로 ROS_DOMAIN_ID 불일치를 빈 화면
        으로 오인한 적이 있다). 상자가 보이면 토픽·프레임·도메인이 다 맞았다는 뜻이다.
        """
        if self._show_zone:
            self._publish([self._zone_marker()])

    def show_target(self, base_mm, grade: str = "NORMAL") -> None:
        """지금 파지하러 가는 열매를 크게 강조한다."""
        from visualization_msgs.msg import Marker
        m = self._new("target", 0, Marker.SPHERE)
        self._set_xyz(m, base_mm)
        m.scale.x = m.scale.y = m.scale.z = TOMATO_D * 1.9
        self._set_color(m, _TARGET_COLOR)
        self._publish([m])

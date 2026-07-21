#!/usr/bin/env python3
"""RP-114  E0 ③④: 로봇별 텔레메트리 취합 — ACS 세 노드가 공유하는 순수 모듈.

로봇 구성이 물리망 분리로 바뀌면서, 로봇 3대분을 하나로 묶는 일이 DG 에서 ACS 로 넘어왔다
(DG 는 로봇 세트마다 하나씩 뜨므로 애초에 전체를 볼 수 없었다). ACS 는 로봇별로
/{robot_id}/telemetry 를 구독해 최신값을 캐시하고, 자체 1Hz 타이머로 배열을 만들어 발행한다.

이 모듈은 rclpy 를 import 하지 않는다 — ROS 노드 없이 단독으로 테스트할 수 있게 하려는 것.
구독을 만드는 subscribe_per_robot() 도 node 객체를 인자로 받아 그 메서드를 호출할 뿐이다.
로그도 두지 않는다(순수 계층). 흐름 로그는 이걸 쓰는 노드 쪽에서 남긴다.

세 노드가 각자 이 모듈을 써서 각자 구독한다(중계 노드를 두지 않는다):
  - patrol_node          가용 판정·DB 배정 스냅샷
  - telemetry_ws_node    웹서비스용 축약본 WebSocket 방송
  - fleet_telemetry_aggregator  QT 용 원본 취합 발행
ROS2 토픽은 다중 구독자를 전제로 한 브로드캐스트라 구독자가 늘어도 발행자 부담이 거의 없고,
서로 프로세스 의존을 만들지 않아 하나가 죽어도 나머지가 산다.
"""
import copy
import threading

from automato_interfaces.msg import FleetMember, FleetTelemetry, RobotTelemetry

# DG 한 세트 → ACS. 어느 로봇인지는 이 네임스페이스가 말해준다(메시지 안엔 robot_id 없음).
ROBOT_TELEMETRY_TOPIC_FMT = "/{robot_id}/telemetry"

# [삭제 예정] 재정의 이전 경로. DG(dg_control)가 아직 이 토픽에 로봇 3대분 배열을 발행한다.
# 팀원의 DG 이전이 끝나면 이 구독을 제거한다.
LEGACY_FLEET_TOPIC = "/automato/telemetry/fleet"

# 구독을 만들려면 로봇 목록이 먼저 있어야 한다(토픽 이름에 robot_id 가 들어가므로).
# DB(robots 테이블)가 단일 출처지만, 구독은 노드 기동 시 한 번만 만드는 정적인 배선이고
# 취합 노드에 DB 의존을 새로 만들지 않으려고 파라미터 기본값으로 둔다.
# 로봇이 늘거나 일부만 띄워 검증할 때는 robot_ids 파라미터로 덮어쓴다.
DEFAULT_ROBOT_IDS = ["dg_01", "dg_02", "dg_03"]


def robot_telemetry_topic(robot_id: str) -> str:
    """로봇 하나의 텔레메트리 토픽 이름."""
    return ROBOT_TELEMETRY_TOPIC_FMT.format(robot_id=robot_id)


class FleetCollector:
    """robot_id -> 그 로봇의 최신 RobotTelemetry 1건(수신마다 통째로 교체).

    ROS 구독 콜백(스레드)이 쓰고, 발행 타이머나 FastAPI 스레드가 읽으므로 락으로 보호한다.
    끊긴 로봇도 지우지 않는다 — 마지막 값이 남아 있어야 header.stamp 가 늙는 것으로
    '연결 끊김'이 드러난다. 배열에서 빼버리면 QT 화면에서 로봇이 깜빡이고
    '끊김'과 '존재하지 않음'을 구분할 수 없다(문서 E0 ④ 발행 규칙).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._robots = {}   # robot_id -> RobotTelemetry (받은 원본 그대로)

    # ------------------------------------------------------------------ #
    # 쓰기 — 새 경로 (/{robot_id}/telemetry)
    # ------------------------------------------------------------------ #
    def update(self, robot_id: str, msg: RobotTelemetry) -> None:
        """로봇 하나의 텔레메트리를 최신값으로 덮어쓴다.

        받은 메시지를 가공 없이 그대로 보관한다. 특히 각 ddago/ddagi 의 header.stamp 를
        건드리지 않는 게 중요하다 — 가용 판정의 '3초 미수신'은 로봇이 찍은 stamp 기준이라,
        중간 계층이 stamp 를 새로 찍으면 죽은 로봇이 영영 신선해 보인다.
        """
        with self._lock:
            self._robots[robot_id] = msg

    # ------------------------------------------------------------------ #
    # 쓰기 — [삭제 예정] 옛 경로 (/automato/telemetry/fleet)
    # ------------------------------------------------------------------ #
    def update_from_legacy_fleet(self, msg: FleetTelemetry) -> list:
        """옛 FleetTelemetry(로봇 3대분 배열) 1건을 로봇별로 갈라 저장한다.

        옛 구조에는 네임스페이스가 없어 로봇 구분이 payload 의 robot_id 뿐이다.
        그래서 여기서만은 msg.ddagos[].robot_id 를 보고 가른다.

        robot_id 가 빈 로봇은 건너뛴다(어느 로봇인지 알 수 없어 캐시 키를 만들 수 없다).
        반환값 = 건너뛴 개수를 호출부가 경고 로그로 남길 수 있도록 알려주는
        (skipped_ddago, skipped_ddagi) 튜플.
        """
        per_robot = {}      # robot_id -> RobotTelemetry(조립 중)
        skipped_ddago = 0
        skipped_ddagi = 0

        def bucket(robot_id):
            item = per_robot.get(robot_id)
            if item is None:
                item = RobotTelemetry()
                item.header = msg.header    # 취합 시각은 옛 메시지의 것을 그대로
                per_robot[robot_id] = item
            return item

        for d in msg.ddagos:
            if not d.robot_id:
                skipped_ddago += 1
                continue
            bucket(d.robot_id).ddagos.append(d)
        for a in msg.ddagis:
            if not a.robot_id:
                skipped_ddagi += 1
                continue
            bucket(a.robot_id).ddagis.append(a)

        with self._lock:
            self._robots.update(per_robot)
        return skipped_ddago, skipped_ddagi

    # ------------------------------------------------------------------ #
    # 읽기
    # ------------------------------------------------------------------ #
    def robot_ids(self) -> list:
        """지금까지 한 번이라도 텔레메트리를 받은 robot_id 목록(정렬).

        내부 dict 순회를 밖에 시키면 락 없이 돌게 되므로(RuntimeError: dict changed size)
        목록 뽑기는 여기서 락 안에 한다.
        """
        with self._lock:
            return sorted(self._robots.keys())

    def get(self, robot_id: str):
        """로봇 하나의 최신 RobotTelemetry(없으면 None). 원본 참조를 준다 — 읽기 전용."""
        with self._lock:
            return self._robots.get(robot_id)

    def snapshot(self) -> list:
        """(robot_id, RobotTelemetry) 목록을 robot_id 오름차순으로.

        락은 '목록만 만들고 즉시 반납'한다. 담기는 메시지는 수신할 때마다 통째로 교체되므로
        (필드를 하나씩 고쳐 쓰지 않는다) 락 밖에서 읽어도 '반쯤 바뀐' 값을 볼 일이 없다.
        """
        with self._lock:
            return sorted(self._robots.items(), key=lambda kv: kv[0])

    # ------------------------------------------------------------------ #
    # 발행용 메시지 조립
    # ------------------------------------------------------------------ #
    def build_fleet_message(self, stamp) -> FleetTelemetry:
        """QT 용 FleetTelemetry 한 건을 만든다(자체 타이머가 1Hz 로 호출).

        stamp : 이번 취합 시각(builtin_interfaces/Time). 노드가 get_clock().now().to_msg() 로 준다.
                개별 ddago/ddagi 의 header.stamp 는 로봇이 찍은 원본 그대로 둔다.

        robots[] 와 함께 [삭제 예정] 필드 ddagos[]/ddagis[] 도 채운다. QT(system_admin_app)가
        아직 옛 필드를 순회하고 있어, 그쪽 이전 전까지 화면이 살아 있게 하는 하위호환 장치다.

        옛 필드로 평탄화할 때 robot_id 를 ACS 가 채워 넣는다 — 로봇은 더 이상 robot_id 를
        채우지 않으므로(네임스페이스로 대체) 그대로 흘리면 QT 가 로봇을 구분하지 못한다.
        원본을 건드리지 않도록 복사본에 쓴다.
        """
        out = FleetTelemetry()
        out.header.stamp = stamp

        for robot_id, telemetry in self.snapshot():
            member = FleetMember()
            member.robot_id = robot_id
            member.telemetry = telemetry
            out.robots.append(member)

            # --- [삭제 예정] 아래 블록은 QT 이전이 끝나면 통째로 지운다 ---
            for d in telemetry.ddagos:
                legacy = copy.deepcopy(d)
                legacy.robot_id = robot_id
                out.ddagos.append(legacy)
            for a in telemetry.ddagis:
                legacy = copy.deepcopy(a)
                legacy.robot_id = robot_id
                out.ddagis.append(legacy)

        return out


# --------------------------------------------------------------------------- #
# 구독 배선 — 노드가 이 함수 하나만 부르면 로봇별 구독이 만들어진다.
# --------------------------------------------------------------------------- #
def subscribe_per_robot(node, robot_ids, on_telemetry, qos=10) -> list:
    """robot_ids 각각에 대해 /{robot_id}/telemetry 구독을 만든다.

    on_telemetry(robot_id, msg) 형태로 호출된다. ROS 구독 콜백은 원래 msg 하나만 받으므로,
    '어느 로봇의 콜백인지'를 함수에 미리 붙여둬야 한다.

    여기서 lambda 를 쓰면 안 된다 —
        for rid in robot_ids:
            node.create_subscription(..., lambda m: on_telemetry(rid, m), qos)
    파이썬 클로저는 값이 아니라 변수를 붙잡으므로, 콜백이 실제로 불릴 때 rid 는 이미
    루프의 마지막 값이다. 결국 모든 구독이 마지막 로봇 이름으로 보고한다(늦은 바인딩 함정).
    기본 인자로 값을 고정(rid=rid)하는 방법도 있지만, 의도가 드러나는 쪽을 골랐다.
    """
    from functools import partial

    subs = []
    for robot_id in robot_ids:
        subs.append(node.create_subscription(
            RobotTelemetry,
            robot_telemetry_topic(robot_id),
            partial(on_telemetry, robot_id),   # 첫 인자를 robot_id 로 고정
            qos,
        ))
    return subs

#!/usr/bin/env python3
"""RP-78 ④ 라우팅/예약 엔진 — 경로 탐색(Dijkstra)과 통로 예약을 담당하는 독립 부품.

이 모듈은 순찰 '전용'이 아니다. 수확·이송에서도 그대로 재사용할 수 있게 순찰 코드와
분리했다. 순찰 루프(patrol_node)는 이 엔진을 '호출만' 한다. 경로 탐색 알고리즘(현재
Dijkstra)을 바꿔도 이 파일 안에서만 바뀌도록 캡슐화한다.

ROS/DB 의존이 전혀 없다 → 그래프만 넣어 단위테스트할 수 있다(DoD 항목).

제공 기능:
  - find_path(start, goal, blocked)  : Dijkstra(통로 length 비용). blocked는 그래프에서
                                       제외. 경로 없으면 None.
  - try_reserve(cid, robot_id)       : 통로가 비었으면 잠그고 True, 남이 쓰면 False.
  - heartbeat(cid, robot_id)         : 보유 중 예약 시각 갱신(살아있음 표시).
  - release(cid, robot_id)           : 보유자만 해제.
  - reap_expired()                   : TTL 지난 예약 자동 해제(죽은 로봇의 통로 영구점유 방지).
  - reserved_corridors(exclude_robot): 지금 (남이) 잡고 있는 통로 id 집합.
  - would_deadlock(robot, cid)        : 이 통로를 기다리면 대기 사이클(데드락)이 생기는지.
  - begin_wait/end_wait(robot[, cid]) : 대기 그래프 갱신(누가 무엇을 기다리는지).
  - reserve_or_wait(cid, robot_id)    : '확인+획득+대기검사'를 원자적으로 → reserved/deadlock/waiting.
  - node_slot(n)                      : 노드 n 의 '자리'를 뜻하는 가상 통로 id(= -n).

예약 자원이 둘인 이유(노드 자리):
  로봇이 차지하는 공간은 '통로 위' 아니면 '노드 위'다. 통로만 예약하면 노드에 서 있는
  로봇이 교통관제에 투명인간이 되어, 남이 '통로는 다 비었으니 가도 된다'고 판단해 그
  자리로 들어온다(정점 충돌). 그래서 노드도 '길이 0짜리 가상 통로'로 예약 대상에 넣는다.
  예약표는 키의 의미를 모르므로 예약/TTL/하트비트/대기 사이클 검사가 전부 그대로 적용된다.

동시성:
  통로 예약표는 로봇 여러 대가 동시에 접근하는 공유 상태다. threading.Lock 으로 보호한다.
  (티켓은 'asyncio 락'을 예시로 들었지만, 본 서비스의 디스패치는 asyncio가 아니라
   '스레드'(로봇당 1 스레드)로 돌기 때문에 threading.Lock 이 맞다. 지켜야 할 불변식은
   '같은 통로를 두 로봇에 동시에 허락하지 않는다'이고, 그 임계구역을 락으로 감싼다.)

안전 속성(가장 중요):
  같은 자원(통로 또는 노드 자리)은 어느 순간에도 최대 한 로봇만 보유한다.
  - 통로 잠금 → 좁은 1차선에서 양끝 마주보기(head-on, 간선 충돌)를 원천 차단.
  - 노드 자리 잠금 → 같은 지점을 두 로봇이 동시에 차지(정점 충돌)하는 것을 차단.
"""
import heapq
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class Route:
    """탐색 결과 경로. nodes[0]=출발, nodes[-1]=목적. corridors[i]는 nodes[i]~nodes[i+1] 통로."""
    nodes: tuple
    corridors: tuple

    @property
    def is_trivial(self) -> bool:
        """출발==목적(이동 없음)."""
        return len(self.nodes) <= 1

    def hops(self) -> list:
        """[(다음_노드, 통로_id), ...] 형태의 세그먼트 목록. 디스패치 루프가 이대로 소비한다."""
        return list(zip(self.nodes[1:], self.corridors))


class RoutingEngine:
    def __init__(self, waypoints, corridors, *, reservation_ttl: float = 15.0,
                 time_fn=time.monotonic):
        """그래프를 메모리에 적재하고 예약표를 초기화한다.

        waypoints: 노드. int(id) 또는 {"waypoint_id": ...} dict 를 담은 iterable.
        corridors: 무방향 간선. {"corridor_id","a","b"[,"length"]} dict 또는
                   (cid, a, b[, length]) 튜플. length(간선 비용)는 선택 —
                   없으면 1.0으로 취급(= 홉 수, 사실상 BFS와 동일 결과).
        reservation_ttl: 이 시간(초) 넘게 하트비트가 없으면 죽은 예약으로 보고 회수.
        time_fn: 단위테스트에서 가짜 시계를 주입하기 위한 통로(기본 단조시계).
        """
        self._time = time_fn
        self._ttl = reservation_ttl

        self._adj = {}     # node -> list[(neighbor, corridor_id)]
        self._pair = {}    # frozenset({u, v}) -> corridor_id
        self._length = {}  # corridor_id -> length(간선 비용). 없으면 1.0
        self._nodes = set()

        for w in waypoints:
            nid = w["waypoint_id"] if isinstance(w, dict) else int(w)
            self._nodes.add(nid)
            self._adj.setdefault(nid, [])
        for c in corridors:
            if isinstance(c, dict):
                cid, a, b = c["corridor_id"], c["a"], c["b"]
                length = c.get("length")
            else:
                # (cid, a, b) 또는 (cid, a, b, length) 튜플 모두 허용
                cid, a, b = c[0], c[1], c[2]
                length = c[3] if len(c) > 3 else None
            self._nodes.update((a, b))
            self._adj.setdefault(a, []).append((b, cid))
            self._adj.setdefault(b, []).append((a, cid))
            self._pair[frozenset((a, b))] = cid
            # length 미제공 → 1.0(홉 수). Dijkstra 전환 후에도 안전한 기본값.
            self._length[cid] = float(length) if length is not None else 1.0

        # 노드도 '길이 0짜리 가상 통로'로 등록한다(자세한 이유는 node_slot 참고).
        # 로봇이 노드에 서 있는 동안 그 자리를 예약하게 해서, '통로는 비었는데 그 지점에
        # 남이 서 있다'는 상황을 예약표가 볼 수 있게 만드는 것이 목적이다.
        # ⚠️ _adj 에는 절대 넣지 않는다 — 가상 통로는 '지나다니는 길'이 아니라 예약 전용
        #    자원이라, find_path 가 이걸 간선으로 보면 없는 길로 경로를 만들어 버린다.
        if 0 in self._nodes:
            # node_slot(0) == 0 이라 corridor_id 0 과 충돌한다. waypoint_id 는 1부터라
            # 실제로 걸릴 일은 없지만, 걸리면 조용히 남의 예약을 덮어쓰므로 즉시 터뜨린다.
            raise ValueError("waypoint_id 0 은 지원하지 않는다(가상 통로 id 와 충돌)")
        for nid in self._nodes:
            self._length[self.node_slot(nid)] = 0.0

        self._reservations = {}   # 자원 id -> (robot_id, last_heartbeat_ts)
                                  # 자원 id: 양수=실제 통로, 음수=노드 자리(node_slot)
        self._waiting = {}        # robot_id -> corridor_id (지금 이 통로를 기다리는 중)
        self._lock = threading.Lock()

    # --------------------------- 그래프 조회 --------------------------- #
    def corridor_between(self, u, v):
        """두 노드가 직접 인접하면 그 통로 id, 아니면 None. (정상 순찰: 인접 지점 직행)"""
        return self._pair.get(frozenset((u, v)))

    def neighbors(self, node) -> list:
        return list(self._adj.get(node, []))

    def find_path(self, start, goal, blocked=frozenset(), blocked_nodes=frozenset()):
        """start→goal 최소 '누적 거리' 경로를 Dijkstra로 찾는다. 없으면 None.

        비용은 통로 length(간선 비용). length 미제공 통로는 1.0 → 사실상 홉 수(BFS와 동일).
        blocked: 제외할 통로 id 집합(막힘 블랙리스트/우회 대상). 이 통로들은 없는 셈 친다.
        blocked_nodes: 지나갈 수 없는 노드 집합(남이 그 자리를 점유 중 등). 그 노드로 들어가는
            길을 전부 없는 셈 쳐서 '지점 하나를 통째로 회피'한다. 통로 blocked 로는 이걸
            표현할 수 없다 — 한 노드에 통로가 여러 개 붙어 있어 하나씩 막아봐야 다른 통로로
            같은 노드에 또 들어가기 때문. start 는 검사하지 않는다(내가 서 있는 자리라
            막혔더라도 거기서 출발해야 한다). goal 이 막혀 있으면 자연히 None 이 된다.
        순찰도(문서 3장) 다음 순찰 포인트까지 이 탐색을 쓴다 — 중간 일반 waypoint를 지나야
        하고, 막히면 우회로가 필요하기 때문. (언제 호출할지는 patrol_node 의 몫)
        """
        if start == goal:
            return Route((start,), ())
        if start not in self._adj or goal not in self._adj:
            return None

        INF = float("inf")
        dist = {start: 0.0}            # node -> 지금까지 알아낸 최소 누적거리
        prev = {start: (None, None)}   # node -> (이전 node, 사용한 corridor)
        heap = [(0.0, start)]          # (누적거리, node) 최소 힙
        while heap:
            d, u = heapq.heappop(heap)
            if u == goal:
                break                  # 목적지를 최소거리로 꺼낸 순간 → 확정
            if d > dist.get(u, INF):
                continue               # 힙에 남은 낡은(이미 더 짧게 갱신된) 항목 → 버림
            for v, cid in self._adj[u]:
                if cid in blocked or v in blocked_nodes:
                    continue           # 막힌 통로/막힌 지점으로 가는 길은 없는 셈 친다
                nd = d + self._length[cid]
                if nd < dist.get(v, INF):   # 더 싼 경로 발견 → 갱신
                    dist[v] = nd
                    prev[v] = (u, cid)
                    heapq.heappush(heap, (nd, v))

        if goal not in prev:
            return None

        # prev 를 goal 부터 거슬러 올라가 경로 복원 (BFS 때와 동일)
        nodes = [goal]
        corridors = []
        cur = goal
        while prev[cur][0] is not None:
            p, cid = prev[cur]
            corridors.append(cid)
            nodes.append(p)
            cur = p
        nodes.reverse()
        corridors.reverse()
        return Route(tuple(nodes), tuple(corridors))

    # --------------------------- 노드(자리) 예약 --------------------------- #
    @staticmethod
    def node_slot(node_id) -> int:
        """노드 n 이 차지하는 '자리'를 가리키는 가상 통로 id(= -n).

        예약표(_reservations)는 키가 무엇을 뜻하는지 모른다 — 그냥 '자원 id → 보유자'다.
        그래서 노드에 고유한 id 하나만 배정해 주면 통로용 예약 코드(try_reserve/release/
        heartbeat/TTL/대기 사이클 검사)를 한 줄도 안 고치고 그대로 재사용할 수 있다.
        실제 corridor_id 는 DB PK라 항상 양수 → 부호만 뒤집으면 절대 겹치지 않는다.
        로그의 '통로 -7' 은 '노드 7 자리'로 바로 읽힌다.

        규칙을 이 함수 한 곳에만 두는 게 요점이다. 부호 뒤집기를 호출부마다 흩어 쓰면
        하나만 빠뜨려도 조용히 엉뚱한 자원을 예약하게 된다.
        """
        return -node_id

    @staticmethod
    def is_node_slot(corridor_id) -> bool:
        """이 자원 id 가 실제 통로가 아니라 노드 자리인가(로그·관측 도구용)."""
        return corridor_id < 0

    @staticmethod
    def node_of_slot(corridor_id):
        """node_slot 의 역변환(자리 id → 노드 id). '-' 규칙을 호출부로 새지 않게 한다."""
        return -corridor_id

    # --------------------------- 통로 예약 --------------------------- #
    def _try_reserve_locked(self, corridor_id, robot_id, now) -> bool:
        """try_reserve 의 락 없는 본체(이미 _lock 보유 중일 때 재사용).

        빈 통로 / 내 것(=하트비트 갱신) / TTL 지난 죽은 예약이면 잠그고 True,
        남이 유효 보유 중이면 False. reserve_or_wait 도 이 판정을 공유한다.
        """
        holder = self._reservations.get(corridor_id)
        if (holder is None                                  # 빈 통로
                or holder[0] == robot_id                    # 내 것(하트비트)
                or (self._ttl is not None                   # 죽은 예약 회수
                    and (now - holder[1]) > self._ttl)):
            self._reservations[corridor_id] = (robot_id, now)
            return True
        return False

    def try_reserve(self, corridor_id, robot_id) -> bool:
        """비었으면(또는 TTL 지난 죽은 예약이면) 잠그고 True. 남이 유효 보유 중이면 False.

        같은 robot_id가 이미 보유 중이면 하트비트만 갱신하고 True(멱등).
        """
        with self._lock:
            return self._try_reserve_locked(corridor_id, robot_id, self._time())

    def heartbeat(self, corridor_id, robot_id) -> bool:
        """보유 중인 통로의 예약 시각을 갱신(이동이 오래 걸릴 때 예약 유지). 보유자만 유효."""
        with self._lock:
            holder = self._reservations.get(corridor_id)
            if holder and holder[0] == robot_id:
                self._reservations[corridor_id] = (robot_id, self._time())
                return True
            return False

    def release(self, corridor_id, robot_id) -> bool:
        """보유자만 해제(남의 예약을 실수로 지우지 않음). 도착/실패/중단 모두에서 호출한다."""
        with self._lock:
            holder = self._reservations.get(corridor_id)
            if holder and holder[0] == robot_id:
                del self._reservations[corridor_id]
                return True
            return False

    def reap_expired(self) -> list:
        """TTL 지난 예약을 모두 해제하고, 해제된 통로 id 목록을 돌려준다(주기 청소용)."""
        with self._lock:
            now = self._time()
            expired = [
                cid for cid, (rid, ts) in self._reservations.items()
                if self._ttl is not None and (now - ts) > self._ttl
            ]
            for cid in expired:
                del self._reservations[cid]
            return expired

    def reserved_corridors(self, exclude_robot=None) -> set:
        """지금 예약된 통로 id 집합. exclude_robot 의 것은 뺀다(내 예약은 우회 대상 아님)."""
        with self._lock:
            return {
                cid for cid, (rid, ts) in self._reservations.items()
                if rid != exclude_robot
            }

    def reservation_snapshot(self) -> dict:
        """예약표 전체를 한 락 안에서 통째로 덤프한다(관측 도구용, 읽기 전용).

        reserved_corridors() + holder_of() 2단계로 물으면 그 사이에 해제된 항목이
        빠져 화면이 한 틱 어긋난다. 또 관측하는 쪽마다 '음수면 자리'라는 규칙을 다시
        구현하게 되므로, 원자적 덤프와 분류를 엔진이 한 번에 책임진다.
        TTL 지난 죽은 예약은 빼고 준다 — 주기 회수(reap_expired)가 아직 안 돈 사이에
        관측만 하면 죽은 로봇이 살아있는 것처럼 보이기 때문.
        반환: {"corridors": {통로id: robot}, "nodes": {노드id: robot}}
        """
        with self._lock:
            now = self._time()
            corridors, nodes = {}, {}
            for cid, (rid, ts) in self._reservations.items():
                if self._ttl is not None and (now - ts) > self._ttl:
                    continue                       # 죽은 예약은 관측에도 안 보인다
                if self.is_node_slot(cid):
                    nodes[self.node_of_slot(cid)] = rid
                else:
                    corridors[cid] = rid
            return {"corridors": corridors, "nodes": nodes}

    def holder_of(self, corridor_id):
        """해당 통로의 현재 보유 robot_id(없으면 None). 주로 테스트/디버깅용."""
        with self._lock:
            holder = self._reservations.get(corridor_id)
            return holder[0] if holder else None

    # --------------------------- 데드락(대기 사이클) 회피 --------------------------- #
    def begin_wait(self, robot_id, corridor_id) -> None:
        """robot_id 가 corridor_id 를 기다리기 시작했음을 기록(대기 그래프 갱신)."""
        with self._lock:
            self._waiting[robot_id] = corridor_id

    def end_wait(self, robot_id) -> None:
        """robot_id 의 대기 상태 해제(예약 성공/포기 시 호출)."""
        with self._lock:
            self._waiting.pop(robot_id, None)

    def would_deadlock(self, robot_id, corridor_id) -> bool:
        """robot_id 가 corridor_id 를 기다리기 시작하면 대기 사이클(데드락)이 생기는가.

        대기 그래프: 노드=로봇, '기다림' 화살표는 로봇당 최대 1개(_waiting).
        corridor_id 를 쥔 로봇부터 '그 로봇이 기다리는 통로 → 그 통로를 쥔 로봇'
        사슬을 따라가다, robot_id 로 되돌아오면 사이클(True). 안 기다리는(또는 빈)
        로봇에 닿으면 안전(False).
        """
        with self._lock:
            return self._would_deadlock_locked(robot_id, corridor_id)

    def _would_deadlock_locked(self, robot_id, corridor_id) -> bool:
        """would_deadlock 의 락 없는 본체(이미 _lock 보유 중일 때 재사용).

        ⑤에서 '확인+획득'과 한 원자 블록으로 묶을 때 이 버전을 호출한다.
        threading.Lock 은 재진입 불가라, 락을 잡은 채 이걸 부른다.
        """
        holder = self._reservations.get(corridor_id)
        if holder is None:
            return False                       # 비어 있으면 기다릴 일이 없다
        cur = holder[0]                        # corridor_id 를 지금 쥔 로봇
        # 사슬 길이 상한 = 예약 수 + 1 (무한루프 방지; 사이클은 그 안에 닫힌다)
        for _ in range(len(self._reservations) + 1):
            if cur == robot_id:
                return True                    # 나에게로 돌아옴 → 사이클
            waited = self._waiting.get(cur)    # cur 가 기다리는 통로(없으면 안 기다림)
            if waited is None:
                return False                   # 안 기다림 → 언젠가 풀림 → 안전
            nxt = self._reservations.get(waited)
            if nxt is None:
                return False                   # 기다리는 통로가 비었으면 곧 잡음 → 안전
            cur = nxt[0]
        return True                            # 상한 초과(비정상) → 보수적으로 사이클 취급

    def reserve_or_wait(self, corridor_id, robot_id) -> str:
        """'확인+획득+대기검사'를 한 락 안에서 원자적으로 처리한다(문서 4-2/4-3).

        try_reserve→would_deadlock→begin_wait 를 따로 부르면 그 틈에 다른 로봇이 상태를
        바꿔 검사가 어긋난다(TOCTOU). 그래서 하나의 임계구역으로 묶는다. 반환:
          "reserved" — 통로 확보(빈 통로/내 것/죽은 예약 회수). 이 로봇의 대기는 해제.
          "deadlock" — 남이 쥐었고 지금 기다리면 대기 사이클 → 기다리지 말고 우회하라.
          "waiting"  — 남이 쥐었지만 안전하게 대기 가능. 대기 그래프에 등록.
        """
        with self._lock:
            if self._try_reserve_locked(corridor_id, robot_id, self._time()):
                self._waiting.pop(robot_id, None)   # 잡았으니 대기 해제
                return "reserved"
            if self._would_deadlock_locked(robot_id, corridor_id):
                self._waiting.pop(robot_id, None)   # 우회할 것이므로 대기 안 함
                return "deadlock"
            self._waiting[robot_id] = corridor_id   # 안전 → 대기 등록
            return "waiting"

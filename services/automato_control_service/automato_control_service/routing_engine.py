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

동시성:
  통로 예약표는 로봇 여러 대가 동시에 접근하는 공유 상태다. threading.Lock 으로 보호한다.
  (티켓은 'asyncio 락'을 예시로 들었지만, 본 서비스의 디스패치는 asyncio가 아니라
   '스레드'(로봇당 1 스레드)로 돌기 때문에 threading.Lock 이 맞다. 지켜야 할 불변식은
   '같은 통로를 두 로봇에 동시에 허락하지 않는다'이고, 그 임계구역을 락으로 감싼다.)

안전 속성(가장 중요):
  같은 통로(corridor)는 어느 순간에도 최대 한 로봇만 보유한다.
  좁은 1차선 통로에서 양끝 마주보기(head-on)를 '통로 전체 잠금'으로 원천 차단한다.
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

        self._reservations = {}   # corridor_id -> (robot_id, last_heartbeat_ts)
        self._lock = threading.Lock()

    # --------------------------- 그래프 조회 --------------------------- #
    def corridor_between(self, u, v):
        """두 노드가 직접 인접하면 그 통로 id, 아니면 None. (정상 순찰: 인접 지점 직행)"""
        return self._pair.get(frozenset((u, v)))

    def neighbors(self, node) -> list:
        return list(self._adj.get(node, []))

    def find_path(self, start, goal, blocked=frozenset()):
        """start→goal 최소 '누적 거리' 경로를 Dijkstra로 찾는다. 없으면 None.

        비용은 통로 length(간선 비용). length 미제공 통로는 1.0 → 사실상 홉 수(BFS와 동일).
        blocked: 제외할 통로 id 집합(막힘 블랙리스트/우회 대상). 이 통로들은 없는 셈 친다.
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
                if cid in blocked:
                    continue           # 막힌/예약된 통로는 없는 셈 친다
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

    # --------------------------- 통로 예약 --------------------------- #
    def try_reserve(self, corridor_id, robot_id) -> bool:
        """비었으면(또는 TTL 지난 죽은 예약이면) 잠그고 True. 남이 유효 보유 중이면 False.

        같은 robot_id가 이미 보유 중이면 하트비트만 갱신하고 True(멱등).
        """
        with self._lock:
            now = self._time()
            holder = self._reservations.get(corridor_id)
            if holder is None:
                self._reservations[corridor_id] = (robot_id, now)
                return True
            hid, ts = holder
            if hid == robot_id:
                self._reservations[corridor_id] = (robot_id, now)   # 하트비트 갱신
                return True
            if self._ttl is not None and (now - ts) > self._ttl:
                # 보유자가 하트비트를 멈춘 지 오래 → 죽은 예약으로 보고 회수
                self._reservations[corridor_id] = (robot_id, now)
                return True
            return False

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

    def holder_of(self, corridor_id):
        """해당 통로의 현재 보유 robot_id(없으면 None). 주로 테스트/디버깅용."""
        with self._lock:
            holder = self._reservations.get(corridor_id)
            return holder[0] if holder else None

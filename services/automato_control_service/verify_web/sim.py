#!/usr/bin/env python3
"""검증 웹 — 시뮬 세계. '진짜' 엔진·디스패처를 가짜 로봇으로 굴리고 상태를 관찰한다.

여기가 4단계의 본체다. 구성은 patrol_node.py 와 의도적으로 똑같다:
  - RoutingEngine   ← 짝(pair)을 뺀 노드 + corridors 로 구성 (진짜 코드, 무수정)
  - PatrolDispatcher← wp_meta 는 짝까지 전부, pair_of 맵 주입 (진짜 코드, 무수정)
  - client          ← 여기만 가짜(FakeNavigateClient)

관측(observability)이 이 파일의 존재 이유다. 검증 대상 코드는 상태를 밖으로 내보내는
창이 없다 — 예약표는 RoutingEngine 안의 dict 이고, 주행 상태는 _navigate() 의 지역변수다.
그래서 '이미 공개돼 있는 것만' 써서 밖에서 들여다본다:
  · 예약표   : engine.holder_of(cid) 를 통로마다 호출 (공개 API. 25통로×10Hz=250회/초라 무해)
  · 블랙리스트: dispatcher._blacklist_active() — 밑줄 이름이지만 '읽기 전용 관찰'로만 쓴다.
                검증 대상 코드를 고치지 않는 게 이 도구의 제1원칙이라 이쪽을 택했다.
  · 로봇 위치 : FakeRobot (가짜 로봇이 위치의 소스오브트루스)
  · 판단 근거 : 디스패처가 남기는 로그를 EventLog 로 받아 화면에 그대로 흘린다.
"""
import threading
import time
from collections import deque

from automato_control_service import automato_db
from automato_control_service.patrol_config import (
    REAP_INTERVAL_SEC,
    RESERVATION_TTL_SEC,
)
from automato_control_service.patrol_dispatcher import PatrolDispatcher
from automato_control_service.routing_engine import RoutingEngine

from fake_navigate import FakeNavigateClient, FakeRobot

# 순찰 대상 조회 — automato_db 의 것과 같은 조건(짝 제외)이지만, 여기서는 task 를
# 만들지 않고 '읽기만' 해야 하므로 accept_patrol_task 를 부르지 않고 직접 조회한다.
_SELECT_TARGETS = (
    "SELECT waypoint_id FROM waypoints "
    " WHERE is_patrol_point = TRUE AND pair_waypoint_id IS NULL "
    " ORDER BY patrol_order"
)


class EventLog:
    """PatrolDispatcher 에 주입하는 로거 겸, 화면으로 흘려보낼 이벤트 버퍼.

    디스패처는 '왜 그렇게 판단했는지'를 전부 로그로 남긴다(세그먼트 하달, 조기 반납,
    막힘 우회, 양보...). 그걸 화면에 그대로 띄우면 맵의 색 변화와 이유가 짝지어진다.

    DEBUG 는 담지 않는다: 하트비트마다 찍히는 줄이라 로봇 3대면 초당 여러 건씩 쌓여
    정작 중요한 판단 로그를 밀어낸다. 예약 상태는 이미 맵 색으로 보이므로 손해가 없다.
    """

    def __init__(self, maxlen: int = 500):
        self._dq = deque(maxlen=maxlen)
        self._lock = threading.Lock()
        self._seq = 0
        self._t0 = time.monotonic()

    def _add(self, level: str, msg: str) -> None:
        with self._lock:
            self._seq += 1
            self._dq.append({
                "seq": self._seq,
                "t": round(time.monotonic() - self._t0, 2),
                "level": level,
                "msg": str(msg),
            })

    def info(self, m): self._add("INFO", m)
    def warn(self, m): self._add("WARN", m)
    def error(self, m): self._add("WARN", m)
    def debug(self, m): pass          # 위 설명 참고 — 일부러 버린다

    def since(self, seq: int) -> list:
        """seq 이후에 생긴 이벤트만. 브로드캐스트가 중복 전송을 피하는 데 쓴다."""
        with self._lock:
            return [e for e in self._dq if e["seq"] > seq]

    def last_seq(self) -> int:
        with self._lock:
            return self._seq


class VerifySim:
    """가짜 로봇 위에서 진짜 교통관제를 돌리는 시뮬 세계(프로세스당 1개)."""

    def __init__(self, pool, *, speed_mps: float = 0.06, spin_rps: float = 0.9):
        graph = automato_db.load_graph(pool)

        # patrol_node.py 와 동일: 짝은 라우팅 그래프에서 뺀다(통로가 없어 고립 노드가 된다).
        routing_nodes = [w for w in graph["waypoints"] if w["pair_of"] is None]
        self.engine = RoutingEngine(
            routing_nodes, graph["corridors"], reservation_ttl=RESERVATION_TTL_SEC)

        self.events = EventLog()
        self.dispatcher = PatrolDispatcher(self.events)
        # wp_meta 는 짝까지 전부 — 짝을 하달하려면 그 좌표와 yaw 가 필요하다.
        self.dispatcher.wp_meta = {
            w["waypoint_id"]: {"x": w["x"], "y": w["y"], "yaw": w["yaw"],
                               "capture": w["is_patrol_point"]}
            for w in graph["waypoints"]
        }
        self.dispatcher.pair_of = {
            w["pair_of"]: w["waypoint_id"]
            for w in graph["waypoints"] if w["pair_of"] is not None
        }

        self.corridor_ids = [c["corridor_id"] for c in graph["corridors"]]
        # (a,b) -> corridor_id. 가짜 로봇이 "지금 지나는 구간이 막혔나"를 물을 때 쓴다.
        self._edge = {}
        for c in graph["corridors"]:
            self._edge[frozenset((c["a"], c["b"]))] = c["corridor_id"]

        with pool.connection() as conn:
            self.targets = [{"waypoint_id": r["waypoint_id"]}
                            for r in conn.execute(_SELECT_TARGETS).fetchall()]

        self.speed_mps = speed_mps
        self.spin_rps = spin_rps
        self._t0 = time.monotonic()
        self._task_seq = 0
        self._lock = threading.Lock()
        self._robots = {}          # robot_id -> dict(robot, client, status, task_id, thread)
        # 사용자가 화면에서 '막았다'고 표시한 통로(진짜 막힘 시뮬). 5단계 조작 UI 가 채운다.
        self._blocked_corridors = set()

        # DB 에 등록된 로봇을 전용 충전소 노드에 세워 둔다.
        for rid, wp in self._charge_nodes(pool).items():
            self.add_robot(rid, wp)

        # 죽은 예약 주기 회수. 실 ACS 는 patrol_node 의 ROS 타이머가 같은 일을 하는데,
        # 엔진 인스턴스를 소유한 주체가 서로 다르므로(저쪽은 ACS, 여기는 이 시뮬)
        # 각자 자기 엔진을 청소해야 한다.
        self._reap_stop = threading.Event()
        self._reap_thread = threading.Thread(
            target=self._reap_loop, name="reap", daemon=True)
        self._reap_thread.start()

    def _heartbeat_standing(self) -> None:
        """서 있는 로봇들의 '자리' 예약을 살려둔다.

        실 ACS 에서는 주행 스레드가 자기 예약을 주기적으로 하트비트한다. 그런데 시뮬에서
        세워만 둔 로봇(초기 배치·작업 끝난 로봇)은 도는 스레드가 없어 아무도 갱신하지
        않는다 → TTL 이 지나면 '가만히 서 있는데 자리가 회수되는' 상태가 되고, 실제로
        그 틈으로 다른 로봇이 그 지점을 관통했다.
        로봇이 살아 있다는 사실 자체가 하트비트의 근거이므로 여기서 대신 찍는다.
        heartbeat 은 보유자만 갱신하므로 남의 자리를 건드리지 않는다(멱등·안전).
        """
        with self._lock:
            standing = [(rid, e["robot"].snapshot().get("waypoint_id"))
                        for rid, e in self._robots.items()]
        for rid, wp in standing:
            if wp is not None:
                self.engine.heartbeat(self.engine.node_slot(wp), rid)

    def _reap_loop(self) -> None:
        """REAP_INTERVAL_SEC 마다 살아있는 자리를 갱신하고 죽은 예약을 회수한다.

        wait() 가 True 를 돌려주면 중지 신호이므로 루프를 끝낸다(sleep 보다 즉시 반응).
        """
        while not self._reap_stop.wait(REAP_INTERVAL_SEC):
            # 순서가 중요하다 — 산 로봇의 자리를 먼저 살려놔야 회수가 오판하지 않는다.
            self._heartbeat_standing()
            reaped = self.engine.reap_expired()
            if reaped:
                self.events.warn(f"죽은 예약 회수(하트비트 끊김): {reaped}")

    # ------------------------------------------------------------------ #
    def _charge_nodes(self, pool) -> dict:
        """robot_id -> 전용 충전소 진입 노드. ACS 와 같은 FK 사슬을 쓴다."""
        out = {}
        with pool.connection() as conn:
            rows = conn.execute("SELECT robot_id FROM robots ORDER BY robot_id").fetchall()
        for r in rows:
            wp = automato_db.get_patrol_start_waypoint(pool, r["robot_id"])
            if wp is not None:
                out[r["robot_id"]] = wp
        return out

    def _claim_slot(self, robot_id: str, waypoint_id: int, prev_wp=None) -> None:
        """로봇이 '서 있는 자리'를 예약표에 반영한다(옛 자리는 반납).

        왜 필요한가: 실 ACS 는 run_patrol 이 출발 지점 자리를 잡고 구간마다 넘겨준다.
        그런데 시뮬에서 세워만 둔 로봇(초기 배치·작업 끝난 로봇)은 그 경로를 안 타므로
        예약표에 안 나타난다 → 가만히 있는 로봇이 교통관제에 투명인간이 되어 남이 그
        지점을 관통한다. 그러면 이 도구가 잡아내려는 결함을 시뮬이 재현조차 못 한다.

        ⚠️ 실물과의 차이: 실 ACS 는 순찰이 끝나면 자리를 반납한다(충전소 복귀 로직이
        아직 없어서다). 여기서는 '로봇은 늘 어딘가에 서 있다'는 물리적 사실을 그대로
        모델링해 세워둔 로봇도 자리를 쥐게 한다. 복귀 로직이 붙으면 양쪽이 같아진다.
        """
        if prev_wp is not None and prev_wp != waypoint_id:
            self.engine.release(self.engine.node_slot(prev_wp), robot_id)
        slot = self.engine.node_slot(waypoint_id)
        if not self.engine.try_reserve(slot, robot_id):
            self.events.warn(
                f"{robot_id} 를 지점 {waypoint_id} 에 세웠지만 그 자리는 "
                f"{self.engine.holder_of(slot)} 가 쥐고 있다(겹침)")

    def add_robot(self, robot_id: str, start_wp: int) -> None:
        meta = self.dispatcher.wp_meta[start_wp]
        robot = FakeRobot(robot_id, meta["x"], meta["y"], meta["yaw"] or 0.0,
                          waypoint_id=start_wp)
        client = FakeNavigateClient(
            robot, self.dispatcher.wp_meta,
            speed_mps=self.speed_mps, spin_rps=self.spin_rps,
            is_edge_blocked=self._is_edge_blocked, logger=self.events)
        with self._lock:
            self._robots[robot_id] = {
                "robot": robot, "client": client, "start_wp": start_wp,
                "status": "IDLE", "task_id": None, "result": None, "thread": None,
                "kind": None,      # 진행 중인 작업 종류 PATROL | MOVE
            }
        self._claim_slot(robot_id, start_wp)    # 충전소에 서 있는 것도 자리 점유다

    def _is_edge_blocked(self, a, b) -> bool:
        """가짜 로봇이 주행 직전에 묻는다: 이 구간 막혔나? (사용자가 화면에서 막은 통로)"""
        cid = self._edge.get(frozenset((a, b)))
        if cid is None:
            return False
        with self._lock:
            return cid in self._blocked_corridors

    # ------------------------------------------------------------------ #
    def set_corridor_blocked(self, corridor_id: int, blocked: bool) -> None:
        """통로를 '진짜 막힘'으로 만들거나 푼다. 로봇이 그 구간에서 result_code=1 을 보고한다."""
        with self._lock:
            if blocked:
                self._blocked_corridors.add(corridor_id)
            else:
                self._blocked_corridors.discard(corridor_id)
        self.events.info(
            f"[조작] 통로 {corridor_id} {'막음' if blocked else '해제'}")

    def _run_task(self, robot_id: str, targets, kind: str) -> dict:
        """로봇 하나에 작업을 시킨다. 로봇당 스레드 1개 — ACS 의 구조와 같다.

        kind="PATROL" 은 전역 1건만 허용한다. 이 도구는 tasks 테이블을 쓰지 않아
        DB 의 부분 유니크 인덱스(ux_tasks_single_active_patrol)가 발동하지 않으므로,
        같은 정책을 메모리에서 그대로 흉내 낸다 — 화면이 실제 운영 모델과 어긋나지 않게.
        kind="MOVE" 는 제한이 없다(수확·이송처럼 순찰과 별개인 작업에 해당).
        """
        with self._lock:
            entry = self._robots.get(robot_id)
            if entry is None:
                return {"ok": False, "reason": "UNKNOWN_ROBOT"}
            if entry["status"] == "RUNNING":
                return {"ok": False, "reason": "ALREADY_RUNNING"}
            if kind == "PATROL" and any(
                    e["status"] == "RUNNING" and e["kind"] == "PATROL"
                    for e in self._robots.values()):
                return {"ok": False, "reason": "PATROL_IN_PROGRESS"}
            self._task_seq += 1
            task_id = self._task_seq
            # 출발 노드는 '지금 서 있는 노드'다. 직전 작업이 끝난 자리에서 이어가야
            # 경로가 실제와 맞는다(항상 충전소에서 시작한다고 보면 순간이동이 된다).
            start_wp = entry["robot"].snapshot()["waypoint_id"] or entry["start_wp"]
            entry.update(status="RUNNING", task_id=task_id, result=None, kind=kind)
            engine, dispatcher = self.engine, self.dispatcher
            client = entry["client"]

        def run():
            try:
                # run_patrol 은 (status, 미방문목록) 을 돌려준다. 검증 화면은 status 만 쓴다.
                result, _unvisited = dispatcher.run_patrol(
                    task_id, robot_id, targets, engine, client, start_wp=start_wp)
            except Exception as exc:  # noqa: BLE001
                self.events.warn(f"{robot_id} 작업 예외: {exc}")
                result = "FAILED"
            with self._lock:
                self._robots[robot_id].update(status="DONE", result=result)
                # run_patrol 은 끝나면서 마지막 자리를 반납한다(실 ACS 동작). 하지만
                # 로봇은 여전히 거기 서 있으므로 시뮬은 그 자리를 도로 쥔다 — 안 그러면
                # 작업 끝난 로봇을 남이 관통하는, 재현하려는 결함 그 자체가 된다.
                wp = self._robots[robot_id]["robot"].snapshot().get("waypoint_id")
                if wp is not None:
                    self._claim_slot(robot_id, wp)
            self.events.info(f"{robot_id} {kind} 종료 → {result}")

        t = threading.Thread(target=run, name=f"{kind}-{robot_id}", daemon=True)
        with self._lock:
            self._robots[robot_id]["thread"] = t
        t.start()
        return {"ok": True, "task_id": task_id, "kind": kind}

    def start_patrol(self, robot_id: str) -> dict:
        """전체 순찰(순찰점 전부 방문). 전역 1건 제한을 받는다."""
        return self._run_task(robot_id, self.targets, "PATROL")

    def goto(self, robot_id: str, waypoint_id: int) -> dict:
        """한 지점으로만 이동(수확·이송에 해당하는 '다른 작업'). 순찰 제한과 무관."""
        return self._run_task(
            robot_id, [{"waypoint_id": int(waypoint_id)}], "MOVE")

    def place(self, robot_id: str, waypoint_id: int) -> dict:
        """로봇을 특정 노드에 즉시 세운다 — 시나리오의 '초기 배치'용(주행이 아니다).

        데드락처럼 특정 초기 조건이 필요한 장면을 만들 때 쓴다. 판정 로직은 전혀
        건드리지 않고 출발 위치만 정하는 것이라, 검증의 진실성에는 영향이 없다.
        """
        with self._lock:
            entry = self._robots.get(robot_id)
            if entry is None:
                return {"ok": False, "reason": "UNKNOWN_ROBOT"}
            if entry["status"] == "RUNNING":
                return {"ok": False, "reason": "ALREADY_RUNNING"}
            meta = self.dispatcher.wp_meta.get(int(waypoint_id))
            if meta is None:
                return {"ok": False, "reason": "UNKNOWN_WAYPOINT"}
            prev_wp = entry["robot"].snapshot().get("waypoint_id")
            entry["robot"].set_pose(meta["x"], meta["y"], yaw=meta["yaw"] or 0.0,
                                    waypoint_id=int(waypoint_id), moving=False,
                                    spinning=False)
            self._claim_slot(robot_id, int(waypoint_id), prev_wp=prev_wp)
        return {"ok": True, "waypoint_id": int(waypoint_id)}

    # ---------------------- 시나리오 ---------------------- #
    # 데드락(대기 사이클)을 만드는 사슬. 통로 3개짜리 일직선이다.
    #   15 -[12-15]- 12 -[9-12]- 9 -[4-9]- 4
    DEADLOCK_CHAIN = (15, 12, 9, 4)
    DEADLOCK_A = "dg_01"        # 15 쪽에서 출발
    DEADLOCK_B = "dg_03"        # 4 쪽에서 출발

    def scenario_deadlock(self) -> dict:
        """두 로봇이 서로의 통로를 기다리는 '대기 사이클'을 확실하게 만든다.

        왜 '동시 출발'로는 안 되나:
          _acquire_segment 는 잡히는 만큼 탐욕적으로 잡는다. 예약 3개를 잡는 데 몇 µs
          밖에 안 걸리는데 스레드 시작 지터는 그보다 훨씬 커서, 먼저 깬 쪽이 사슬 전체를
          가져가 버린다. 그러면 다른 쪽은 그냥 기다릴 뿐 — 사이클이 안 생긴다.

        그래서 초기 조건을 명시적으로 만든다:
          ① B 를 사슬 끝(4)에 세우고, B 가 '이미 마지막 통로에 들어서 있는' 상태로 둔다
             (엔진 공개 API try_reserve 로 예약만 선점. 곧바로 B 가 그 통로를 실제 주행한다).
          ② A 를 15 에서 출발시키면 앞 통로 2개만 쥐고 마지막 통로에서 막혀 9 에 선다.
          ③ B 를 출발시키면 자기 통로 1개만 쥐고 가운데 통로에서 막혀 9 로 향한다.
          ④ 이제 A 는 B 의 통로를, B 는 A 의 통로를 원한다 → 사이클 → reserve_or_wait 가
             deadlock 을 반환하고 나중에 요청한 쪽이 즉시 양보(블랙리스트+우회)한다.
        """
        self.reset()
        chain = self.DEADLOCK_CHAIN
        head, tail = chain[0], chain[-1]
        a, b = self.DEADLOCK_A, self.DEADLOCK_B
        self.place(a, head)
        self.place(b, tail)

        # 사슬의 마지막 통로(9-4)를 B 가 이미 점유한 상태로 만든다.
        far = self.engine.corridor_between(chain[-2], chain[-1])
        if far is None:
            return {"ok": False, "reason": "CHAIN_BROKEN"}
        self.engine.try_reserve(far, b)
        self.events.info(
            f"[시나리오] 데드락 — {b} 가 통로 {far}({chain[-2]}-{tail})에 진입한 상태에서 "
            f"{a}({head}→{tail}) 와 {b}({tail}→{head}) 가 마주 본다")

        r1 = self.goto(a, tail)          # A: 앞 통로 2개만 쥐고 9 에서 막힘
        time.sleep(1.0)                  # A 가 확보를 마칠 시간
        r2 = self.goto(b, head)          # B: 자기 통로만 쥐고 가운데에서 막힘
        return {"ok": True, "started": {a: r1, b: r2},
                "chain": list(chain), "primed_corridor": far}

    # ------------------------------------------------------------------ #
    def snapshot(self, since_seq: int = 0) -> dict:
        """화면에 뿌릴 현재 상태 한 장. 브로드캐스트 루프가 10Hz 로 호출한다."""
        # 예약표: 엔진이 한 락 안에서 통째로 덤프해 준다(통로/자리 분리, TTL 죽은 것 제외).
        # 통로마다 holder_of 를 묻던 방식은 자리까지 세면 왕복이 배로 늘고, 묻는 사이에
        # 상태가 바뀌면 화면이 어긋난다.
        snap = self.engine.reservation_snapshot()
        reservations = {str(cid): rid for cid, rid in snap["corridors"].items()}
        node_holders = {str(n): rid for n, rid in snap["nodes"].items()}
        avoid = self.dispatcher.blacklist_view(self.engine)   # 락 1회로 끝낸다

        with self._lock:
            robots = []
            for rid, e in self._robots.items():
                s = e["robot"].snapshot()
                s.update(status=e["status"], task_id=e["task_id"],
                         result=e["result"], kind=e["kind"])
                robots.append(s)
            blocked = sorted(self._blocked_corridors)

        return {
            "t": round(time.monotonic() - self._t0, 2),
            # 화면이 '지금 보고 있는 게 시뮬인지 실물인지'를 늘 알 수 있게 매 틱 붙인다.
            # LIVE 는 ACS 가 죽으면 connected=False 가 되지만 SIM 은 항상 살아있다.
            "mode": "SIM",
            "connected": True,
            "robots": sorted(robots, key=lambda r: r["robot_id"]),
            "reservations": reservations,
            # 지점 자리 점유: {노드id: 로봇}. 통로 예약과 키 공간이 겹치므로 따로 낸다
            # (같은 dict 에 음수로 섞으면 화면이 통로 번호로 오해한다).
            "node_holders": node_holders,
            # 회피 중(블랙리스트) = 막힘/양보로 재계획에서 잠시 제외된 통로/지점
            "avoiding": avoid["corridors"],
            "avoiding_nodes": avoid["nodes"],
            "blocked": blocked,
            "events": self.events.since(since_seq),
            "seq": self.events.last_seq(),
        }

    def last_seq(self) -> int:
        """마지막 이벤트 번호. SIM/LIVE 를 같은 방식으로 다루기 위한 공통 메서드."""
        return self.events.last_seq()

    def reset(self) -> None:
        """진행 중인 주행을 멈추고 로봇을 충전소로 되돌린다(시나리오 다시 하기)."""
        with self._lock:
            entries = list(self._robots.values())
        for e in entries:
            e["client"].shutdown()
        for e in entries:
            t = e["thread"]
            if t is not None:
                t.join(timeout=5)
        # 남은 예약 강제 회수 — 통로뿐 아니라 '자리'도 비워야 한다. 통로만 돌면 로봇을
        # 충전소로 되돌려 놓고도 자리 예약이 남아, 다음 시나리오에서 아무도 그 지점에
        # 못 들어가는 유령 점유가 된다.
        snap = self.engine.reservation_snapshot()
        for cid, holder in snap["corridors"].items():
            self.engine.release(cid, holder)
        for node_id, holder in snap["nodes"].items():
            self.engine.release(self.engine.node_slot(node_id), holder)
        with self._lock:
            self._blocked_corridors.clear()
            for rid, e in self._robots.items():
                meta = self.dispatcher.wp_meta[e["start_wp"]]
                e["robot"].set_pose(meta["x"], meta["y"], yaw=meta["yaw"] or 0.0,
                                    waypoint_id=e["start_wp"], moving=False,
                                    spinning=False)
                e["client"] = FakeNavigateClient(
                    e["robot"], self.dispatcher.wp_meta,
                    speed_mps=self.speed_mps, spin_rps=self.spin_rps,
                    is_edge_blocked=self._is_edge_blocked, logger=self.events)
                e.update(status="IDLE", task_id=None, result=None, thread=None,
                         kind=None)
                self._claim_slot(rid, e["start_wp"])   # 충전소 자리를 다시 쥔다
        self.events.info("[조작] 시뮬 초기화")

    def shutdown(self) -> None:
        self._reap_stop.set()               # 청소 스레드 먼저 세운다
        self._reap_thread.join(timeout=2)
        with self._lock:
            entries = list(self._robots.values())
        for e in entries:
            e["client"].shutdown()

#!/usr/bin/env python3
"""RP-78 순찰 — 순찰 지점 방문 순서·촬영 판정·방문 마킹 (composition 분리).

automato_node(ROS 표면)에서 '순찰 동작 결정' 로직을 떼어낸 클래스. rclpy 노드를 직접
참조하지 않고, 필요한 것(logger, 라우팅 engine, Navigate 액션 client)을 인자로 받아
동작한다 → ROS 를 안 띄우고 fake engine/client 로 단위 테스트할 수 있다.

지점과 지점 사이를 '예약하며 이동하는' 부분(세그먼트 예약·룩어헤드·막힘 우회)은
route_runner.RouteRunner 로 떼어냈다 — 시나리오2 수확도 같은 규칙으로 이동해야 하는데
알고리즘이 두 벌이면 예약 규칙이 갈라져 교통관제가 조용히 깨지기 때문이다.
여기 남은 것은 **순찰이라서 하는 일**뿐이다:
  - 어느 지점을 어떤 순서로 갈지(run_patrol / _visit / sweep 재시도)
  - 언제 찍을지(_build_segment_goal — 문서 E2 20번 판정식, 짝 끼워넣기)
  - 무엇을 '방문했다'로 칠지(_mark_visited)
이 순찰 고유 로직은 _PatrolHooks 를 통해 RouteRunner.drive 안으로 주입된다.
"""
import math
import time

from automato_control_service.patrol_config import (
    BLOCK_GIVEUP_SEC,
    CAPTURE_DIR_GATE_DEG,
    PATROL_START_WAYPOINT_ID,
    RESERVE_POLL_SEC,
    SERVER_WAIT_SEC,
)
from automato_control_service.route_runner import DriveHooks, RouteRunner

# 촬영 방향 게이트를 라디안으로 (설정은 사람이 읽기 쉬운 도(°)로 둔다).
_CAPTURE_DIR_GATE_RAD = math.radians(CAPTURE_DIR_GATE_DEG)


def _norm_angle(a):
    """각도를 -pi ~ pi 로 접는다. 짝은 부모와 정반대(예: 1.57 ↔ -1.57)라, 접지 않으면
    두 방향의 차이가 -3.14 로 나와 '거의 같은 방향'으로 오판할 수 있다."""
    return math.atan2(math.sin(a), math.cos(a))


class _PatrolHooks(DriveHooks):
    """RouteRunner.drive 의 빈칸 4개를 순찰 로직으로 채운다(지점 하나 이동당 1개 생성).

    drive 는 '촬영'도 '짝'도 '방문'도 모른다. 그 개념이 필요한 순간마다 이 객체를 통해
    PatrolDispatcher 에게 되묻는 구조다. 이동 1회 동안만 사는 값(visited·client·task_id)
    을 여기 담아 두어, drive 의 인자 목록이 순찰 사정으로 불어나지 않게 한다.
    """

    def __init__(self, disp, engine, client, task_id, robot_id, visited):
        self._d = disp
        self._engine = engine
        self._client = client
        self._task_id = task_id
        self._robot_id = robot_id
        self._visited = visited

    def build_goal(self, seg_wps, seg_start):
        """예약한 경로에 방향 게이트 촬영 판정을 얹어 실제 하달 배열을 만든다."""
        return self._d._build_segment_goal(seg_wps, self._visited, seg_start)

    def on_segment_done(self, hadal, capture_ids, parents, last_wp, code):
        """촬영이 끝난 지점을 방문 완료로 올린다(짝이 있으면 짝까지 끝나야 인정)."""
        self._d._mark_visited(
            hadal, capture_ids, parents, last_wp, code, self._visited)

    def normalize_wp(self, wp):
        """짝 id 로 온 보고를 부모 id 로 되돌린다(짝은 라우팅 그래프에 없다)."""
        return self._d._parent_of(wp)

    # finalize 는 기본(DriveHooks, 아무것도 안 함)을 그대로 쓴다.
    # 옛날에는 '도착했는데 짝 촬영이 남았으면 제자리에서 다시 하달'했다. 그 시절 짝은
    # 부모와 같은 자리라 제자리 회전만 하면 됐다. 지금은 방향 게이트가 '지나는 방향에
    # 맞는 것만' 찍고, 좌표가 갈라진 짝은 반대로 지날 때 찍힌다. 제자리 재하달은 진행
    # 방향이 없어 무엇을 찍을지 정할 수 없고, 자칫 180° 회전을 유발한다. 그래서 뺐다.
    # 촬영은 세그먼트 주행 중(방향이 확실할 때)에만 일어나고, 못 찍은 지점은 sweep 이
    # '다시 지나가며(방향 있음)' 재시도한다.


class PatrolDispatcher:
    """순찰 1건을 실행한다(로봇당 스레드가 run_patrol 호출).

    노드에서 넘겨받는 것:
      - logger: 생성자에서 1회 (ROS 로거를 그대로 사용).
      - runner: 생성자에서 1회. **노드가 만든 것을 수확 디스패처와 함께 공유**한다
                (블랙리스트·wp_meta 가 갈리면 교통관제가 어긋난다). 생략하면 스스로
                하나 만든다 — 순찰만 돌리는 테스트·시뮬레이터를 위한 편의다.
      - engine/client: run_patrol 인자로 매번 (rclpy 엔티티는 노드가 만든다).
    스스로 소유하는 공유 상태:
      - pair_of : 부모 waypoint_id -> 짝 waypoint_id. 그래프 로드 시 노드가 채운다
                  (1회, 읽기전용). 짝 촬영은 순찰 고유 개념이라 runner 로 가지 않았다.
      - wp_meta 는 runner 소유다(주행 골에 좌표가 필요하다) — 아래 프로퍼티로 넘겨준다.
    """

    def __init__(self, logger, runner=None):
        self._log = logger
        self.runner = runner if runner is not None else RouteRunner(logger)
        # 부모 waypoint_id -> 짝 waypoint_id (같은 자리, 반대 촬영 방향).
        # 부모에 도착해 촬영한 뒤 이 짝을 추가로 하달해 제자리 회전 촬영을 시킨다.
        self.pair_of = {}

    @property
    def wp_meta(self):
        """waypoint_id -> {x,y,yaw,capture}. 실체는 runner 가 들고 있다.

        주행 골(Navigate)에 좌표가 필요해 소유권은 runner 로 넘겼지만, 순찰도 촬영 판정에
        같은 표를 본다. 노드·테스트가 예전처럼 dispatcher.wp_meta 로 읽고 쓸 수 있게
        여기서 통째로 위임한다(표가 두 벌이 되면 좌표와 촬영 판정이 어긋난다).
        """
        return self.runner.wp_meta

    @wp_meta.setter
    def wp_meta(self, value):
        self.runner.wp_meta = value

    # ---------------------------- 순찰 본체 ---------------------------- #
    def run_patrol(self, task_id, robot_id, waypoints, engine, client,
                   start_wp=None, entry_wp=None) -> tuple:
        """순찰 지점을 순서대로 방문. 반환: (status, unvisited_waypoint_ids, last_wp).

        last_wp: 순찰이 끝난 시점 로봇이 서 있는 노드. 이 자리 예약을 '쥔 채로' 반환하고
          (finally 에서 반납하지 않는다) 복귀 주행이 이어받는다. 시작 전 실패(서버 미기동/
          방문 지점 없음)면 None.

        status: 'COMPLETED' | 'COMPLETED_PARTIAL' | 'FAILED'.
        unvisited_waypoint_ids: sweep 후에도 못 간 순찰 지점 목록(E2 9-1 의 그 필드).
          COMPLETED/FAILED 면 빈 리스트다 — 못 간 지점 목록이 의미 있는 것은
          '끝까지 돌았지만 일부를 못 간' PARTIAL 뿐이고, FAILED 의 task_failed 알림에는
          애초에 이 목록이 들어가지 않는다(문서 13번). 호출부(노드)가 이 목록을
          patrol_completed 페이로드에 그대로 싣는다.

        engine/client 는 노드(ROS 표면)가 만들어 넘긴다 — 이 클래스는 rclpy 엔티티를
        생성하지 않고 받은 것만 사용한다.
        start_wp: 이 로봇이 서 있는 출발 노드(전용 충전소의 진입 노드). None 이면
                  설정 상수 PATROL_START_WAYPOINT_ID 로 폴백한다.
        """
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            self._log.warn(
                f"{robot_id} Navigate 액션 서버 미기동 → task {task_id} FAILED")
            return "FAILED_ABORTED", [], None

        targets = [wp["waypoint_id"] for wp in waypoints]
        if not targets:
            return "COMPLETED", [], None       # 방문할 지점이 없음

        visited = set()
        # 순찰 시작 노드(로봇 전용 충전소의 진입 노드). 그래프(wp_meta)에 있으면 current 로 두고
        # 첫 순찰 지점도 drive 로 이동해 '첫 구간까지 통로 예약'으로 보호한다.
        # 미설정/미상이면 옛 동작으로 폴백: 첫 지점만 예약 없이 직행(이 구간은 통로 보호 없음).
        # start_wp(로봇별, DB 유도)가 우선이고, 없을 때만 전역 설정 상수를 쓴다.
        start = start_wp if start_wp is not None else PATROL_START_WAYPOINT_ID
        # entry_wp 가 주어지면(실배포) 충전기 탈출+진입 노드 경유를 한다. 없으면(테스트·
        # 시뮬) 로봇이 이미 start 노드에 서 있다고 보고 곧장 순찰한다.
        lead_in = False
        if start and start in self.wp_meta:
            current = start
            remaining = targets
            lead_in = entry_wp is not None
            self._log.info(f"순찰 시작 노드 {start} 에서 출발 task={task_id}")
        else:
            self._log.warn(
                f"순찰 시작 waypoint({start}) 미설정/그래프에 없음 → 첫 지점 예약 없이 "
                f"직행(폴백) task={task_id}")
            current = targets[0]
            remaining = targets[1:]
            # 통로는 못 잡고 가지만 '도착해서 설 자리'는 미리 잡는다. 짝이 있는 지점이면
            # 같은 배열 안에서 제자리 회전까지 하는데, 그동안 자리가 비어 보이면 남이
            # 그 지점으로 들어온다. 아래 공통 블록의 예약은 같은 로봇이라 그대로 성공한다.
            if not engine.try_reserve(engine.node_slot(current), robot_id):
                self._log.warn(
                    f"첫 지점 {current} 자리를 남(로봇 "
                    f"{engine.holder_of(engine.node_slot(current))})이 쥐고 있다")
            hadal, cap_ids, cap_parents = self._build_segment_goal(
                [current], visited, None)
            code, last_wp = self.runner._dispatch_segment(
                client, task_id, hadal, cap_ids)
            if code != 0:
                engine.release(engine.node_slot(current), robot_id)
                self._log.warn(f"첫 순찰 지점 도달 실패 → task {task_id} FAILED")
                return "FAILED_ABORTED", [], current
            self._mark_visited(hadal, cap_ids, cap_parents, last_wp, code, visited)

        # 출발선에서 '지금 서 있는 자리'부터 잡는다. 첫 구간의 drive 가 잡아주긴
        # 하지만 그 전에 짝 촬영(제자리 회전)이 낀 경로가 있어, 그동안 이 로봇이 예약표에
        # 안 보이면 남이 그 지점으로 들어온다. 이 예약은 구간마다 drive 가 이어받아
        # 순찰 내내 유지되고, 맨 끝에서 아래 finally 가 반납한다.
        start_slot = engine.node_slot(current)
        if engine.try_reserve(start_slot, robot_id):
            self._log.info(f"출발 지점 {current} 자리 확보 task={task_id}")
        else:
            self._log.warn(
                f"출발 지점 {current} 자리를 남(로봇 {engine.holder_of(start_slot)})이 "
                f"쥐고 있다 task={task_id} — 예약표와 실제 위치가 어긋남")

        # 순찰 지점: 세그먼트(연속 통로 묶음) 단위로 이동(시작 노드가 있으면 첫 지점부터)
        skipped = []
        # 여기서부터 로봇은 '서 있는 자리'를 계속 쥔 채 구간을 이어간다(drive 가
        # 서로 넘겨준다). 마지막 한 장은 순찰 전체를 소유하는 이 함수가 반납해야 하므로
        # 어떤 경로로 빠져나가든 finally 를 지나게 감싼다.
        try:
            # 충전기 탈출(언도킹) + 순찰 진입 노드까지. entry_wp 가 주어질 때만 한다.
            if lead_in:
                outcome, current = self._lead_in(
                    engine, client, task_id, robot_id, current, entry_wp)
                if outcome == "aborted":
                    return "FAILED_ABORTED", [], current
            for target in remaining:
                outcome, current = self._visit(
                    engine, client, task_id, robot_id, current, target, visited)
                if outcome == "aborted":
                    return "FAILED_ABORTED", [], current
                # 문서 22: 이 지점 우회로도 없을 때(skipped) 로봇이 '갇혔는지'(남은 순찰
                # 지점 어디로도 못 감) 본다. 갇혔으면 T_block 재시도 후에도 못 나가면 막힘
                # 확정 → 22-1(순찰 실패 후 충전소 복귀). 아니면 이 지점만 건너뛰고 계속.
                if outcome == "skipped" and self._stranded_after_block(
                        engine, robot_id, current, targets, visited):
                    return "FAILED_BLOCKED", [], current
                if target not in visited:
                    skipped.append(target)

            # 건너뛴 지점 마지막에 1회 재시도(문서 23번의 sweep — 1회로 고정)
            for target in skipped:
                outcome, current = self._visit(
                    engine, client, task_id, robot_id, current, target, visited)
                if outcome == "aborted":
                    return "FAILED_ABORTED", [], current

            if all(t in visited for t in targets):
                return "COMPLETED", [], current
            # 못 간 지점이 남으면 몇 개든 COMPLETED_PARTIAL 이다(문서 E2 23번).
            # 예전엔 '한 곳만 방문했으면 FAILED' 규칙이 있었으나 문서에 근거가 없다.
            # 순찰은 끝까지 돌았고 일부를 못 간 것이지 실패한 것이 아니다 — 그래서
            # 이 경로에서는 task_failed 알림도 보내지 않는다. FAILED 는 로봇이 중단을
            # 보고했을 때(aborted)와 막힘 확정 복귀(22-1)에서만 나온다.
            # 순찰 순서(targets)를 지켜 미방문 목록을 만든다(집합 차집합은 순서를 잃는다).
            unvisited = [t for t in targets if t not in visited]
            return "COMPLETED_PARTIAL", unvisited, current
        finally:
            # 순찰이 끝나도 마지막 자리는 '반납하지 않는다' — 복귀 주행(_return_and_dock)이
            # 같은 로봇 자격으로 이 자리를 이어받아 도킹 성공 시점에 한 번에 해제한다.
            # 순찰이 자리를 놓는 찰나에 남이 그 자리로 들어오는 것을 막기 위함이다.
            # 최종 반납 책임은 호출부(_patrol_job)로 넘어간다: 복귀하면 도킹 후 해제하고,
            # FAILED 로 끝나면 즉시 반납한다.
            self._log.info(
                f"순찰 종료 task={task_id} 지점 {current} 자리 유지 → 복귀에 인계")

    def _lead_in(self, engine, client, task_id, robot_id, current, entry_wp):
        """순찰 목표를 돌기 전, 충전기에서 빠져나와 진입 노드까지 간다.
        반환: (outcome, 현재 노드). outcome 'arrived' | 'aborted'.

        ① 언도킹 — 로봇은 충전기 '안'에 도킹돼 있고 ACS 는 current(전용 충전소의 진입
           노드)에 서 있다고 가정한다. 그대로 첫 목표로 출발하면 로봇이 15cm 충전 공간
           안에서 크게 돌 수 있다(도킹 방향과 첫 이동 방향이 벌어질 때). 먼저 진입 노드
           '그 자리'로 한 스텝만 하달해 정면으로 빠져나오게 한다. current 자리는 위에서
           이미 예약했다. 실제 하달은 RouteRunner.undock_step 이 한다 — 수확(충전소·
           수확지·예냉실 출발)도 같은 함수를 쓴다. 언도킹 코드가 두 벌이 되면 한쪽만
           고쳐 순찰은 되는데 수확은 안 되는 종류의 버그가 생긴다.
        ② 진입 노드 — 첫 촬영 목표로 갈 때 '최단 경로'가 반대 방향에서 접근해 촬영이
           방향 게이트에 막히는 것을 피하려, 지정 진입 노드를 먼저 거친다. 여기서는
           촬영하지 않는다(훅 없는 평범한 주행). 지정이 없거나 이미 그 노드면 생략한다.
        """
        # ① 언도킹: [current] 한 노드만 하달(촬영 없음). 좌표·yaw 는 undock_step 이 정한다.
        # 하트비트를 넘기는 이유: 이 자리는 위에서 이미 예약했는데, 언도킹이 15초를 넘기면
        # 갱신이 없어 TTL 만료로 회수된다(도킹에 넘기는 것과 같은 이유).
        if not self.runner.undock_step(
                client, task_id, current,
                heartbeat=(engine, [engine.node_slot(current)], robot_id)):
            self._log.warn(f"언도킹 하달 실패 task={task_id} 노드 {current}")
            return "aborted", current
        self._log.info(f"언도킹 완료 task={task_id} → 노드 {current}")

        # ② 진입 노드까지 촬영 없이 이동(hooks 안 넘김 = 기본 주행).
        if entry_wp and entry_wp in self.wp_meta and entry_wp != current:
            self._log.info(f"순찰 진입 노드 {entry_wp} 경유 task={task_id}")
            outcome, current = self.runner.drive(
                engine, client, task_id, robot_id, current, entry_wp)
            if outcome == "aborted":
                return "aborted", current
        return "arrived", current

    def _visit(self, engine, client, task_id, robot_id, current, target, visited):
        """순찰 지점 하나를 방문한다. 반환: (outcome, 도달한 노드).

        RouteRunner.drive 를 감싸며 '방문했다'의 판정만 맡는다:
          · 오는 길에 이미 찍힌 지점이면 이동조차 하지 않는다(방향 게이트가 지나는 김에
            찍었을 수 있다). 다시 가도 촬영하지 않으므로 순수한 헛걸음이다.
          · 짝(18·19)은 corridors 에 없어 그리로는 경로 탐색이 안 된다. 경로는 부모 노드
            (같은 자리)로 찾고, 촬영은 방향 게이트가 세그먼트 주행 중에 짝으로 바꿔 찍는다.
          · 촬영 대상이 아닌 목표(순찰 지점이 아닌 노드)는 도달만으로 방문으로 친다.
            촬영이 방문의 근거인 지점은 _mark_visited 가 넣어 준다.
        """
        if target in visited:
            self._log.info(
                f"지점 {target} 은 오는 길에 이미 촬영됨 → 목표에서 제외 task={task_id}")
            return "arrived", current
        route_target = self._parent_of(target)   # 짝이면 부모 노드로, 아니면 그대로
        hooks = _PatrolHooks(self, engine, client, task_id, robot_id, visited)
        outcome, current = self.runner.drive(
            engine, client, task_id, robot_id, current, route_target, hooks)
        if (outcome == "arrived"
                and not self.wp_meta.get(target, {}).get("capture")):
            visited.add(target)
        return outcome, current

    # ---------------------------- 막힘 확정 판정(문서 22(b) → 22-1) ---------------------------- #
    def _stranded_after_block(self, engine, robot_id, current, targets, visited):
        """current 에서 남은 순찰 지점 어디로도 못 가는가(막힘 확정 판정 → 22-1).

        _visit 이 skipped(우회로도 없음)를 낸 직후 부른다. 남은 미방문 순찰 지점 중 하나라도
        지금 도달 가능하면 갇힌 게 아니다(문서 22(a): 다른 지점으로 스킵). 전부 도달 불가면
        갇힌 것이므로 T_block(BLOCK_GIVEUP_SEC) 동안 재시도하며 통로가 풀리길 기다린다
        (문서 22(b)). 그래도 못 나가면 True → 호출부(run_patrol)가 22-1 복귀로 넘어간다.

        서 있는 자리는 폴링마다 하트비트를 갱신한다. 여기서 기다리는 동안에는 주행
        하트비트(_dispatch_segment)가 안 도는데, T_block(60초)이 RESERVATION_TTL_SEC
        (15초)의 네 배라 갱신이 없으면 **반드시** 자리가 죽은 예약으로 회수된다. 하필
        통로가 붐벼 막힌 상황이라, 그 순간 '이 지점 비었다'고 남에게 알리는 꼴이 된다
        (자원 양보 대기 RouteRunner._reserve_with_wait 와 같은 처리).
        """
        deadline = time.monotonic() + BLOCK_GIVEUP_SEC
        standing_slot = engine.node_slot(current)
        while True:
            if self._escapable(engine, robot_id, current, targets, visited):
                return False
            if time.monotonic() >= deadline:
                self._log.warn(
                    f"{robot_id} 위치 {current} 에서 남은 순찰 지점 전부 도달 불가 · "
                    f"T_block({BLOCK_GIVEUP_SEC}s) 초과 → 막힘 확정(22-1 복귀)")
                return True
            engine.heartbeat(standing_slot, robot_id)   # 서 있는 자리 TTL 방어
            time.sleep(RESERVE_POLL_SEC)

    def _escapable(self, engine, robot_id, current, targets, visited):
        """current 에서 남은 미방문 순찰 지점 중 하나라도 지금 도달 가능한가.

        남이 점유한 통로(reserved_corridors)와 시간 만료 전 블랙리스트를 제외한 그래프로
        find_path 를 돌려, 미방문 순찰 지점 하나라도 경로가 나오면 True(어디론가는 갈 수 있음).
        내 예약은 제외한다 — 내가 쥔 자리·통로는 나한테는 막힘이 아니다.
        """
        # 블랙리스트·자원 분류는 주행 엔진(RouteRunner)이 소유한다 — 순찰·수확이 공유하는
        # 상태라, 여기서 따로 들면 '막혔다'는 판정이 두 벌이 된다.
        blocked = (engine.reserved_corridors(exclude_robot=robot_id)
                   | self.runner._blacklist_active())
        corridors, nodes = self.runner._split_blocked(engine, blocked)
        for t in targets:
            if t in visited:
                continue
            # 짝(18·19)은 그래프에 없다 → 부모 노드(같은 자리)로 도달 가능성을 본다.
            node = self._parent_of(t)
            if engine.find_path(current, node, blocked=corridors,
                                blocked_nodes=nodes) is not None:
                return True
        return False

    def drive_to_point(self, task_id, robot_id, current, target, engine, client):
        """current→target 을 촬영 없이(전 구간 capture=false) 한 번 주행한다.

        E4(순찰 종료 후 충전소 복귀)와 22-1(막힘 실패 복귀)이 공통으로 쓰는 진입점이다.
        순찰(run_patrol)이 촬영·방문·sweep 판정에 얽매인 것과 달리, 이건 '한 지점까지 가서
        선다'만 한다. 속은 순찰과 똑같은 RouteRunner.drive 라 복귀 주행도 통로를 예약하며
        움직인다 — 복귀라고 예약 없이 달리면 그게 다른 로봇의 새 막힘 원인이 된다.

        **도착 방향은 target 의 DB yaw 로 고정한다.** 도킹 진입 노드의 yaw_coord 는
        '마커를 정면으로 보는 방향'인데, 주행 기본 규칙은 통과 노드를 '진행 방향'으로
        세운다(_travel_yaw). 그러면 어느 쪽에서 오느냐로 도착 자세가 크게 달라져
        (wp15→wp24 는 176.6°, wp16→wp24 는 81.7°, DB 값은 87.3°) 마커를 비스듬히
        보게 되고, 코너 한 면이 짧게 읽혀 검출에서 탈락한다 → 도킹이 '마커 없음'으로
        실패한다. 언도킹(undock_step)에는 같은 방어가 이미 있었고 그 짝이 빠져 있었다.

        반환: (outcome, 도달한 노드).
          'arrived' 목표(충전소 진입 노드) 도달 → 다음은 도킹.
          'skipped' 우회로도 없어 못 감 → 호출부가 22-2(현장 정지)로 넘긴다.
          'aborted' 로봇이 중단 보고 / Navigate 서버 미기동.
        """
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            self._log.warn(
                f"{robot_id} Navigate 서버 미기동 → 복귀 주행 불가 task={task_id}")
            return "aborted", current
        # 도착 방향(마지막 노드에만 적용). 없으면 None → 기존 규칙(진행 방향)에 맡긴다.
        final_yaw = self.runner.entry_yaw(target, task_id)
        self._log.info(
            f"복귀 주행 시작 task={task_id} {robot_id} {current}→{target}(충전소) "
            f"도착 yaw={'미지정' if final_yaw is None else f'{final_yaw:.2f}'}")
        # 훅을 안 넘긴다 = DriveHooks 기본값(촬영·짝·방문 마킹 없는 평범한 주행).
        # 예전에는 _navigate(capture=False) 로 순찰 로직을 '껐'지만, 지금은 그 로직이
        # 애초에 drive 밖(_PatrolHooks)에 있어 안 넘기면 그만이다.
        return self.runner.drive(
            engine, client, task_id, robot_id, current, target,
            final_yaw=final_yaw)

    # ---------------------------- 촬영 판정(방향 게이트, RP-EX) ---------------------------- #
    def _build_segment_goal(self, seg_wps, visited, seg_start):
        """예약 확보한 노드 목록 → (하달 배열, 촬영 id 집합, 방문마킹 대상 목록).

        이 함수는 '순찰 경로일 때'만 불린다 — _PatrolHooks.build_goal 을 통해서만 들어오고,
        복귀(E4)·실패 복귀(22-1)·수확 주행은 애초에 훅을 안 넘겨 여기 오지 않는다.

        노드마다 '지나는 방향'을 보고 무엇을 찍을지 정한다(방향 게이트):
          · 노드 자신의 촬영 방향이 진행 방향과 맞으면(≤게이트, 미방문) 그 노드를 찍는다.
          · 아니면 그 자리의 반대 방향 촬영(짝)이 진행 방향과 맞으면 짝을 찍는다.
            짝은 하달 배열에 '짝 id'로 넣는다 — 좌표·yaw 가 짝 것이라 로봇이 그 자세로 선다
            (예약은 부모 자리 그대로 → 예약 경로 seg_wps 는 안 건드린다).
          · 둘 다 아니면 통과(capture=false). 반대 방향으로 지날 때 찍히므로 헛것이 아니다.

        왜 방향을 보나: 카메라가 로봇 옆 한쪽에 고정돼 있어, 통로를 지나는 방향이 곧 어느
        베드를 찍느냐다. 방향을 안 보면 반대로 지날 때도 찍으라고 해 '제자리 180° 뒤돌아
        찍기'(좁은 통로에서 물리적으로 불가능)를 유발한다.

        seg_start: 이 세그먼트 진입 직전 노드(첫 노드의 진입 방향 계산용). None 이면
        진행 방향을 알 수 없어 아무것도 안 찍고 통과시킨다(언도킹·폴백의 단일 노드 하달).

        세 번째 반환값(방문마킹 대상)은 이번에 촬영한 id 들이다 — 노드든 짝이든 '찍은 것'을
        방문 완료로 올린다(각 촬영이 독립 목표라 부모-짝 묶음 판정이 없다).
        """
        hadal, capture_ids, parents = [], set(), []
        for i, wp in enumerate(seg_wps):
            travel = self._travel_dir(seg_wps, i, seg_start)
            shot = self._pick_capture(wp, travel, visited)
            if shot is None:
                hadal.append(wp)            # 통과 (capture=false, 노드 그대로)
                continue
            hadal.append(shot)              # 노드 자신 또는 짝의 id(좌표가 딸려온다)
            capture_ids.add(shot)
            parents.append(shot)
        return hadal, capture_ids, parents

    def _travel_dir(self, seg_wps, i, seg_start):
        """seg_wps[i] 에 도착할 때의 진행 방향(rad). 못 구하면 None.

        직전 노드(첫 노드면 seg_start)에서 이 노드로 향하는 방향이다. 로봇은 실제로 그
        방향을 보고 이 노드에 도착하므로(navigate_server 가 이동 방향으로 정렬), 게이트가
        이 값을 기준으로 '지금 방향으로 찍을 수 있나'를 판정하면 로봇 동작과 일치한다.
        """
        prev = seg_wps[i - 1] if i > 0 else seg_start
        if prev is None:
            return None
        a = self.wp_meta.get(prev)
        b = self.wp_meta.get(seg_wps[i])
        if not a or not b:
            return None
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None                     # 두 점이 겹침(이례적) → 방향 불명
        return math.atan2(dy, dx)

    def _pick_capture(self, node, travel, visited):
        """이 노드를 지날 때 찍을 대상: 노드 자신 id / 짝 id / None(통과).

        travel(진행 방향)이 None 이면 방향을 몰라 아무것도 안 찍는다 — 엉뚱한 방향으로
        찍느니 방향이 확실한 다음 기회에 찍는 편이 안전하다.
        """
        if travel is None:
            return None
        meta = self.wp_meta.get(node, {})
        if (meta.get("capture") and node not in visited
                and self._dir_ok(meta.get("yaw"), travel)):
            return node                     # 후보 ① 노드 자신
        pair = self.pair_of.get(node)       # 후보 ② 짝(같은 자리, 반대 방향)
        if pair is not None and pair not in visited:
            pmeta = self.wp_meta.get(pair)
            if pmeta is None:
                self._log.warn(
                    f"짝 {pair}(부모 {node}) 좌표를 그래프에서 못 찾음 → 건너뜀")
            elif self._dir_ok(pmeta.get("yaw"), travel):
                return pair
        return None

    @staticmethod
    def _dir_ok(shoot_yaw, travel):
        """찍을 방향과 진행 방향의 차이가 게이트 이내인가(= 180° 뒤돌기가 아닌가)."""
        if shoot_yaw is None:
            return False
        return abs(_norm_angle(shoot_yaw - travel)) <= _CAPTURE_DIR_GATE_RAD

    def _mark_visited(self, hadal, capture_ids, parents, last_wp, code, visited):
        """이번 하달에서 '촬영까지 끝난' 대상을 방문 완료로 올린다.

        parents 는 이번에 촬영한 id(노드 또는 짝)다. 각 촬영이 독립 목표라 옛날처럼
        '부모+짝이 다 끝나야 인정'하는 묶음 판정이 없다 — 찍었으면 그 id 를 방문으로 친다.

        code == 0 이면 배열을 끝까지 소화한 것이라 전부 인정한다. 중간에 끊겼으면 로봇이
        실제로 도달한 last_wp(그래프 노드)까지만 인정한다. 하달 배열에는 짝 id 가 들어
        있을 수 있고 last_wp 는 그래프 노드(부모)로 정규화돼 오므로, 둘을 같은 그래프
        노드로 비교해 도달 범위를 찾는다.
        """
        if code == 0:
            done = set(hadal)
        elif last_wp is None:
            return
        else:
            j = None
            for idx, h in enumerate(hadal):
                if h == last_wp or self._parent_of(h) == last_wp:
                    j = idx                 # 같은 그래프 노드의 마지막 위치까지 도달
            if j is None:
                return
            done = set(hadal[:j + 1])
        for wp in parents:
            if wp in done:
                visited.add(wp)

    def _parent_of(self, wp):
        """짝 id 면 부모 id 로 바꾼다(짝이 아니면 그대로).

        예약·진행도 계산은 라우팅 그래프(짝이 없는 그래프) 기준이라, 로봇이 짝에서 멈춰
        last_waypoint_id 로 짝 id 를 돌려주면 그 노드를 못 알아본다. 짝은 부모와 같은
        자리이므로 부모로 바꿔 주면 그대로 성립한다. 짝은 두어 개뿐이라 역맵을 따로
        들지 않고 즉석에서 찾는다.
        """
        for parent, pair in self.pair_of.items():
            if pair == wp:
                return parent
        return wp

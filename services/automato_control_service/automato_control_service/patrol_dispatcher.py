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
from automato_control_service.patrol_config import (
    PATROL_START_WAYPOINT_ID,
    SERVER_WAIT_SEC,
)
from automato_control_service.route_runner import DriveHooks, RouteRunner


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

    def build_goal(self, seg_wps):
        """예약한 경로에 촬영 판정과 짝을 얹어 실제 하달 배열을 만든다."""
        return self._d._build_segment_goal(seg_wps, self._visited)

    def on_segment_done(self, hadal, capture_ids, parents, last_wp, code):
        """촬영이 끝난 지점을 방문 완료로 올린다(짝이 있으면 짝까지 끝나야 인정)."""
        self._d._mark_visited(
            hadal, capture_ids, parents, last_wp, code, self._visited)

    def normalize_wp(self, wp):
        """짝 id 로 온 보고를 부모 id 로 되돌린다(짝은 라우팅 그래프에 없다)."""
        return self._d._parent_of(wp)

    def finalize(self, target, held):
        """도착했는데 촬영이 남았으면, 이동 없이 촬영만 다시 하달한다.

        촬영(짝이 있으면 제자리 회전 촬영까지)은 보통 마지막 세그먼트의 하달 배열 안에서
        이미 끝나고 방문 마킹도 마쳤다. 다만 두 경우엔 도달했는데도 촬영이 남는다:
          ① 마지막 배열이 짝 바로 앞에서 끊겼다(부모만 찍고 반대쪽을 못 찍음)
          ② 이미 목표 지점에 서 있어(current == target) drive 가 한 번도 안 움직였다
             — sweep 재시도가 여기 해당한다
        로봇이 그 자리에 서 있으므로 이동 없이 촬영만 다시 하달한다. 쥐고 있는
        자원(held)은 아직 반납 전이고 하트비트로 유지되므로 회전하는 동안 남이 못 들어온다.
        """
        if target in self._visited:
            return
        d = self._d
        hadal, cap_ids, cap_parents = d._build_segment_goal(
            [target], self._visited)
        if not cap_ids:
            return
        d._log.info(
            f"촬영 미완 지점 {target} 재하달 task={self._task_id} "
            f"(이동 없음, 촬영={sorted(cap_ids)})")
        code, last_wp = d.runner._dispatch_segment(
            self._client, self._task_id, hadal, cap_ids,
            heartbeat=(self._engine, held, self._robot_id))
        d._mark_visited(
            hadal, cap_ids, cap_parents, last_wp, code, self._visited)


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
                   start_wp=None) -> tuple:
        """순찰 지점을 순서대로 방문. 반환: (status, unvisited_waypoint_ids).

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
            return "FAILED", []

        targets = [wp["waypoint_id"] for wp in waypoints]
        if not targets:
            return "COMPLETED", []             # 방문할 지점이 없음

        visited = set()
        # 순찰 시작 노드(로봇 전용 충전소의 진입 노드). 그래프(wp_meta)에 있으면 current 로 두고
        # 첫 순찰 지점도 drive 로 이동해 '첫 구간까지 통로 예약'으로 보호한다.
        # 미설정/미상이면 옛 동작으로 폴백: 첫 지점만 예약 없이 직행(이 구간은 통로 보호 없음).
        # start_wp(로봇별, DB 유도)가 우선이고, 없을 때만 전역 설정 상수를 쓴다.
        start = start_wp if start_wp is not None else PATROL_START_WAYPOINT_ID
        if start and start in self.wp_meta:
            current = start
            remaining = targets
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
                [current], visited)
            code, last_wp = self.runner._dispatch_segment(
                client, task_id, hadal, cap_ids)
            if code != 0:
                engine.release(engine.node_slot(current), robot_id)
                self._log.warn(f"첫 순찰 지점 도달 실패 → task {task_id} FAILED")
                return "FAILED", []
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
            for target in remaining:
                outcome, current = self._visit(
                    engine, client, task_id, robot_id, current, target, visited)
                if outcome == "aborted":
                    return "FAILED", []
                if target not in visited:
                    skipped.append(target)

            # 건너뛴 지점 마지막에 1회 재시도(문서 23번의 sweep — 1회로 고정)
            for target in skipped:
                outcome, current = self._visit(
                    engine, client, task_id, robot_id, current, target, visited)
                if outcome == "aborted":
                    return "FAILED", []

            if all(t in visited for t in targets):
                return "COMPLETED", []
            # 못 간 지점이 남으면 몇 개든 COMPLETED_PARTIAL 이다(문서 E2 23번).
            # 예전엔 '한 곳만 방문했으면 FAILED' 규칙이 있었으나 문서에 근거가 없다.
            # 순찰은 끝까지 돌았고 일부를 못 간 것이지 실패한 것이 아니다 — 그래서
            # 이 경로에서는 task_failed 알림도 보내지 않는다. FAILED 는 로봇이 중단을
            # 보고했을 때(aborted)와 막힘 확정 복귀(22-1)에서만 나온다.
            # 순찰 순서(targets)를 지켜 미방문 목록을 만든다(집합 차집합은 순서를 잃는다).
            unvisited = [t for t in targets if t not in visited]
            return "COMPLETED_PARTIAL", unvisited
        finally:
            # 순찰이 끝나면 마지막 자리를 반납한다. 로봇은 아직 거기 서 있으므로 이
            # 시점부터 교통관제에 안 보인다 — 충전소 복귀가 붙으면 복귀 경로가 자리를
            # 이어받게 되고, 그때 이 반납은 복귀 도착 지점으로 옮겨가야 한다.
            engine.release(engine.node_slot(current), robot_id)
            self._log.info(
                f"순찰 종료 task={task_id} 지점 {current} 자리 반납 "
                f"(복귀 로직 전까지 이 지점은 교통관제에 비어 보인다)")

    def _visit(self, engine, client, task_id, robot_id, current, target, visited):
        """순찰 지점 하나를 방문한다. 반환: (outcome, 도달한 노드).

        RouteRunner.drive 를 감싸며 '방문했다'의 판정만 맡는다:
          · 오는 길에 이미 찍힌 지점이면 이동조차 하지 않는다. 문서 20번 판정식대로면
            다시 가도 미방문이 아니라 촬영하지 않으므로 순수한 헛걸음이다.
          · 촬영 대상이 아닌 목표(순찰 지점이 아닌 노드)는 도달만으로 방문으로 친다.
            촬영이 방문의 근거인 지점은 _mark_visited 가 이미 넣어 준다.
        """
        if target in visited:
            self._log.info(
                f"지점 {target} 은 오는 길에 이미 촬영됨 → 목표에서 제외 task={task_id}")
            return "arrived", current
        hooks = _PatrolHooks(self, engine, client, task_id, robot_id, visited)
        outcome, current = self.runner.drive(
            engine, client, task_id, robot_id, current, target, hooks)
        if (outcome == "arrived"
                and not self.wp_meta.get(target, {}).get("capture")):
            visited.add(target)
        return outcome, current

    # ---------------------------- 촬영 판정(문서 E2 20번) ---------------------------- #
    def _build_segment_goal(self, seg_wps, visited):
        """예약 확보한 노드 목록 → (하달 배열, 촬영 대상 id 집합, 촬영 대상 부모 목록).

        촬영 여부는 문서 판정식을 **노드마다** 적용한다:
            capture = (순찰 지점) AND (이번 task 에서 미방문)
        '배열의 마지막 하나만' 이 아니다 — 우회 경로가 아직 안 찍은 순찰 지점을 지나가면
        지나는 김에 찍어야 나중에 그 지점을 목표로 다시 오지 않는다.

        짝(같은 자리·반대 촬영 방향)이 있는 노드는 문서 20-1 대로 **바로 뒤에 연달아**
        끼워 넣는다. 촬영 카메라가 로봇 한쪽에 고정돼 있어 통로를 한 번 지나면 한쪽 베드만
        찍히기 때문이다. 로봇은 직전 원소와 좌표가 같으면 주행이 아니라 제자리 회전(Spin)
        으로 분기하므로 Goal 을 한 번 더 보낼 필요가 없다. 짝은 corridors 에 없어 통로
        예약도 필요 없다 — 부모 자리를 그대로 쓴다.

        짝에 별도 waypoint_id 를 주는 이유는 사진마다 고유 식별자가 남아야 detection_logs
        와 병해충 알림이 '어느 지점의 어느 방향'인지 특정할 수 있기 때문이다.

        세 번째 반환값(부모 목록)은 방문 마킹용이다. 짝 자신은 순찰 지점이 아니라
        방문 큐에 넣지 않는다.
        """
        hadal, capture_ids, parents = [], set(), []
        for wp in seg_wps:
            hadal.append(wp)
            meta = self.wp_meta.get(wp, {})
            if not (meta.get("capture") and wp not in visited):
                continue                    # 순찰 지점이 아니거나 이미 찍음 → 통과만
            capture_ids.add(wp)
            parents.append(wp)
            pair = self.pair_of.get(wp)
            if pair is None:
                continue                    # 짝 없는 지점 — 한 방향만 찍고 끝
            if pair not in self.wp_meta:
                self._log.warn(
                    f"짝 {pair}(부모 {wp}) 좌표를 그래프에서 못 찾음 → 한쪽만 촬영")
                continue
            hadal.append(pair)
            capture_ids.add(pair)
        return hadal, capture_ids, parents

    def _mark_visited(self, hadal, capture_ids, parents, last_wp, code, visited):
        """이번 하달에서 '촬영까지 끝난' 순찰 지점을 방문 완료로 올린다.

        문서 20-1: 방문 마킹은 **짝의 촬영이 끝난 뒤** 부모 id 로 한다. 부모를 찍은 시점에
        마킹해버리면 그 직후 재계획이 끼어들었을 때 짝이 '이미 방문한 지점의 짝'이 되어
        영구 미촬영으로 남는다.

        code == 0 이면 배열을 끝까지 소화한 것이라 전부 인정한다. 중간에 끊겼으면 로봇이
        실제로 도달한 last_wp 까지만 인정하고, 짝이 있는 지점은 그 짝도 도달 범위 안에
        있어야 마킹한다.
        """
        if code == 0:
            done = set(hadal)
        elif last_wp is None:
            return
        else:
            try:
                j = hadal.index(last_wp)
            except ValueError:
                return                      # 배열 밖 노드 → 판정 불가, 아무것도 안 함
            done = set(hadal[:j + 1])
        for wp in parents:
            if wp not in done:
                continue
            pair = self.pair_of.get(wp)
            if pair is not None and pair in capture_ids and pair not in done:
                self._log.warn(
                    f"부모 {wp} 는 찍었으나 짝 {pair} 미촬영 → 방문 미완으로 남김")
                continue
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

#!/usr/bin/env python3
"""데모 촬영용 — 수확지에서 예냉실로 이송하고 바구니를 비우는 것만 따로 실행한다.

ACS 의 수확 task(E2~E6) 중 뒷부분(E5 이송 + E6 하역)만 떼어 낸 것이다. 정식 경로로는
'수확 접수 → 수확지 주행 → 도킹 → 수확 → 이송 → 하역' 전체를 돌려야 이 구간에 도달하는데,
영상에 담을 것이 이송·도킹·하역 세 장면뿐이라 앞단(수확)을 매번 기다릴 이유가 없다.

    수확지(HARVEST_01) ──주행──▶ 예냉실(PRECOOL_01) ──H마커 도킹──▶ 바구니 들어올리기

■ ACS 코드를 그대로 import 한다 (이 파일에 로직을 다시 쓰지 않는다)
경로탐색·통로예약·도킹·하역을 여기서 새로 구현하면 '이 스크립트가 잘 도는 것'만 증명되고
정작 검증하려는 ACS 코드는 하나도 검증되지 않는다(verify_web 과 같은 원칙). 그래서
RouteRunner·RoutingEngine·docking·HarvestDispatcher 를 실물 그대로 불러 쓴다 —
데모에서 움직인 경로는 ACS 가 수확 task 로 움직이는 경로와 같은 코드다.

■ 왜 패키지 밖(tools/)인가
ROS 노드가 아니라 실행 스크립트다. automato_control_service/ 안에 넣으면 colcon 빌드·
setup.py 에 얽히는데, 밖에 두면 find_packages() 가 잡지 않아 빌드 영향이 0 이다
(verify_web 을 밖에 둔 것과 같은 이유).

■ 먼저 띄워 둘 것 — 로봇 쪽 노드 3개
    1) ddago  : ros2 launch ddago_control ddago_bringup.launch.py dry_run:=false
                (주행 /ddago/navigate + 바닥 H마커 도킹 /ddago/floor_dock)
                ⚠ dry_run 기본값이 true 다. 그대로 두면 바퀴가 안 굴러간다.
    2) ddagi  : ros2 launch ddagi_harvest harvest_with_ai.launch.py with_ai:=false
                (하역 /ddagi/unload. 하역만 쓸 거라 AI·카메라는 띄우지 않는다)
                ⚠ 하역 경로(unload_path.json)를 그 로봇에서 티칭해 두어야 한다.
                   없으면 Goal 을 거절한다(teach_unload.py teach).
    3) dg     : ros2 run dg_control dcs_node --ros-args -p robot_id:=dg_01
                (ACS ↔ 로봇 중계. 이 스크립트가 붙는 액션이 여기서 열린다)

■ 실행 (워크스페이스가 둘이라 소싱도 둘. 나중 것이 앞에 오는 겹쳐쓰기라 순서가 중요하다)
    source /opt/ros/jazzy/setup.bash
    source ~/roscamp-repo-1/equip/automato_ws/install/setup.bash          # 액션 정의
    source ~/roscamp-repo-1/services/automato_control_service/install/setup.bash
    set -a; source ~/roscamp-repo-1/services/database/.env; set +a        # DATABASE_URL

    cd ~/roscamp-repo-1/services/automato_control_service
    python3 tools/goto_precool.py dg_01                    # 이송 → 도킹 → 하역
    python3 tools/goto_precool.py dg_01 --from HARVEST_02  # 두 번째 수확지에서 출발
    python3 tools/goto_precool.py dg_01 --no-unload        # 하역 빼고 리허설

  ROS_DOMAIN_ID 는 로봇에 맞춘 값이어야 액션이 보인다(로봇마다 다르다).

■ 조심할 점 3가지
  (1) ACS 를 동시에 돌리지 않는다. 이 스크립트는 자기만의 예약표(RoutingEngine)를 만든다.
      ACS 가 순찰을 돌리는 중이면 두 예약표가 서로를 못 보고 같은 통로에 두 로봇이 들어간다.
  (2) 도킹과 하역을 쪼개지 않는다. DG 는 '직전 도킹이 성공한 task_id' 의 하역만 받아준다
      (도킹 안 한 자리에서 팔이 움직이는 것을 막는 안전장치). 그래서 한 프로세스에서
      연달아 해야 하고, 사이에 다른 주행이 끼면 그 게이트가 닫힌다.
  (3) tasks/harvest_batches/unload_logs 에 아무것도 남기지 않는다. task_id 는 임의 번호
      (--task-id, 기본 9001)다. DG 의 게이트는 숫자 비교뿐이라 동작에는 지장이 없지만,
      실적·입고 기록이 필요한 검증이라면 정식 수확 접수(POST /internal/v1/tasks/harvest)를
      써야 한다. 이 스크립트는 촬영용이다.
"""
import argparse
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from automato_interfaces.action import Navigate, Unload

from automato_control_service import automato_db, docking
from automato_control_service.harvest_dispatcher import HarvestDispatcher
from automato_control_service.patrol_config import (
    RESERVATION_TTL_SEC,
    SERVER_WAIT_SEC,
)
from automato_control_service.route_runner import RouteRunner
from automato_control_service.routing_engine import RoutingEngine

# 임의 번호. tasks 에 이 행을 만들지 않으므로 DB 와 겹칠 걱정이 없고, 로그에서
# '데모로 움직인 것'을 한눈에 가르려 실제 task_id 와 멀찍이 띄운 값을 쓴다.
DEFAULT_TASK_ID = 9001
DEFAULT_FROM = "HARVEST_01"


def build_engine(pool, runner, log):
    """DB 그래프를 읽어 예약 엔진을 만들고 runner 에 좌표를 채운다. (automato_node._get_engine 과 동일)

    라우팅 그래프에 짝(pair)은 넣지 않는다 — 짝은 같은 자리에서 방향만 바꾸는 촬영 전용
    지점이라 통로(corridor)가 없어서, 그래프에 노드로 섞이면 Dijkstra 가 '도달할 수 없는
    목적지'를 후보로 잡는다. 반대로 wp_meta(좌표)에는 짝까지 전부 넣는다(하달에 좌표가 필요).
    """
    graph = automato_db.load_graph(pool)
    routing_nodes = [w for w in graph["waypoints"] if w["pair_of"] is None]
    engine = RoutingEngine(routing_nodes, graph["corridors"],
                           reservation_ttl=RESERVATION_TTL_SEC)
    runner.wp_meta = {
        w["waypoint_id"]: {"x": w["x"], "y": w["y"],
                           "yaw": w["yaw"], "capture": w["is_patrol_point"]}
        for w in graph["waypoints"]
    }
    log.info(f"[PRECOOL-DEMO] 라우팅 그래프 로드: 노드 {len(routing_nodes)} / "
             f"통로 {len(graph['corridors'])}")
    return engine


def resolve_points(pool, from_id, log):
    """출발 지점과 예냉실을 조회한다. 반환: (출발 dict, 예냉실 dict). 하나라도 없으면 (None, None).

    출발지를 task_point_id 로 받는 이유: 사람이 'HARVEST_01' 이라고 말하는 자리를 노드
    번호(20)로 옮기는 일을 손으로 하면 틀린다. 조회는 DB 가 유일한 출처다.
    """
    src = automato_db.get_task_point(pool, from_id)
    if src is None:
        log.error(f"[PRECOOL-DEMO] 출발 지점 {from_id} 가 task_points 에 없다")
        return None, None
    dst = automato_db.get_precool_point(pool)
    if dst is None:
        log.error("[PRECOOL-DEMO] 예냉실(point_type=PRECOOL)이 task_points 에 없다")
        return None, None
    log.info(f"[PRECOOL-DEMO] {src['task_point_id']}(노드 {src['waypoint_id']}) → "
             f"{dst['task_point_id']}(노드 {dst['waypoint_id']})")
    if src["point_type"] == "CHARGE":
        # 충전소는 15cm 칸 안에 후진 도킹돼 있어, 그대로 첫 구간을 하달하면 로봇이 그
        # 안에서 크게 돈다. 순찰만 언도킹 한 스텝(_lead_in)을 갖고 있고 이 스크립트에는
        # 없다 — 수확지 출발이 전제라서다.
        log.warn("[PRECOOL-DEMO] 충전소에서 출발한다 — 언도킹 절차가 없어 충전칸 안에서 "
                 "크게 돌 수 있다(수확지 출발 권장)")
    return src, dst


def run_demo(node, pool, robot_id, from_id, task_id, do_unload):
    """이송 → 도킹 → 하역을 순서대로. 반환: 성공 여부.

    ACS 의 수확 디스패처(E5~E6)와 같은 순서·같은 함수를 부른다. 다른 것은 딱 둘이다:
    앞단(수확)이 없고, DB 에 기록하지 않는다(save_* 콜백을 넘기지 않는다).
    """
    log = node.get_logger()
    runner = RouteRunner(log)
    engine = build_engine(pool, runner, log)
    src, dst = resolve_points(pool, from_id, log)
    if src is None:
        return False

    current = src["waypoint_id"]
    target = dst["waypoint_id"]
    label = dst["task_point_id"]
    if current not in runner.wp_meta or target not in runner.wp_meta:
        log.error(f"[PRECOOL-DEMO] 노드가 라우팅 그래프에 없다(출발 {current} / 목표 {target})")
        return False

    # 액션 클라이언트는 spin 시작 전에 만든다(ACS prewarm_clients 와 같은 이유 — 실행 중
    # rclpy 엔티티 생성을 피하고, 어디에 붙는지 기동 로그에 드러나게 한다).
    method = docking.method_for(label)          # PRECOOL_* → floor
    dock_type, dock_suffix = docking.action_spec(method)
    nav_client = ActionClient(node, Navigate, f"/{robot_id}/navigate")
    dock_client = ActionClient(node, dock_type, f"/{robot_id}/{dock_suffix}")
    unload_client = ActionClient(node, Unload, f"/{robot_id}/unload")
    log.info(f"[PRECOOL-DEMO] 액션 대상: /{robot_id}/navigate, "
             f"/{robot_id}/{dock_suffix}, /{robot_id}/unload")

    # drive 는 spin_wait(Event 대기)으로 결과를 받으므로, 누군가 다른 스레드에서 계속
    # spin 하고 있어야 한다. ACS main 과 같은 구성(백그라운드 executor + 작업 스레드).
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, name="rclpy_spin", daemon=True).start()

    try:
        # --- 출발 전 서버 점검: 쓸 액션 셋을 '미리 한꺼번에' 확인한다 ---
        # 정식 수확 흐름(harvest_dispatcher)은 Navigate 만 미리 보는데, 여기서는 셋을 다
        # 본다. 하역이 마지막 단계라, 안 보고 출발하면 이송·도킹을 다 끝낸 뒤에야
        # 'Ddagi 서버 없음'으로 실패한다 — 그러면 로봇을 수확지로 되돌려 놓고 다시 찍어야
        # 한다. 촬영 도구에서는 그 손실이 30초 대기보다 크다.
        # (안 걸러내면 Goal 수락 타임아웃을 다 기다린 뒤에야 실패한다는 것도 같은 이유다.)
        missing = [
            name for name, client in (
                (f"/{robot_id}/navigate", nav_client),
                (f"/{robot_id}/{dock_suffix}", dock_client),
                *(((f"/{robot_id}/unload", unload_client),) if do_unload else ()),
            )
            if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC)
        ]
        if missing:
            log.error(f"[PRECOOL-DEMO] 액션 서버 미기동: {', '.join(missing)} "
                      f"— dcs_node / ddago_bringup / harvest_with_ai 기동 확인")
            return False

        # --- ① 예냉실까지 이송 (통로 예약·막힘 우회 전부 ACS 것 그대로) ---
        log.info(f"[PRECOOL-DEMO] 이송 시작 task={task_id} {robot_id} {current} → {target}")
        outcome, current = runner.drive(
            engine, nav_client, task_id, robot_id, current, target)
        if outcome != "arrived":
            # skipped = 우회로가 없어 포기 / aborted = 로봇이 중단 보고.
            log.error(f"[PRECOOL-DEMO] 이송 실패({outcome}) — 로봇 위치 {current}")
            return False
        log.info(f"[PRECOOL-DEMO] 예냉실 도착 노드 {current}")

        # --- ② 바닥 H 마커 도킹 ---
        # heartbeat 를 넘기는 이유: 도킹은 마커 탐색~후진까지 수십 초가 걸리는데 그동안
        # 주행 하트비트가 멎어, 안 넘기면 예약 TTL(15초)에 걸려 '지금 서 있는 자리'가
        # 회수된다. 단독 실행이라 남이 들어올 일은 없지만, ACS 와 같은 호출 형태를
        # 유지해야 이 데모가 실제 경로를 검증하는 의미가 있다.
        slot = engine.node_slot(current)
        success, code, msg = docking.dock(
            log, task_id, robot_id, label, method, dock_client,
            heartbeat=(engine, [slot], robot_id))
        if not success:
            log.error(f"[PRECOOL-DEMO] 예냉실 도킹 실패(code={code}): {msg}")
            return False
        log.info(f"[PRECOOL-DEMO] 예냉실 도킹 완료 @ {label}")

        if not do_unload:
            log.info("[PRECOOL-DEMO] --no-unload 라 하역은 건너뛴다")
            return True

        # --- ③ 하역(바구니 들어올리기) ---
        # HarvestDispatcher._unload 를 그대로 쓴다. 하역은 '서버 대기 → Goal → phase
        # 피드백 → 타임아웃 → result_code 판정' 이 한 벌인데, 그 절차를 여기 옮겨 쓰면
        # ACS 와 갈라져 데모가 실제 동작을 대변하지 못한다.
        #   save_unload=None : 입고 기록을 남기지 않는다(그래서 앞의 집계 dict 는 쓰이지
        #                      않는다 — 개수는 실제로 딴 게 아니라 넘길 값도 없다).
        dispatcher = HarvestDispatcher(log, runner)
        ok = dispatcher._unload(task_id, robot_id, unload_client, {}, None,
                                heartbeat=(engine, [slot], robot_id))
        if not ok:
            # 정식 수확 task 에서 하역은 '보너스'라 실패해도 task 를 뒤집지 않는다. 하지만
            # 이 스크립트의 목적이 하역 장면이므로 여기서는 실패로 알린다.
            log.error("[PRECOOL-DEMO] 하역 실패 — 위 로그의 code/message 확인")
            return False
        log.info("[PRECOOL-DEMO] 하역 완료")
        return True
    finally:
        # drive 는 '지금 서 있는 자리'를 일부러 쥔 채 돌아온다(다음 구간이 이어받게).
        # 뒷단계가 없는 이 스크립트가 반납하지 않으면 그 자리가 예약된 채로 남는다
        # — 프로세스가 죽으면 예약표도 같이 사라지지만, 순서를 ACS 와 같게 둔다.
        engine.release(engine.node_slot(current), robot_id)
        log.info(f"[PRECOOL-DEMO] 노드 {current} 자리 반납")
        executor.shutdown()


def main():
    ap = argparse.ArgumentParser(
        description="데모용: 수확지 → 예냉실 이송 + 도킹 + 바구니 하역")
    ap.add_argument("robot_id",
                    help="DCS 의 robot_id (액션 이름 /{robot_id}/navigate 에 쓰인다)")
    ap.add_argument("--from", dest="from_id", default=DEFAULT_FROM,
                    help=f"출발 지점 task_point_id (기본 {DEFAULT_FROM})")
    ap.add_argument("--task-id", type=int, default=DEFAULT_TASK_ID,
                    help=f"액션에 실을 task_id (기본 {DEFAULT_TASK_ID}, DB 에 남기지 않음)")
    ap.add_argument("--no-unload", action="store_true",
                    help="하역을 건너뛴다(주행·도킹만 리허설)")
    args = ap.parse_args()

    rclpy.init()
    node = Node("precool_demo")
    pool = None
    ok = False
    try:
        pool = automato_db.create_pool()
        ok = run_demo(node, pool, args.robot_id, args.from_id,
                      args.task_id, not args.no_unload)
    except KeyboardInterrupt:
        node.get_logger().warn("[PRECOOL-DEMO] 중단(Ctrl+C) — 로봇은 그 자리에 선다")
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(f"[PRECOOL-DEMO] 예외로 종료: {exc}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if pool is not None:
            try:
                pool.close()
            except Exception:  # noqa: BLE001
                pass
    # 성공/실패를 종료코드로도 알린다 — 촬영 중엔 로그를 다 못 보므로 쉘에서 바로 갈린다.
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

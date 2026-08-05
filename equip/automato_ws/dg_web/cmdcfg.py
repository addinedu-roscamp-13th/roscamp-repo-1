#!/usr/bin/env python3
"""dg_web — 대시보드가 실행하는 명령의 정의·편집 저장소.

control_server.py(웹)와 dashboard.sh(CLI)가 **같은 정의 한 벌**을 쓰도록 여기 모은다.
예전에는 명령이 dashboard.sh 에 박혀 있어서, 로봇을 바꿔 config_file 하나 다르게
주려 해도 스크립트를 고쳐야 했다. 이제 대시보드에서 고칠 수 있다.

편집은 두 단계로 열려 있다(하이브리드).
  1) params  — 템플릿의 <이름> 자리에 들어갈 값만 바꾼다. 명령 구조는 그대로라 안전하다.
  2) cmdline — 명령줄 전체를 통째로 바꾼다. 이게 있으면 params 는 무시된다.
둘 다 되돌리기(reset)로 기본값 복귀. 기본값은 아래 DEFAULTS 가 단일 출처다.

치환 자리는 `{}` 가 아니라 `<>` 를 쓴다. ROS 액션 goal 이 `'{task_id: 1, ...}'` 처럼
중괄호 YAML 이라 `{}` 를 쓰면 서로 부딪힌다.

편집분만 commands.local.json 에 남는다(기본값과 같으면 파일에 안 쌓인다).
이 파일은 장비마다 다른 값이라 **커밋 대상이 아니다**(dg_ai_target.json 과 같은 성격).

⚠️ 최종 명령줄은 `bash -c` 로 실행된다. 즉 여기 값은 **셸 명령 그대로**다. 이 대시보드는
127.0.0.1 전용이라 그걸 전제로 열어 둔 것이고, 외부에 노출할 서버라면 절대 이렇게 두면 안 된다.

CLI (dashboard.sh 가 쓴다):
    python3 cmdcfg.py <key>     # 최종 명령줄 한 줄 출력
    python3 cmdcfg.py --list    # 키 목록
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OVERRIDES = os.path.join(HERE, 'commands.local.json')

# 명령 정의. group 은 대시보드에서 묶어 보여주는 단위일 뿐 실행에 영향이 없다.
# params 값의 ${ROBOT_ID} 는 bash 가 실행 시점에 펼친다(dashboard.sh 가 export 한다).
DEFAULTS = {
    # ── 노드 기동 (dashboard.sh start <노드>) ──
    'dcs': {
        'group': '노드 기동',
        'label': 'DCS (dg_control dcs_node)',
        'template': 'ros2 run dg_control dcs_node --ros-args -p robot_id:=<robot_id>',
        'params': {'robot_id': '${ROBOT_ID}'},
    },
    'acs': {
        'group': '노드 기동',
        'label': 'ACS 시뮬',
        'template': 'ros2 run dg_sim acs_sim --ros-args -p robot_id:=<robot_id>',
        'params': {'robot_id': '${ROBOT_ID}'},
    },
    'ddago': {
        'group': '노드 기동',
        'label': 'DdaGo 시뮬',
        'template': 'ros2 run dg_sim ddago_sim --ros-args -p robot_id:=<robot_id>',
        'params': {'robot_id': '${ROBOT_ID}'},
    },
    'ddagi': {
        'group': '노드 기동',
        'label': 'Ddagi 시뮬',
        'template': 'ros2 run dg_sim ddagi_sim --ros-args -p robot_id:=<robot_id>',
        'params': {'robot_id': '${ROBOT_ID}'},
    },
    'dg_ai': {
        'group': '노드 기동',
        'label': 'DG AI 시뮬 (TCP :9100)',
        'template': 'ros2 run dg_sim dg_ai_sim',
        'params': {},
    },
    'rosbridge': {
        'group': '노드 기동',
        'label': 'rosbridge (:9090)',
        'template': 'ros2 run rosbridge_server rosbridge_websocket',
        'params': {},
    },

    # ── 시나리오 트리거 (ACS 시뮬의 서비스를 부른다) ──
    'test': {
        'group': '시나리오 트리거',
        'label': 'E1·E2 순찰 시작',
        'template': 'ros2 service call /acs_sim/start_patrol std_srvs/srv/Trigger',
        'params': {},
    },
    'harvest-move': {
        'group': '시나리오 트리거',
        'label': 'S2 E2 수확 이동+도킹',
        'template': 'ros2 service call /acs_sim/start_harvest_move std_srvs/srv/Trigger',
        'params': {},
    },
    'harvest': {
        'group': '시나리오 트리거',
        'label': 'S2 E3 수확 시작',
        'template': 'ros2 service call /acs_sim/start_harvest std_srvs/srv/Trigger',
        'params': {},
    },
    'unload': {
        'group': '시나리오 트리거',
        'label': 'S2 E6 하역 시작',
        'template': 'ros2 service call /acs_sim/start_unload std_srvs/srv/Trigger',
        'params': {},
    },
    'return-dock': {
        'group': '시나리오 트리거',
        'label': 'E4 복귀 주행+충전소 도킹',
        'template': 'ros2 service call /acs_sim/start_return std_srvs/srv/Trigger',
        'params': {},
    },
    'telemetry': {
        'group': '시나리오 트리거',
        'label': 'E0 텔레메트리 시작',
        'template': ('ros2 service call /ddago_sim/start_telemetry std_srvs/srv/Trigger; '
                     'ros2 service call /ddagi_sim/start_telemetry std_srvs/srv/Trigger'),
        'params': {},
    },
    'telemetry-stop': {
        'group': '시나리오 트리거',
        'label': 'E0 텔레메트리 중지',
        'template': ('ros2 service call /ddago_sim/stop_telemetry std_srvs/srv/Trigger; '
                     'ros2 service call /ddagi_sim/stop_telemetry std_srvs/srv/Trigger'),
        'params': {},
    },

    # ── 도킹 단발 하달 (ACS 역할로 DCS 에 goal 을 직접 보낸다) ──
    # 마커리스(floor/reflective)는 goal 에 마커 규격이 없다. 목표 정차값 0 = 로봇 노드 기본값.
    'floor-dock': {
        'group': '도킹 단발 하달',
        'label': 'H마커 도킹 (수확지·예냉실)',
        'template': ("ros2 action send_goal /<robot_id>/floor_dock "
                     "automato_interfaces/action/FloorDock "
                     "'{task_id: <task_id>, task_point_id: <task_point_id>, "
                     "wall_gap_m: <wall_gap_m>, lateral_offset_m: <lateral_offset_m>}' --feedback"),
        'params': {'robot_id': '${ROBOT_ID}', 'task_id': '1024',
                   'task_point_id': 'HARVEST_01',
                   'wall_gap_m': '0.0', 'lateral_offset_m': '0.0'},
    },
    'reflective-dock': {
        'group': '도킹 단발 하달',
        'label': '반사테이프 도킹 (충전소)',
        'template': ("ros2 action send_goal /<robot_id>/reflective_dock "
                     "automato_interfaces/action/ReflectiveDock "
                     "'{task_id: <task_id>, task_point_id: <task_point_id>, "
                     "stop_gap_m: <stop_gap_m>}' --feedback"),
        'params': {'robot_id': '${ROBOT_ID}', 'task_id': '1024',
                   'task_point_id': 'CHARGE_01', 'stop_gap_m': '0.0'},
    },
    'dock': {
        'group': '도킹 단발 하달',
        'label': 'ChArUco 도킹 (휴면 경로, 수동 확인용)',
        'template': ("ros2 action send_goal /<robot_id>/dock automato_interfaces/action/Dock "
                     "'{task_id: <task_id>, task_point_id: <task_point_id>, "
                     "marker_id: \"<marker_id>\", dictionary: <dictionary>, "
                     "squares_x: <squares_x>, squares_y: <squares_y>, "
                     "square_size_m: <square_size_m>, marker_size_m: <marker_size_m>}' --feedback"),
        'params': {'robot_id': '${ROBOT_ID}', 'task_id': '1024',
                   'task_point_id': 'CHARGE_01', 'marker_id': '500',
                   'dictionary': 'DICT_5X5_1000', 'squares_x': '6', 'squares_y': '5',
                   'square_size_m': '0.024', 'marker_size_m': '0.018'},
    },
}

# ── 실서버(실장비) 프로파일 ───────────────────────────────────────────────
# 위 DEFAULTS 는 전부 시뮬 스택이다. 아래는 **실제로 도는 것**들이라 화면을 아예 나눈다
# (/real 페이지). 섞어 두면 실장비가 붙은 채로 시뮬 버튼을 눌러 액션 서버가 둘씩 생기는
# 사고가 난다 — 이름이 같은 서버가 둘이면 goal 이 어디로 갈지 알 수 없다.
#
# host: 'local'  = 이 노트북에서 실행
#       'robot'  = 로봇 온보드에서 실행(대시보드가 dg_agent 에 명령줄을 보낸다)
REAL_DEFAULTS = {
    # ── 노트북에서 도는 실서비스 ──
    'real-dcs': {
        'check': 'dg_control/lib/dg_control/dcs_node',
        # DCS 는 시뮬이든 실서버든 **하나만 띄우는 프로세스**다. 화면에 따라 로그가
        # /tmp/dash_dcs.log 와 /tmp/dash_real-dcs.log 로 갈리면 메시지 시계열이 반토막 난다.
        'log': 'dcs',
        'group': '실서버 · 노트북',
        'label': 'DG Control Service (실 DCS)',
        'template': 'ros2 run dg_control dcs_node --ros-args -p robot_id:=<robot_id>',
        'params': {'robot_id': '${ROBOT_ID}'},
    },
    'real-rosbridge': {
        'check': 'rosbridge_server/rosbridge_websocket',
        'group': '실서버 · 노트북',
        'label': 'rosbridge (:9090)',
        'template': 'ros2 run rosbridge_server rosbridge_websocket',
        'params': {},
    },

    'real-ai': {
        'check': 'dg_ai_service.analysis_server',
        'group': '실서버 · 노트북',
        'label': 'DG AI Service (실 YOLO 추론)',
        # 두 가지를 각각 다른 곳에서 가져온다.
        #  · dg_ai_service 모듈  → install/setup.bash 가 PYTHONPATH 에 넣어 준다(venv 아님)
        #  · ultralytics 등 추론 의존성 → automato_ws/.venv (uv 로 만든 것)
        # 빌드용 시스템 python 과 섞이지 않게 venv 를 따로 둔다. install/setup.bash 를
        # 명령 안에 넣어 두는 이유: 이 줄만 터미널에 복사해도 그대로 돌아야 한다
        # (대시보드로 띄울 때는 start-key 가 대신 해 주지만, 손으로 쓸 때 안 되면 헷갈린다).
        'template': ('source ${WS}/install/setup.bash && '
                     'source ${WS}/.venv/bin/activate && '
                     'python -m dg_ai_service.analysis_server'),
        'params': {},
    },
    # ↓ 이 둘은 사람마다·장비마다 띄우는 방법이 달라서 **명령 전체를 파라미터 하나로** 둔다.
    # 모양을 고정해 두면(예: bash -ic '<alias>') 스크립트를 부르고 싶을 때 그 틀을 못 벗어난다.
    # 아래 기본값은 어디까지나 출발점이고, 화면의 cmd 칸에서 통째로 바꿔 쓰면 된다.
    #   예) $HOME/nav.sh            (중복기동 방지·정상종료·status 를 이미 갖춘 스크립트)
    #       bash -ic 'nav2_ddago02' (alias 로 띄우고 싶을 때 — alias 는 대화형 셸에서만 펼쳐진다)
    #       ros2 launch pinky_navigation bringup_launch.xml ...
    'real-nav2': {
        'check': 'bt_navigator',    # 어떻게 띄우든 nav2 면 이 노드가 뜬다
        # nav.sh 는 nav2 를 백그라운드로 띄우고 **자기 로그 파일에** 쌓는다. 그래서 대시보드가
        # 잡는 실행 출력에는 스크립트가 찍은 몇 줄만 남고 정작 nav2 로그는 안 보인다.
        # 두 파일을 한 화면에 나란히 보여주려고 여기 같이 적어 둔다.
        # 장비 하나에 네임스페이스가 여럿 뜰 일이 없으므로 파일명은 고정으로 본다.
        'extra_logs': ['/tmp/nav_single.log'],
        'group': '실서버 · 노트북',
        'label': 'Nav2 (로봇 주행)',
        'template': '<cmd>',
        'params': {'cmd': '$HOME/nav.sh'},
    },
    'real-rviz': {
        'check': 'rviz2',
        'group': '실서버 · 노트북',
        'label': 'RViz (2D Pose Estimate 용)',
        'template': '<cmd>',
        'params': {'cmd': 'rviz2'},
    },
    # ── 로봇 온보드 (dg_agent 를 거쳐 실행) ──
    'robot-bringup': {
        'check': 'pinky_bringup bringup_robot.launch',
        'group': '실서버 · 로봇 온보드',
        'label': '로봇 bringup (라이다·모터·odom)',
        'host': 'robot',
        'template': 'ros2 launch pinky_bringup bringup_robot.launch.xml',
        'params': {},
    },
    'robot-ddago': {
        'check': 'ddago_bringup.launch',
        'group': '실서버 · 로봇 온보드',
        'label': 'DdaGo Control (navigate·telemetry·도킹 2종)',
        'host': 'robot',
        # config_file 기본값이 ddago01.yaml 이라 로봇을 바꾸면 반드시 지정해야 한다.
        # floor_calib_file 은 로봇에서 직접 캘리브한 값이라 로봇마다 다르다.
        # ⚠ 로봇 .bashrc 는 ROS·pinky_pro 까지만 자동 source 하고 automato_ws 는
        # alias(automato)로만 두고 있다. 그래서 ddago_control 을 찾으려면 여기서 직접
        # source 해야 한다 — 안 하면 'package not found' 로 그냥 실패한다.
        'template': ('source <ws_setup> && '
                     'ros2 launch ddago_control ddago_bringup.launch.py '
                     'robot_id:=<robot_id> dry_run:=<dry_run> '
                     'config_file:=$(ros2 pkg prefix ddago_control)'
                     '/share/ddago_control/config/reflective_dock/<config_yaml> '
                     'floor_calib_file:=<floor_calib>'),
        # ⚠ 여기서만 ${ROBOT_ID} 를 쓰면 안 된다. 로봇 명령은 **로봇에서** 펼쳐지는데
        # 로봇 .bashrc 의 ROBOT_ID 는 장비 이름(ddago02)이라 노트북 값(dg_02)과 다르다.
        # 그래서 대시보드 쪽 값을 여기서 확정해 넣는다(화면에도 실제 값이 그대로 보인다).
        # robot_id 자체는 로봇에서 로그 표기용이지만, 노트북과 달라 보이면 로그를 맞대 볼 때 헷갈린다.
        'params': {'ws_setup': '$HOME/roscamp-repo-1/equip/automato_ws/install/setup.bash',
                   'robot_id': os.environ.get('ROBOT_ID', 'dg_01'), 'dry_run': 'false',
                   'config_yaml': 'ddago02.yaml',
                   'floor_calib': '$HOME/floor_dock_ws/floor_calib.npz'},
    },
    # ── 충전소 복귀 (ACS 없이 사람이 직접) ──
    # oneshot: 켜고 끄는 서비스가 아니라 **한 번 돌고 끝나는 동작**이다. 서비스 카드에
    # 뜨면 토글 버튼 모델과 안 맞으므로 real_server 가 목록에서 걸러낸다.
    # 좌표(x·y·yaw)는 실행 시점에 effective(..., extra=) 로 주입한다 — 파일에 저장하면
    # 실행마다 commands.local.json 이 바뀌고 동시 실행에서 레이스가 난다.
    'real-return-nav': {
        'oneshot': True,
        'check': 'action send_goal .*navigate',
        'group': '실서버 · 충전소 복귀',
        'label': '① 복귀 주행 (DCS 경유 Navigate)',
        # ACS 가 하던 것과 같은 인터페이스를 탄다. waypoint 하나(capture:false)만 주면
        # 경로는 Nav2 가 짠다. DCS→DdaGo 중계를 거치므로 navigate_server 의 출발 정렬·
        # yaw 보정을 그대로 받고, 취소 중계도 이미 구현돼 있다.
        'template': ("ros2 action send_goal /<robot_id>/navigate "
                     "automato_interfaces/action/Navigate "
                     "'{task_id: <task_id>, waypoints: [{waypoint_id: <wp_id>, "
                     "x: <x>, y: <y>, yaw: <yaw>, capture: false}]}' --feedback"),
        'params': {'robot_id': '${ROBOT_ID}', 'task_id': '0', 'wp_id': '0',
                   'x': '0.0', 'y': '0.0', 'yaw': '0.0'},
    },
    'real-return-dock': {
        'oneshot': True,
        'check': 'action send_goal .*reflective_dock',
        'group': '실서버 · 충전소 복귀',
        'label': '② 충전소 도킹 (ReflectiveDock)',
        # 마커리스 도킹이라 goal 에 좌표가 없다. 정차값 0 = 로봇 config yaml 의 기본값.
        'template': ("ros2 action send_goal /<robot_id>/reflective_dock "
                     "automato_interfaces/action/ReflectiveDock "
                     "'{task_id: <task_id>, task_point_id: <task_point_id>, "
                     "stop_gap_m: <stop_gap_m>}' --feedback"),
        'params': {'robot_id': '${ROBOT_ID}', 'task_id': '0',
                   'task_point_id': 'CHARGE_01', 'stop_gap_m': '0.0'},
    },
}

# 프로파일·호스트 기본값을 한 번에 채운다(항목마다 적으면 빠뜨리기 쉽다).
for _d in DEFAULTS.values():
    _d.setdefault('profile', 'sim')
    _d.setdefault('host', 'local')
    _d.setdefault('oneshot', False)
for _d in REAL_DEFAULTS.values():
    _d.setdefault('profile', 'real')
    _d.setdefault('host', 'local')
    _d.setdefault('oneshot', False)
DEFAULTS.update(REAL_DEFAULTS)

_PLACEHOLDER = re.compile(r'<([a-z_][a-z0-9_]*)>')


def load_overrides():
    """편집분을 읽는다. 파일이 없거나 깨졌으면 '편집 없음'으로 본다
    (대시보드가 못 뜨는 것보다 기본값으로 도는 편이 낫다)."""
    try:
        with open(OVERRIDES, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_overrides(data):
    with open(OVERRIDES, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False, sort_keys=True)


def params_of(key, overrides=None):
    """기본 params 에 편집분을 덮어쓴 '현재 값'. 정의에 없는 이름은 버린다
    (템플릿이 바뀌어 사라진 파라미터가 편집분에 남아 계속 따라다니지 않게)."""
    base = dict(DEFAULTS[key]['params'])
    ov = (overrides if overrides is not None else load_overrides()).get(key) or {}
    for name, val in (ov.get('params') or {}).items():
        if name in base:
            base[name] = str(val)
    return base


def cmdline_of(key, overrides=None):
    """명령줄 전체 편집분. 없으면 None."""
    ov = (overrides if overrides is not None else load_overrides()).get(key) or {}
    line = (ov.get('cmdline') or '').strip()
    return line or None


def effective(key, overrides=None, extra=None):
    """실제로 실행될 명령줄. cmdline 편집이 있으면 그것, 없으면 템플릿+params.

    extra 는 **실행 시점에만** 정해지는 값(복귀 좌표 등)을 끼워 넣는다. 파일에 저장하지
    않으므로 편집분이 실행마다 바뀌지 않는다.
    ⚠ cmdline(명령줄 전체 편집)이 있으면 extra 는 무시된다 — 좌표가 안 들어간 명령이
    그대로 나간다. 이건 서버 사전점검에서 막는다(cmdcfg 는 정의 저장소일 뿐이다)."""
    if key not in DEFAULTS:
        raise KeyError(key)
    ov = overrides if overrides is not None else load_overrides()
    line = cmdline_of(key, ov)
    if line:
        return line
    vals = params_of(key, ov)
    if extra:
        # 정의에 없는 이름은 버린다(params_of 와 같은 규칙)
        vals.update({k: str(v) for k, v in extra.items() if k in vals})
    # 정의에 없는 자리표시자는 손대지 않고 그대로 둔다 — 조용히 지우면 원인 찾기 어렵다.
    return _PLACEHOLDER.sub(lambda m: vals.get(m.group(1), m.group(0)),
                            DEFAULTS[key]['template'])


def describe(key, overrides=None):
    ov = overrides if overrides is not None else load_overrides()
    d = DEFAULTS[key]
    line = cmdline_of(key, ov)
    cur = params_of(key, ov)
    return {
        'key': key,
        'group': d['group'],
        'label': d['label'],
        'profile': d['profile'],
        'host': d['host'],
        'oneshot': d['oneshot'],
        'template': d['template'],
        'default_params': d['params'],
        'params': cur,
        'log_tag': log_tag(key),
        'cmdline': line or '',
        'effective': effective(key, ov),
        'edited': bool(line) or cur != d['params'],
    }


def log_tag(key):
    """이 명령의 로그 파일 꼬리표(/tmp/dash_<태그>.log). 기본은 키 이름이고,
    다른 명령과 **같은 프로세스**를 쓰는 항목만 log 로 따로 지정해 한 파일로 모은다."""
    return DEFAULTS[key].get('log') or key


def describe_all(profile=None):
    """profile 을 주면 그 프로파일(sim/real)만. 시뮬 화면과 실서버 화면이 서로의 명령을
    보지 못하게 하는 것이 이 분리의 목적이다."""
    ov = load_overrides()
    return [describe(k, ov) for k in DEFAULTS
            if profile is None or DEFAULTS[k]['profile'] == profile]


def set_command(key, params=None, cmdline=None):
    """편집분 저장. params 는 준 이름만 갱신한다. cmdline 은 빈 문자열이면 해제.
    기본값과 같아지면 편집분에서 지워 파일이 군더더기 없이 남게 한다."""
    if key not in DEFAULTS:
        raise KeyError(key)
    ov = load_overrides()
    entry = dict(ov.get(key) or {})

    if params:
        cur = dict(entry.get('params') or {})
        for name, val in params.items():
            if name in DEFAULTS[key]['params']:
                cur[name] = str(val)
        # 기본값과 같은 항목은 편집분에서 뺀다
        cur = {n: v for n, v in cur.items() if v != DEFAULTS[key]['params'][n]}
        if cur:
            entry['params'] = cur
        else:
            entry.pop('params', None)

    if cmdline is not None:
        line = str(cmdline).strip()
        if line:
            entry['cmdline'] = line
        else:
            entry.pop('cmdline', None)

    if entry:
        ov[key] = entry
    else:
        ov.pop(key, None)
    save_overrides(ov)
    return describe(key)


def reset(key):
    """이 명령의 편집분을 전부 버리고 기본값으로."""
    if key not in DEFAULTS:
        raise KeyError(key)
    ov = load_overrides()
    if ov.pop(key, None) is not None:
        save_overrides(ov)
    return describe(key)


if __name__ == '__main__':
    arg = sys.argv[1] if len(sys.argv) > 1 else ''
    if arg == '--list':
        print('\n'.join(DEFAULTS))
    elif arg == '--log':
        k = sys.argv[2] if len(sys.argv) > 2 else ''
        if k not in DEFAULTS:
            sys.stderr.write('cmdcfg: 알 수 없는 명령 키: %r\n' % k)
            sys.exit(1)
        print(log_tag(k))
    elif arg in DEFAULTS:
        print(effective(arg))
    else:
        sys.stderr.write('cmdcfg: 알 수 없는 명령 키: %r\n' % arg)
        sys.exit(1)

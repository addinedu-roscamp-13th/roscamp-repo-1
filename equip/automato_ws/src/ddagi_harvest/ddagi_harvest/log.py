#!/usr/bin/env python3
"""수확 로직의 진단 출력 싱크 — 단독 실행은 print, ROS 노드는 노드 로거로.

파지·루프 코드는 print() 대신 이 모듈의 log()/warn()/error() 를 부른다. 기본
싱크는 print 라 `python3 harvest.py` 단독 실행이 그대로 동작하고, 노드는 기동 시
set_sink() 로 노드 로거를 꽂아 journald·`ros2 launch` 출력에 남게 한다.

왜 필요한가: print 는 터미널이 아닐 때 파이썬이 블록 버퍼링을 해서 프로세스가 끝나야
쏟아진다. 액션 서버로 돌리면 터미널을 붙일 수 없어 **실패 원인이 실시간으로 안 보인다**
(실측: 4분 30초짜리 수확에서 노드 로그가 868바이트 — 파지 로그가 전부 버퍼에 갇혔다).

    from ddagi_harvest.log import log, warn, error
    log("배치 3개")
    warn("standoff 확보 실패")

노드 쪽:
    from ddagi_harvest import log as L
    L.set_sink(node.get_logger().info, node.get_logger().warning,
               node.get_logger().error)
"""
from __future__ import annotations

_info = print
_warn = print
_error = print


def set_sink(info=None, warning=None, err=None) -> None:
    """출력 싱크 교체. 인자를 생략하면 그 레벨은 그대로 둔다."""
    global _info, _warn, _error
    if info is not None:
        _info = info
    if warning is not None:
        _warn = warning
    if err is not None:
        _error = err


def reset_sink() -> None:
    """print 로 되돌린다(테스트·단독 실행용)."""
    global _info, _warn, _error
    _info = _warn = _error = print


def _fmt(parts) -> str:
    """앞뒤 개행을 턴다 — 로거가 줄바꿈을 스스로 붙이는데, 앞에 \\n 이 붙은 메시지는
    레벨·타임스탬프 접두 없이 맨 줄로 나가 필터링(grep '\\[INFO\\]')에서 새어나간다.
    단독 실행(print)에서는 문단 구분이 조금 촘촘해질 뿐이라 손해가 없다."""
    return " ".join(str(p) for p in parts).strip("\n")


def log(*parts) -> None:
    _info(_fmt(parts))


def warn(*parts) -> None:
    _warn(_fmt(parts))


def error(*parts) -> None:
    _error(_fmt(parts))

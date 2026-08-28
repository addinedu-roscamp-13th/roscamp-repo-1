"""엔트리 포인트: QApplication + ROS 워커 스레드 부트스트랩."""
from __future__ import annotations

import signal
import sys

from PyQt6.QtWidgets import QApplication

from .ros.ros_worker import RosWorker
from .ui.main_window import MainWindow


def main(args=None) -> int:
    # Ctrl+C로 종료 가능하게
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv if args is None else args)

    worker = RosWorker()
    window = MainWindow(worker)
    worker.start()          # ROS spin 스레드 시작
    window.show()

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

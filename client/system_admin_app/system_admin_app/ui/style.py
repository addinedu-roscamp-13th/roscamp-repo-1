"""색상 팔레트와 QSS 테마. 참고 이미지(팜 그린)를 따른다."""

# 상태 레벨 색상
LEVEL_COLOR = {
    "ok":   "#2e7d32",   # green
    "warn": "#ef6c00",   # orange
    "crit": "#c62828",   # red
    "idle": "#9e9e9e",   # gray (미지원/미수신)
}

# 그래프용 로봇별 색상 (참고 이미지: DG1 파랑, DG2 빨강, DG3 초록)
ROBOT_COLOR = {
    "dg_01": "#1e88e5",
    "dg_02": "#e53935",
    "dg_03": "#43a047",
}

GREEN_DARK = "#14351f"
GREEN = "#1b5e20"

QSS = """
QWidget { font-family: 'Noto Sans CJK KR', 'Malgun Gothic', sans-serif; font-size: 13px; }
QMainWindow, QTabWidget::pane { background: #f4f6f4; }

#TopBar { background: %(green_dark)s; color: #eaf3ea; }
#TopBar QLabel { color: #eaf3ea; }

QTabBar::tab {
    padding: 8px 18px; background: #e3e9e3; color: #33413a;
    border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px;
}
QTabBar::tab:selected { background: %(green)s; color: white; }

#DGCard {
    background: white; border: 1px solid #dde3dd; border-radius: 10px;
}
#CardTitle { font-size: 16px; font-weight: 700; color: #22331f; }
#SectionLabel { color: #6b7b6b; font-weight: 600; }

QFrame[class="Tile"] {
    background: #f6faf6; border: 1px solid #e3ebe3; border-radius: 8px;
}
QFrame[class="TileUnsupported"] {
    background: #f2f2f2; border: 1px dashed #cfcfcf; border-radius: 8px;
}

#WarnPanel {
    background: #fdecea; border: 1px solid #f5c6c0; border-radius: 8px; color: #b71c1c;
    padding: 6px 10px;
}

QPushButton {
    padding: 7px 12px; border-radius: 6px; background: #e8efe8; border: 1px solid #cfd9cf;
}
QPushButton:hover { background: #dbe6db; }
QPushButton:pressed { background: #cfe0cf; }
QPushButton#Estop {
    background: #c62828; color: white; font-weight: 700; border: none;
}
QPushButton#Estop:hover { background: #b71c1c; }

#Log { background: #101512; color: #cfe8cf; font-family: monospace; border-radius: 6px; }
""" % {"green_dark": GREEN_DARK, "green": GREEN}


def badge_qss(level: str) -> str:
    color = LEVEL_COLOR.get(level, LEVEL_COLOR["idle"])
    return (
        f"background:{color}; color:white; border-radius:10px; "
        f"padding:2px 10px; font-weight:700;"
    )

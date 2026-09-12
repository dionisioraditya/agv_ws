"""Modern Industrial Dark Theme Stylesheet for AGV HMI."""

MODERN_DARK_THEME = """
/* Global Window & Fonts */
QWidget {
    background-color: #13141f;
    color: #e2e8f0;
    font-family: 'Segoe UI', 'Ubuntu', 'Helvetica Neue', sans-serif;
    font-size: 13px;
    selection-background-color: #4f46e5;
    selection-color: #ffffff;
}

/* Card Panels */
QFrame#Card {
    background-color: #1c1d2e;
    border: 1px solid #2e3148;
    border-radius: 12px;
    padding: 12px;
}

QFrame#HeaderCard {
    background-color: #1e2033;
    border: 1px solid #333752;
    border-radius: 12px;
    padding: 14px;
}

/* Group Boxes */
QGroupBox {
    background-color: #1c1d2e;
    border: 1px solid #2e3148;
    border-radius: 12px;
    margin-top: 24px;
    padding: 16px;
    font-weight: bold;
    font-size: 13px;
    color: #94a3b8;
}

QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 0 8px;
    background-color: transparent;
    color: #38bdf8;
}

/* Labels */
QLabel#TitleLabel {
    font-size: 18px;
    font-weight: bold;
    color: #f8fafc;
}

QLabel#SubtitleLabel {
    font-size: 12px;
    color: #94a3b8;
}

QLabel#SectionLabel {
    font-size: 14px;
    font-weight: bold;
    color: #38bdf8;
    letter-spacing: 0.5px;
}

QLabel#ValueLabel {
    font-size: 15px;
    font-weight: bold;
    color: #f1f5f9;
}

/* Status Pill */
QLabel#StatusPill {
    font-size: 13px;
    font-weight: bold;
    padding: 6px 14px;
    border-radius: 14px;
    background-color: #334155;
    color: #cbd5e1;
    border: 1px solid #475569;
}

/* Buttons */
QPushButton {
    background-color: #2a2d42;
    color: #f1f5f9;
    border: 1px solid #3d4263;
    border-radius: 8px;
    padding: 8px 16px;
    font-weight: 600;
    min-height: 22px;
}

QPushButton:hover {
    background-color: #383c5a;
    border-color: #4f557d;
}

QPushButton:pressed {
    background-color: #222538;
}

QPushButton:disabled {
    background-color: #1e202f;
    color: #64748b;
    border-color: #272a3e;
}

/* Start Button (Emerald Glow) */
QPushButton#StartButton {
    background-color: #059669;
    color: #ffffff;
    border: 1px solid #10b981;
    border-radius: 10px;
    font-size: 15px;
    font-weight: bold;
    padding: 12px 24px;
}

QPushButton#StartButton:hover {
    background-color: #10b981;
    border-color: #34d399;
}

QPushButton#StartButton:pressed {
    background-color: #047857;
}

/* Cancel / Danger Button */
QPushButton#DangerButton {
    background-color: #dc2626;
    color: #ffffff;
    border: 1px solid #ef4444;
    border-radius: 10px;
    font-size: 14px;
    font-weight: bold;
    padding: 12px 20px;
}

QPushButton#DangerButton:hover {
    background-color: #ef4444;
    border-color: #f87171;
}

QPushButton#DangerButton:pressed {
    background-color: #b91c1c;
}

/* Primary Action Button */
QPushButton#PrimaryButton {
    background-color: #4f46e5;
    color: #ffffff;
    border: 1px solid #6366f1;
    border-radius: 8px;
    font-size: 13px;
    font-weight: bold;
    padding: 8px 16px;
}

QPushButton#PrimaryButton:hover {
    background-color: #6366f1;
}

QPushButton#PrimaryButton:pressed {
    background-color: #4338ca;
}

/* Waypoint Add Chip */
QPushButton#WaypointChip {
    background-color: #22253b;
    color: #e2e8f0;
    border: 1px solid #383e5e;
    border-radius: 8px;
    padding: 10px 14px;
    font-size: 13px;
    text-align: left;
    font-weight: 600;
}

QPushButton#WaypointChip:hover {
    background-color: #2e3352;
    border-color: #38bdf8;
    color: #38bdf8;
}

/* Lists and Tables */
QListWidget, QTableWidget {
    background-color: #181928;
    border: 1px solid #2e3148;
    border-radius: 10px;
    padding: 6px;
    color: #f1f5f9;
    gridline-color: #272a3e;
    font-size: 13px;
}

QListWidget::item {
    background-color: #22253b;
    border: 1px solid #313652;
    border-radius: 8px;
    padding: 8px 12px;
    margin-bottom: 4px;
    color: #f1f5f9;
}

QListWidget::item:hover {
    background-color: #2b304c;
    border-color: #475079;
}

QListWidget::item:selected {
    background-color: #343c68;
    border: 1px solid #6366f1;
    color: #ffffff;
}

QTableWidget::item {
    padding: 6px;
    border-bottom: 1px solid #23263b;
}

QTableWidget::item:selected {
    background-color: #313861;
    color: #ffffff;
}

QHeaderView::section {
    background-color: #1f2134;
    color: #94a3b8;
    padding: 8px;
    border: none;
    border-bottom: 2px solid #333854;
    font-weight: bold;
    font-size: 12px;
    text-transform: uppercase;
}

/* Line Edit */
QLineEdit {
    background-color: #181928;
    border: 1px solid #333752;
    border-radius: 8px;
    padding: 8px 12px;
    color: #f8fafc;
    font-size: 13px;
}

QLineEdit:focus {
    border: 1px solid #38bdf8;
    background-color: #1e2033;
}

/* Scrollbars */
QScrollBar:vertical {
    border: none;
    background: #151622;
    width: 10px;
    margin: 0px;
    border-radius: 5px;
}

QScrollBar::handle:vertical {
    background: #333752;
    min-height: 20px;
    border-radius: 5px;
}

QScrollBar::handle:vertical:hover {
    background: #4a5075;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
"""


def get_status_style(state_name: str) -> str:
    """Return styling tuple (bg_color, border_color, text_color) based on FSM state."""
    state = state_name.upper()
    if state == "NAVIGATING":
        return "background-color: #064e3b; border: 1px solid #10b981; color: #34d399;"
    elif state in ("MISSION_RECEIVED", "PREPARING"):
        return "background-color: #1e3a8a; border: 1px solid #3b82f6; color: #93c5fd;"
    elif state == "MISSION_COMPLETED":
        return "background-color: #065f46; border: 1px solid #34d399; color: #a7f3d0;"
    elif state in ("MISSION_FAILED", "EMERGENCY"):
        return "background-color: #7f1d1d; border: 1px solid #ef4444; color: #fca5a5;"
    elif state == "MISSION_CANCELED":
        return "background-color: #78350f; border: 1px solid #f59e0b; color: #fde68a;"
    else:  # IDLE
        return "background-color: #27273a; border: 1px solid #434461; color: #94a3b8;"

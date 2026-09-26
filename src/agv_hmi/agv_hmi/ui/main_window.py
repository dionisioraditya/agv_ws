#!/usr/bin/env python3
from PyQt5.QtWidgets import (
    QMainWindow, QStackedWidget, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QPushButton, QFrame
)
from PyQt5.QtCore import Qt, QEvent

from agv_hmi.storage import WaypointStorage
from agv_hmi.ros_worker import RosWorker
from agv_hmi.ui.styles import MODERN_DARK_THEME
from agv_hmi.ui.dashboard_page import DashboardPage
from agv_hmi.ui.settings_page import SettingsPage


class MainWindow(QMainWindow):
    """Main Application Window hosting Dashboard and Settings pages with Window Controls."""

    def __init__(self, storage: WaypointStorage, ros_worker: RosWorker):
        super().__init__()
        self.storage = storage
        self.ros_worker = ros_worker

        self.setWindowTitle("AGV Mission Manager & Waypoint HMI")
        # Locked target resolution: 1024 x 600
        self.resize(1024, 600)
        self.setMinimumSize(1024, 600)

        # Ensure native window decorations have minimize, maximize, and close
        self.setWindowFlags(
            Qt.Window |
            Qt.WindowMinimizeButtonHint |
            Qt.WindowMaximizeButtonHint |
            Qt.WindowCloseButtonHint
        )

        # Apply Global Modern Dark Theme
        self.setStyleSheet(MODERN_DARK_THEME)

        self._init_ui()

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ----------------- Top Title & Window Control Bar -----------------
        self.title_bar = QFrame()
        self.title_bar.setObjectName("WindowTitleBar")
        title_bar_layout = QHBoxLayout(self.title_bar)
        title_bar_layout.setContentsMargins(16, 4, 12, 4)
        title_bar_layout.setSpacing(8)

        # Title Branding
        brand_lbl = QLabel("🤖  AGV MISSION MANAGER & WAYPOINT HMI")
        brand_lbl.setStyleSheet(
            "font-size: 12px; font-weight: bold; color: #94a3b8; letter-spacing: 0.5px;"
        )
        title_bar_layout.addWidget(brand_lbl)
        title_bar_layout.addStretch()

        # Window Control Buttons (Minimize, Fullscreen/Restore, Close)
        self.btn_min = QPushButton("─")
        self.btn_min.setObjectName("WindowControlBtn")
        self.btn_min.setToolTip("Minimize")
        self.btn_min.setCursor(Qt.PointingHandCursor)
        self.btn_min.clicked.connect(self.showMinimized)
        title_bar_layout.addWidget(self.btn_min)

        self.btn_fullscreen = QPushButton("⛶")
        self.btn_fullscreen.setObjectName("WindowControlBtn")
        self.btn_fullscreen.setToolTip("Layar Penuh / Fullscreen (F11)")
        self.btn_fullscreen.setCursor(Qt.PointingHandCursor)
        self.btn_fullscreen.clicked.connect(self.toggle_fullscreen)
        title_bar_layout.addWidget(self.btn_fullscreen)

        self.btn_close = QPushButton("✕")
        self.btn_close.setObjectName("WindowCloseBtn")
        self.btn_close.setToolTip("Tutup Aplikasi")
        self.btn_close.setCursor(Qt.PointingHandCursor)
        self.btn_close.clicked.connect(self.close)
        title_bar_layout.addWidget(self.btn_close)

        main_layout.addWidget(self.title_bar)

        # ----------------- Pages Stack -----------------
        self.stacked_widget = QStackedWidget()
        main_layout.addWidget(self.stacked_widget)

        # Page 0: Front Panel / Dashboard
        self.dashboard_page = DashboardPage(self.storage, self.ros_worker)
        self.stacked_widget.addWidget(self.dashboard_page)

        # Page 1: Settings / Waypoint Management
        self.settings_page = SettingsPage(self.storage, self.ros_worker)
        self.stacked_widget.addWidget(self.settings_page)

        # Connect page transitions
        self.dashboard_page.switch_to_settings.connect(self._goto_settings)
        self.settings_page.switch_to_dashboard.connect(self._goto_dashboard)

        # Automatically refresh dashboard waypoint bank when settings modify points
        self.settings_page.waypoints_modified.connect(self.dashboard_page.refresh_waypoints_bank)

    def toggle_fullscreen(self):
        """Toggle between Fullscreen and locked 1024x600 Windowed mode."""
        if self.isFullScreen():
            self.showNormal()
            self.setFixedSize(1024, 600)
            self.btn_fullscreen.setText("⛶")
            self.btn_fullscreen.setToolTip("Layar Penuh / Fullscreen (F11)")
        else:
            self.setMaximumSize(16777215, 16777215)
            self.setMinimumSize(0, 0)
            self.showFullScreen()
            self.btn_fullscreen.setText("🗗")
            self.btn_fullscreen.setToolTip("Keluar Fullscreen / Mode Jendela 1024x600 (F11 / Esc)")

    def changeEvent(self, event):
        """Detect window state changes (maximize/fullscreen/restore)."""
        super().changeEvent(event)
        if event.type() == QEvent.WindowStateChange:
            if self.isFullScreen():
                self.btn_fullscreen.setText("🗗")
                self.btn_fullscreen.setToolTip("Keluar Fullscreen / Mode Jendela (F11 / Esc)")
            else:
                self.btn_fullscreen.setText("⛶")
                self.btn_fullscreen.setToolTip("Layar Penuh / Fullscreen (F11)")

    def keyPressEvent(self, event):
        """Support F11 and Escape shortcut for fullscreen toggle."""
        if event.key() == Qt.Key_F11:
            self.toggle_fullscreen()
            event.accept()
        elif event.key() == Qt.Key_Escape and self.isFullScreen():
            self.toggle_fullscreen()
            event.accept()
        else:
            super().keyPressEvent(event)

    def _goto_settings(self):
        self.settings_page.load_table_data()
        self.stacked_widget.setCurrentIndex(1)

    def _goto_dashboard(self):
        self.dashboard_page.refresh_waypoints_bank()
        self.stacked_widget.setCurrentIndex(0)

    def closeEvent(self, event):
        if self.ros_worker:
            self.ros_worker.stop()
        event.accept()

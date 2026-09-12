#!/usr/bin/env python3
from PyQt5.QtWidgets import QMainWindow, QStackedWidget, QWidget, QVBoxLayout
from PyQt5.QtCore import Qt

from agv_hmi.storage import WaypointStorage
from agv_hmi.ros_worker import RosWorker
from agv_hmi.ui.styles import MODERN_DARK_THEME
from agv_hmi.ui.dashboard_page import DashboardPage
from agv_hmi.ui.settings_page import SettingsPage


class MainWindow(QMainWindow):
    """Main Application Window hosting Dashboard and Settings pages."""

    def __init__(self, storage: WaypointStorage, ros_worker: RosWorker):
        super().__init__()
        self.storage = storage
        self.ros_worker = ros_worker

        self.setWindowTitle("AGV Mission Manager & Waypoint HMI")
        self.resize(1080, 720)
        self.setMinimumSize(900, 600)

        # Apply Global Modern Dark Theme
        self.setStyleSheet(MODERN_DARK_THEME)

        self._init_ui()

    def _init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # QStackedWidget for multi-page navigation
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

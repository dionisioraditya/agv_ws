#!/usr/bin/env python3
from typing import List
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QListWidget, QListWidgetItem, QFrame, QScrollArea, QMessageBox
)
from PyQt5.QtCore import Qt, pyqtSignal

from agv_hmi.storage import WaypointStorage
from agv_hmi.ui.styles import get_status_style


class DashboardPage(QWidget):
    """Front Panel Page: Task Queue creation, Mission Start/Cancel, and Telemetry."""

    switch_to_settings = pyqtSignal()

    def __init__(self, storage: WaypointStorage, ros_worker, parent=None):
        super().__init__(parent)
        self.storage = storage
        self.ros_worker = ros_worker

        self.current_state_name = "IDLE"
        self._init_ui()
        self._connect_signals()
        self.refresh_waypoints_bank()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(16)

        # ----------------- 1. HEADER CARD (Telemetry & FSM State) -----------------
        header_card = QFrame()
        header_card.setObjectName("HeaderCard")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(16, 12, 16, 12)

        # Left Header: App Title & Mission Info
        left_header = QVBoxLayout()
        title_label = QLabel("AGV MISSION CONTROL")
        title_label.setObjectName("TitleLabel")
        self.active_mission_label = QLabel("Active Mission: None | Current Goal: -")
        self.active_mission_label.setObjectName("SubtitleLabel")
        left_header.addWidget(title_label)
        left_header.addWidget(self.active_mission_label)
        header_layout.addLayout(left_header, stretch=2)

        # Middle Header: Robot Live Coordinates
        pose_box = QVBoxLayout()
        pose_title = QLabel("ROBOT GLOBAL POSE (TF)")
        pose_title.setObjectName("SubtitleLabel")
        self.pose_label = QLabel("X: --  |  Y: --  |  Yaw: --")
        self.pose_label.setObjectName("ValueLabel")
        pose_box.addWidget(pose_title)
        pose_box.addWidget(self.pose_label)
        header_layout.addLayout(pose_box, stretch=2)

        # Right Header: FSM State Pill & Settings Button
        right_header = QHBoxLayout()
        right_header.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        right_header.setSpacing(12)

        self.status_pill = QLabel("IDLE")
        self.status_pill.setObjectName("StatusPill")
        self.status_pill.setStyleSheet(get_status_style("IDLE"))
        right_header.addWidget(self.status_pill)

        self.btn_settings = QPushButton("⚙ Setting Point")
        self.btn_settings.setObjectName("PrimaryButton")
        self.btn_settings.setCursor(Qt.PointingHandCursor)
        self.btn_settings.clicked.connect(self.switch_to_settings.emit)
        right_header.addWidget(self.btn_settings)

        header_layout.addLayout(right_header, stretch=1)
        layout.addWidget(header_card)

        # ----------------- 2. MAIN SECTION (2 COLUMNS) -----------------
        main_layout = QHBoxLayout()
        main_layout.setSpacing(16)

        # LEFT COLUMN: WAYPOINT BANK
        wp_bank_card = QFrame()
        wp_bank_card.setObjectName("Card")
        wp_bank_layout = QVBoxLayout(wp_bank_card)
        wp_bank_layout.setContentsMargins(14, 14, 14, 14)
        wp_bank_layout.setSpacing(10)

        bank_header_layout = QHBoxLayout()
        bank_title = QLabel("DAFTAR POINT TERSEDIA")
        bank_title.setObjectName("SectionLabel")
        bank_header_layout.addWidget(bank_title)

        btn_refresh_bank = QPushButton("🔄")
        btn_refresh_bank.setToolTip("Muat ulang daftar point")
        btn_refresh_bank.setFixedWidth(36)
        btn_refresh_bank.clicked.connect(self.refresh_waypoints_bank)
        bank_header_layout.addWidget(btn_refresh_bank)
        wp_bank_layout.addLayout(bank_header_layout)

        bank_hint = QLabel("Klik point untuk menambahkan ke antrian task:")
        bank_hint.setObjectName("SubtitleLabel")
        wp_bank_layout.addWidget(bank_hint)

        # Scrollable area for waypoint chips
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("background-color: transparent; border: none;")
        self.wp_chips_widget = QWidget()
        self.wp_chips_layout = QVBoxLayout(self.wp_chips_widget)
        self.wp_chips_layout.setContentsMargins(0, 0, 0, 0)
        self.wp_chips_layout.setSpacing(8)
        self.wp_chips_layout.addStretch()
        scroll.setWidget(self.wp_chips_widget)
        wp_bank_layout.addWidget(scroll)

        main_layout.addWidget(wp_bank_card, stretch=2)

        # RIGHT COLUMN: TASK QUEUE BUILDER
        queue_card = QFrame()
        queue_card.setObjectName("Card")
        queue_layout = QVBoxLayout(queue_card)
        queue_layout.setContentsMargins(14, 14, 14, 14)
        queue_layout.setSpacing(10)

        queue_title = QLabel("ANTRIAN TASK (MISSION QUEUE)")
        queue_title.setObjectName("SectionLabel")
        queue_layout.addWidget(queue_title)

        # List Widget for Queue
        self.queue_list = QListWidget()
        self.queue_list.setDragDropMode(QListWidget.InternalMove)
        queue_layout.addWidget(self.queue_list)

        # Queue Modification Buttons
        queue_actions = QHBoxLayout()
        queue_actions.setSpacing(8)

        self.btn_move_up = QPushButton("▲ Naik")
        self.btn_move_up.clicked.connect(self._move_item_up)
        queue_actions.addWidget(self.btn_move_up)

        self.btn_move_down = QPushButton("▼ Turun")
        self.btn_move_down.clicked.connect(self._move_item_down)
        queue_actions.addWidget(self.btn_move_down)

        self.btn_remove_item = QPushButton("✖ Hapus Pilihan")
        self.btn_remove_item.clicked.connect(self._remove_selected_item)
        queue_actions.addWidget(self.btn_remove_item)

        self.btn_clear_queue = QPushButton("🗑 Bersihkan Antrian")
        self.btn_clear_queue.clicked.connect(self._clear_queue)
        queue_actions.addWidget(self.btn_clear_queue)

        queue_layout.addLayout(queue_actions)

        # Command Preview Label
        self.preview_label = QLabel("Preview Misi: [Kosong]")
        self.preview_label.setObjectName("SubtitleLabel")
        queue_layout.addWidget(self.preview_label)

        main_layout.addWidget(queue_card, stretch=3)
        layout.addLayout(main_layout, stretch=1)

        # ----------------- 3. BOTTOM CONTROL BAR -----------------
        bottom_bar = QFrame()
        bottom_bar.setObjectName("Card")
        bottom_layout = QHBoxLayout(bottom_bar)
        bottom_layout.setContentsMargins(16, 12, 16, 12)
        bottom_layout.setSpacing(16)

        self.btn_start = QPushButton("▶  START MISSION")
        self.btn_start.setObjectName("StartButton")
        self.btn_start.setCursor(Qt.PointingHandCursor)
        self.btn_start.clicked.connect(self._on_start_clicked)
        bottom_layout.addWidget(self.btn_start, stretch=3)

        self.btn_cancel = QPushButton("⏹  BATALKAN / CANCEL")
        self.btn_cancel.setObjectName("DangerButton")
        self.btn_cancel.setCursor(Qt.PointingHandCursor)
        self.btn_cancel.clicked.connect(self._on_cancel_clicked)
        bottom_layout.addWidget(self.btn_cancel, stretch=1)

        layout.addWidget(bottom_bar)

    def _connect_signals(self):
        """Connect ROS Worker signals to UI update slots."""
        if self.ros_worker:
            self.ros_worker.robot_pose_updated.connect(self._update_robot_pose)
            self.ros_worker.mission_status_received.connect(self._update_mission_status)

    def refresh_waypoints_bank(self):
        """Populate the waypoint bank from storage."""
        # Clear existing chips
        for i in reversed(range(self.wp_chips_layout.count())):
            item = self.wp_chips_layout.itemAt(i)
            if item.widget():
                item.widget().deleteLater()

        waypoints = self.storage.load_all()
        if not waypoints:
            empty_lbl = QLabel("Belum ada waypoint tersimpan.\nBuka 'Setting Point' untuk menambah.")
            empty_lbl.setObjectName("SubtitleLabel")
            empty_lbl.setAlignment(Qt.AlignCenter)
            self.wp_chips_layout.insertWidget(0, empty_lbl)
            return

        for name, data in waypoints.items():
            btn = QPushButton(f"📍  {name}")
            btn.setObjectName("WaypointChip")
            btn.setCursor(Qt.PointingHandCursor)
            x, y, yaw = data.get('x', 0.0), data.get('y', 0.0), data.get('yaw_deg', 0.0)
            btn.setToolTip(f"X: {x} m, Y: {y} m, Yaw: {yaw}°")
            btn.clicked.connect(lambda checked, wp_name=name: self._add_to_queue(wp_name))
            self.wp_chips_layout.insertWidget(self.wp_chips_layout.count() - 1, btn)

    def _add_to_queue(self, wp_name: str):
        """Append waypoint to active queue list."""
        count = self.queue_list.count() + 1
        item = QListWidgetItem(f"{count}.  {wp_name}")
        item.setData(Qt.UserRole, wp_name)
        self.queue_list.addItem(item)
        self._update_queue_display()

    def _update_queue_display(self):
        """Re-number items in the list and update preview text."""
        items = []
        for i in range(self.queue_list.count()):
            item = self.queue_list.item(i)
            raw_name = item.data(Qt.UserRole)
            items.append(raw_name)
            item.setText(f"{i + 1}.  {raw_name}")

        if items:
            self.preview_label.setText(f"Preview Misi: {' ➔ '.join(items)}")
        else:
            self.preview_label.setText("Preview Misi: [Kosong]")

    def _move_item_up(self):
        row = self.queue_list.currentRow()
        if row > 0:
            item = self.queue_list.takeItem(row)
            self.queue_list.insertItem(row - 1, item)
            self.queue_list.setCurrentRow(row - 1)
            self._update_queue_display()

    def _move_item_down(self):
        row = self.queue_list.currentRow()
        if row >= 0 and row < self.queue_list.count() - 1:
            item = self.queue_list.takeItem(row)
            self.queue_list.insertItem(row + 1, item)
            self.queue_list.setCurrentRow(row + 1)
            self._update_queue_display()

    def _remove_selected_item(self):
        row = self.queue_list.currentRow()
        if row >= 0:
            self.queue_list.takeItem(row)
            self._update_queue_display()

    def _clear_queue(self):
        if self.queue_list.count() > 0:
            self.queue_list.clear()
            self._update_queue_display()

    def get_current_queue(self) -> List[str]:
        return [
            self.queue_list.item(i).data(Qt.UserRole)
            for i in range(self.queue_list.count())
        ]

    def _on_start_clicked(self):
        queue = self.get_current_queue()
        if not queue:
            QMessageBox.warning(
                self,
                "Antrian Kosong",
                "Silakan pilih minimal satu point untuk membuat antrian misi sebelum menekan Start."
            )
            return

        cmd_str = ",".join(queue)
        if self.ros_worker:
            self.ros_worker.send_mission(queue)
            QMessageBox.information(
                self,
                "Misi Dimulai",
                f"Misi berhasil dikirim ke Mission Manager:\n{cmd_str}"
            )

    def _on_cancel_clicked(self):
        reply = QMessageBox.question(
            self,
            "Konfirmasi Pembatalan",
            "Apakah Anda yakin ingin membatalkan misi yang sedang berjalan?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes and self.ros_worker:
            self.ros_worker.cancel_mission()

    def _update_robot_pose(self, pose: dict):
        """Update live telemetry coordinates."""
        if pose and pose.get('available'):
            x = pose.get('x', 0.0)
            y = pose.get('y', 0.0)
            yaw = pose.get('yaw_deg', 0.0)
            frame = pose.get('frame_id', 'map')
            self.pose_label.setText(f"X: {x:.3f} m  |  Y: {y:.3f} m  |  Yaw: {yaw:.1f}° ({frame})")
        else:
            self.pose_label.setText("TF Pose tidak tersedia")

    def _update_mission_status(self, data: dict):
        """Update FSM status pill and active mission indicators."""
        state = data.get('state', 'IDLE')
        self.current_state_name = state
        self.status_pill.setText(state)
        self.status_pill.setStyleSheet(get_status_style(state))

        active_mission = data.get('active_mission') or "None"
        current_obj = data.get('current_objective') or "-"
        remaining = data.get('remaining_objectives') or []
        rem_str = f" (Sisa: {len(remaining)})" if remaining else ""

        self.active_mission_label.setText(
            f"Active: {active_mission} | Target Saat Ini: {current_obj}{rem_str}"
        )

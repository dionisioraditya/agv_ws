#!/usr/bin/env python3
import math
from typing import Optional, Set
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QFrame,
    QInputDialog, QMessageBox, QDialog, QLineEdit, QDialogButtonBox,
    QFormLayout, QDoubleSpinBox, QGridLayout, QCheckBox
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer

from agv_hmi.storage import WaypointStorage, yaw_to_quaternion


class EditPointDialog(QDialog):
    """Dialog to manually inspect or fine-tune waypoint coordinates."""

    def __init__(self, name: str, data: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Edit Waypoint: {name}")
        self.resize(360, 280)
        self.name = name

        layout = QVBoxLayout(self)
        form = QFormLayout()

        self.name_edit = QLineEdit(name)
        self.x_edit = QLineEdit(str(data.get('x', 0.0)))
        self.y_edit = QLineEdit(str(data.get('y', 0.0)))
        self.z_edit = QLineEdit(str(data.get('z', 0.0)))
        self.yaw_edit = QLineEdit(str(data.get('yaw_deg', 0.0)))

        form.addRow("Nama Point:", self.name_edit)
        form.addRow("X (meter):", self.x_edit)
        form.addRow("Y (meter):", self.y_edit)
        form.addRow("Z (meter):", self.z_edit)
        form.addRow("Yaw (derajat):", self.yaw_edit)

        layout.addLayout(form)

        button_box = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

    def get_values(self):
        new_name = self.name_edit.text().strip()
        x = float(self.x_edit.text() or 0.0)
        y = float(self.y_edit.text() or 0.0)
        z = float(self.z_edit.text() or 0.0)
        yaw_deg = float(self.yaw_edit.text() or 0.0)

        yaw_rad = math.radians(yaw_deg)
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)

        return new_name, {
            'x': x, 'y': y, 'z': z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'yaw_deg': yaw_deg
        }


class ResponsiveDpadWidget(QWidget):
    """Responsive D-Pad container that dynamically scales buttons while maintaining exact 1:1 square symmetry."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.grid = QGridLayout(self)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setSpacing(8)
        self.grid.setAlignment(Qt.AlignCenter)

        self.btn_jog_fwd = QPushButton("▲\nMAJU\n[W]")
        self.btn_jog_fwd.setObjectName("DPadButton")
        self.btn_jog_fwd.setCursor(Qt.PointingHandCursor)

        self.btn_jog_left = QPushButton("◄\nKIRI\n[A]")
        self.btn_jog_left.setObjectName("DPadButton")
        self.btn_jog_left.setCursor(Qt.PointingHandCursor)

        self.btn_jog_stop = QPushButton("🛑\nSTOP\n[Spc]")
        self.btn_jog_stop.setObjectName("StopJogButton")
        self.btn_jog_stop.setCursor(Qt.PointingHandCursor)

        self.btn_jog_right = QPushButton("►\nKANAN\n[D]")
        self.btn_jog_right.setObjectName("DPadButton")
        self.btn_jog_right.setCursor(Qt.PointingHandCursor)

        self.btn_jog_back = QPushButton("▼\nMUNDUR\n[S]")
        self.btn_jog_back.setObjectName("DPadButton")
        self.btn_jog_back.setCursor(Qt.PointingHandCursor)

        self.buttons = [
            self.btn_jog_fwd,
            self.btn_jog_left,
            self.btn_jog_stop,
            self.btn_jog_right,
            self.btn_jog_back
        ]

        self.grid.addWidget(self.btn_jog_fwd, 0, 1)
        self.grid.addWidget(self.btn_jog_left, 1, 0)
        self.grid.addWidget(self.btn_jog_stop, 1, 1)
        self.grid.addWidget(self.btn_jog_right, 1, 2)
        self.grid.addWidget(self.btn_jog_back, 2, 1)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        w = self.width()
        h = self.height()
        side = min(w, h)
        spacing = self.grid.spacing()
        # Scale between 55px and 120px depending on screen/panel space
        btn_size = max(55, min(120, int((side - 2 * spacing) / 3)))
        font_pt = max(9, min(13, int(btn_size * 0.13)))

        for btn in self.buttons:
            btn.setFixedSize(btn_size, btn_size)
            f = btn.font()
            f.setPointSize(font_pt)
            btn.setFont(f)


class SettingsPage(QWidget):
    """Settings Page for Waypoint Management, Calibration, and Manual Robot Teleop."""

    switch_to_dashboard = pyqtSignal()
    waypoints_modified = pyqtSignal()

    def __init__(self, storage: WaypointStorage, ros_worker, parent=None):
        super().__init__(parent)
        self.storage = storage
        self.ros_worker = ros_worker
        self.latest_pose = None

        # Teleop state
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.pressed_keys: Set[int] = set()

        # Timer for continuous jog publishing (20 Hz)
        self.jog_timer = QTimer(self)
        self.jog_timer.setInterval(50)
        self.jog_timer.timeout.connect(self._on_jog_timer_tick)

        # Allow page to receive key events
        self.setFocusPolicy(Qt.StrongFocus)

        self._init_ui()
        self._connect_signals()
        self.load_table_data()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(10)

        # ----------------- 1. HEADER CARD -----------------
        header_card = QFrame()
        header_card.setObjectName("HeaderCard")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(14, 8, 14, 8)

        left_header = QVBoxLayout()
        title_label = QLabel("MANAJEMEN POINT")
        title_label.setObjectName("TitleLabel")
        desc_label = QLabel(
            "Gerakkan robot manual (W/A/S/D / D-Pad), rekam posisi saat ini (Teach-in), atau kelola daftar titik."
        )
        desc_label.setObjectName("SubtitleLabel")
        left_header.addWidget(title_label)
        left_header.addWidget(desc_label)
        header_layout.addLayout(left_header, stretch=3)

        self.btn_back = QPushButton("⬅ Kembali ke Dashboard")
        self.btn_back.setObjectName("PrimaryButton")
        self.btn_back.setCursor(Qt.PointingHandCursor)
        self.btn_back.clicked.connect(self._on_back_clicked)
        header_layout.addWidget(self.btn_back, stretch=1, alignment=Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(header_card)

        # ----------------- 2. MAIN SECTION (2 COLUMNS) -----------------
        main_layout = QHBoxLayout()
        main_layout.setSpacing(16)

        # LEFT COLUMN: WAYPOINT TABLE & MANAGEMENT (stretch=3)
        table_card = QFrame()
        table_card.setObjectName("Card")
        table_layout = QVBoxLayout(table_card)
        table_layout.setContentsMargins(14, 14, 14, 14)
        table_layout.setSpacing(10)

        table_header = QHBoxLayout()
        table_title = QLabel("DAFTAR POINT TERSIMPAN (YAML)")
        table_title.setObjectName("SectionLabel")
        table_header.addWidget(table_title)

        self.lbl_count = QLabel("Total: 0 point")
        self.lbl_count.setObjectName("SubtitleLabel")
        table_header.addWidget(self.lbl_count)
        table_header.addStretch()

        table_layout.addLayout(table_header)

        self.table = QTableWidget()
        self.table.setColumnCount(5)
        self.table.setHorizontalHeaderLabels([
            "Nama Point", "X (m)", "Y (m)", "Yaw (°)", "Terakhir Diupdate"
        ])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.Stretch)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setSelectionMode(QTableWidget.SingleSelection)
        table_layout.addWidget(self.table)

        # Action Buttons under Table
        actions_grid = QGridLayout()
        actions_grid.setSpacing(8)

        self.btn_teach = QPushButton("📍 Ambil Posisi Robot Sekarang (Teach Point)")
        self.btn_teach.setObjectName("StartButton")
        self.btn_teach.setCursor(Qt.PointingHandCursor)
        self.btn_teach.clicked.connect(self._on_teach_point_clicked)
        actions_grid.addWidget(self.btn_teach, 0, 0, 1, 2)

        self.btn_update_pose = QPushButton("🔄 Set Koordinat Baru ke Point Terpilih")
        self.btn_update_pose.setCursor(Qt.PointingHandCursor)
        self.btn_update_pose.clicked.connect(self._on_overwrite_with_current_pose)
        actions_grid.addWidget(self.btn_update_pose, 1, 0)

        self.btn_edit = QPushButton("✏ Edit Nilai")
        self.btn_edit.setCursor(Qt.PointingHandCursor)
        self.btn_edit.clicked.connect(self._on_edit_manual_clicked)
        actions_grid.addWidget(self.btn_edit, 1, 1)

        self.btn_delete = QPushButton("🗑 Hapus Point")
        self.btn_delete.setObjectName("DangerButton")
        self.btn_delete.setCursor(Qt.PointingHandCursor)
        self.btn_delete.clicked.connect(self._on_delete_point_clicked)
        actions_grid.addWidget(self.btn_delete, 2, 0, 1, 2)

        table_layout.addLayout(actions_grid)
        main_layout.addWidget(table_card, stretch=3)

        # RIGHT COLUMN: MANUAL JOG / TELEOP PANEL (stretch=2)
        teleop_card = QFrame()
        teleop_card.setObjectName("Card")
        teleop_layout = QVBoxLayout(teleop_card)
        teleop_layout.setContentsMargins(14, 14, 14, 14)
        teleop_layout.setSpacing(12)

        teleop_title = QLabel("MANUAL JOGGING / TELEOP")
        teleop_title.setObjectName("SectionLabel")
        teleop_layout.addWidget(teleop_title)

        # Pose monitor feedback inside teleop panel
        pose_box = QFrame()
        pose_box.setStyleSheet("background-color: #191a2a; border-radius: 8px; padding: 6px;")
        pose_box_layout = QVBoxLayout(pose_box)
        pose_box_layout.setContentsMargins(8, 6, 8, 6)
        pose_box_layout.setSpacing(2)

        p_lbl = QLabel("Posisi Robot Saat Ini (TF):")
        p_lbl.setObjectName("SubtitleLabel")
        self.pose_feedback_lbl = QLabel("Membaca koordinat robot...")
        self.pose_feedback_lbl.setObjectName("ValueLabel")
        self.pose_feedback_lbl.setStyleSheet("font-size: 13px; color: #38bdf8;")
        pose_box_layout.addWidget(p_lbl)
        pose_box_layout.addWidget(self.pose_feedback_lbl)
        teleop_layout.addWidget(pose_box)

        # Speed adjustment controls
        speed_layout = QGridLayout()
        speed_layout.setSpacing(6)

        lbl_lin = QLabel("Linear (m/s):")
        lbl_lin.setObjectName("SubtitleLabel")
        self.spin_linear = QDoubleSpinBox()
        self.spin_linear.setRange(0.05, 1.0)
        self.spin_linear.setSingleStep(0.05)
        self.spin_linear.setValue(0.35)
        self.spin_linear.setStyleSheet("background-color: #181928; color: #fff; padding: 4px;")
        speed_layout.addWidget(lbl_lin, 0, 0)
        speed_layout.addWidget(self.spin_linear, 0, 1)

        lbl_ang = QLabel("Angular (rad/s):")
        lbl_ang.setObjectName("SubtitleLabel")
        self.spin_angular = QDoubleSpinBox()
        self.spin_angular.setRange(0.1, 2.5)
        self.spin_angular.setSingleStep(0.1)
        self.spin_angular.setValue(0.8)
        self.spin_angular.setStyleSheet("background-color: #181928; color: #fff; padding: 4px;")
        speed_layout.addWidget(lbl_ang, 1, 0)
        speed_layout.addWidget(self.spin_angular, 1, 1)

        teleop_layout.addLayout(speed_layout)

        # D-Pad Button Layout (Responsive square container)
        self.dpad_container = ResponsiveDpadWidget()
        self.btn_jog_fwd = self.dpad_container.btn_jog_fwd
        self.btn_jog_left = self.dpad_container.btn_jog_left
        self.btn_jog_stop = self.dpad_container.btn_jog_stop
        self.btn_jog_right = self.dpad_container.btn_jog_right
        self.btn_jog_back = self.dpad_container.btn_jog_back

        self.btn_jog_fwd.pressed.connect(self._on_fwd_pressed)
        self.btn_jog_fwd.released.connect(self._on_jog_released)
        self.btn_jog_left.pressed.connect(self._on_left_pressed)
        self.btn_jog_left.released.connect(self._on_jog_released)
        self.btn_jog_stop.clicked.connect(self._emergency_stop_jog)
        self.btn_jog_right.pressed.connect(self._on_right_pressed)
        self.btn_jog_right.released.connect(self._on_jog_released)
        self.btn_jog_back.pressed.connect(self._on_back_pressed)
        self.btn_jog_back.released.connect(self._on_jog_released)

        teleop_layout.addWidget(self.dpad_container, stretch=1)

        # Keyboard control active notice
        self.chk_keyboard = QCheckBox("Aktifkan Kontrol Keyboard (W/A/S/D)")
        self.chk_keyboard.setChecked(True)
        self.chk_keyboard.setStyleSheet("color: #a5b4fc; font-weight: bold;")
        teleop_layout.addWidget(self.chk_keyboard)

        tip_lbl = QLabel("Tahan tombol mouse atau tombol keyboard untuk menggerakkan robot.")
        tip_lbl.setObjectName("SubtitleLabel")
        tip_lbl.setWordWrap(True)
        teleop_layout.addWidget(tip_lbl)

        main_layout.addWidget(teleop_card, stretch=2)

        layout.addLayout(main_layout, stretch=1)

    def _connect_signals(self):
        if self.ros_worker:
            self.ros_worker.robot_pose_updated.connect(self._on_pose_received)

    def _on_pose_received(self, pose: dict):
        self.latest_pose = pose
        if pose and pose.get('available'):
            x = pose.get('x', 0.0)
            y = pose.get('y', 0.0)
            yaw = pose.get('yaw_deg', 0.0)
            self.pose_feedback_lbl.setText(f"X: {x:.3f} m  |  Y: {y:.3f} m  |  Yaw: {yaw:.1f}°")
        else:
            self.pose_feedback_lbl.setText("TF robot tidak tersedia")

    # ------------------- Manual Jogging Slots -------------------

    def _on_fwd_pressed(self):
        self._start_jog(self.spin_linear.value(), 0.0)

    def _on_back_pressed(self):
        self._start_jog(-self.spin_linear.value(), 0.0)

    def _on_left_pressed(self):
        self._start_jog(0.0, self.spin_angular.value())

    def _on_right_pressed(self):
        self._start_jog(0.0, -self.spin_angular.value())

    def _on_jog_released(self):
        if not self.pressed_keys:
            self._emergency_stop_jog()

    def _start_jog(self, vx: float, wz: float):
        self.current_vx = vx
        self.current_wz = wz
        self._send_cmd(vx, wz)
        if not self.jog_timer.isActive():
            self.jog_timer.start()

    def _on_jog_timer_tick(self):
        self._send_cmd(self.current_vx, self.current_wz)

    def _emergency_stop_jog(self):
        self.jog_timer.stop()
        self.current_vx = 0.0
        self.current_wz = 0.0
        self.pressed_keys.clear()
        self._send_cmd(0.0, 0.0)

    def _send_cmd(self, vx: float, wz: float):
        if self.ros_worker:
            self.ros_worker.send_teleop(vx, wz)

    # ------------------- Keyboard Teleop Handling -------------------

    def keyPressEvent(self, event):
        if not self.chk_keyboard.isChecked() or event.isAutoRepeat():
            super().keyPressEvent(event)
            return

        key = event.key()
        if key in (Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D, Qt.Key_Space):
            self.pressed_keys.add(key)
            self._update_keyboard_velocity()
            event.accept()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if not self.chk_keyboard.isChecked() or event.isAutoRepeat():
            super().keyReleaseEvent(event)
            return

        key = event.key()
        if key in self.pressed_keys:
            self.pressed_keys.discard(key)
            self._update_keyboard_velocity()
            event.accept()
        else:
            super().keyReleaseEvent(event)

    def _update_keyboard_velocity(self):
        if Qt.Key_Space in self.pressed_keys:
            self._emergency_stop_jog()
            return

        vx = 0.0
        wz = 0.0
        lin_speed = self.spin_linear.value()
        ang_speed = self.spin_angular.value()

        if Qt.Key_W in self.pressed_keys:
            vx += lin_speed
        if Qt.Key_S in self.pressed_keys:
            vx -= lin_speed
        if Qt.Key_A in self.pressed_keys:
            wz += ang_speed
        if Qt.Key_D in self.pressed_keys:
            wz -= ang_speed

        if vx == 0.0 and wz == 0.0:
            self._emergency_stop_jog()
        else:
            self._start_jog(vx, wz)

    def _on_back_clicked(self):
        self._emergency_stop_jog()
        self.switch_to_dashboard.emit()

    # ------------------- Table & Waypoint Management -------------------

    def load_table_data(self):
        """Populate the table with stored waypoints from YAML."""
        waypoints = self.storage.load_all()
        self.table.setRowCount(0)
        self.lbl_count.setText(f"Total: {len(waypoints)} point")

        for row, (name, data) in enumerate(waypoints.items()):
            self.table.insertRow(row)

            item_name = QTableWidgetItem(name)
            item_name.setFlags(item_name.flags() ^ Qt.ItemIsEditable)

            item_x = QTableWidgetItem(f"{data.get('x', 0.0):.3f}")
            item_x.setTextAlignment(Qt.AlignCenter)
            item_x.setFlags(item_x.flags() ^ Qt.ItemIsEditable)

            item_y = QTableWidgetItem(f"{data.get('y', 0.0):.3f}")
            item_y.setTextAlignment(Qt.AlignCenter)
            item_y.setFlags(item_y.flags() ^ Qt.ItemIsEditable)

            item_yaw = QTableWidgetItem(f"{data.get('yaw_deg', 0.0):.1f}")
            item_yaw.setTextAlignment(Qt.AlignCenter)
            item_yaw.setFlags(item_yaw.flags() ^ Qt.ItemIsEditable)

            updated = data.get('updated_at', '-')
            item_updated = QTableWidgetItem(updated)
            item_updated.setTextAlignment(Qt.AlignCenter)
            item_updated.setFlags(item_updated.flags() ^ Qt.ItemIsEditable)

            self.table.setItem(row, 0, item_name)
            self.table.setItem(row, 1, item_x)
            self.table.setItem(row, 2, item_y)
            self.table.setItem(row, 3, item_yaw)
            self.table.setItem(row, 4, item_updated)

    def _get_selected_waypoint_name(self) -> Optional[str]:
        selected_rows = self.table.selectionModel().selectedRows()
        if not selected_rows:
            return None
        row = selected_rows[0].row()
        item = self.table.item(row, 0)
        return item.text() if item else None

    def _on_teach_point_clicked(self):
        """Read current robot pose and add a new waypoint."""
        pose = None
        if self.ros_worker:
            pose = self.ros_worker.get_current_pose()
        if not pose:
            pose = self.latest_pose

        if not pose or not pose.get('available'):
            QMessageBox.critical(
                self,
                "TF Gagal",
                "Gagal membaca posisi global robot dari TF ('map' -> 'base_footprint').\nPastikan localization / Nav2 sedang aktif."
            )
            return

        x = pose.get('x', 0.0)
        y = pose.get('y', 0.0)
        yaw = pose.get('yaw_deg', 0.0)

        default_name = f"point_{self.table.rowCount() + 1}"
        name, ok = QInputDialog.getText(
            self,
            "Tambah Point Baru (Teach Point)",
            f"Posisi Robot Saat Ini:\nX: {x:.3f} m, Y: {y:.3f} m, Yaw: {yaw:.1f}°\n\nMasukkan Nama Point:",
            text=default_name
        )

        if ok and name.strip():
            clean_name = name.strip()
            self.storage.save_waypoint(clean_name, pose)
            self.load_table_data()
            self.waypoints_modified.emit()
            if self.ros_worker:
                self.ros_worker.notify_waypoints_updated()

            QMessageBox.information(
                self,
                "Sukses Disimpan",
                f"Point '{clean_name}' berhasil disimpan dengan koordinat robot saat ini!"
            )

    def _on_overwrite_with_current_pose(self):
        """Overwrite the selected waypoint's coordinates with current robot pose."""
        wp_name = self._get_selected_waypoint_name()
        if not wp_name:
            QMessageBox.warning(self, "Pilih Point", "Pilih salah satu baris point di tabel terlebih dahulu.")
            return

        pose = None
        if self.ros_worker:
            pose = self.ros_worker.get_current_pose()
        if not pose:
            pose = self.latest_pose

        if not pose or not pose.get('available'):
            QMessageBox.critical(
                self,
                "TF Gagal",
                "Gagal membaca posisi global robot dari TF saat ini."
            )
            return

        x = pose.get('x', 0.0)
        y = pose.get('y', 0.0)
        yaw = pose.get('yaw_deg', 0.0)

        reply = QMessageBox.question(
            self,
            "Konfirmasi Set Koordinat Baru",
            f"Apakah Anda yakin ingin mengatur ulang koordinat point '{wp_name}' ke posisi robot saat ini?\n\n"
            f"Koordinat Baru: X={x:.3f}, Y={y:.3f}, Yaw={yaw:.1f}°",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )

        if reply == QMessageBox.Yes:
            self.storage.save_waypoint(wp_name, pose)
            self.load_table_data()
            self.waypoints_modified.emit()
            if self.ros_worker:
                self.ros_worker.notify_waypoints_updated()

            QMessageBox.information(
                self,
                "Koordinat Diperbarui",
                f"Point '{wp_name}' berhasil di-update ke posisi robot terkini!"
            )

    def _on_edit_manual_clicked(self):
        wp_name = self._get_selected_waypoint_name()
        if not wp_name:
            QMessageBox.warning(self, "Pilih Point", "Pilih salah satu baris point di tabel terlebih dahulu.")
            return

        wp_data = self.storage.get_waypoint(wp_name)
        if not wp_data:
            return

        dialog = EditPointDialog(wp_name, wp_data, self)
        if dialog.exec_() == QDialog.Accepted:
            try:
                new_name, new_coords = dialog.get_values()
                if new_name != wp_name:
                    self.storage.delete_waypoint(wp_name)
                self.storage.save_waypoint(new_name, new_coords)
                self.load_table_data()
                self.waypoints_modified.emit()
                if self.ros_worker:
                    self.ros_worker.notify_waypoints_updated()
            except ValueError as e:
                QMessageBox.critical(self, "Input Error", f"Format nilai angka salah: {e}")

    def _on_delete_point_clicked(self):
        """Delete selected waypoint."""
        wp_name = self._get_selected_waypoint_name()
        if not wp_name:
            QMessageBox.warning(self, "Pilih Point", "Pilih baris point yang ingin dihapus terlebih dahulu.")
            return

        reply = QMessageBox.question(
            self,
            "Konfirmasi Hapus",
            f"Apakah Anda yakin ingin menghapus point '{wp_name}' secara permanen?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            if self.storage.delete_waypoint(wp_name):
                self.load_table_data()
                self.waypoints_modified.emit()
                if self.ros_worker:
                    self.ros_worker.notify_waypoints_updated()
                QMessageBox.information(self, "Terhapus", f"Point '{wp_name}' telah dihapus.")

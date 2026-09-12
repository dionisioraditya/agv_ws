#!/usr/bin/env python3
import math
from typing import Optional
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QTableWidget, QTableWidgetItem, QHeaderView, QFrame,
    QInputDialog, QMessageBox, QDialog, QLineEdit, QDialogButtonBox,
    QFormLayout
)
from PyQt5.QtCore import Qt, pyqtSignal

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


class SettingsPage(QWidget):
    """Settings Page for Waypoint Management and Robot Pose Calibration (Teach-in)."""

    switch_to_dashboard = pyqtSignal()
    waypoints_modified = pyqtSignal()

    def __init__(self, storage: WaypointStorage, ros_worker, parent=None):
        super().__init__(parent)
        self.storage = storage
        self.ros_worker = ros_worker
        self.latest_pose = None

        self._init_ui()
        self._connect_signals()
        self.load_table_data()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(16)

        # ----------------- 1. HEADER CARD -----------------
        header_card = QFrame()
        header_card.setObjectName("HeaderCard")
        header_layout = QHBoxLayout(header_card)
        header_layout.setContentsMargins(16, 12, 16, 12)

        left_header = QVBoxLayout()
        title_label = QLabel("MANAJEMEN POINT & KALIBRASI")
        title_label.setObjectName("TitleLabel")
        desc_label = QLabel("Tambah titik baru via posisi robot (Teach-in), set koordinat baru, atau hapus titik.")
        desc_label.setObjectName("SubtitleLabel")
        left_header.addWidget(title_label)
        left_header.addWidget(desc_label)
        header_layout.addLayout(left_header, stretch=3)

        self.btn_back = QPushButton("⬅ Kembali ke Dashboard")
        self.btn_back.setObjectName("PrimaryButton")
        self.btn_back.setCursor(Qt.PointingHandCursor)
        self.btn_back.clicked.connect(self.switch_to_dashboard.emit)
        header_layout.addWidget(self.btn_back, stretch=1, alignment=Qt.AlignRight | Qt.AlignVCenter)

        layout.addWidget(header_card)

        # ----------------- 2. LIVE ROBOT POSITION BANNER -----------------
        pose_card = QFrame()
        pose_card.setObjectName("Card")
        pose_layout = QHBoxLayout(pose_card)
        pose_layout.setContentsMargins(16, 10, 16, 10)

        pose_lbl_title = QLabel("📍 Posisi Global Robot Saat Ini (TF):")
        pose_lbl_title.setObjectName("SectionLabel")
        pose_layout.addWidget(pose_lbl_title)

        self.pose_feedback_lbl = QLabel("Membaca koordinat robot...")
        self.pose_feedback_lbl.setObjectName("ValueLabel")
        pose_layout.addWidget(self.pose_feedback_lbl)
        pose_layout.addStretch()

        layout.addWidget(pose_card)

        # ----------------- 3. WAYPOINT TABLE -----------------
        table_card = QFrame()
        table_card.setObjectName("Card")
        table_layout = QVBoxLayout(table_card)
        table_layout.setContentsMargins(14, 14, 14, 14)
        table_layout.setSpacing(12)

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

        # ----------------- 4. ACTION BUTTONS -----------------
        actions_layout = QHBoxLayout()
        actions_layout.setSpacing(12)

        # Teach Point Button
        self.btn_teach = QPushButton("📍 Tambah Point (Ambil Posisi Robot Sekarang)")
        self.btn_teach.setObjectName("StartButton")
        self.btn_teach.setCursor(Qt.PointingHandCursor)
        self.btn_teach.clicked.connect(self._on_teach_point_clicked)
        actions_layout.addWidget(self.btn_teach)

        # Overwrite with current pose
        self.btn_update_pose = QPushButton("🔄 Set Koordinat Baru ke Point Terpilih")
        self.btn_update_pose.setCursor(Qt.PointingHandCursor)
        self.btn_update_pose.clicked.connect(self._on_overwrite_with_current_pose)
        actions_layout.addWidget(self.btn_update_pose)

        # Manual Edit
        self.btn_edit = QPushButton("✏ Edit Nilai")
        self.btn_edit.setCursor(Qt.PointingHandCursor)
        self.btn_edit.clicked.connect(self._on_edit_manual_clicked)
        actions_layout.addWidget(self.btn_edit)

        # Delete Point
        self.btn_delete = QPushButton("🗑 Hapus Point")
        self.btn_delete.setObjectName("DangerButton")
        self.btn_delete.setCursor(Qt.PointingHandCursor)
        self.btn_delete.clicked.connect(self._on_delete_point_clicked)
        actions_layout.addWidget(self.btn_delete)

        table_layout.addLayout(actions_layout)
        layout.addWidget(table_card, stretch=1)

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
            self.pose_feedback_lbl.setText("Tidak dapat membaca TF robot saat ini")

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
            "Tambah Point Baru",
            f"Koordinat Robot Saat Ini:\nX: {x:.3f} m, Y: {y:.3f} m, Yaw: {yaw:.1f}°\n\nMasukkan Nama Point:",
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
                    # Rename -> delete old and save new
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

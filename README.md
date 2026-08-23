<h1 align="center">🌐 Robotic UAJY – Autonomous Mobile Robot (AMR) / AGV Workspace</h1>

<p align="center">
  Repository ini berisi <b>source code</b>, model 3D CAD/URDF terbaru (Fusion 360), dan konfigurasi <b>Robot AMR/AGV</b> yang dikembangkan menggunakan <b>ROS 2 Humble</b>.<br>
  Mendukung penuh <b>Simulasi Gazebo Classic</b> dan <b>Robot Fisik (Real-World)</b> dengan <b>SLAM Toolbox</b>, <b>Nav2</b>, serta <b>Custom Planners</b>.
</p>

---

## 🎯 Fitur Utama & Pembaharuan Terbaru
- 🦾 **Model URDF & Visual STL Presisi:** Di-export langsung dari Autodesk Fusion 360 dengan inersia, collision, caster wheels, dan sensor mount yang akurat.
- 🌍 **Simulasi Gazebo Lengkap:** Integrasi `diff_drive` controller, sensor LiDAR 360°, IMU, dan joint state publisher.
- 🗺️ **Dual Mode SLAM & Nav2:** Launch file terpadu untuk mapping dan autonomous navigation baik di simulasi maupun robot fisik.
- 🕹️ **Interactive Keyboard Teleop:** Kontrol pergerakan halus dengan penyesuaian kecepatan linier dan anguler secara *real-time*.

---

## 💻 Tech Stack & Hardware

| Komponen | Spesifikasi / Deskripsi |
|:---|:---|
| **ROS Distribution** | ROS 2 Humble Hawksbill (Ubuntu 22.04 LTS) |
| **SBC (Robot Fisik)** | NVIDIA Jetson Orin Nano / Jetson Series |
| **LiDAR Scanner** | RPLIDAR A2M8 (360° Laser Scan) |
| **Depth Camera** | Orbbec Astra Pro Plus / Intel RealSense |
| **IMU Sensor** | Bosch BNO055 (9-DOF IMU) |
| **Motor Controller** | Custom Serial MCU (`/dev/ttyACM0`) dengan Encoder Odometry |

---

## 📦 Struktur Package

```
agv_ws/src/
├── agv_description/     # Model URDF/Xacro, mesh 3D STL, konfigurasi RViz/Gazebo & launch simulasi
├── agv_controller/      # Odom translator (motor-encoder), keyboard teleop, joystick, nav2 launch
├── agv_filmware/        # Driver sensor hardware (BNO055 IMU)
├── agv_scene/           # Mission manager & data logger untuk pengujian eksperimen
├── my_nav2_planners/    # Custom Nav2 Global/Local Planner plugins (C++)
├── agv_cpp_examples/    # Contoh node ROS 2 C++
└── agv_py_examples/     # Contoh node ROS 2 Python
```

---

## ⚙️ 1. Instalasi & Build

### A. Dependensi ROS 2
```bash
sudo apt update && sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-plugins \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-twist-mux \
    ros-humble-rplidar-ros
```

### B. Build Workspace
```bash
cd ~/agv_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

---

## 🎮 2. Panduan Menjalankan Simulasi (Gazebo)

### A. Tampilkan Model Robot di RViz2 (Visual Check)
Untuk memeriksa joint dan visualisasi mesh 3D dengan GUI slider joint:
```bash
source ~/agv_ws/install/setup.bash
ros2 launch agv_description display.launch.py
```

---

### B. Menjalankan Gazebo + Keyboard Teleop
1. **Terminal 1 – Buka Gazebo World & Spawn Robot:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_description gazebo.launch.py
   ```
2. **Terminal 2 – Keyboard Teleoperation:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_controller teleop.launch.py
   ```
   *Kontrol Keyboard:*
   * `W` / `↑` : Maju
   * `S` / `↓` : Mundur
   * `A` / `←` : Belok Kiri
   * `D` / `→` : Belok Kanan
   * `SPACE` / `X` : Berhenti
   * `Q` / `Z` : Tambah / Kurang Kecepatan Linier
   * `E` / `C` : Tambah / Kurang Kecepatan Anguler

---

### C. Simulasi SLAM Mapping (All-in-One)
Membuka Gazebo, SLAM Toolbox Online Async, dan RViz2 dalam satu perintah:
1. **Terminal 1 – Launch Mapping Simulasi:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_description sim_mapping.launch.py
   ```
2. **Terminal 2 – Kemudikan Robot dengan Teleop:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_controller teleop.launch.py
   ```
3. **Menyimpan Map yang Sudah Dibuat:**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/agv_ws/map_sim_save
   ```

---

### D. Simulasi Nav2 Autonomous Navigation (All-in-One)
1. **Terminal 1 – Launch Nav2 Simulasi:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_description sim_navigation.launch.py
   ```
2. Di RViz2:
   * Berikan **2D Pose Estimate** awal robot jika menggunakan AMCL, atau langsung gunakan **Nav2 Goal** untuk navigasi waypoint.

---

## 🤖 3. Panduan Menjalankan Robot Fisik (Real-World)

### A. Bringup Robot Fisik (Hardware & Sensor)
Menjalankan `odom_translator.py`, driver `bno055_driver`, RPLidar (`lidar_head-v2`), Astra Camera, dan `twist_mux`:
```bash
source ~/agv_ws/install/setup.bash
ros2 launch agv_description agv_bringup.launch.py
```

---

### B. SLAM Mapping di Dunia Nyata
1. **Terminal 1 – Bringup Robot:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_description agv_bringup.launch.py
   ```
2. **Terminal 2 – SLAM Toolbox & RViz2:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_description mapping.launch.py
   ```
3. **Terminal 3 – Teleop Robot:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_controller teleop.launch.py
   ```
4. **Terminal 4 – Simpan Map Setelah Selesai:**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/agv_ws/map_koridor_save
   ```

---

### C. Lokalisasi & Navigasi Otonom (Nav2) di Robot Fisik
1. **Terminal 1 – Bringup Robot:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_description agv_bringup.launch.py
   ```
2. **Terminal 2 – Lokalisasi SLAM Toolbox (Menggunakan Map yang Disimpan):**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_description localization.launch.py
   ```
3. **Terminal 3 – Nav2 Navigation Stack:**
   ```bash
   source ~/agv_ws/install/setup.bash
   ros2 launch agv_controller agv_navigation.launch.py
   ```

---

## 🌲 4. Arsitektur TF Tree & Konvensi Frame

```
map
 └── odom (diterbitkan oleh SLAM Toolbox / Odom Translator / Gazebo Diff-Drive)
      └── base_footprint (pusat rotasi di permukaan lantai)
           └── base_link-v3 (chassis robot utama)
                ├── lidar_housing-v2 ── lidar_head-v2 (Frame sensor Lidar 360° /scan)
                ├── camera_base-v3 ── camera_lens_1-v2 (Frame kamera depth)
                ├── imu_link-v1 (Frame sensor IMU BNO055 /imu/out)
                ├── wheel_bracket_left-v4 ── wheel_left-v2
                ├── wheel_bracket_right-v1 ── wheel_right-v1
                ├── castrol_bracket_left-v2 ── ... ── castrol_wheel_left-v2
                └── castrol_bracket_right-v1 ── ... ── castrol_wheel_right-v1
```

---

## 🚀 5. Mission Manager (Pengujian Eksperimen)

Untuk menjalankan skenario misi otomatis multi-waypoint:

```bash
# Terminal 1: Jalankan node mission manager
ros2 run agv_scene mission_manager_data --ros-args -p experiment_id:=S1_trial_01

# Terminal 2: Kirim perintah single/multi target
# Target Tunggal:
ros2 topic pub --once /mission_command std_msgs/msg/String "{data: 'point1'}"

# Multi Target:
ros2 topic pub --once /mission_command std_msgs/msg/String "{data: 'point1,point2,home'}"
```

# agv_datacollector

Package ROS 2 Humble untuk pengumpulan dataset Vision-Language Model (VLM) / Vision-Language Navigation (VLN) pada AGV.

Package ini merekam stream kamera Orbbec Astra Pro Plus (1920x1080 @ 30 FPS) langsung ke format video `.mp4` dan secara otomatis menghasilkan metadata tersinkronisasi `.json` yang menghubungkan setiap `frame_id` dengan status robot (`cmd_vel`, `odom`, dan posisi global localization dari TF `map -> base_footprint`).

Hasil perekaman otomatis disimpan di direktori:
```text
agv_ws/video/rgb/
```

## Fitur Utama
1. **Multi-Threaded Asynchronous Encoding**: Menggunakan antrean memori *thread-safe* terpisah antara subscriber ROS dan `cv2.VideoWriter`. Tidak membebani callback executor dan mencegah frame drop pada 1080p 30fps.
2. **Synchronized Metadata JSON**:
   - `frame_id`: Indeks urutan frame video (0, 1, 2, ...).
   - `timestamp_ros` & `timestamp_iso`: Timestamp presisi ROS dan ISO-8601.
   - `cmd_vel`: Kecepatan linear ($x, y$) dan angular ($z$).
   - `odom`: Posisi lokal odometri ($x, y, z$, *yaw* derajat & radian, serta kecepatan aktual).
   - `global_pose`: Posisi global robot pada peta (`map -> base_footprint` via SLAM / TF).
3. **Fleksibel**: Parameter dapat dikonfigurasi melalui file YAML ataupun argumen CLI saat me-launch.

## Cara Menggunakan

### 1. Build Package (jika belum di-build)
```bash
cd /home/agv_ws
colcon build --packages-select agv_datacollector
source install/setup.bash
```

### 2. Menjalankan Perekaman Standar
Pastikan robot bringup / driver kamera sudah berjalan, lalu jalankan:
```bash
ros2 launch agv_datacollector record_rgb.launch.py
```
File akan otomatis diberi nama berdasarkan timestamp saat ini, contoh:
- `vlm_20260908_101530.mp4`
- `vlm_20260908_101530_meta.json`

### 3. Menjalankan dengan Nama Sesi Khusus
Untuk mempermudah pengelompokan skenario dataset (misal skenario lorong atau ruangan tertentu):
```bash
ros2 launch agv_datacollector record_rgb.launch.py session_name:=trial_koridor_lab1
```
Output:
- `trial_koridor_lab1.mp4`
- `trial_koridor_lab1_meta.json`

### 4. Menyesuaikan Parameter Lain (Opsional)
```bash
ros2 launch agv_datacollector record_rgb.launch.py \
    session_name:=exp_speed_test \
    image_topic:=/camera/color/image_raw \
    target_fps:=30.0 \
    target_width:=1920 \
    target_height:=1080
```

### 5. Menghentikan Perekaman
Cukup tekan **`Ctrl + C`** di terminal launcher. Node akan:
1. Mengosongkan sisa buffer frame di memori ke file video.
2. Menutup file `.mp4`.
3. Menulis file `_meta.json`.
4. Menampilkan konfirmasi jumlah frame yang berhasil disimpan.

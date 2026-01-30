<h1 align="center">🌐 Robotic UAJY – Automated Guided Vehicle (AGV) Workspace</h1>

<p align="center">
  Repository ini berisi <b>source code</b> dan konfigurasi <b>robot AGV KSR</b> yang dikembangkan menggunakan <b>ROS2 Humble</b>.<br>
  Digunakan untuk keperluan <b>simulasi, penelitian, dan pengembangan path planning</b> sistem robotika.
</p>

---

## 🎯 Goals
Repository ini bertujuan untuk:
- 🧠 Menyediakan lingkungan simulasi <b>AGV (Automated Guided Vehicle)</b> berbasis ROS2.  
- 🤖 Menjadi platform <b>Research & Development</b> untuk algoritma **path planning** dan **AI navigation**.  
- 🔧 Menjadi pondasi untuk integrasi robot fisik berbasis **Jetson Orin + ROS2**.

---

## 💻 Tech Stack

| Komponen | Keterangan |
|:--|:--|
| **Framework** | ROS2 Humble |
| **Languages** | Python, C++, XACRO |
| **Libraries** | ROS2 Nodes, RViz, Gazebo |
| **OS Support** | Ubuntu 22.04 LTS |

---

## ⚙️ Hardware Setup

| Part | Deskripsi |
|:--|:--|
| 🧠 **Nvidia Jetson Orin Nano** | Single Board Computer |
| 🎥 **Intel Realsense Astra Pro Plus** | Depth Camera |
| 🛰️ **RPLIDAR A2M8** | 360° Lidar Scanner |
| 🧭 **BNO055** | Inertial Measuring Unit |
---

## 📜 Installation & Usage Guide

### 🧩 1. Install Dependencies
```bash
sudo apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-velocity-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-ros2controlcli \
    ros-humble-xacro \
    ros-humble-ros-gz*
```

---

### 📦 2. Clone Repository
```bash
cd ~
git clone https://github.com/dionisioraditya/agv_ws.git
cd agv_ws
```

---

### ⚙️ 3. Build Project
> 💡 Always build before running the simulation
```bash
cd ~/agv_ws
colcon build
```

---

### 🦾 4. Display Robot Model on RViz
```bash
# Open new terminal
cd ~/agv_ws
source install/setup.bash
ros2 launch agv_description display.launch.py
```

---

### 🕹️ 5. Run Full Simulation (Gazebo + Joystick)

#### 🪄 Terminal 1 – Build Workspace
```bash
cd ~/agv_ws
colcon build
```

#### 🌍 Terminal 2 – Launch Gazebo Simulation
```bash
cd ~/agv_ws
source install/setup.bash
ros2 launch agv_description gazebo.launch.py
```

#### 🎮 Terminal 3 – Launch Controller
```bash
cd ~/agv_ws
source install/setup.bash

# Show available arguments
ros2 launch agv_controller controller.launch.py --show-args

# Option 1: Simple Control
ros2 launch agv_controller controller.launch.py

# Option 2: Differential Drive Control
ros2 launch agv_controller controller.launch.py use_simple_control:=false

# Joystick Teleoperation
ros2 launch agv_controller joystick_teleop.launch.py
```

## Robot ROS Node Topic
### Depth Camera & Lidar
Use rviz for lidar or depth camera visualization
#### Depth Camera topic
```bash
# make sure you have cloned the Astra Camera repository (link on .gitmodules)
# Open new terminal
cd ~/agv_ws
. install/setup.bash
ros2 launch astra_camera astra_pro.launch.xml uvc_product_id:=0x050f
```

#### Lidar topic topic
```bash
# Open new terminal
cd ~/agv_ws
. install/setup.bash
ros2 launch rplidar_ros rplidar_a2m8_launch.py serial_port:=/dev/rplidar

```
### Robot Odometry (Differential Kinematic & IMU)
#### IMU Node
```bash
# Open new terminal
cd ~agv_ws
. install/setup.bash
ros2 run agv_filmware bno055_driver
```
#### Odometry Node
```bash
# Open new terminal
cd ~agv_ws
. install/setup.bash
ros2 run agv_controller odom_translator.py
```
#### Keyboard controller
```bash
# Open new terminal
cd ~agv_ws
. install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# I = Forward
# K = Stop
# L = Turn right
# J = Turn left
# , = Reverse
```
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist 
import tf2_ros
import serial
import math

class OdomTranslator(Node):
    def __init__(self):
        super().__init__('odom_translator')
        
        self.ser = None 
        self.last_cmd = ""
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.001)
        except Exception as e:
            self.get_logger().error(f"Gagal koneksi Serial: {e}")

        # 2. Parameter Fisik AGV
        self.wheel_radius = 0.057  # 5 cm
        self.wheel_base = 0.39    # 30 cm
        
        # State Robot
        self.x = 0.0
        self.y = 0.0
        self.last_time = self.get_clock().now()
        self.current_quat = [0.0, 0.0, 0.0, 1.0] # [x,y,z,w] dari IMU

        # Subscriber & Publisher
        self.imu_sub = self.create_subscription(
            Imu, 
            '/imu/out', 
            self.imu_callback, 
            qos_profile_sensor_data
        )
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

       
        self.create_timer(0.02, self.update_logic)

    def cmd_vel_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            return

        v = msg.linear.x
        w = msg.angular.z
        
        vr_rad = (2 * v + w * self.wheel_base) / (2 * self.wheel_radius)
        vl_rad = (2 * v - w * self.wheel_base) / (2 * self.wheel_radius)

        vr_rad = max(-5.0, min(5.0, vr_rad))
        vl_rad = max(-5.0, min(5.0, vl_rad))

        dir_r = "F" if vr_rad >= 0 else "B"
        dir_l = "F" if vl_rad >= 0 else "B"

        current_cmd = f"R{dir_r}{abs(vr_rad):.2f} L{dir_l}{abs(vl_rad):.2f}"
        
        if current_cmd != self.last_cmd:
            self.ser.write((current_cmd + "\n").encode('utf-8'))
            self.last_cmd = current_cmd
    def imu_callback(self, msg):
        # Ambil quanternion dari IMU
        self.current_quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]

    def update_logic(self):
        if self.ser is None or not self.ser.is_open:
            return

        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                
                if "VR:" in line and "VL:" in line:
                    parts = dict(item.split(":") for item in line.split(","))
                    vr = float(parts['VR'])
                    vl = float(parts['VL'])

                # DIFFERENTIAL KINEMATIC
                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds / 1e9
                self.last_time = now
                if dt <= 0: return

                # Menghitung kecepatan linear (v) robot
                v = (vr + vl) * self.wheel_radius / 2.0
                
                # Mengambil Yaw dari orientasi IMU BNO055
                siny_cosp = 2 * (self.current_quat[3] * self.current_quat[2] + self.current_quat[0] * self.current_quat[1])
                cosy_cosp = 1 - 2 * (self.current_quat[1]**2 + self.current_quat[2]**2)
                yaw = math.atan2(siny_cosp, cosy_cosp)
                yaw = yaw - 1.5708

                # Update Posisi X dan Y (Integrasi Euler)
                self.x += v * math.cos(yaw) * dt
                self.y += v * math.sin(yaw) * dt

                # Publikasi Odometry
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.child_frame_id = "base_footprint"

                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation.x = self.current_quat[0]
                odom.pose.pose.orientation.y = self.current_quat[1]
                odom.pose.pose.orientation.z = self.current_quat[2]
                odom.pose.pose.orientation.w = self.current_quat[3]
                
                self.odom_pub.publish(odom)

                # Broadcast TF (Transformasi untuk RViz)
                t = TransformStamped()
                t.header = odom.header
                t.child_frame_id = "base_link"
                t.child_frame_id = "base_footprint"
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.rotation = odom.pose.pose.orientation
                self.tf_broadcaster.sendTransform(t)
                yaw_deg = math.degrees(yaw)
        
                self.get_logger().info(
                    f"POSISI -> X: {self.x:6.2f} | Y: {self.y:6.2f} | Yaw: {yaw_deg:6.1f}°"
                )  
            except:
                continue

def main():
    rclpy.init()
    node = OdomTranslator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
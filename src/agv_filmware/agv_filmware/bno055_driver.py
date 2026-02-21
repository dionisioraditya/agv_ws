#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from rclpy.qos import qos_profile_sensor_data

import board
import busio
import adafruit_bno055


def quaternion_to_euler(w, x, y, z):
    """Konversi quaternion (w, x, y, z) → Euler (roll, pitch, yaw) dalam radian."""
    # Roll (x-axis)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    # Pitch (y-axis)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(+1.0, t2))
    pitch = math.asin(t2)

    # Yaw (z-axis)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class BNO055Driver(Node):
    def __init__(self):
        super().__init__("bno055_driver")

        i2c = busio.I2C(board.SCL, board.SDA)

        try:
            self.bno = adafruit_bno055.BNO055_I2C(i2c)
            self.get_logger().info("BNO055 connected successfully!")
            self.is_connected = True
            self.bno.mode = adafruit_bno055.IMUPLUS_MODE 
            # self.bno.mode = adafruit_bno055.NDOF_MODE 
            self.get_logger().info("BNO055 connected in IMUPLUS mode!")

        except Exception as e:
            self.get_logger().error(f"Failed to connect to BNO055: {e}")
            self.is_connected = False

        # Publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/out", qos_profile_sensor_data)
        self.euler_pub = self.create_publisher(Vector3, "/imu/euler", 10)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"

        # 100 Hz
        self.timer = self.create_timer(0.01, self.timer_callback)
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Konversi Euler (radian) kembali ke Quaternion (w, x, y, z)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return w, x, y, z

    def timer_callback(self):
        if not self.is_connected:
            return
        
        accel = self.bno.acceleration
        gyro = self.bno.gyro
        quat = self.bno.quaternion
        # calib = self.bno.calibration_status
        # if quat is None:
        #     self.get_logger().warn("Data Quaternion NONE! Sensor mungkin hang atau kabel longgar.")
        #     return
        # Cek status kalibrasi di logger
        # sys, g, a, m = calib
        # if sys < 1:
        #     self.get_logger().info(f"Menunggu Kalibrasi... Status Sys: {sys} (Gerakkan IMU angka 8!)")

        if accel is None or gyro is None or quat is None:
            return

        w_raw, x_raw, y_raw, z_raw = quat
        roll, pitch, yaw = quaternion_to_euler(w_raw, x_raw, y_raw, z_raw)

        #yaw_corrected = yaw + (math.pi / 2)
        yaw_corrected = yaw
        #yaw_corrected = yaw + math.pi
 
        w_new, x_new, y_new, z_new = self.euler_to_quaternion(roll, pitch, yaw_corrected)

     
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        
        
        self.imu_msg.orientation.w = w_new
        self.imu_msg.orientation.x = x_new
        self.imu_msg.orientation.y = y_new
        self.imu_msg.orientation.z = z_new

        #PUBLISH (Data ini yang akan dibaca oleh Slam Toolbox & Odom)
        self.imu_pub.publish(self.imu_msg)

        # Publikasikan Euler untuk monitoring di terminal
        yaw_deg_final = math.degrees(yaw_corrected)
        euler_msg = Vector3()
        euler_msg.x = math.degrees(roll)
        euler_msg.y = math.degrees(pitch)
        euler_msg.z = yaw_deg_final
        self.euler_pub.publish(euler_msg)

        # Debug print
        # self.get_logger().info(f"Yaw Terkoreksi: {yaw_deg_final:6.2f} | Sensor Asli: {math.degrees(yaw):6.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO055Driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

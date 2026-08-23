#!/usr/bin/env python3
"""
Interactive Keyboard Teleoperation Node for AGV Differential Drive Robot.
Publishes geometry_msgs/msg/Twist to /cmd_vel.
"""

import os
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
\033[1;36m=======================================================
          AGV KEYBOARD TELEOPERATION CONTROLLER
=======================================================\033[0m
\033[1;32mControl Keys:\033[0m
   \033[1m[W] / [↑]\033[0m : Move Forward
   \033[1m[S] / [↓]\033[0m : Move Backward
   \033[1m[A] / [←]\033[0m : Turn Left
   \033[1m[D] / [→]\033[0m : Turn Right
   \033[1m[SPACE] / [X]\033[0m : Emergency Stop (Brake)

\033[1;33mSpeed Adjustments:\033[0m
   \033[1m[Q] / [Z]\033[0m : Linear Speed  (+ / -)
   \033[1m[E] / [C]\033[0m : Angular Speed (+ / -)

\033[1;31mPress Ctrl+C to exit safely.\033[0m
-------------------------------------------------------
"""


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('agv_keyboard_teleop')

        # Declare parameters
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('default_linear_speed', 0.4)
        self.declare_parameter('default_angular_speed', 1.0)
        self.declare_parameter('linear_step', 0.05)
        self.declare_parameter('angular_step', 0.1)
        self.declare_parameter('publish_rate', 20.0)

        cmd_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.target_linear_speed = self.get_parameter('default_linear_speed').get_parameter_value().double_value
        self.target_angular_speed = self.get_parameter('default_angular_speed').get_parameter_value().double_value
        self.linear_step = self.get_parameter('linear_step').get_parameter_value().double_value
        self.angular_step = self.get_parameter('angular_step').get_parameter_value().double_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Twist, cmd_topic, 10)

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_key_time = self.get_clock().now()

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(f"Keyboard Teleop started. Publishing to '{cmd_topic}'.")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(self.linear_x)
        msg.angular.z = float(self.angular_z)
        self.publisher_.publish(msg)

    def print_status(self, action=""):
        status_line = (
            f"\r\033[K\033[1;34m[Status]\033[0m {action:<18} | "
            f"Lin Speed: \033[1;32m{self.target_linear_speed:.2f} m/s\033[0m | "
            f"Ang Speed: \033[1;33m{self.target_angular_speed:.2f} rad/s\033[0m | "
            f"Active: Lin=\033[1m{self.linear_x:+.2f}\033[0m, Ang=\033[1m{self.angular_z:+.2f}\033[0m"
        )
        sys.stdout.write(status_line)
        sys.stdout.flush()

    def process_key(self, key):
        action = ""

        # Movement Keys
        if key in ('w', 'W', '\x1b[A'):  # Forward
            self.linear_x = self.target_linear_speed
            self.angular_z = 0.0
            action = "FORWARD ↑"
        elif key in ('s', 'S', '\x1b[B'):  # Backward
            self.linear_x = -self.target_linear_speed
            self.angular_z = 0.0
            action = "BACKWARD ↓"
        elif key in ('a', 'A', '\x1b[D'):  # Turn Left
            self.linear_x = 0.0
            self.angular_z = self.target_angular_speed
            action = "TURN LEFT ←"
        elif key in ('d', 'D', '\x1b[C'):  # Turn Right
            self.linear_x = 0.0
            self.angular_z = -self.target_angular_speed
            action = "TURN RIGHT →"
        elif key in (' ', 'x', 'X'):  # Stop
            self.linear_x = 0.0
            self.angular_z = 0.0
            action = "STOPPED ■"

        # Speed Tuning Keys
        elif key in ('q', 'Q'):
            self.target_linear_speed = round(min(2.0, self.target_linear_speed + self.linear_step), 2)
            action = f"Lin Speed: +{self.linear_step}"
        elif key in ('z', 'Z'):
            self.target_linear_speed = round(max(0.05, self.target_linear_speed - self.linear_step), 2)
            action = f"Lin Speed: -{self.linear_step}"
        elif key in ('e', 'E'):
            self.target_angular_speed = round(min(5.0, self.target_angular_speed + self.angular_step), 2)
            action = f"Ang Speed: +{self.angular_step}"
        elif key in ('c', 'C'):
            self.target_angular_speed = round(max(0.1, self.target_angular_speed - self.angular_step), 2)
            action = f"Ang Speed: -{self.angular_step}"

        self.print_status(action)


def get_tty_fd():
    if sys.stdin.isatty():
        return sys.stdin.fileno()
    try:
        return os.open('/dev/tty', os.O_RDWR)
    except Exception:
        return sys.stdin.fileno()


def get_key(fd, settings, timeout=0.05):
    """Read a key without pressing enter (supporting arrow escape sequences)."""
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            raw = os.read(fd, 1)
            key = raw.decode('utf-8', errors='ignore')
            if key == '\x1b':  # Escape sequence for arrow keys
                rlist_extra, _, _ = select.select([fd], [], [], 0.05)
                if rlist_extra:
                    extra = os.read(fd, 2).decode('utf-8', errors='ignore')
                    key += extra
        else:
            key = ''
    except Exception:
        key = ''
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        except Exception:
            pass
    return key


def main(args=None):
    fd = get_tty_fd()
    try:
        settings = termios.tcgetattr(fd)
    except Exception as e:
        settings = None

    print(BANNER)

    rclpy.init(args=args)
    teleop_node = KeyboardTeleop()
    teleop_node.print_status("READY")

    try:
        while rclpy.ok():
            if settings is not None:
                key = get_key(fd, settings, timeout=0.05)
                if key:
                    if key == '\x03':  # Ctrl+C
                        break
                    teleop_node.process_key(key)
            rclpy.spin_once(teleop_node, timeout_sec=0.01)
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        # Publish zero velocity on shutdown
        stop_msg = Twist()
        teleop_node.publisher_.publish(stop_msg)
        # Restore terminal settings
        if settings is not None:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, settings)
            except Exception:
                pass
        print("\n\033[1;32mTeleoperation stopped safely. Terminal restored.\033[0m")
        teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

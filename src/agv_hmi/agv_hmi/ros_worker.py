#!/usr/bin/env python3
import json
import math
from typing import Optional, Dict, Any

from PyQt5.QtCore import QThread, pyqtSignal

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException

from agv_hmi.storage import quaternion_to_yaw


class RosHmiNode(Node):
    """ROS 2 Node for HMI operations."""

    def __init__(self, worker_ref):
        super().__init__('agv_hmi_node')
        self.worker = worker_ref

        # Frames
        self.map_frame = 'map'
        self.base_frame = 'base_footprint'
        self.fallback_frame = 'base_link'

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publishers
        self.mission_cmd_pub = self.create_publisher(String, '/mission_command', 10)
        self.mission_cancel_pub = self.create_publisher(Bool, '/mission_cancel', 10)
        self.reload_wp_pub = self.create_publisher(Bool, '/reload_waypoints', 10)
        self.cmd_vel_key_pub = self.create_publisher(Twist, '/cmd_vel_key', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.mission_status_sub = self.create_subscription(
            String,
            '/mission_status',
            self._mission_status_callback,
            10
        )

        # Periodic pose check timer (5 Hz)
        self.pose_timer = self.create_timer(0.2, self._pose_timer_callback)

    def _mission_status_callback(self, msg: String):
        """Handle incoming status messages from Mission Manager."""
        try:
            data = json.loads(msg.data)
            self.worker.mission_status_received.emit(data)
        except Exception as e:
            self.get_logger().warn(f"Failed to parse /mission_status: {e}")

    def _pose_timer_callback(self):
        """Periodically sample robot pose from TF."""
        pose = self.get_robot_pose(timeout_sec=0.03)
        if pose:
            self.worker.robot_pose_updated.emit(pose)

    def get_robot_pose(self, timeout_sec: float = 0.1) -> Optional[Dict[str, Any]]:
        """Lookup current transform from map to base_footprint (or base_link)."""
        target_frames = [self.base_frame, self.fallback_frame]

        for target in target_frames:
            try:
                now = Time()
                trans = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    target,
                    now,
                    timeout=Duration(seconds=timeout_sec)
                )
                t = trans.transform.translation
                r = trans.transform.rotation

                yaw_rad = quaternion_to_yaw(r.x, r.y, r.z, r.w)
                yaw_deg = round(math.degrees(yaw_rad), 2)

                return {
                    'available': True,
                    'frame_id': self.map_frame,
                    'child_frame_id': target,
                    'x': round(t.x, 4),
                    'y': round(t.y, 4),
                    'z': round(t.z, 4),
                    'qx': round(r.x, 6),
                    'qy': round(r.y, 6),
                    'qz': round(r.z, 6),
                    'qw': round(r.w, 6),
                    'yaw_rad': round(yaw_rad, 4),
                    'yaw_deg': yaw_deg,
                }
            except TransformException:
                continue

        # If map is not available, try odom frame
        try:
            trans = self.tf_buffer.lookup_transform(
                'odom',
                self.base_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec)
            )
            t = trans.transform.translation
            r = trans.transform.rotation
            yaw_rad = quaternion_to_yaw(r.x, r.y, r.z, r.w)
            return {
                'available': True,
                'frame_id': 'odom (fallback)',
                'child_frame_id': self.base_frame,
                'x': round(t.x, 4),
                'y': round(t.y, 4),
                'z': round(t.z, 4),
                'qx': round(r.x, 6),
                'qy': round(r.y, 6),
                'qz': round(r.z, 6),
                'qw': round(r.w, 6),
                'yaw_rad': round(yaw_rad, 4),
                'yaw_deg': round(math.degrees(yaw_rad), 2),
            }
        except TransformException:
            pass

        return None

    def publish_mission_command(self, waypoints_str: str):
        """Publish mission command string (e.g. 'point1,point2,home')."""
        msg = String()
        msg.data = waypoints_str
        self.mission_cmd_pub.publish(msg)
        self.get_logger().info(f"Published mission command: '{waypoints_str}'")

    def publish_mission_cancel(self):
        """Publish mission cancel signal."""
        msg = Bool()
        msg.data = True
        self.mission_cancel_pub.publish(msg)
        self.get_logger().info("Published mission cancel")

    def notify_waypoints_reloaded(self):
        """Notify other nodes that waypoints YAML has been updated."""
        msg = Bool()
        msg.data = True
        self.reload_wp_pub.publish(msg)

    def publish_teleop_cmd(self, linear_x: float, angular_z: float):
        """Publish velocity command for manual robot teleop."""
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_key_pub.publish(msg)
        self.cmd_vel_pub.publish(msg)


class RosWorker(QThread):
    """QThread worker to spin ROS 2 node without blocking the Qt main GUI loop."""

    mission_status_received = pyqtSignal(dict)
    robot_pose_updated = pyqtSignal(dict)
    log_message = pyqtSignal(str, str)

    def __init__(self):
        super().__init__()
        self.node: Optional[RosHmiNode] = None
        self._is_running = True

    def run(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = RosHmiNode(self)
        self.log_message.emit("INFO", "HMI ROS 2 Node initialized")

        while rclpy.ok() and self._is_running:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.node:
            self.node.destroy_node()

    def stop(self):
        self._is_running = False
        self.wait(2000)

    def send_mission(self, queue_list):
        """Send task queue list as mission command."""
        if not self.node:
            return
        cmd_str = ",".join([item.strip() for item in queue_list if item.strip()])
        if cmd_str:
            self.node.publish_mission_command(cmd_str)

    def cancel_mission(self):
        """Abort/cancel active mission."""
        if self.node:
            self.node.publish_mission_cancel()

    def get_current_pose(self) -> Optional[Dict[str, Any]]:
        """Directly query current robot pose via TF."""
        if self.node:
            return self.node.get_robot_pose(timeout_sec=0.2)
        return None

    def notify_waypoints_updated(self):
        """Trigger waypoints reload in ROS ecosystem."""
        if self.node:
            self.node.notify_waypoints_reloaded()

    def send_teleop(self, linear_x: float, angular_z: float):
        """Send teleop velocity command."""
        if self.node:
            self.node.publish_teleop_cmd(linear_x, angular_z)

    def stop_robot(self):
        """Send zero velocity to stop robot."""
        if self.node:
            self.node.publish_teleop_cmd(0.0, 0.0)

#!/usr/bin/env python3
"""
RGB Video & Synchronized Telemetry Recorder Node
Developed for AGV VLM (Vision-Language Model) Dataset Collection.
Records 1080p 30FPS camera stream to MP4 and saves synchronized metadata (JSON)
containing frame_id, timestamp, cmd_vel, odom, and global localization.
"""

import os
import math
import time
import json
import queue
import threading
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_ros import Buffer, TransformListener, TransformException


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw angle (in radians) from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class RgbRecorderNode(Node):
    def __init__(self):
        super().__init__('rgb_recorder_node')

        # ----------------- Parameters -----------------
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('output_dir', '~/agv_ws/video/rgb')
        self.declare_parameter('session_name', '')
        self.declare_parameter('target_width', 1920)
        self.declare_parameter('target_height', 1080)
        self.declare_parameter('target_fps', 30.0)
        self.declare_parameter('video_codec', 'mp4v')
        self.declare_parameter('queue_size', 150)
        self.declare_parameter('auto_detect_resolution', True)
        self.declare_parameter('save_metadata', True)

        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        raw_output_dir = self.get_parameter('output_dir').value
        if raw_output_dir and '/home/diordty' in raw_output_dir and not os.path.exists('/home/diordty'):
            raw_output_dir = raw_output_dir.replace('/home/diordty', os.path.expanduser('~'))
        if not raw_output_dir:
            raw_output_dir = '~/agv_ws/video/rgb'
        self.output_dir = os.path.abspath(os.path.expanduser(raw_output_dir))

        self.session_name = self.get_parameter('session_name').value
        self.target_width = int(self.get_parameter('target_width').value)
        self.target_height = int(self.get_parameter('target_height').value)
        self.target_fps = float(self.get_parameter('target_fps').value)
        self.video_codec = self.get_parameter('video_codec').value
        self.queue_size = int(self.get_parameter('queue_size').value)
        self.auto_detect_resolution = bool(self.get_parameter('auto_detect_resolution').value)
        self.save_metadata = bool(self.get_parameter('save_metadata').value)

        # ----------------- Ensure Directory -----------------
        os.makedirs(self.output_dir, exist_ok=True)

        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        if not self.session_name:
            self.session_name = f'vlm_{timestamp_str}'

        self.video_filepath = os.path.join(self.output_dir, f'{self.session_name}.mp4')
        self.meta_filepath = os.path.join(self.output_dir, f'{self.session_name}_meta.json')

        self.get_logger().info('====================================================')
        self.get_logger().info('    AGV VLM DATASET COLLECTOR: RGB VIDEO RECORDER   ')
        self.get_logger().info('====================================================')
        self.get_logger().info(f'Output Video   : {self.video_filepath}')
        self.get_logger().info(f'Output Metadata: {self.meta_filepath}')
        self.get_logger().info(f'Target Setting : {self.target_width}x{self.target_height} @ {self.target_fps} FPS')
        self.get_logger().info(f'Subscribing    : {self.image_topic}')

        # ----------------- CV & Threading -----------------
        self.bridge = CvBridge()
        self.frame_queue = queue.Queue(maxsize=self.queue_size)
        self.stop_event = threading.Event()

        # Telemetry State Caches
        self.state_lock = threading.Lock()
        self.latest_cmd_vel = {
            'linear_x': 0.0,
            'linear_y': 0.0,
            'angular_z': 0.0
        }
        self.latest_odom = {
            'x': 0.0,
            'y': 0.0,
            'z': 0.0,
            'yaw_rad': 0.0,
            'yaw_deg': 0.0,
            'linear_speed': 0.0,
            'angular_speed': 0.0
        }

        # TF Listener for Global Localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----------------- Subscribers -----------------
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # ----------------- Background Worker -----------------
        self.frame_id_counter = 0
        self.start_wall_time = time.time()
        self.writer_thread = threading.Thread(target=self._writer_worker, daemon=True)
        self.writer_thread.start()

    def log_info(self, msg: str):
        if rclpy.ok():
            try:
                self.get_logger().info(msg)
                return
            except Exception:
                pass
        print(f'[rgb_recorder_node] [INFO]: {msg}')

    def cmd_vel_callback(self, msg: Twist):
        with self.state_lock:
            self.latest_cmd_vel = {
                'linear_x': round(msg.linear.x, 4),
                'linear_y': round(msg.linear.y, 4),
                'angular_z': round(msg.angular.z, 4)
            }

    def odom_callback(self, msg: Odometry):
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        with self.state_lock:
            self.latest_odom = {
                'x': round(msg.pose.pose.position.x, 4),
                'y': round(msg.pose.pose.position.y, 4),
                'z': round(msg.pose.pose.position.z, 4),
                'yaw_rad': round(yaw, 4),
                'yaw_deg': round(math.degrees(yaw), 2),
                'linear_speed': round(msg.twist.twist.linear.x, 4),
                'angular_speed': round(msg.twist.twist.angular.z, 4)
            }

    def get_global_pose(self):
        """Lookup transform from map_frame to base_frame."""
        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                now,
                timeout=Duration(seconds=0.05)
            )
            t = trans.transform.translation
            r = trans.transform.rotation
            yaw = quaternion_to_yaw(r.x, r.y, r.z, r.w)
            return {
                'available': True,
                'x': round(t.x, 4),
                'y': round(t.y, 4),
                'z': round(t.z, 4),
                'yaw_rad': round(yaw, 4),
                'yaw_deg': round(math.degrees(yaw), 2),
                'reference_frame': self.map_frame
            }
        except TransformException:
            return {
                'available': False,
                'x': None,
                'y': None,
                'z': None,
                'yaw_rad': None,
                'yaw_deg': None,
                'reference_frame': self.map_frame
            }

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert ROS image: {e}')
            return

        # Snapshot synchronized telemetry
        with self.state_lock:
            cmd_vel_snapshot = dict(self.latest_cmd_vel)
            odom_snapshot = dict(self.latest_odom)

        global_pose_snapshot = self.get_global_pose()

        header_stamp = msg.header.stamp
        timestamp_ros = header_stamp.sec + (header_stamp.nanosec * 1e-9)
        iso_str = datetime.now().isoformat()

        metadata_item = {
            'frame_id': self.frame_id_counter,
            'timestamp_ros': round(timestamp_ros, 6),
            'timestamp_iso': iso_str,
            'cmd_vel': cmd_vel_snapshot,
            'odom': odom_snapshot,
            'global_pose': global_pose_snapshot
        }
        self.frame_id_counter += 1

        try:
            self.frame_queue.put_nowait((cv_image, metadata_item))
        except queue.Full:
            self.get_logger().warn('Recording queue is full! Dropping frame to preserve real-time sync.')

    def _writer_worker(self):
        video_writer = None
        all_metadata = []
        frames_written = 0
        actual_width = self.target_width
        actual_height = self.target_height

        self.get_logger().info('Video writer background worker initialized and ready.')

        while not (self.stop_event.is_set() and self.frame_queue.empty()):
            try:
                cv_image, meta_item = self.frame_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            h, w = cv_image.shape[:2]

            # Lazy init VideoWriter on first valid frame
            if video_writer is None:
                if self.auto_detect_resolution:
                    actual_width = w
                    actual_height = h
                    self.get_logger().info(f'Auto-detected camera stream resolution: {w}x{h}')
                else:
                    actual_width = self.target_width
                    actual_height = self.target_height

                fourcc = cv2.VideoWriter_fourcc(*self.video_codec)
                video_writer = cv2.VideoWriter(
                    self.video_filepath,
                    fourcc,
                    self.target_fps,
                    (actual_width, actual_height)
                )

                if not video_writer.isOpened():
                    self.get_logger().error(f'Failed to open VideoWriter with codec {self.video_codec}. Fallback to MJPG.')
                    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                    self.video_filepath = os.path.splitext(self.video_filepath)[0] + '.avi'
                    video_writer = cv2.VideoWriter(
                        self.video_filepath,
                        fourcc,
                        self.target_fps,
                        (actual_width, actual_height)
                    )

            # Resize if dimensions differ from writer
            if (w, h) != (actual_width, actual_height):
                cv_image = cv2.resize(cv_image, (actual_width, actual_height))

            video_writer.write(cv_image)
            all_metadata.append(meta_item)
            frames_written += 1

            if frames_written % int(self.target_fps * 2) == 0:
                elapsed = frames_written / self.target_fps
                self.get_logger().info(
                    f'Recording... [{frames_written} frames | ~{elapsed:.1f}s | Queue: {self.frame_queue.qsize()}]'
                )

            self.frame_queue.task_done()

        # ----------------- Flush & Close -----------------
        if video_writer is not None:
            video_writer.release()
            self.log_info(f'Successfully saved video: {self.video_filepath} ({frames_written} frames)')

        if self.save_metadata:
            session_summary = {
                'session_info': {
                    'session_name': self.session_name,
                    'start_time': datetime.fromtimestamp(self.start_wall_time).isoformat(),
                    'end_time': datetime.now().isoformat(),
                    'total_frames': frames_written,
                    'target_fps': self.target_fps,
                    'resolution': [actual_width, actual_height],
                    'topics': {
                        'image': self.image_topic,
                        'cmd_vel': self.cmd_vel_topic,
                        'odom': self.odom_topic,
                        'map_frame': self.map_frame,
                        'base_frame': self.base_frame
                    }
                },
                'frames': all_metadata
            }
            try:
                with open(self.meta_filepath, 'w', encoding='utf-8') as f:
                    json.dump(session_summary, f, indent=2)
                self.log_info(f'Successfully saved metadata: {self.meta_filepath}')
            except Exception as e:
                print(f'[rgb_recorder_node] [ERROR]: Failed to write metadata JSON: {e}')

    def stop_recording(self):
        self.log_info('Stopping recording, flushing remaining frames from buffer...')
        self.stop_event.set()
        self.writer_thread.join(timeout=10.0)
        self.log_info('Recording completed cleanly.')


def main(args=None):
    rclpy.init(args=args)
    node = RgbRecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop_recording()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

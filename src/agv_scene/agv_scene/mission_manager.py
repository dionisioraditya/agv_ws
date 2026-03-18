#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.state = "idle"

        self.vel_pub = self.create_publisher(Twist, '/cmd_vel_dock', 10)

        self.goal_objective_sub = self.create_subscription(
            String,
            '/goal_objective',
            self.goal_objective_callback,
            10
        )

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def goal_objective_callback(self, msg):
        self.get_logger().info(f"Received goal objective: {msg.data}")

        valid_goals = ["home", "point1", "point2", "point3"]

        if msg.data in valid_goals:
            self.state = msg.data
            self.send_goal()
        else:
            self.get_logger().warn(f"Unknown goal objective: {msg.data}")

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        if self.state == "home":
            goal_msg.pose.pose.position.x = -0.26
            goal_msg.pose.pose.position.y = 0.03
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.x = 0.0262233215972649
            goal_msg.pose.pose.orientation.y = -0.0013580498481072893
            goal_msg.pose.pose.orientation.z = -0.9948616978004114
            goal_msg.pose.pose.orientation.w = 0.10124328249164331

        elif self.state == "point1":
            goal_msg.pose.pose.position.x = 9.114886283874512
            goal_msg.pose.pose.position.y = -0.2326909452676773
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = -0.5399881524008281
            goal_msg.pose.pose.orientation.w = 0.8416726176291707

        elif self.state == "point2":
            goal_msg.pose.pose.position.x = 9.016785621643066
            goal_msg.pose.pose.position.y = -3.9377059936523438
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = 0.9999727640375905
            goal_msg.pose.pose.orientation.w = 0.007380459539976924

        elif self.state == "point3":
            goal_msg.pose.pose.position.x = 6.176360130310059
            goal_msg.pose.pose.position.y = -3.965082883834839
            goal_msg.pose.pose.position.z = 0.0
            goal_msg.pose.pose.orientation.x = 0.0
            goal_msg.pose.pose.orientation.y = 0.0
            goal_msg.pose.pose.orientation.z = -0.9990995879598777
            goal_msg.pose.pose.orientation.w = 0.0424265640654813

        else:
            self.get_logger().warn("No valid state set for navigation.")
            return

        self._action_client.wait_for_server()

        self.get_logger().info(f"Sending goal to: {self.state}")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'distance_remaining'):
            self.get_logger().info(
                f"Distance remaining: {feedback.distance_remaining:.2f}"
            )

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.state = "idle"
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')

            if self.state == "home":
                self.get_logger().info("Arrived at home, start docking")
                self.docking()
            else:
                self.state = "idle"

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Navigation aborted')
            self.state = "idle"

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation canceled')
            self.state = "idle"

        else:
            self.get_logger().warn(f'Navigation ended with status: {status}')
            self.state = "idle"

    def docking(self):
        self.get_logger().info("Docking process started")
        # coming soon docking algorithm
        
        ###########################
        msg = Twist()
        self.vel_pub.publish(msg)
        self.state = "idle"


def main(args=None):
    rclpy.init(args=args)
    agv_scene = MissionManager()
    rclpy.spin(agv_scene)
    agv_scene.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
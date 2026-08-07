#!/usr/bin/env python3

import json
from collections import deque
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ============================================================
# FINITE STATE MACHINE
# ============================================================

class MissionState(Enum):
    IDLE = auto()
    MISSION_RECEIVED = auto()
    NAVIGATING = auto()
    MISSION_COMPLETED = auto()
    MISSION_FAILED = auto()
    MISSION_CANCELED = auto()


# ============================================================
# MISSION DATA
# ============================================================

@dataclass
class Mission:
    """
    Satu mission terdiri atas satu atau lebih objective.

    Contoh:
        Mission 001
        ├── point1
        ├── point2
        └── home
    """

    mission_id: str
    objectives: Deque[str] = field(default_factory=deque)


# ============================================================
# MISSION MANAGER NODE
# ============================================================

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ----------------------------------------------------
        # FSM
        # ----------------------------------------------------

        self.current_state = MissionState.IDLE

        # ----------------------------------------------------
        # Mission Management
        # ----------------------------------------------------

        # Queue untuk mission yang belum dieksekusi
        self.mission_queue: Deque[Mission] = deque()

        # Mission yang sedang aktif
        self.active_mission: Optional[Mission] = None

        # Objective yang sedang dieksekusi
        self.current_objective: Optional[str] = None

        # Counter ID mission
        self.mission_counter = 0

        # ----------------------------------------------------
        # Nav2 Goal Handle
        # ----------------------------------------------------

        self.current_goal_handle = None

        # ----------------------------------------------------
        # Waypoint Database
        # ----------------------------------------------------
        #
        # Untuk sementara waypoint masih disimpan di sini.
        # Nantinya dapat dipindahkan ke file YAML.
        #
        # Format:
        #
        # waypoint_name:
        #     x
        #     y
        #     z
        #     qx
        #     qy
        #     qz
        #     qw
        #
        # ----------------------------------------------------

        self.waypoints = {

            "home": {
                "x": -0.26,
                "y": 0.03,
                "z": 0.0,
                "qx": 0.0262233215972649,
                "qy": -0.0013580498481072893,
                "qz": -0.9948616978004114,
                "qw": 0.10124328249164331,
            },

            "point1": {
                "x": 9.114886283874512,
                "y": -0.2326909452676773,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": -0.5399881524008281,
                "qw": 0.8416726176291707,
            },

            "point2": {
                "x": 9.016785621643066,
                "y": -3.9377059936523438,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": 0.9999727640375905,
                "qw": 0.007380459539976924,
            },

            "point3": {
                "x": 6.176360130310059,
                "y": -3.965082883834839,
                "z": 0.0,
                "qx": 0.0,
                "qy": 0.0,
                "qz": -0.9990995879598777,
                "qw": 0.0424265640654813,
            },
        }

        # ----------------------------------------------------
        # ROS2 Subscriber
        # ----------------------------------------------------

        # Format mission:
        #
        # point1
        #
        # atau:
        #
        # point1,point2,home
        #
        self.mission_sub = self.create_subscription(
            String,
            '/mission_command',
            self.mission_callback,
            10
        )

        # Cancel mission aktif
        self.cancel_sub = self.create_subscription(
            Bool,
            '/mission_cancel',
            self.cancel_callback,
            10
        )

        # ----------------------------------------------------
        # ROS2 Publisher
        # ----------------------------------------------------

        # Publisher status Mission Manager
        self.status_pub = self.create_publisher(
            String,
            '/mission_status',
            10
        )

        # ----------------------------------------------------
        # Nav2 Action Client
        # ----------------------------------------------------

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # ----------------------------------------------------
        # Initialization
        # ----------------------------------------------------

        self.get_logger().info(
            "Mission Management System started"
        )

        self.transition_to(
            MissionState.IDLE,
            note="Mission Manager initialized"
        )

    # ========================================================
    # RECEIVE MISSION
    # ========================================================

    def mission_callback(self, msg: String):
        """
        Menerima satu mission.

        Format:

            point1

        atau:

            point1,point2,home

        Satu message dianggap sebagai satu mission.
        """

        raw_command = msg.data.strip()

        if not raw_command:
            self.get_logger().warn(
                "Received empty mission command"
            )
            return

        # ----------------------------------------------------
        # Parse objective
        # ----------------------------------------------------

        objectives = [
            objective.strip().lower()
            for objective in raw_command.split(',')
            if objective.strip()
        ]

        if len(objectives) == 0:
            self.get_logger().warn(
                "Mission does not contain any objective"
            )
            return

        # ----------------------------------------------------
        # Validate objective
        # ----------------------------------------------------

        invalid_objectives = [
            objective
            for objective in objectives
            if objective not in self.waypoints
        ]

        if invalid_objectives:
            self.get_logger().warn(
                "Mission rejected. Unknown objective(s): "
                + ", ".join(invalid_objectives)
            )

            self.publish_status(
                note=(
                    "Mission rejected: invalid objective(s) "
                    + ", ".join(invalid_objectives)
                )
            )

            return

        # ----------------------------------------------------
        # Create Mission
        # ----------------------------------------------------

        self.mission_counter += 1

        mission_id = f"mission_{self.mission_counter:03d}"

        mission = Mission(
            mission_id=mission_id,
            objectives=deque(objectives)
        )

        # Masukkan ke Mission Queue
        self.mission_queue.append(mission)

        self.get_logger().info(
            f"Received {mission_id}: {objectives}"
        )

        self.get_logger().info(
            f"Mission queue size: {len(self.mission_queue)}"
        )

        self.publish_status(
            note=f"{mission_id} added to mission queue"
        )

        # ----------------------------------------------------
        # Jika tidak ada mission aktif, langsung jalankan
        # ----------------------------------------------------

        if self.active_mission is None:
            self.start_next_mission()

    # ========================================================
    # START NEXT MISSION
    # ========================================================

    def start_next_mission(self):
        """
        Mengambil mission berikutnya dari mission queue.
        """

        # Jangan menjalankan mission baru jika masih ada
        # mission yang aktif.
        if self.active_mission is not None:
            return

        # Jika queue kosong, kembali ke IDLE.
        if not self.mission_queue:

            if self.current_state != MissionState.IDLE:
                self.transition_to(
                    MissionState.IDLE,
                    note="No mission remaining"
                )

            return

        # ----------------------------------------------------
        # Ambil mission paling depan
        # ----------------------------------------------------

        self.active_mission = self.mission_queue.popleft()

        self.current_objective = None

        self.get_logger().info(
            f"Starting {self.active_mission.mission_id}"
        )

        self.transition_to(
            MissionState.MISSION_RECEIVED,
            note=(
                f"{self.active_mission.mission_id} "
                "started"
            )
        )

        # Mulai objective pertama
        self.execute_next_objective()

    # ========================================================
    # EXECUTE OBJECTIVE
    # ========================================================

    def execute_next_objective(self):
        """
        Mengambil objective berikutnya dari active mission.
        """

        if self.active_mission is None:
            return

        # ----------------------------------------------------
        # Jika seluruh objective telah selesai
        # ----------------------------------------------------

        if not self.active_mission.objectives:
            self.complete_active_mission()
            return

        # ----------------------------------------------------
        # Dequeue objective
        # ----------------------------------------------------

        self.current_objective = (
            self.active_mission.objectives.popleft()
        )

        self.get_logger().info(
            f"Executing objective: {self.current_objective}"
        )

        self.get_logger().info(
            "Remaining objectives: "
            f"{list(self.active_mission.objectives)}"
        )

        self.publish_status(
            note=(
                f"Executing objective "
                f"{self.current_objective}"
            )
        )

        # Kirim objective ke Nav2
        self.send_navigation_goal()

    # ========================================================
    # SEND NAVIGATION GOAL
    # ========================================================

    def send_navigation_goal(self):
        """
        Mengirim pose dari current objective ke Nav2.
        """

        if self.current_objective is None:
            self.get_logger().error(
                "No current objective"
            )
            self.fail_active_mission(
                "No current objective"
            )
            return

        waypoint = self.waypoints.get(
            self.current_objective
        )

        if waypoint is None:
            self.get_logger().error(
                f"Waypoint not found: "
                f"{self.current_objective}"
            )

            self.fail_active_mission(
                f"Waypoint not found: "
                f"{self.current_objective}"
            )
            return

        # ----------------------------------------------------
        # Construct NavigateToPose Goal
        # ----------------------------------------------------

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # Position
        goal_msg.pose.pose.position.x = waypoint["x"]
        goal_msg.pose.pose.position.y = waypoint["y"]
        goal_msg.pose.pose.position.z = waypoint["z"]

        # Orientation Quaternion
        goal_msg.pose.pose.orientation.x = waypoint["qx"]
        goal_msg.pose.pose.orientation.y = waypoint["qy"]
        goal_msg.pose.pose.orientation.z = waypoint["qz"]
        goal_msg.pose.pose.orientation.w = waypoint["qw"]

        # ----------------------------------------------------
        # Check Nav2 Action Server
        # ----------------------------------------------------

        self.get_logger().info(
            "Waiting for Nav2 NavigateToPose server..."
        )

        server_available = (
            self._action_client.wait_for_server(
                timeout_sec=5.0
            )
        )

        if not server_available:
            self.get_logger().error(
                "NavigateToPose action server "
                "is not available"
            )

            self.fail_active_mission(
                "Nav2 NavigateToPose server unavailable"
            )
            return

        # ----------------------------------------------------
        # Send Goal
        # ----------------------------------------------------

        self.get_logger().info(
            f"Sending Nav2 goal: "
            f"{self.current_objective}"
        )

        self._send_goal_future = (
            self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback
            )
        )

        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    # ========================================================
    # NAV2 GOAL RESPONSE
    # ========================================================

    def goal_response_callback(self, future):
        """
        Dipanggil ketika Nav2 memberikan response terhadap goal.
        """

        try:
            goal_handle = future.result()

        except Exception as error:
            self.get_logger().error(
                f"Failed to send goal: {error}"
            )

            self.fail_active_mission(
                f"Failed to send navigation goal: {error}"
            )

            return

        # ----------------------------------------------------
        # Goal rejected
        # ----------------------------------------------------

        if not goal_handle.accepted:

            self.get_logger().warn(
                f"Navigation goal rejected: "
                f"{self.current_objective}"
            )

            self.current_goal_handle = None

            self.fail_active_mission(
                "Navigation goal rejected"
            )

            return

        # ----------------------------------------------------
        # Goal accepted
        # ----------------------------------------------------

        self.current_goal_handle = goal_handle

        self.get_logger().info(
            f"Navigation goal accepted: "
            f"{self.current_objective}"
        )

        if self.current_state != MissionState.NAVIGATING:

            self.transition_to(
                MissionState.NAVIGATING,
                note=(
                    f"Navigating to "
                    f"{self.current_objective}"
                )
            )

        else:

            # Untuk objective berikutnya dalam mission
            self.publish_status(
                note=(
                    f"Navigating to "
                    f"{self.current_objective}"
                )
            )

        # ----------------------------------------------------
        # Request Result
        # ----------------------------------------------------

        self._get_result_future = (
            goal_handle.get_result_async()
        )

        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    # ========================================================
    # NAV2 FEEDBACK
    # ========================================================

    def feedback_callback(self, feedback_msg):
        """
        Menerima feedback NavigateToPose dari Nav2.
        """

        feedback = feedback_msg.feedback

        distance_remaining = getattr(
            feedback,
            'distance_remaining',
            None
        )

        if distance_remaining is not None:

            self.get_logger().info(
                f"[{self.current_objective}] "
                f"Distance remaining: "
                f"{distance_remaining:.2f} m"
            )

    # ========================================================
    # NAV2 RESULT
    # ========================================================

    def get_result_callback(self, future):
        """
        Menangani hasil akhir objective navigasi.
        """

        try:
            result = future.result()

        except Exception as error:

            self.get_logger().error(
                f"Navigation result error: {error}"
            )

            self.current_goal_handle = None

            self.fail_active_mission(
                f"Navigation result error: {error}"
            )

            return

        status = result.status

        self.current_goal_handle = None

        # ----------------------------------------------------
        # SUCCESS
        # ----------------------------------------------------

        if status == GoalStatus.STATUS_SUCCEEDED:

            completed_objective = (
                self.current_objective
            )

            self.get_logger().info(
                f"Objective succeeded: "
                f"{completed_objective}"
            )

            self.current_objective = None

            # Jika masih ada objective:
            #
            # point1 -> success
            # point2 -> next
            #
            if (
                self.active_mission is not None
                and self.active_mission.objectives
            ):

                self.execute_next_objective()

            # Jika objective queue kosong,
            # mission selesai.
            else:

                self.complete_active_mission()

        # ----------------------------------------------------
        # ABORTED
        # ----------------------------------------------------

        elif status == GoalStatus.STATUS_ABORTED:

            self.get_logger().warn(
                f"Objective aborted: "
                f"{self.current_objective}"
            )

            self.fail_active_mission(
                f"Navigation aborted at "
                f"{self.current_objective}"
            )

        # ----------------------------------------------------
        # CANCELED
        # ----------------------------------------------------

        elif status == GoalStatus.STATUS_CANCELED:

            self.get_logger().warn(
                f"Objective canceled: "
                f"{self.current_objective}"
            )

            self.cancel_active_mission()

        # ----------------------------------------------------
        # OTHER STATUS
        # ----------------------------------------------------

        else:

            self.get_logger().warn(
                f"Navigation ended with "
                f"unknown status: {status}"
            )

            self.fail_active_mission(
                f"Unknown navigation status: {status}"
            )

    # ========================================================
    # MISSION COMPLETED
    # ========================================================

    def complete_active_mission(self):
        """
        Dipanggil ketika seluruh objective dalam mission
        telah berhasil.
        """

        if self.active_mission is None:
            return

        mission_id = self.active_mission.mission_id

        self.transition_to(
            MissionState.MISSION_COMPLETED,
            note=f"{mission_id} completed"
        )

        self.get_logger().info(
            f"{mission_id} COMPLETED"
        )

        # Clear active mission
        self.active_mission = None
        self.current_objective = None
        self.current_goal_handle = None

        # FSM kembali IDLE
        self.transition_to(
            MissionState.IDLE,
            note=f"{mission_id} finished"
        )

        # Jalankan mission berikutnya jika ada
        self.start_next_mission()

    # ========================================================
    # MISSION FAILED
    # ========================================================

    def fail_active_mission(self, reason: str):
        """
        Dipanggil jika objective gagal dan menyebabkan
        mission dianggap gagal.
        """

        if self.active_mission is None:

            self.transition_to(
                MissionState.MISSION_FAILED,
                note=reason
            )

            self.transition_to(
                MissionState.IDLE
            )

            return

        mission_id = self.active_mission.mission_id

        self.transition_to(
            MissionState.MISSION_FAILED,
            note=f"{mission_id}: {reason}"
        )

        self.get_logger().error(
            f"{mission_id} FAILED: {reason}"
        )

        # Semua objective yang tersisa dibuang karena
        # mission dianggap gagal.
        self.active_mission.objectives.clear()

        self.active_mission = None
        self.current_objective = None
        self.current_goal_handle = None

        self.transition_to(
            MissionState.IDLE,
            note=f"{mission_id} terminated"
        )

        # Jalankan mission berikutnya
        self.start_next_mission()

    # ========================================================
    # CANCEL REQUEST
    # ========================================================

    def cancel_callback(self, msg: Bool):
        """
        Cancel mission aktif.

        Command:

        ros2 topic pub --once /mission_cancel \
        std_msgs/msg/Bool "{data: true}"
        """

        if not msg.data:
            return

        # Tidak ada mission aktif
        if self.active_mission is None:

            self.get_logger().warn(
                "No active mission to cancel"
            )

            return

        # Mission ada tetapi goal belum memiliki handle
        if self.current_goal_handle is None:

            self.get_logger().warn(
                "Navigation goal is not active yet"
            )

            return

        self.get_logger().info(
            f"Cancel requested for "
            f"{self.active_mission.mission_id}"
        )

        cancel_future = (
            self.current_goal_handle.cancel_goal_async()
        )

        cancel_future.add_done_callback(
            self.cancel_response_callback
        )

    # ========================================================
    # CANCEL RESPONSE
    # ========================================================

    def cancel_response_callback(self, future):

        try:
            cancel_response = future.result()

        except Exception as error:

            self.get_logger().error(
                f"Cancel request failed: {error}"
            )

            return

        if len(cancel_response.goals_canceling) > 0:

            self.get_logger().info(
                "Nav2 accepted cancel request"
            )

        else:

            self.get_logger().warn(
                "Nav2 rejected cancel request"
            )

    # ========================================================
    # MISSION CANCELED
    # ========================================================

    def cancel_active_mission(self):
        """
        Membersihkan active mission setelah Nav2
        mengonfirmasi STATUS_CANCELED.
        """

        if self.active_mission is None:
            return

        mission_id = self.active_mission.mission_id

        self.transition_to(
            MissionState.MISSION_CANCELED,
            note=f"{mission_id} canceled"
        )

        self.get_logger().warn(
            f"{mission_id} CANCELED"
        )

        self.active_mission.objectives.clear()

        self.active_mission = None
        self.current_objective = None
        self.current_goal_handle = None

        self.transition_to(
            MissionState.IDLE,
            note=f"{mission_id} canceled and cleared"
        )

        # Mission setelahnya tetap boleh jalan
        self.start_next_mission()

    # ========================================================
    # FSM TRANSITION
    # ========================================================

    def transition_to(
        self,
        new_state: MissionState,
        note: str = ""
    ):
        """
        Mengatur transisi Finite State Machine.
        """

        old_state = self.current_state

        self.current_state = new_state

        self.get_logger().info(
            f"FSM: "
            f"{old_state.name} -> "
            f"{new_state.name}"
        )

        self.publish_status(note=note)

    # ========================================================
    # STATUS PUBLISHER
    # ========================================================

    def publish_status(self, note: str = ""):
        """
        Publish kondisi Mission Manager dalam format JSON.
        """

        msg = String()

        if self.active_mission is not None:

            active_mission_id = (
                self.active_mission.mission_id
            )

            remaining_objectives = list(
                self.active_mission.objectives
            )

        else:

            active_mission_id = None
            remaining_objectives = []

        queued_missions = [
            mission.mission_id
            for mission in self.mission_queue
        ]

        status_data = {
            "state": self.current_state.name,
            "active_mission": active_mission_id,
            "current_objective": self.current_objective,
            "remaining_objectives": remaining_objectives,
            "queued_missions": queued_missions,
            "note": note,
        }

        msg.data = json.dumps(status_data)

        self.status_pub.publish(msg)


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    mission_manager = MissionManager()

    try:

        rclpy.spin(mission_manager)

    except KeyboardInterrupt:

        pass

    finally:

        mission_manager.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
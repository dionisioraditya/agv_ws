#!/usr/bin/env python3

import csv
import json
import time

from collections import deque
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, auto
from pathlib import Path
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
    mission_id: str
    objectives: Deque[str] = field(default_factory=deque)

    # Untuk kebutuhan eksperimen
    total_objectives: int = 0
    completed_objectives: int = 0

    received_time: float = 0.0
    started_time: Optional[float] = None
    first_goal_sent_time: Optional[float] = None


# ============================================================
# MISSION MANAGER NODE
# ============================================================

class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')

        # ====================================================
        # ROS2 PARAMETERS FOR EXPERIMENT
        # ====================================================

        self.declare_parameter(
            'experiment_id',
            'default_experiment'
        )

        self.declare_parameter(
            'log_directory',
            str(Path.home() / 'agv_experiment_logs')
        )

        self.experiment_id = (
            self.get_parameter('experiment_id')
            .get_parameter_value()
            .string_value
        )

        log_directory = (
            self.get_parameter('log_directory')
            .get_parameter_value()
            .string_value
        )

        # ====================================================
        # FSM
        # ====================================================

        self.current_state = MissionState.IDLE

        # ====================================================
        # MISSION MANAGEMENT
        # ====================================================

        self.mission_queue: Deque[Mission] = deque()

        self.active_mission: Optional[Mission] = None

        self.current_objective: Optional[str] = None

        self.mission_counter = 0

        # ====================================================
        # NAV2 GOAL HANDLE
        # ====================================================

        self.current_goal_handle = None

        # ====================================================
        # TIMING VARIABLES FOR EXPERIMENT
        # ====================================================

        self.current_objective_start_time = None

        self.previous_objective_end_time = None

        self.current_objective_transition_latency = 0.0

        self.last_distance_remaining = None

        # ====================================================
        # WAYPOINT DATABASE
        # ====================================================

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

        # ====================================================
        # ROS2 SUBSCRIBERS
        # ====================================================

        self.mission_sub = self.create_subscription(
            String,
            '/mission_command',
            self.mission_callback,
            10
        )

        self.cancel_sub = self.create_subscription(
            Bool,
            '/mission_cancel',
            self.cancel_callback,
            10
        )

        # ====================================================
        # ROS2 PUBLISHERS
        # ====================================================

        self.status_pub = self.create_publisher(
            String,
            '/mission_status',
            10
        )

        # ====================================================
        # NAV2 ACTION CLIENT
        # ====================================================

        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        # ====================================================
        # EXPERIMENT LOGGER
        # ====================================================

        self.setup_csv_logger(log_directory)

        self.get_logger().info(
            "======================================"
        )
        self.get_logger().info(
            "Mission Management System started"
        )
        self.get_logger().info(
            f"Experiment ID: {self.experiment_id}"
        )
        self.get_logger().info(
            f"Log directory: {self.session_directory}"
        )
        self.get_logger().info(
            "======================================"
        )

        self.transition_to(
            MissionState.IDLE,
            note="Mission Manager initialized"
        )

    # ========================================================
    # CSV LOGGER SETUP
    # ========================================================

    def setup_csv_logger(self, log_directory):

        session_timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        self.session_directory = (
            Path(log_directory)
            / f"{self.experiment_id}_{session_timestamp}"
        )

        self.session_directory.mkdir(
            parents=True,
            exist_ok=True
        )

        # ----------------------------------------------------
        # Event Log
        # ----------------------------------------------------

        self.event_file = open(
            self.session_directory / "mission_events.csv",
            mode="w",
            newline="",
            encoding="utf-8"
        )

        self.event_writer = csv.writer(
            self.event_file
        )

        self.event_writer.writerow([
            "timestamp",
            "monotonic_time_s",
            "experiment_id",
            "mission_id",
            "state",
            "event",
            "current_objective",
            "remaining_objectives",
            "mission_queue_size",
            "distance_remaining_m",
            "note"
        ])

        # ----------------------------------------------------
        # Objective Summary
        # ----------------------------------------------------

        self.objective_file = open(
            self.session_directory / "objective_summary.csv",
            mode="w",
            newline="",
            encoding="utf-8"
        )

        self.objective_writer = csv.writer(
            self.objective_file
        )

        self.objective_writer.writerow([
            "experiment_id",
            "mission_id",
            "objective",
            "status",
            "execution_time_s",
            "transition_latency_s"
        ])

        # ----------------------------------------------------
        # Mission Summary
        # ----------------------------------------------------

        self.mission_file = open(
            self.session_directory / "mission_summary.csv",
            mode="w",
            newline="",
            encoding="utf-8"
        )

        self.mission_writer = csv.writer(
            self.mission_file
        )

        self.mission_writer.writerow([
            "experiment_id",
            "mission_id",
            "total_objectives",
            "completed_objectives",
            "objective_completion_rate_percent",
            "mission_status",
            "queue_wait_time_s",
            "dispatch_latency_s",
            "mission_execution_time_s"
        ])

        self.event_file.flush()
        self.objective_file.flush()
        self.mission_file.flush()

    # ========================================================
    # LOG EVENT
    # ========================================================

    def log_event(
        self,
        event,
        note=""
    ):

        mission_id = ""

        remaining_objectives = []

        if self.active_mission is not None:

            mission_id = (
                self.active_mission.mission_id
            )

            remaining_objectives = list(
                self.active_mission.objectives
            )

        self.event_writer.writerow([
            datetime.now().isoformat(
                timespec='milliseconds'
            ),

            f"{time.monotonic():.6f}",

            self.experiment_id,

            mission_id,

            self.current_state.name,

            event,

            self.current_objective
            if self.current_objective
            else "",

            json.dumps(
                remaining_objectives
            ),

            len(self.mission_queue),

            (
                f"{self.last_distance_remaining:.3f}"
                if self.last_distance_remaining
                is not None
                else ""
            ),

            note
        ])

        self.event_file.flush()

    # ========================================================
    # RECEIVE MISSION
    # ========================================================

    def mission_callback(self, msg: String):

        raw_command = msg.data.strip()

        receive_time = time.monotonic()

        if not raw_command:

            self.get_logger().warn(
                "Received empty mission command"
            )

            self.log_event(
                "MISSION_REJECTED",
                "Empty mission"
            )

            return

        # ----------------------------------------------------
        # Parse Objectives
        # ----------------------------------------------------

        objectives = [
            objective.strip().lower()
            for objective in raw_command.split(',')
            if objective.strip()
        ]

        if len(objectives) == 0:

            self.log_event(
                "MISSION_REJECTED",
                "No objective"
            )

            return

        # ----------------------------------------------------
        # Validate Objectives
        # ----------------------------------------------------

        invalid_objectives = [
            objective
            for objective in objectives
            if objective not in self.waypoints
        ]

        if invalid_objectives:

            reason = (
                "Invalid objective(s): "
                + ", ".join(invalid_objectives)
            )

            self.get_logger().warn(
                reason
            )

            self.log_event(
                "MISSION_REJECTED",
                reason
            )

            self.publish_status(
                note=reason
            )

            return

        # ----------------------------------------------------
        # Create Mission
        # ----------------------------------------------------

        self.mission_counter += 1

        mission_id = (
            f"mission_{self.mission_counter:03d}"
        )

        mission = Mission(
            mission_id=mission_id,
            objectives=deque(objectives),
            total_objectives=len(objectives),
            completed_objectives=0,
            received_time=receive_time
        )

        self.mission_queue.append(
            mission
        )

        self.get_logger().info(
            f"Received {mission_id}: "
            f"{objectives}"
        )

        self.log_event(
            "MISSION_ENQUEUED",
            f"{mission_id}: {objectives}"
        )

        self.publish_status(
            note=(
                f"{mission_id} added "
                "to mission queue"
            )
        )

        # Jika tidak ada mission aktif
        if self.active_mission is None:
            self.start_next_mission()

    # ========================================================
    # START NEXT MISSION
    # ========================================================

    def start_next_mission(self):

        if self.active_mission is not None:
            return

        if not self.mission_queue:

            if (
                self.current_state
                != MissionState.IDLE
            ):

                self.transition_to(
                    MissionState.IDLE,
                    note="No mission remaining"
                )

            return

        self.active_mission = (
            self.mission_queue.popleft()
        )

        self.active_mission.started_time = (
            time.monotonic()
        )

        self.current_objective = None

        self.previous_objective_end_time = None

        queue_wait = (
            self.active_mission.started_time
            - self.active_mission.received_time
        )

        self.get_logger().info(
            f"Starting "
            f"{self.active_mission.mission_id}"
        )

        self.log_event(
            "MISSION_STARTED",
            f"Queue wait: "
            f"{queue_wait:.6f} s"
        )

        self.transition_to(
            MissionState.MISSION_RECEIVED,
            note=(
                f"{self.active_mission.mission_id} "
                "started"
            )
        )

        self.execute_next_objective()

    # ========================================================
    # EXECUTE NEXT OBJECTIVE
    # ========================================================

    def execute_next_objective(self):

        if self.active_mission is None:
            return

        if not self.active_mission.objectives:

            self.complete_active_mission()
            return

        self.current_objective = (
            self.active_mission
            .objectives
            .popleft()
        )

        # ----------------------------------------------------
        # Measure transition latency
        # ----------------------------------------------------

        now = time.monotonic()

        if self.previous_objective_end_time is None:

            self.current_objective_transition_latency = 0.0

        else:

            self.current_objective_transition_latency = (
                now
                - self.previous_objective_end_time
            )

        self.current_objective_start_time = now

        self.get_logger().info(
            f"Executing objective: "
            f"{self.current_objective}"
        )

        self.log_event(
            "OBJECTIVE_SELECTED",
            f"Transition latency: "
            f"{self.current_objective_transition_latency:.6f} s"
        )

        self.publish_status(
            note=(
                f"Executing objective "
                f"{self.current_objective}"
            )
        )

        self.send_navigation_goal()

    # ========================================================
    # SEND NAVIGATION GOAL
    # ========================================================

    def send_navigation_goal(self):

        if self.current_objective is None:

            self.fail_active_mission(
                "No current objective"
            )

            return

        waypoint = self.waypoints.get(
            self.current_objective
        )

        if waypoint is None:

            self.fail_active_mission(
                f"Waypoint not found: "
                f"{self.current_objective}"
            )

            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal_msg.pose.pose.position.x = (
            waypoint["x"]
        )

        goal_msg.pose.pose.position.y = (
            waypoint["y"]
        )

        goal_msg.pose.pose.position.z = (
            waypoint["z"]
        )

        goal_msg.pose.pose.orientation.x = (
            waypoint["qx"]
        )

        goal_msg.pose.pose.orientation.y = (
            waypoint["qy"]
        )

        goal_msg.pose.pose.orientation.z = (
            waypoint["qz"]
        )

        goal_msg.pose.pose.orientation.w = (
            waypoint["qw"]
        )

        # ----------------------------------------------------
        # Wait for Nav2 Server
        # ----------------------------------------------------

        server_available = (
            self._action_client.wait_for_server(
                timeout_sec=5.0
            )
        )

        if not server_available:

            self.fail_active_mission(
                "Nav2 NavigateToPose "
                "server unavailable"
            )

            return

        goal_sent_time = time.monotonic()

        # Waktu goal pertama dikirim
        if (
            self.active_mission is not None
            and
            self.active_mission
            .first_goal_sent_time is None
        ):

            self.active_mission.first_goal_sent_time = (
                goal_sent_time
            )

        self.log_event(
            "GOAL_SENT",
            f"Goal sent to "
            f"{self.current_objective}"
        )

        self._send_goal_future = (
            self._action_client
            .send_goal_async(
                goal_msg,
                feedback_callback=
                self.feedback_callback
            )
        )

        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    # ========================================================
    # GOAL RESPONSE
    # ========================================================

    def goal_response_callback(self, future):

        try:

            goal_handle = future.result()

        except Exception as error:

            self.fail_active_mission(
                f"Failed to send goal: {error}"
            )

            return

        if not goal_handle.accepted:

            self.log_event(
                "GOAL_REJECTED"
            )

            self.current_goal_handle = None

            self.fail_active_mission(
                "Navigation goal rejected"
            )

            return

        self.current_goal_handle = (
            goal_handle
        )

        self.log_event(
            "GOAL_ACCEPTED"
        )

        if (
            self.current_state
            != MissionState.NAVIGATING
        ):

            self.transition_to(
                MissionState.NAVIGATING,
                note=(
                    f"Navigating to "
                    f"{self.current_objective}"
                )
            )

        else:

            self.publish_status(
                note=(
                    f"Navigating to "
                    f"{self.current_objective}"
                )
            )

        self._get_result_future = (
            goal_handle.get_result_async()
        )

        self._get_result_future.add_done_callback(
            self.get_result_callback
        )

    # ========================================================
    # NAV2 FEEDBACK
    # ========================================================

    def feedback_callback(
        self,
        feedback_msg
    ):

        feedback = feedback_msg.feedback

        distance_remaining = getattr(
            feedback,
            'distance_remaining',
            None
        )

        if distance_remaining is not None:

            self.last_distance_remaining = (
                distance_remaining
            )

            self.get_logger().info(
                f"[{self.current_objective}] "
                f"Distance remaining: "
                f"{distance_remaining:.2f} m"
            )

    # ========================================================
    # NAV2 RESULT
    # ========================================================

    def get_result_callback(
        self,
        future
    ):

        try:

            result = future.result()

        except Exception as error:

            self.fail_active_mission(
                f"Navigation result error: "
                f"{error}"
            )

            return

        status = result.status

        self.current_goal_handle = None

        if (
            status
            == GoalStatus.STATUS_SUCCEEDED
        ):

            self.finish_current_objective(
                "SUCCEEDED"
            )

            if (
                self.active_mission is not None
                and
                self.active_mission.objectives
            ):

                self.execute_next_objective()

            else:

                self.complete_active_mission()

        elif (
            status
            == GoalStatus.STATUS_ABORTED
        ):

            self.finish_current_objective(
                "FAILED"
            )

            self.fail_active_mission(
                "Navigation aborted"
            )

        elif (
            status
            == GoalStatus.STATUS_CANCELED
        ):

            self.finish_current_objective(
                "CANCELED"
            )

            self.cancel_active_mission()

        else:

            self.finish_current_objective(
                "FAILED"
            )

            self.fail_active_mission(
                f"Unknown navigation status: "
                f"{status}"
            )

    # ========================================================
    # OBJECTIVE FINISH + LOGGER
    # ========================================================

    def finish_current_objective(
        self,
        objective_status
    ):

        if (
            self.current_objective_start_time
            is None
        ):
            execution_time = 0.0

        else:
            execution_time = (
                time.monotonic()
                - self.current_objective_start_time
            )

        if (
            objective_status == "SUCCEEDED"
            and
            self.active_mission is not None
        ):

            self.active_mission.completed_objectives += 1

        self.objective_writer.writerow([
            self.experiment_id,

            (
                self.active_mission.mission_id
                if self.active_mission
                else ""
            ),

            (
                self.current_objective
                if self.current_objective
                else ""
            ),

            objective_status,

            f"{execution_time:.6f}",

            f"{self.current_objective_transition_latency:.6f}"
        ])

        self.objective_file.flush()

        self.log_event(
            f"OBJECTIVE_{objective_status}",
            f"Execution time: "
            f"{execution_time:.6f} s"
        )

        self.previous_objective_end_time = (
            time.monotonic()
        )

        self.current_objective_start_time = None

        self.current_objective = None

        self.last_distance_remaining = None

    # ========================================================
    # MISSION COMPLETED
    # ========================================================

    def complete_active_mission(self):

        if self.active_mission is None:
            return

        mission = self.active_mission

        self.transition_to(
            MissionState.MISSION_COMPLETED,
            note=(
                f"{mission.mission_id} "
                "completed"
            )
        )

        self.write_mission_summary(
            mission,
            "COMPLETED"
        )

        self.get_logger().info(
            f"{mission.mission_id} COMPLETED"
        )

        self.active_mission = None

        self.current_objective = None

        self.current_goal_handle = None

        self.transition_to(
            MissionState.IDLE,
            note=(
                f"{mission.mission_id} "
                "finished"
            )
        )

        self.start_next_mission()

    # ========================================================
    # MISSION FAILED
    # ========================================================

    def fail_active_mission(
        self,
        reason
    ):

        if self.active_mission is None:

            self.transition_to(
                MissionState.MISSION_FAILED,
                note=reason
            )

            self.transition_to(
                MissionState.IDLE
            )

            return

        mission = self.active_mission

        self.transition_to(
            MissionState.MISSION_FAILED,
            note=reason
        )

        self.write_mission_summary(
            mission,
            "FAILED"
        )

        self.get_logger().error(
            f"{mission.mission_id} FAILED: "
            f"{reason}"
        )

        mission.objectives.clear()

        self.active_mission = None

        self.current_objective = None

        self.current_goal_handle = None

        self.transition_to(
            MissionState.IDLE,
            note=(
                f"{mission.mission_id} "
                "terminated"
            )
        )

        self.start_next_mission()

    # ========================================================
    # CANCEL REQUEST
    # ========================================================

    def cancel_callback(
        self,
        msg: Bool
    ):

        if not msg.data:
            return

        if self.active_mission is None:

            self.get_logger().warn(
                "No active mission to cancel"
            )

            self.log_event(
                "CANCEL_REJECTED",
                "No active mission"
            )

            return

        if self.current_goal_handle is None:

            self.get_logger().warn(
                "Navigation goal is not active yet"
            )

            return

        self.log_event(
            "CANCEL_REQUESTED"
        )

        cancel_future = (
            self.current_goal_handle
            .cancel_goal_async()
        )

        cancel_future.add_done_callback(
            self.cancel_response_callback
        )

    # ========================================================
    # CANCEL RESPONSE
    # ========================================================

    def cancel_response_callback(
        self,
        future
    ):

        try:

            cancel_response = future.result()

        except Exception as error:

            self.log_event(
                "CANCEL_ERROR",
                str(error)
            )

            return

        if (
            len(cancel_response.goals_canceling)
            > 0
        ):

            self.log_event(
                "CANCEL_ACCEPTED"
            )

        else:

            self.log_event(
                "CANCEL_REJECTED",
                "Nav2 rejected cancel request"
            )

    # ========================================================
    # MISSION CANCELED
    # ========================================================

    def cancel_active_mission(self):

        if self.active_mission is None:
            return

        mission = self.active_mission

        self.transition_to(
            MissionState.MISSION_CANCELED,
            note=(
                f"{mission.mission_id} "
                "canceled"
            )
        )

        self.write_mission_summary(
            mission,
            "CANCELED"
        )

        mission.objectives.clear()

        self.active_mission = None

        self.current_objective = None

        self.current_goal_handle = None

        self.transition_to(
            MissionState.IDLE,
            note=(
                f"{mission.mission_id} "
                "canceled and cleared"
            )
        )

        self.start_next_mission()

    # ========================================================
    # WRITE MISSION SUMMARY
    # ========================================================

    def write_mission_summary(
        self,
        mission,
        mission_status
    ):

        end_time = time.monotonic()

        if mission.started_time is None:

            queue_wait_time = 0.0
            mission_execution_time = 0.0

        else:

            queue_wait_time = (
                mission.started_time
                - mission.received_time
            )

            mission_execution_time = (
                end_time
                - mission.started_time
            )

        if (
            mission.first_goal_sent_time is None
            or
            mission.started_time is None
        ):

            dispatch_latency = 0.0

        else:

            dispatch_latency = (
                mission.first_goal_sent_time
                - mission.started_time
            )

        if mission.total_objectives > 0:

            completion_rate = (
                mission.completed_objectives
                / mission.total_objectives
            ) * 100.0

        else:

            completion_rate = 0.0

        self.mission_writer.writerow([
            self.experiment_id,

            mission.mission_id,

            mission.total_objectives,

            mission.completed_objectives,

            f"{completion_rate:.2f}",

            mission_status,

            f"{queue_wait_time:.6f}",

            f"{dispatch_latency:.6f}",

            f"{mission_execution_time:.6f}"
        ])

        self.mission_file.flush()

    # ========================================================
    # FSM TRANSITION
    # ========================================================

    def transition_to(
        self,
        new_state,
        note=""
    ):

        old_state = self.current_state

        self.current_state = new_state

        self.get_logger().info(
            f"FSM: "
            f"{old_state.name} -> "
            f"{new_state.name}"
        )

        self.log_event(
            "STATE_TRANSITION",
            (
                f"{old_state.name}"
                f" -> "
                f"{new_state.name}. "
                f"{note}"
            )
        )

        self.publish_status(
            note=note
        )

    # ========================================================
    # STATUS PUBLISHER
    # ========================================================

    def publish_status(
        self,
        note=""
    ):

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

            "state":
                self.current_state.name,

            "active_mission":
                active_mission_id,

            "current_objective":
                self.current_objective,

            "remaining_objectives":
                remaining_objectives,

            "queued_missions":
                queued_missions,

            "note":
                note
        }

        msg.data = json.dumps(
            status_data
        )

        self.status_pub.publish(
            msg
        )

    # ========================================================
    # CLOSE LOG FILES
    # ========================================================

    def close_log_files(self):

        for file_handle in [
            self.event_file,
            self.objective_file,
            self.mission_file
        ]:

            if (
                file_handle
                and
                not file_handle.closed
            ):

                file_handle.flush()
                file_handle.close()


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    mission_manager = MissionManager()

    try:

        rclpy.spin(
            mission_manager
        )

    except KeyboardInterrupt:

        mission_manager.get_logger().info(
            "Mission Manager stopped"
        )

    finally:

        mission_manager.close_log_files()

        mission_manager.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
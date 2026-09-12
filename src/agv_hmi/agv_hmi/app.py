#!/usr/bin/env python3
import sys
import signal
from PyQt5.QtWidgets import QApplication

import rclpy

from agv_hmi.storage import WaypointStorage
from agv_hmi.ros_worker import RosWorker
from agv_hmi.ui.main_window import MainWindow


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Enable Ctrl+C in terminal to terminate Qt application
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Initialize Qt Application
    app = QApplication(sys.argv)
    app.setApplicationName("AGV HMI")

    # Storage and ROS Worker
    storage = WaypointStorage()
    ros_worker = RosWorker()
    ros_worker.start()

    # Create and display Main Window
    window = MainWindow(storage, ros_worker)
    window.show()

    # Qt Event Loop
    exit_code = app.exec_()

    # Clean shutdown
    ros_worker.stop()
    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()

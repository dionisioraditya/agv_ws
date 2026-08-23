import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource

import xacro

def generate_launch_description():
    pkg_description = get_package_share_directory('agv_description')

    # 1. Model Robot (URDF)
    xacro_file = os.path.join(pkg_description, 'urdf', 'agv.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    pkg_controller = get_package_share_directory('agv_controller')
    twist_mux_config = os.path.join(pkg_controller, 'config', 'twist_mux.yaml')

    actions = [
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            parameters=[twist_mux_config],
            remappings=[
                ('cmd_vel_out', 'cmd_vel')
            ]
        ),
        # Robot State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),
        # Odom Translator (Motor & Encoder)
        Node(
            package='agv_controller',
            executable='odom_translator.py',
            output='screen'
        ),
        # IMU BNO055
        Node(
            package='agv_filmware',
            executable='bno055_driver'
        ),
    ]

    # RPLidar A2M8
    try:
        pkg_lidar = get_package_share_directory('rplidar_ros')
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_lidar, 'launch', 'rplidar_a2m8_launch.py')
                ),
                launch_arguments={
                    'serial_port': '/dev/rplidar',
                    'frame_id': 'lidar_head-v2'
                }.items()
            )
        )
    except PackageNotFoundError:
        actions.append(LogInfo(msg="Package 'rplidar_ros' not found. Skipping lidar launch."))

    # Astra Camera
    try:
        pkg_astra = get_package_share_directory('astra_camera')
        actions.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(pkg_astra, 'launch', 'astra_pro.launch.xml')
                ),
                launch_arguments={
                    'uvc_product_id': '0x050f',
                    'camera_frame': 'camera_lens_1-v2'
                }.items()
            )
        )
    except PackageNotFoundError:
        actions.append(LogInfo(msg="Package 'astra_camera' not found. Skipping astra camera launch."))

    return LaunchDescription(actions)

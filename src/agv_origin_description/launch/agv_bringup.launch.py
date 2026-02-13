import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_description = get_package_share_directory('agv_origin_description')
    pkg_lidar = get_package_share_directory('rplidar_ros')

    # 1. Model Robot (URDF)
    xacro_file = os.path.join(pkg_description, 'urdf', 'agv_origin.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # 2. Node Hardware
    return LaunchDescription([
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
            parameters=[{'robot_description': robot_description}]
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
        # RPLidar A2M8
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_lidar, 'launch', 'rplidar_a2m8_launch.py')
            ),
            launch_arguments={
                'serial_port': '/dev/rplidar',
                'frame_id': 'lidar_1'
            }.items()
        )
    ])
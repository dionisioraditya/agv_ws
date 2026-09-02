import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_agv_description = get_package_share_directory('agv_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Try default hospital world
    try:
        hospital_share = get_package_share_directory('aws_robomaker_hospital_world')
        default_world = os.path.join(hospital_share, 'worlds', 'hospital.world')
    except PackageNotFoundError:
        default_world = 'empty'

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Gazebo world file path or alias (hospital / empty)'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to false for headless simulation'
    )

    x_pose_arg = DeclareLaunchArgument(
        name='x_pose',
        default_value='0.0',
        description='Initial x spawn position in Gazebo'
    )

    y_pose_arg = DeclareLaunchArgument(
        name='y_pose',
        default_value='-2.5',
        description='Initial y spawn position in Gazebo'
    )

    # 1. Gazebo Simulation with Robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'use_sim_time': 'true'
        }.items()
    )

    # 2. SLAM Toolbox Online Async
    slam_params = os.path.join(pkg_agv_description, 'config', 'mapper_params_online_async.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': slam_params
        }.items()
    )

    # 3. RViz2
    rviz_config_file = os.path.join(
        pkg_agv_description, 'config', 'slam_tool_box.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        x_pose_arg,
        y_pose_arg,
        gazebo_launch,
        slam_launch,
        rviz_node
    ])

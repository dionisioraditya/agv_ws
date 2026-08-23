import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    pkg_agv_description = get_package_share_directory('agv_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_config = os.path.join(pkg_agv_description, 'config', 'nav2_params.yaml')

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'gui': 'true'
        }.items()
    )

    # 2. Nav2 Navigation Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_config,
            'autostart': 'true',
        }.items()
    )

    # 3. RViz2
    rviz_config_file = os.path.join(
        pkg_agv_description, 'config', 'display.rviz'
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
        gazebo_launch,
        nav2_launch,
        rviz_node
    ])

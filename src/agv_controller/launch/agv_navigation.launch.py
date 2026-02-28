import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    pkg_origin_description = get_package_share_directory('agv_origin_description')
    nav2_config = os.path.join(pkg_origin_description, 'config', 'nav2_params_myplanner.yaml')

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    return LaunchDescription([
        GroupAction(
            actions=[
                SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav2_launch_file),
                    launch_arguments={
                        'use_sim_time': 'False',
                        'params_file': nav2_config,
                        'autostart': 'True',
                    }.items()
                )
            ]
        )
    ])
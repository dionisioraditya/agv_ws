import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Direktori package
    pkg_agv_description = get_package_share_directory('agv_origin_description')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_agv_controller = get_package_share_directory('agv_controller')

    # 1. Bringup robot
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv_description, 'launch', 'agv_bringup.launch.py')
        )
    )

    # 2. SLAM Toolbox Localization
    slam_params_file = os.path.join(
        pkg_agv_description, 'config', 'mapper_params_online_async.yaml'
    )
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'false'
        }.items()
    )

    # 3. Navigation Controller
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv_controller, 'launch', 'agv_navigation.launch.py')
        )
    )

    # 4. RViz2
    rviz_config_file = os.path.join(
        pkg_agv_description, 'config', 'slam_tool_box.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        bringup_launch,
        localization_launch,
        nav_launch,
        rviz_node
    ])
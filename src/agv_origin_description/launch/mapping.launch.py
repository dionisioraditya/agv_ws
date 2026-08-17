import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Bringup robot
    pkg_agv_description = get_package_share_directory('agv_origin_description')
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv_description, 'launch', 'agv_bringup.launch.py')
        )
    )

    # 2. SLAM Toolbox
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        )
    )

    # 3. RViz2 dengan file konfigurasi SLAM
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
        slam_launch,
        rviz_node
    ])
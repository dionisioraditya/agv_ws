import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_agv_description = get_package_share_directory('agv_description')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_config = os.path.join(pkg_agv_description, 'config', 'nav2_params.yaml')

    map_val = context.perform_substitution(LaunchConfiguration('map'))
    if not os.path.isabs(map_val):
        map_val = os.path.abspath(map_val)

    # 1. Gazebo Simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_agv_description, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
            'yaw_pose': LaunchConfiguration('yaw_pose'),
            'use_sim_time': 'true'
        }.items()
    )

    # 2. Full Nav2 Bringup (Map Server + AMCL + Navigation Stack)
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_val,
            'params_file': nav2_config,
            'use_sim_time': 'true',
            'autostart': 'true',
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

    return [
        gazebo_launch,
        nav2_bringup_launch,
        rviz_node
    ]


def generate_launch_description():
    pkg_agv_description = get_package_share_directory('agv_description')

    try:
        hospital_share = get_package_share_directory('aws_robomaker_hospital_world')
        default_world = os.path.join(hospital_share, 'worlds', 'hospital.world')
    except PackageNotFoundError:
        default_world = 'empty'

    # Check for existing hospital map yaml
    default_map = os.path.expanduser('~/agv_ws/map_sim_hospital_save.yaml')
    if not os.path.exists(default_map):
        default_map = os.path.join(pkg_agv_description, 'maps', 'map_lab.yaml')

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Gazebo world file path or alias (hospital / empty)'
    )

    map_arg = DeclareLaunchArgument(
        name='map',
        default_value=default_map,
        description='Full path to map .yaml file'
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

    z_pose_arg = DeclareLaunchArgument(
        name='z_pose',
        default_value='0.10',
        description='Initial z spawn position in Gazebo'
    )

    yaw_pose_arg = DeclareLaunchArgument(
        name='yaw_pose',
        default_value='0.0',
        description='Initial yaw spawn orientation in Gazebo'
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        gui_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        yaw_pose_arg,
        OpaqueFunction(function=launch_setup)
    ])

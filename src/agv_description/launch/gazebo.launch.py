#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'agv_description'

    try:
        pkg_share = get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    # Add package share parent to GAZEBO_MODEL_PATH so Gazebo can load package:// meshes
    pkg_share_parent = os.path.dirname(pkg_share)
    src_parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

    gazebo_model_paths = [pkg_share_parent, src_parent]
    current_gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if current_gazebo_model_path:
        gazebo_model_paths.append(current_gazebo_model_path)

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':'.join(gazebo_model_paths)
    )

    default_xacro = os.path.join(pkg_share, 'urdf', 'agv.xacro')
    if not os.path.exists(default_xacro):
        default_xacro = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'urdf', 'agv.xacro')
        )

    default_world = os.path.join(gazebo_ros_share, 'worlds', 'empty.world')

    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=default_xacro,
        description='Absolute path to robot URDF/Xacro file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=default_world,
        description='Gazebo world file path'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Set to false to run headless'
    )

    x_pose_arg = DeclareLaunchArgument(
        name='x_pose',
        default_value='0.0',
        description='Initial x spawn position in Gazebo'
    )

    y_pose_arg = DeclareLaunchArgument(
        name='y_pose',
        default_value='0.0',
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

    # Gazebo Classic launch (gzserver + gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'server': 'true',
            'init': 'true',
            'factory': 'true',
            'force_system': 'true',
        }.items()
    )

    # Robot Description Parameter via xacro command
    robot_description_content = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Spawn Entity Node
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_agv',
        output='screen',
        arguments=[
            '-entity', 'agv',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            '-Y', LaunchConfiguration('yaw_pose'),
            '-package_to_model'
        ]
    )

    return LaunchDescription([
        set_gazebo_model_path,
        model_arg,
        use_sim_time_arg,
        world_arg,
        gui_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        yaw_pose_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
    ])

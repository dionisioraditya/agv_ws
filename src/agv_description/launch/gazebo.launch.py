#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
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

    # Check for aws_robomaker_hospital_world
    hospital_share = None
    try:
        hospital_share = get_package_share_directory('aws_robomaker_hospital_world')
        gazebo_model_paths.extend([
            os.path.join(hospital_share, 'models'),
            os.path.join(hospital_share, 'fuel_models'),
            os.path.join(hospital_share, 'worlds'),
        ])
    except PackageNotFoundError:
        pass

    current_gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if current_gazebo_model_path:
        gazebo_model_paths.append(current_gazebo_model_path)

    # Resource paths MUST include Gazebo system directories (/usr/share/gazebo-11) for shaders/textures
    gazebo_system_paths = ['/usr/share/gazebo-11', '/usr/share/gazebo']
    gazebo_resource_paths = list(gazebo_system_paths) + list(gazebo_model_paths)
    current_gazebo_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    if current_gazebo_resource_path:
        gazebo_resource_paths.append(current_gazebo_resource_path)

    full_model_path = ':'.join([p for p in dict.fromkeys(gazebo_model_paths) if os.path.exists(p)])
    full_resource_path = ':'.join([p for p in dict.fromkeys(gazebo_resource_paths) if os.path.exists(p)])

    os.environ['GAZEBO_MODEL_PATH'] = full_model_path
    os.environ['GAZEBO_RESOURCE_PATH'] = full_resource_path

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=full_model_path
    )

    set_gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=full_resource_path
    )

    # Resolve world path
    world_val = context.perform_substitution(LaunchConfiguration('world'))
    if world_val in ('hospital', 'hospital.world') and hospital_share:
        resolved_world = os.path.join(hospital_share, 'worlds', 'hospital.world')
    elif world_val in ('hospital_two_floors', 'hospital_two_floors.world') and hospital_share:
        resolved_world = os.path.join(hospital_share, 'worlds', 'hospital_two_floors.world')
    elif world_val in ('hospital_three_floors', 'hospital_three_floors.world') and hospital_share:
        resolved_world = os.path.join(hospital_share, 'worlds', 'hospital_three_floors.world')
    elif world_val in ('empty', 'empty.world'):
        resolved_world = os.path.join(gazebo_ros_share, 'worlds', 'empty.world')
    elif not os.path.isabs(world_val) and hospital_share and os.path.exists(os.path.join(hospital_share, 'worlds', world_val)):
        resolved_world = os.path.join(hospital_share, 'worlds', world_val)
    else:
        resolved_world = world_val

    # Gazebo Classic launch (gzserver + gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': resolved_world,
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
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return [
        set_gazebo_model_path,
        set_gazebo_resource_path,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node
    ]


def generate_launch_description():
    pkg_name = 'agv_description'

    try:
        pkg_share = get_package_share_directory(pkg_name)
    except PackageNotFoundError:
        pkg_share = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    default_xacro = os.path.join(pkg_share, 'urdf', 'agv.xacro')
    if not os.path.exists(default_xacro):
        default_xacro = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'urdf', 'agv.xacro')
        )

    # Default world: check if hospital world exists, otherwise empty.world
    try:
        hospital_share = get_package_share_directory('aws_robomaker_hospital_world')
        default_world = os.path.join(hospital_share, 'worlds', 'hospital.world')
    except PackageNotFoundError:
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
        description='Gazebo world file path or alias (hospital / empty)'
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

    return LaunchDescription([
        model_arg,
        use_sim_time_arg,
        world_arg,
        gui_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        yaw_pose_arg,
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    import sys
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())

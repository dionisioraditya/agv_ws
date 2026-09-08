import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_datacollector = get_package_share_directory('agv_datacollector')
    default_config_file = os.path.join(pkg_datacollector, 'config', 'collector_params.yaml')

    # Declare Launch Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to parameter YAML file'
    )

    session_name_arg = DeclareLaunchArgument(
        'session_name',
        default_value='',
        description='Name of the recording session (leave empty for timestamped auto-name)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/color/image_raw',
        description='Image topic from Orbbec Astra camera'
    )

    target_fps_arg = DeclareLaunchArgument(
        'target_fps',
        default_value='30.0',
        description='Target recording FPS'
    )

    target_width_arg = DeclareLaunchArgument(
        'target_width',
        default_value='1920',
        description='Target frame width'
    )

    target_height_arg = DeclareLaunchArgument(
        'target_height',
        default_value='1080',
        description='Target frame height'
    )

    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/home/diordty/agv_ws/video/rgb',
        description='Directory where MP4 and JSON metadata will be stored'
    )

    recorder_node = Node(
        package='agv_datacollector',
        executable='rgb_recorder',
        name='rgb_recorder_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'session_name': LaunchConfiguration('session_name'),
                'image_topic': LaunchConfiguration('image_topic'),
                'target_fps': LaunchConfiguration('target_fps'),
                'target_width': LaunchConfiguration('target_width'),
                'target_height': LaunchConfiguration('target_height'),
                'output_dir': LaunchConfiguration('output_dir'),
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        session_name_arg,
        image_topic_arg,
        target_fps_arg,
        target_width_arg,
        target_height_arg,
        output_dir_arg,
        recorder_node
    ])

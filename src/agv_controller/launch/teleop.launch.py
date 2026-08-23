#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cmd_topic_arg = DeclareLaunchArgument(
        name='cmd_vel_topic',
        default_value='cmd_vel',
        description='Topic name for velocity commands'
    )

    linear_speed_arg = DeclareLaunchArgument(
        name='default_linear_speed',
        default_value='0.4',
        description='Initial linear speed in m/s'
    )

    angular_speed_arg = DeclareLaunchArgument(
        name='default_angular_speed',
        default_value='1.0',
        description='Initial angular speed in rad/s'
    )

    teleop_node = Node(
        package='agv_controller',
        executable='keyboard_teleop.py',
        name='agv_keyboard_teleop',
        output='screen',
        prefix='xterm -e' if False else '',  # runs directly in current terminal
        parameters=[{
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'default_linear_speed': LaunchConfiguration('default_linear_speed'),
            'default_angular_speed': LaunchConfiguration('default_angular_speed'),
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        cmd_topic_arg,
        linear_speed_arg,
        angular_speed_arg,
        teleop_node,
    ])


if __name__ == '__main__':
    import sys
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    sys.exit(ls.run())

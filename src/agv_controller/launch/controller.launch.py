import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description(): 

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="false",

    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
        description="Radius of the wheels in meters"
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.3",
        description="Distance between the wheels in meters"
    )

    #use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    controller_config = os.path.join(
        get_package_share_directory('agv_controller'),
        'config',
        'simple_velocity_controller.yaml'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ]
        
    )

    simple_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'simple_velocity_controller',
            '--controller-manager',
            '/controller_manager'
        ]
    )

    simple_controller_py = Node(
        package='agv_controller',
        executable='simple_controller.py',
        parameters=[
            {'wheel_radius': wheel_radius},
            {'wheel_separation': wheel_separation}
        ],
        #condition = IfCondition(use_python)
    )
    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        #controller_manager_node, 
        joint_state_broadcaster_spawner,
        simple_controller,
        simple_controller_py
    ])


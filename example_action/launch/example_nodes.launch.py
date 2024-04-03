from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_action',
            executable='example_action_server',
            output='screen'
        ),
        Node(
            package='example_action',
            executable='example_action_client',
            output='screen'
        )
    ])
